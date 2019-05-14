#include "crustcrawler_hardware/crustcrawler.hpp"
#include <iostream>

namespace crustcrawler_hardware {
	Crustcrawler::Crustcrawler(ros::NodeHandle &robot_nh) {
		ROS_DEBUG("Initializing Crustcrawler");
		// Initialize dynamixel driver
		std::string path = robot_nh.param<std::string>("device_name", "/dev/ttyUSB0");
		int baud = robot_nh.param<int>("baud_rate", 1000000);
		bool full_arm = robot_nh.param<bool>("full_arm", false);
		if(full_arm) {
			ROS_WARN("Using all joints");
			_ids = {0, 1, 2, 3, 4, 5, 6};
		} else {
			ROS_WARN("Using truncated arm");
			_ids = {0, 1, 2, 3};
		}
		ROS_DEBUG("Creating handles 'ros_control'");
		for(auto id : _ids) {
			joints[id].ctrl = dynamixel::CtrlType::None;
			// Joint 1 is a combination of ID 1 & 2, only register it once
			if(id != 2) {
				std::string name("joint_" + std::to_string((id <= 1? id + 1 : id)));
				hardware_interface::JointStateHandle state_handle(name,
						&(joints[id].position),
						&(joints[id].velocity),
						&(joints[id].effort));
				hardware_interface::JointHandle handle(state_handle, &(joints[id].command));
				joint_states_.registerHandle(state_handle);
				effort_int_.registerHandle(handle);
				velocity_int_.registerHandle(handle);
				position_int_.registerHandle(handle);
			}
		}
		ROS_DEBUG("Registering interfaces");
		registerInterface(&joint_states_);
		registerInterface(&effort_int_);
		registerInterface(&velocity_int_);
		registerInterface(&position_int_);
		ROS_DEBUG("Initializing Dynamixel driver, path: %s, baud rate: %d",
				path.c_str(), baud);
		_comm                = std::make_unique<dynamixel::Comm>(path, baud);
		_status_read         = _comm->make_mx_reader(_ids);
		_torque_read         = _comm->make_reader(dynamixel::Addr::TorqueEnable, _ids);
		_hardware_error_read = _comm->make_reader(dynamixel::Addr::HardwareError, _ids);
		// Check that we can detect the required number of servos
		const auto res = _torque_read->read_data();
		const auto expected_size = _ids.size();
		if(res.size() != expected_size) {
			const auto scan = _comm->scan();
			for(const auto& servo : scan) {
				ROS_WARN("Found servo with ID: %u, type: %u", servo.first, servo.second);
			}
			// This is very hacky, but since `dynamixel_sdk` can't properly ping ID 0
			// we allow for one less servo to be found
			ROS_FATAL("Too few servos connected, found %zu, expected %zu",
					scan.size(), expected_size - 1);
			throw std::runtime_error("Too few servos connected, found "
					+ std::to_string(scan.size()) + ", expected "
					+ std::to_string(expected_size - 1));
		}
		ROS_INFO("Crustcrawler hardware initialized");
	}

	Crustcrawler::~Crustcrawler() {
		if(_comm != nullptr){
			enable(false);
			const auto res = write_torque();
			if(res != dynamixel::Result::Success) {
				// Use `std::cerr` here instead of ROS_* since ROS will be shutdown
				// once we get here.
				//
				// Note the strange sequence below is to get color, this is read and
				// '\033[0m' resets the colors
				std::cerr << "\033[31mFATAL: Error sending torque disable, error: \033[0m"
					<< res << std::endl;
			} else {
				// Output in beautiful blue to ease users
				std::cout << "\033[34mCrustcrawler shutdown successfully \033[0m\n";
			}
		}
	}

	dynamixel::Result Crustcrawler::write_torque() {
		if(!_torque_updates.empty()){
			const auto res = _comm->bulk_write(_torque_updates);
			if(res != dynamixel::Result::Success) {
				ROS_WARN_STREAM("Error writing torque update: " << res);
			}
			_torque_updates.clear();
			return res;
		}
		return dynamixel::Result::Success;
	}

	void Crustcrawler::enable(bool enable) {
		if(enable) {
			ROS_WARN("Enabling torque on all Crustcrawler joints!");
		} else {
			ROS_INFO("Disabling torque");
		}
		std::vector<dynamixel::IdAddrVal> stop_update;
		for(const auto& id : _ids) {
			dynamixel::IdAddrVal stop;
			stop.id = id;
			stop.addr = dynamixel::Addr::TorqueEnable;
			stop.value = static_cast<uint8_t>(enable);
			stop_update.push_back(stop);
		}
		_torque_updates.swap(stop_update);
	}

	void Crustcrawler::read(const ros::Time &time, const ros::Duration &period) {
		const auto status = _status_read->read_data();
		for(const auto& stat : status) {
			joints[stat.id].position    = dynamixel::from_pos(stat.position);
			joints[stat.id].velocity    = dynamixel::from_vel(stat.velocity);
			joints[stat.id].effort      = dynamixel::from_eff(stat.current);
			joints[stat.id].pwm         = dynamixel::from_pwm(stat.pwm);
			joints[stat.id].voltage     = dynamixel::from_voltage(stat.input_voltage);
			joints[stat.id].temperature = dynamixel::from_temp(stat.temperature);
		}
	}

	void Crustcrawler::write(const ros::Time&, const ros::Duration&) {
		std::vector<dynamixel::IdAddrVal> updates;
		updates.reserve(_ids.size());
		for(auto id : _ids) {
			if(!joints[id].enabled) {
				// If the joint is not enabled ignore
				continue;
			}
			dynamixel::IdAddrVal val;
			val.id = id;
			// Special case, this ensures that ID 1 == 2
			// for commands, NOTE the `val.id = id` above
			if(id == 2) {
				id = 1;
			}
			switch(joints[id].ctrl) {
				case dynamixel::CtrlType::None:
					ROS_WARN("Tried to send command to servo %i, but it has no controller", id);
					continue;
				case dynamixel::CtrlType::Position:
					val.addr = dynamixel::Addr::GoalPosition;
					val.value = dynamixel::to_pos(joints[id].command);
					break;
				case dynamixel::CtrlType::Velocity:
					val.addr = dynamixel::Addr::GoalVelocity;
					val.value = dynamixel::to_vel(joints[id].command);
					break;
				case dynamixel::CtrlType::Effort:
					val.addr = dynamixel::Addr::GoalCurrent;
					val.value = dynamixel::to_eff(joints[id].command);
					break;
			}
			updates.push_back(val);
		}
		if(updates.size() > 0) {
			const auto res = _comm->bulk_write(updates);
			if(res != dynamixel::Result::Success) {
				ROS_WARN_STREAM("Error writing command to servos, error:" << res);
			}
		}
		write_torque();
	}

	void Crustcrawler::doSwitch(const std::list<hardware_interface::ControllerInfo> &start,
			const std::list<hardware_interface::ControllerInfo>& stop) {
		std::vector<dynamixel::IdAddrVal> mode_updates;
		for(auto info : start) {
			dynamixel::CtrlType new_ctrl = dynamixel::CtrlType::None;
			if(info.type == "position_controllers/JointPositionController") {
				new_ctrl = dynamixel::CtrlType::Position;
			} else if(info.type == "joint_state_controller/JointStateController"){
				// Ignore by default!
				continue;
			} else if(info.type == "velocity_controllers/JointVelocityController"){
				new_ctrl = dynamixel::CtrlType::Velocity;
			} else if(info.type == "effort_controllers/JointEffortController"){
				new_ctrl = dynamixel::CtrlType::Effort;
			} else if(info.type == "position_controllers/JointTrajectoryController") {
				new_ctrl = dynamixel::CtrlType::Position;
			}
			if(new_ctrl == dynamixel::CtrlType::None) {
				ROS_ERROR("No compatible control mode found for %s", info.type.c_str());
				continue;
			}
			for(auto claim : info.claimed_resources) {
				for(auto res : claim.resources){
					ROS_DEBUG("Changing %s to controller: %i", res.c_str(), static_cast<int>(new_ctrl));
					dynamixel::IdAddrVal msg;
					msg.addr = dynamixel::Addr::OperatingMode;
					msg.value = static_cast<uint8_t>(new_ctrl);
					if(res == "joint_1"){
						msg.id = 0;
						joints[0].ctrl = new_ctrl;
					} else if(res == "joint_2") {
						msg.id = 1;
						joints[1].ctrl = new_ctrl;
						joints[2].ctrl = new_ctrl;
						// NOTE this is a special case for the double joint
						dynamixel::IdAddrVal m;
						m.addr = dynamixel::Addr::OperatingMode;
						m.value = static_cast<uint8_t>(new_ctrl);
						m.id = 2;
						mode_updates.push_back(m);
					} else if(res == "joint_3") {
						msg.id = 3;
						joints[3].ctrl = new_ctrl;
					} else if(res == "joint_4") {
						msg.id = 4;
						joints[4].ctrl = new_ctrl;
					} else if(res == "joint_5") {
						msg.id = 5;
						joints[5].ctrl = new_ctrl;
					} else if(res == "joint_6") {
						msg.id = 6;
						joints[6].ctrl = new_ctrl;
					} else {
						ROS_WARN("Unknown resource claimed: %s", res.c_str());
						continue;
					}
					mode_updates.push_back(msg);
				}
			}
		}
		if(mode_updates.size() > 0) {
			const auto res = _comm->bulk_write(mode_updates);
			if(res != dynamixel::Result::Success) {
				ROS_WARN_STREAM("Error changing mode for servos, error: " << res);
			}
		}
	}

	void Crustcrawler::readStatus(const ros::Time&, const ros::Duration&) {
		const auto torque = _torque_read->read_data();
		const auto hw_error = _hardware_error_read->read_data();
		if(torque.size() == _ids.size()) {
			for(const auto& pair : torque) {
				joints[pair.first].enabled = static_cast<bool>(pair.second);
			}
		} else {
			ROS_ERROR("Not all servos responded to torque read!");
		}
		if(hw_error.size() == _ids.size()) {
			for(const auto& pair : torque) {
				joints[pair.first].err.raw = pair.second;
			}
		} else {
			ROS_ERROR("Not all servos responded to HW error read!");
		}
	}

	void Crustcrawler::produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat, const dynamixel::ServoID& id) {
		// Initial summary of joint
		stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Joint is good");
		const auto torque_str = joints[id].enabled ? "enabled" : "disabled";
		auto mode = "None";
		switch(joints[id].ctrl) {
			case dynamixel::CtrlType::None:     break;
			case dynamixel::CtrlType::Position: mode = "Position"; break;
			case dynamixel::CtrlType::Velocity: mode = "Velocity"; break;
			case dynamixel::CtrlType::Effort:   mode = "Effort"; break;
		}
		// Add data not published other places
		stat.addf("torque", "%s", torque_str);
		stat.addf("mode", "%s", mode);
		stat.addf("pwm", "%.3f", joints[id].pwm);
		stat.addf("voltage", "%.1fV", joints[id].voltage);
		stat.addf("temperature", "%.1fC", joints[id].temperature);
		// Check temperature, if high we warn users
		if(joints[id].temperature > 65) {
			stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::WARN,
					"Temperature is large %.1fC",
					joints[id].temperature);
		}
		// Check hardware error and update summary with higher value
		// status if necessary
		if(joints[id].err.bits.overload) {
			stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR,
					"Overload detected");
		}
		if(joints[id].err.bits.electrical_shock) {
			stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR,
					"Electrical shock detected");
		}
		if(joints[id].err.bits.encoder_error) {
			stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR,
					"Encoder error detected");
		}
		if(joints[id].err.bits.overheating) {
			stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR,
					"Overheating detected");
		}
		if(joints[id].err.bits.input_voltage) {
			stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR,
					"Input voltage error detected");
		}
	}
}
