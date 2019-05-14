#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>

#include "crustcrawler_hardware/dynamixel.hpp"

#define NUM_JOINTS 7

namespace crustcrawler_hardware {
	// Helper class to keep track of single joint state
	struct Joint {
		// Is the joint enabled and will actuate?
		bool enabled;
		// Desired command value
		double command;
		// Standard information about joint
		double position;
		double velocity;
		double effort;
		// Additional information gathered
		double pwm;
		double voltage;
		double temperature;
		// Current control type
		dynamixel::CtrlType ctrl;
		// Possible errors
		dynamixel::HwError err;
	};

	/**
	 * This class represents the hardware interface to the Crustcrawler robot which
	 * is `ros_control` compatible.
	 *
	 * For more information:
	 *  - https://github.com/ros-controls/ros-control/wiki/hardware_interface
	 *  - https://roscon.ros.org/2014/wp-content/uploads/2014/07/ros_control_an_overview.pdf
	 */
	class Crustcrawler : public hardware_interface::RobotHW {
		public:
			Crustcrawler(ros::NodeHandle&);
			~Crustcrawler();
			void read(const ros::Time&, const ros::Duration&) override;
			void readStatus(const ros::Time&, const ros::Duration&);
			void write(const ros::Time&, const ros::Duration&) override;
			void doSwitch(const std::list<hardware_interface::ControllerInfo>&,
					const std::list<hardware_interface::ControllerInfo>&) override;
			void enable(bool enable);
			void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper&, const dynamixel::ServoID&);
			const std::vector<dynamixel::ServoID>& connected_servos() const {return _ids;}

		private:
			dynamixel::Result write_torque();
			// References to dynamixel servos
			std::unique_ptr<dynamixel::Comm> _comm;
			std::unique_ptr<dynamixel::MxBulkReader> _status_read;
			std::unique_ptr<dynamixel::GroupReader> _torque_read;
			std::unique_ptr<dynamixel::GroupReader> _hardware_error_read;
			// ros_control interfaces:
			hardware_interface::JointStateInterface joint_states_;
			hardware_interface::EffortJointInterface effort_int_;
			hardware_interface::VelocityJointInterface velocity_int_;
			hardware_interface::PositionJointInterface position_int_;
			// State and command arrays
			Joint joints[NUM_JOINTS];
			// List of IDs in use
			std::vector<dynamixel::ServoID> _ids;
			std::vector<dynamixel::IdAddrVal> _torque_updates;
	};
}
