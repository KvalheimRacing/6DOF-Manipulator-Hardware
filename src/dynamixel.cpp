#include "crustcrawler_hardware/dynamixel.hpp"

#include <iostream>
#include <stdexcept>
#include <string>

namespace dynamixel {
	// Dynamixel servo memory size map
	static constexpr uint8_t memsize[256] = {
		2, 0, 4, 0, 0, 0, 1, 1, // 00
		1, 1, 1, 1, 1, 1, 0, 0, // 08
		0, 0, 0, 0, 4, 0, 0, 0, // 16
		4, 0, 0, 0, 0, 0, 0, 1, // 24
		2, 0, 2, 0, 2, 0, 2, 0, // 32
		4, 0, 0, 0, 4, 0, 0, 0, // 40
		4, 0, 0, 0, 4, 0, 0, 0, // 48
		0, 0, 0, 0, 0, 0, 0, 1, // 56
		1, 1, 0, 0, 1, 1, 1, 0, // 64
		0, 0, 0, 0, 2, 0, 2, 0, // 72
		2, 0, 2, 0, 2, 0, 0, 0, // 80
		2, 0, 2, 0, 0, 0, 0, 0, // 88
		0, 0, 1, 0, 2, 0, 2, 0, // 96
		4, 0, 0, 0, 4, 0, 0, 0, // 104
		4, 0, 0, 0, 4, 0, 0, 0, // 112
		2, 0, 1, 1, 2, 0, 2, 0, // 120
		4, 0, 0, 0, 4, 0, 0, 0, // 128
		4, 0, 0, 0, 4, 0, 0, 0, // 136
		2, 0, 1, 0, 0, 0, 0, 0 // 144
	};

	std::ostream& operator<<(std::ostream& os, const Result& res) {
		switch(res) {
			case Result::Success:
				os << "success";
				break;
			case Result::PortBusy:
				os << "USB port busy";
				break;
			case Result::TxFail:
				os << "failed to transmit instruction packet (TxFail)";
				break;
			case Result::RxFail:
				os << "failed to receive instruction packet (RxFail)";
				break;
			case Result::TxError:
				os << "incorrect instruction packet (TxError)";
				break;
			case Result::RxWaiting:
				os << "already receiving status packet (RxWaiting)";
				break;
			case Result::RxTimeout:
				os << "timeout waiting for status packet (RxTimeout)";
				break;
			case Result::RxCorrupt:
				os << "received incorrect status packet (RxCorrupt)";
				break;
			case Result::NotAvailable:
				os << "communication not available";
				break;
		}
		return os;
	}

	Comm::Comm(std::string device_name, int baud_rate) {
		_data = std::make_shared<Data>();
		_data->_port.reset(PortHandler::getPortHandler(device_name.c_str()));
		// Force protocol 2.0
		_data->_packet.reset(PacketHandler::getPacketHandler(2.0));

		if(!_data->_port->openPort()){
			throw std::runtime_error("Could not open serial port: '" +
					device_name + "'");
		}
		if(!_data->_port->setBaudRate(baud_rate)){
			throw std::runtime_error("Could not set baud rate: " +
					std::to_string(baud_rate));
		}
	}

	boost::optional<ModelID> Comm::ping(const ServoID id) const {
		ModelID result;
		uint8_t err;
		const auto ping_res = static_cast<Result>(_data->_packet->ping(_data->_port.get(), id, &result, &err));
		if(ping_res == Result::Success){
			return result;
		} else {
			std::cerr << "\033[31m[Comm::ping] problem sending ping to servo with ID:\033[0m "
				<< static_cast<int>(id) << ", error: "
				<< ping_res << std::endl;
			std::cerr << "\033[31m[Comm::ping] additional error:\033[0m "
				<< _data->_packet->getRxPacketError(err)
				<< std::endl;
		}
		return {};
	}

	std::vector<std::pair<ServoID, ModelID>> Comm::scan() {
		std::vector<std::pair<ServoID, ModelID>> res;
		std::vector<uint8_t> ids;
		const auto scan_res = static_cast<Result>(_data->_packet->broadcastPing(_data->_port.get(), ids));
		if(scan_res == Result::Success) {
			for(auto id : ids) {
				if(auto model = ping(id)) {
					res.push_back(std::make_pair(id, *model));
				} else {
					std::cerr << "\033[31m[Comm::scan] problem pinging servo with ID:\033[0m "
						<< static_cast<int>(id) << std::endl;
				}
			}
		} else {
			std::cerr << "\033[31m[Comm::scan] problem broadcasting ping:\033[0m " << scan_res << std::endl;
		}
		return res;
	}

	std::unique_ptr<GroupReader> Comm::make_reader(Addr addr, std::vector<ServoID> ids) {
		return std::unique_ptr<GroupReader>(new GroupReader(addr, std::move(ids), _data));
	}

	std::unique_ptr<MxBulkReader> Comm::make_mx_reader(std::vector<ServoID> ids) {
		return std::unique_ptr<MxBulkReader>(new MxBulkReader(std::move(ids), _data));
	}

	Result Comm::bulk_write(const std::vector<IdAddrVal> data) {
		auto writer = GroupBulkWrite(_data->_port.get(), _data->_packet.get());
		for(auto update : data) {
			auto raw_addr = static_cast<uint16_t>(update.addr);
			auto addr_size = memsize[raw_addr];
			if(!writer.addParam(update.id, raw_addr, addr_size,
						reinterpret_cast<uint8_t*>(&update.value))) {
				std::cerr << "\033[31m[Comm::bulk_write] Could not write "
					<< raw_addr << "=" << update.value
					<< " [" << addr_size << "] for ID:\033[0m "
					<< static_cast<int>(update.id)
					<< std::endl;
			}
		}
		return static_cast<Result>(writer.txPacket());
	}

	GroupReader::GroupReader(const Addr addr, std::vector<ServoID> ids,
			std::shared_ptr<Comm::Data> comm):
		_addr(addr), _ids(std::move(ids)), _comm(std::move(comm)),
		_reader(_comm->_port.get(), _comm->_packet.get(),
				static_cast<uint16_t>(_addr), memsize[static_cast<uint16_t>(_addr)]) {
		// Since we will always read the same parameters we add them now
		for(auto id : _ids) {
			_reader.addParam(id);
		}
	}

	std::vector<std::pair<ServoID, uint32_t>> GroupReader::read_data() {
		std::vector<std::pair<ServoID, uint32_t>> res;
		Result send_res = static_cast<Result>(_reader.txRxPacket());
		if(send_res == Result::Success) {
			res.reserve(_ids.size());
			auto raw_addr = static_cast<uint16_t>(_addr);
			for(auto id : _ids) {
				if(_reader.isAvailable(id, raw_addr, memsize[raw_addr])) {
					res.push_back(std::make_pair(
								id, _reader.getData(id, raw_addr, memsize[raw_addr])));
				} else {
					std::cerr << "\033[31m[GroupReader::read_data] No data available for:\033[0m "
						<< id << std::endl;
				}
			}
		} else {
			std::cerr << "\033[31m[GroupReader::read_data] Error reading from servos, error:\033[0m "
				<< send_res << std::endl;
		}
		return res;
	}

	MxBulkReader::MxBulkReader(std::vector<ServoID> ids, std::shared_ptr<Comm::Data> comm):
		_ids(std::move(ids)), _comm(std::move(comm)),
		_reader(_comm->_port.get(), _comm->_packet.get(), _start_addr, _len) {
		for(const auto& id: _ids) {
			_reader.addParam(id);
		}
	}

	std::vector<MxBulkResult> MxBulkReader::read_data() {
		std::vector<MxBulkResult> result;
		// Tell all servos that we would like to read from them
		Result send_res = static_cast<Result>(_reader.txRxPacket());
		if(send_res == Result::Success){
			// Since the above call indicates we will receive some data we reserve
			// the maximum needed space to avoid re-allocations
			result.reserve(_ids.size());
			for(auto id : _ids) {
				MxBulkResult res;
				res.id = id;
				const uint16_t pwm_addr = static_cast<uint16_t>(Addr::PresentPWM);
				if(_reader.isAvailable(id, pwm_addr, memsize[pwm_addr])) {
					res.pwm = _reader.getData(id, pwm_addr, memsize[pwm_addr]);
				} else {
					std::cerr << "\033[31m[MxBulkReader::read_data] PWM not available for servo:\033[0m "
						<< static_cast<int>(id) << std::endl;
				}
				const uint16_t current_addr = static_cast<uint16_t>(Addr::PresentCurrent);
				if(_reader.isAvailable(id, current_addr, memsize[current_addr])) {
					res.current = _reader.getData(id, current_addr, memsize[current_addr]);
				} else {
					std::cerr << "\033[31m[MxBulkReader::read_data] Current not available for servo:\033[0m "
						<< static_cast<int>(id) << std::endl;
				}
				const uint16_t velocity_addr = static_cast<uint16_t>(Addr::PresentVelocity);
				if(_reader.isAvailable(id, velocity_addr, memsize[velocity_addr])) {
					res.velocity = _reader.getData(id, velocity_addr, memsize[velocity_addr]);
				} else {
					std::cerr << "\033[31m[MxBulkReader::read_data] Velocity not available for servo:\033[0m "
						<< static_cast<int>(id) << std::endl;
				}
				const uint16_t position_addr = static_cast<uint16_t>(Addr::PresentPosition);
				if(_reader.isAvailable(id, position_addr, memsize[position_addr])) {
					res.position = _reader.getData(id, position_addr, memsize[position_addr]);
				} else {
					std::cerr << "\033[31m[MxBulkReader::read_data] Position not available for servo:\033[0m "
						<< static_cast<int>(id) << std::endl;
				}
				const uint16_t input_voltage_addr = static_cast<uint16_t>(Addr::PresentVoltage);
				if(_reader.isAvailable(id, input_voltage_addr, memsize[input_voltage_addr])) {
					res.input_voltage = _reader.getData(id, input_voltage_addr, memsize[input_voltage_addr]);
				} else {
					std::cerr << "\033[31m[MxBulkReader::read_data] Voltage not available for servo:\033[0m "
						<< static_cast<int>(id) << std::endl;
				}
				const uint16_t temperature_addr = static_cast<uint16_t>(Addr::PresentTemperature);
				if(_reader.isAvailable(id, temperature_addr, memsize[temperature_addr])) {
					res.temperature = _reader.getData(id, temperature_addr, memsize[temperature_addr]);
				} else {
					std::cerr << "\033[31m[MxBulkReader::read_data] Temperature not available for servo:\033[0m "
						<< static_cast<int>(id) << std::endl;
				}
				result.push_back(res);
			}
		} else {
			std::cerr << "\033[31m[MxBulkReader::read_data] Error reading bulk data, error:\033[0m "
				<< send_res << std::endl;
		}
		return result;
	}

	double from_pos(const uint32_t pos) {
		const auto pos2 = static_cast<int32_t>(pos);
		return (static_cast<double>(pos2) - 2048.0) * 0.00154;
	}
	uint32_t to_pos(const double pos) {
		return static_cast<uint32_t>(pos / 0.00154 + 2048.0);
	}
	// '0.229' converts the value to RPM, '9.54...' converts to rad/s
	double from_vel(const uint32_t vel) {
		const auto vel2 = static_cast<int32_t>(vel);
		return static_cast<double>(vel2) * (0.229 / 9.5492965964254);
	}
	uint32_t to_vel(const double vel) {
		return static_cast<uint32_t>(vel / (0.229 / 9.5492965964254));
	}
	double from_eff(const uint16_t eff) {
		const auto eff2 = static_cast<int16_t>(eff);
		return (static_cast<double>(eff) * 3.36) / 1000.0;
	}
	uint16_t to_eff(const double eff) {
		return static_cast<uint16_t>((eff * 1000.0) / 3.36);
	}

	double from_pwm(const uint16_t pwm) {
		double value = static_cast<double>(static_cast<int16_t>(pwm));
		value /= 885.0;
		return value;
	}

	double from_voltage(const uint16_t volt) {
		return static_cast<double>(volt) / 10.0;
	}

	double from_temp(const uint8_t temp) {
		return static_cast<double>(temp);
	}

}
