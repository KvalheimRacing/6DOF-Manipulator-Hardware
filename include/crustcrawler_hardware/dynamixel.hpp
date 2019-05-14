#pragma once

#include <boost/optional.hpp>
#include <cstdint>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include <dynamixel_sdk/dynamixel_sdk.h>

namespace dynamixel {
	using ServoID = uint8_t;
	using ModelID = uint16_t;

	enum class Addr : uint16_t {
		/* EEPROM (read-only) */
		ModelNumber        = 0, //
		FirmwareVersion    = 6,
		/* EEPROM (read-slow write) */
		ReturnDelay        = 9, // Time between reception of instruction packet and return of status packet
		OperatingMode      = 11,
		InternalMaxTemp    = 31,
		MaxTorque          = 38,
		/* RAM (read-write) */
		TorqueEnable       = 64, // Turns motor on/off
		LED                = 65,
		HardwareError      = 70,
		GoalPWM            = 100,
		GoalCurrent        = 102,
		GoalVelocity       = 104,
		GoalPosition       = 116,
		Moving             = 122,
		PresentPWM         = 124,
		PresentCurrent     = 126,
		PresentVelocity    = 128,
		PresentPosition    = 132,
		PresentVoltage     = 144,
		PresentTemperature = 146,
	};

	enum class Result : int {
		Success = COMM_SUCCESS,  	  // tx or rx packet communication success
		PortBusy = COMM_PORT_BUSY,  	  // Port is busy (in use)
		TxFail = COMM_TX_FAIL,		  // Failed transmit instruction packet
		RxFail = COMM_RX_FAIL,		  // Failed get status packet
		TxError = COMM_TX_ERROR,	  // Incorrect instruction packet
		RxWaiting = COMM_RX_WAITING,	  // Now recieving status packet
		RxTimeout = COMM_RX_TIMEOUT,	  // There is no status packet
		RxCorrupt = COMM_RX_CORRUPT,	  // Incorrect status packet
		NotAvailable = COMM_NOT_AVAILABLE,//
	};

	// Pretty printing facilities for errors
	std::ostream& operator<<(std::ostream&, const Result&);

	// Controller types modeled on
	// http://support.robotis.com/en/product/actuator/dynamixel/mx_series/mx-64(2.0).htm#bookmark5
	enum class CtrlType: uint8_t {
		None     = 100,
		Position = 3,
		Velocity = 1,
		Effort   = 0,
	};

	typedef union {
		struct {
			uint8_t bit6_7: 2;
			uint8_t overload: 1;
			uint8_t electrical_shock: 1;
			uint8_t encoder_error: 1;
			uint8_t overheating: 1;
			uint8_t bit1: 1;
			uint8_t input_voltage: 1;
		} bits;
		uint8_t raw;
	} HwError;

	struct MxBulkResult {
		ServoID id;
		uint16_t pwm;
		uint16_t current;
		uint32_t velocity;
		uint32_t position;
		uint16_t input_voltage;
		uint8_t temperature;
	};

	struct IdAddrVal {
		ServoID id;
		Addr addr;
		uint32_t value;
	};

	// Forward pointer for Comm class
	class GroupReader;
	class MxBulkReader;

	class Comm{
		public:
			/**
			 * Create a new Dynamixel communication channel
			 */
			Comm(std::string device_name, int baud_rate=1000000);
			/**
			 * Ping a servo with the given ID
			 */
			boost::optional<ModelID> ping(const ServoID id) const;
			/**
			 * Scan for all connected servos and return their ID and model
			 */
			std::vector<std::pair<ServoID, ModelID>> scan();
			/**
			 * Create a group reader object that can be used to query a number of servos
			 * simultaneously for the same address repeatedly
			 */
			std::unique_ptr<GroupReader> make_reader(Addr addr, std::vector<ServoID> ids);
			/**
			 * Create a bulk reader for MX servos that read PWM, Current, Velocity
			 * and Position all at once.
			 */
			std::unique_ptr<MxBulkReader> make_mx_reader(std::vector<ServoID> ids);
			/**
			 * Write `value` to `addr` for all servos given
			 */
			Result bulk_write(const std::vector<IdAddrVal> data);
			/**
			 * POD structure that is shared between communication classes to ensure that
			 * references are always valid
			 */
			struct Data {
				std::unique_ptr<PortHandler> _port;
				std::unique_ptr<PacketHandler> _packet;
			};
		private:
			std::shared_ptr<Data> _data;
	};

	/**
	 * Read the given address from all specified servos
	 */
	class GroupReader {
		friend class Comm;
		public:
			/**
			 * Blocking read data from the servos
			 */
			std::vector<std::pair<ServoID, uint32_t>> read_data();

		private:
			GroupReader(const Addr addr, std::vector<ServoID> ids, std::shared_ptr<Comm::Data> comm);

			const Addr _addr;
			const std::vector<ServoID> _ids;
			std::shared_ptr<Comm::Data> _comm;
			GroupSyncRead _reader;
	};

	/**
	 * Read PWM, current, velocity and position from MX servos
	 */
	class MxBulkReader {
		friend class Comm;
		public:
			/**
			 * Blocking read data from the servos
			 */
			std::vector<MxBulkResult> read_data();

		private:
			MxBulkReader(std::vector<ServoID> ids, std::shared_ptr<Comm::Data> comm);

			static constexpr uint16_t _len = 2 + 2 + 4 + 4 + 4 + 4 + 2 + 1;
			static constexpr uint16_t _start_addr = static_cast<uint16_t>(Addr::PresentPWM);
			std::vector<ServoID> _ids;
			std::shared_ptr<Comm::Data> _comm;
			GroupSyncRead _reader;
	};

	// Define helper functions to convert raw bytes
	double from_pos(const uint32_t pos);
	uint32_t to_pos(const double pos);
	double from_vel(const uint32_t vel);
	uint32_t to_vel(const double vel);
	double from_eff(const uint16_t eff);
	uint16_t to_eff(const double eff);
	double from_pwm(const uint16_t pwm);
	double from_voltage(const uint16_t volt);
	double from_temp(const uint8_t temp);
}
