/*
trossenrobots.h - openframeworks based classes for managing trossen robots
Copyright (c) 2016 Mark J Shavlik.  All right reserved.This file is part of myRobotSketch.

myRobotSketch is free software : you can redistribute it and / or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

myRobotSketch is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with myRobotSketch.If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <stdint.h>
#include <utility>    
#include <map>
#include <iostream>
#include <string>
#include <sstream>     

namespace RobotArtists {

	// from GlobalArm.h

	/* RAM REGISTER ADDRESSES */
	enum Registers {
		AX_TORQUE_ENABLE = 24, AX_LED, AX_CW_COMPLIANCE_MARGIN, AX_CCW_COMPLIANCE_MARGIN, AX_CW_COMPLIANCE_SLOPE, AX_CCW_COMPLIANCE_SLOPE,
		AX_GOAL_POSITION_L, AX_GOAL_POSITION_H, AX_GOAL_SPEED_L, AX_GOAL_SPEED_H, AX_TORQUE_LIMIT_L, AX_TORQUE_LIMIT_H, AX_PRESENT_POSITION_L, AX_PRESENT_POSITION_H,
		AX_PRESENT_SPEED_L, AX_PRESENT_SPEED_H, AX_PRESENT_LOAD_L, AX_PRESENT_LOAD_H, AX_PRESENT_VOLTAGE, AX_PRESENT_TEMPERATURE, AX_REGISTERED_INSTRUCTION,
		AX_PAUSE_TIME, AX_MOVING, AX_LOCK, AX_PUNCH_L, AX_PUNCH_H
	};

	enum ArmIDs {	PINCHER_ARMID = 1, REACTOR_ARMID, WIDOWX};

	enum ArmServoCounts {	PINCHER_SERVO_COUNT=5, REACTOR_SERVO_COUNT=8, WIDOWX_SERVO_COUNT = 6};

	enum ServoIDs {	FIRST_SERVO=1,
		WIDOWX_SID_BASE = FIRST_SERVO, WIDOWX_SID_SHOULDER, WIDOWX_SID_ELBOW, WIDOWX_SID_WRIST, WIDOWX_SID_WRISTROT, WIDOWX_SID_GRIP,
		REACTOR_SID_BASE = FIRST_SERVO, REACTOR_SID_RSHOULDER, REACTOR_SID_LSHOULDER, REACTOR_SID_RELBOW, REACTOR_SID_LELBOW, REACTOR_SID_WRIST, REACTOR_SID_WRISTROT, REACTOR_SID_GRIP,
		PINCHER_SID_BASE = FIRST_SERVO, PINCHER_SID_SHOULDER, PINCHER_SID_ELBOW, PINCHER_SID_WRIST, PINCHER_SID_GRIP
	};

	// low level commands
	enum robotLowLevelCommand : uint8_t {
		unKnownCommand = 255, NoArmCommand = 0, EmergencyStopCommand = 17, SleepArmCommand = 96, HomeArmCommand = 80, HomeArm90Command = 88,
		setArm3DCylindricalStraightWristAndGoHomeCommand = 48, setArm3DCartesian90DegreeWristAndGoHomeCommand = 40,
		setArm3DCartesianStraightWristAndGoHomeCommand = 32, setArm3DCylindrical90DegreeWristAndGoHomeCommand = 56,
		setArmBackhoeJointAndGoHomeCommand = 64, IDPacketCommand = 80, getServoRegisterCommand = 129, setServoRegisterCommand = 130, analogReadCommand = 200
	};

	// OF Free, Trossen specific so it can be used w/o openframeworks

	// lib is not dev'd or tested for multi threading yet

	//http://learn.trossenrobotics.com/arbotix/arbotix-communication-controllers/31-arm-link-reference.html
	//http://learn.trossenrobotics.com/arbotix/arbotix-getting-started/38-arbotix-m-hardware-overview.html#&panel1-8

	// from firmware IKM_BACKHOE not 100% supported
	enum robotArmMode { IKM_IK3D_CARTESIAN, IKM_IK3D_CARTESIAN_90, IKM_CYLINDRICAL, IKM_CYLINDRICAL_90, IKM_BACKHOE, IKM_NOT_DEFINED };

	enum robotArmJointType { X, Y, Z, wristAngle, wristRotate, Gripper, JointNotDefined };

	// only 1 supported
	enum RobotTypeID { PhantomXReactorArm, PhantomXPincherArm, WidowX, unknownRobotType, AllRobotTypes };

	typedef std::pair<robotArmMode, RobotTypeID> robotType;
	typedef std::pair<robotType, robotArmJointType> SpecificJoint; // backhoe gets different handling, see is spec. Its not fully supported here

	inline robotType createRobotType(robotArmMode mode, RobotTypeID id) {
		return robotType(mode, id);
	}
	inline robotType createDefaultRobotType(RobotTypeID id) {
		return robotType(IKM_CYLINDRICAL, id);
	}
	inline robotType createUndefinedRobotType() {
		return createRobotType(IKM_NOT_DEFINED, unknownRobotType);
	}
	inline bool robotTypeIsError(robotType type) {
		return type == createUndefinedRobotType();
	}
	inline SpecificJoint createJoint(robotArmJointType joint, robotArmMode mode, RobotTypeID id) {
		return SpecificJoint(createRobotType(mode, id), joint);
	}

	// tracing helper
	inline std::string echoJointType(SpecificJoint joint) {
		std::stringstream buffer;
		buffer << "SpecificJoint<robotType, robotArmJointType> = " << joint.first.first << ", " << joint.first.second << ", " << joint.second;
		return buffer.str();
	}

	// initial attempt at a generic robot definition, bugbug once understood move to the right object location
	class RobotBaseClass  {
	public:
		static uint8_t minDelta() { return 0; }
		static uint8_t maxDelta() { return 254; }
	};

	// pure virtual base class, low level data without range checking so only use derived classes
	// the Interbotix software uses a block of memory to manage a robot arm.  This class manages that block of memory. its a pure virtual class as its too
	// low level, missing things like range checking
	class RobotState : public RobotBaseClass {
	public:
		static const uint16_t count = 17;
		robotLowLevelCommand getStartCommand(robotType type);

		RobotState(uint8_t *pose) { setPose(pose); }

		virtual void virtfunction() = 0;
		void setPose(uint8_t *pose) { this->pose = pose; }
		void echoRawData();
		void set(uint16_t offset, uint8_t b);
		void setDelta(uint8_t value = 128) { if (value > maxDelta()) value = maxDelta(); set(deltaValBytesOffset, value); }
		void setButton(uint8_t value = 0) { set(buttonByteOffset, value); }
		void setLowLevelCommand(uint8_t cmd = 0) { set(extValBytesOffset, cmd); }
		void setLowLevelX(int x, int magicNumer=0) { set(xHighByteOffset, xLowByteOffset, x + magicNumer); } // no validation at this level use with care
		void setLowLevelY(int y) { set(yHighByteOffset, yLowByteOffset, y); }
		void setLowLevelZ(int z) { if (z > 250) z = 250; set(zHighByteOffset, zLowByteOffset, z); }
		void setLowLevelWristAngle(int a, int magicNumer = 90) { set(wristAngleHighByteOffset, wristAngleLowByteOffset, a + magicNumer); };
		void setLowLevelWristRotate(int a) { set(wristRotateHighByteOffset, wristRotateLowByteOffset, a); };
		void setLowLevelGripper(int distance) { set(gripperHighByteOffset, gripperLowByteOffset, distance); };

		int getX() { return get(xHighByteOffset, xLowByteOffset); }
		int getY() { return get(yHighByteOffset, yLowByteOffset); }
		int getZ() { return get(zHighByteOffset, zLowByteOffset); }
		int getWristAngle() { return get(wristAngleHighByteOffset, wristAngleLowByteOffset); }
		int getWristRotate() { return get(wristRotateHighByteOffset, wristRotateLowByteOffset); }
		int getGripper() { return get(gripperHighByteOffset, gripperLowByteOffset); }

		uint8_t *getPose();

		void set(uint16_t high, uint16_t low, int val);
		int get(uint16_t high, uint16_t low);
		static const uint16_t headerByteOffset = 0;
		static const uint16_t xHighByteOffset = 1;
		static const uint16_t xLowByteOffset = 2;
		static const uint16_t yHighByteOffset = 3;
		static const uint16_t yLowByteOffset = 4;
		static const uint16_t zHighByteOffset = 5;
		static const uint16_t zLowByteOffset = 6;
		static const uint16_t wristAngleHighByteOffset = 7;
		static const uint16_t wristAngleLowByteOffset = 8;
		static const uint16_t wristRotateHighByteOffset = 9;
		static const uint16_t wristRotateLowByteOffset = 10;
		static const uint16_t gripperHighByteOffset = 11;
		static const uint16_t gripperLowByteOffset = 12;
		static const uint16_t deltaValBytesOffset = 13;
		static const uint16_t buttonByteOffset = 14;//bugbug not supported
		static const uint16_t extValBytesOffset = 15;
		static const uint16_t checksum = 16;
		uint8_t RobotState::calcChkSum(uint8_t *pose, int start, int end);
		uint8_t getChkSum(); // hide to prevent using data out side of this object as much as possible

		uint8_t lowByte(uint16_t a) { return a % 256; }
		uint8_t highByte(uint16_t a) { return (a / 256) % 256; }
		uint8_t *pose = nullptr; // data to send
	};

	class RobotCommandInterface : public RobotState {
	public:
		RobotCommandInterface(uint8_t *pose) : RobotState(pose) {  }
	private:
		virtual void virtfunction() {};
	};

	//bugbug at some point this needs to be moved out of a trossen specific file as its OF dependent

	// works for any trossen robot
	class ofRobotSerial : public ofSerial {
	public:
		ofRobotSerial() {}

		bool waitForSerial(int retries);
		int readAllBytes(uint8_t* bytes, int bytesRequired = 5);
		int readBytesInOneShot(uint8_t* bytes, int bytesMax = 100);
		int readLine(uint8_t* bytes, int bytesMax = 100);
		void write(uint8_t* data, int count);
		robotType waitForRobot(string& name, int retries);
		vector <ofSerialDeviceInfo>& getDevices() {
			buildDeviceList();
			return devices;
		}
		//http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm
		int getPosition(ServoIDs id) { return getServoRegister(id, AX_PRESENT_POSITION_L, 2); }
		void setPosition(ServoIDs id, int value) { setServoRegister(id, AX_PRESENT_POSITION_L, 1, value); }
		void setLED(ServoIDs id, int value) { setServoRegister(id, AX_LED, 1, value); }
		int  getLED(ServoIDs id) { return getServoRegister(id, AX_LED, 1); }
		int  getTempature(ServoIDs id) { return getServoRegister(id, AX_PRESENT_TEMPERATURE, 1); }
		int  getVoltage(ServoIDs id) { return getServoRegister(id, AX_PRESENT_VOLTAGE, 1); }

		void getPose();

	protected:
		int getServoRegister(ServoIDs id, Registers registerNumber, int length);
		void setServoRegister(ServoIDs id, Registers registerNumber, int length, int dataToSend);
		void echoRawBytes(uint8_t *bytes, int count);
		bool idPacket(uint8_t *bytes, int size);
		robotType ArmIDResponsePacket(uint8_t *bytes, int count);
	};

	class RobotValueRanges {
	public:

		std::map<SpecificJoint, int> minValue;
		std::map<SpecificJoint, int> maxValue;
		std::map<SpecificJoint, int> defaultValue;
	};

	// stores only valid values for specific joints, does validation, defaults and other things, but no high end logic around motion
	class RobotJoints : public RobotState {
	public:
		// constructor required
		RobotJoints() : RobotState(nullptr) {}
		RobotJoints(const robotType& typeOfRobot) : RobotState(nullptr) { this->typeOfRobot = typeOfRobot; };
		RobotJoints(uint8_t* pose, const robotType& typeOfRobot) : RobotState(pose) { this->typeOfRobot = typeOfRobot; }
		RobotJoints(uint8_t* pose) : RobotState(pose) { typeOfRobot = createUndefinedRobotType(); }

		void setDefault(SpecificJoint joint, int value);
		void setMin(SpecificJoint joint, int value);
		void setMax(SpecificJoint joint, int value);

		void setX(int x);
		void setY(int y);
		void setZ(int z);
		void setWristAngle(int a);
		void setWristRotate(int a);
		void setGripper(int distance);

		static void oneTimeSetup();

		bool inRange(robotArmJointType type, int value);

		int getDefaultValue(robotArmJointType type);

		// ranges based on device
		int getMin(robotArmJointType type);
		int getMax(robotArmJointType type);
		int getMid(robotArmJointType type);
		bool isCartesion() { return (typeOfRobot.first == IKM_IK3D_CARTESIAN || typeOfRobot.first == IKM_IK3D_CARTESIAN_90); }
		bool isCylindrical() { return (typeOfRobot.first == IKM_CYLINDRICAL || typeOfRobot.first == IKM_CYLINDRICAL_90); }
		int addMagicNumber() { return isCylindrical() ? 0 : 512; }

		//Set 3D Cartesian mode / straight wrist and go to home etc
		robotType setStartState();
		robotType getRobotType() { return typeOfRobot; }
		void setDefaultState();
		void setUserDefinedRanges(SpecificJoint joint, shared_ptr<RobotValueRanges>);

	protected:

	private:
		// user defined ranges
		static RobotValueRanges hardwareRanges;
		shared_ptr<RobotValueRanges> userDefinedRanges = nullptr;

		static void set(SpecificJoint type, int min, int max, int defaultvalue);
		robotType typeOfRobot;// required
		void virtfunction() {};
	};
}