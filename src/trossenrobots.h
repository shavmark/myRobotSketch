#pragma once

#include <stdint.h>
#include <utility>    
#include <map>
#include <iostream>
#include <string>
#include <sstream>     

namespace RobotArtists {

	// OF Free, Trossen specific so it can be used w/o openframeworks

	// lib is not dev'd or tested for multi threading yet

	//http://learn.trossenrobotics.com/arbotix/arbotix-communication-controllers/31-arm-link-reference.html


	// from firmware IKM_BACKHOE not 100% supported
	enum robotArmMode { IKM_IK3D_CARTESIAN, IKM_IK3D_CARTESIAN_90, IKM_CYLINDRICAL, IKM_CYLINDRICAL_90, IKM_BACKHOE, IKM_NOT_DEFINED };
	enum robotArmJointType { X, Y, Z, wristAngle, wristRotate, Gripper, JointNotDefined };
	// only 1 supported
	enum RobotTypeID { InterbotiXPhantomXReactorArm, unknownRobotType };

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
	// low level commands
	enum robotLowLevelCommand : uint8_t {
		unKnownCommand = 255, NoArmCommand = 0, EmergencyStop = 17, SleepArm = 96, HomeArm = 80, HomeArm90 = 88, setArm3DCylindricalStraightWristAndGoHome = 48,
		setArm3DCartesian90DegreeWristAndGoHome = 40, setArm3DCartesianStraightWristAndGoHome = 32, setArm3DCylindrical90DegreeWristAndGoHome = 56, setArmBackhoeJointAndGoHome = 64
	};

	// tracing helper
	inline std::string echoJointType(SpecificJoint joint) {
		std::stringstream buffer;
		buffer << "SpecificJoint<robotType, robotArmJointType> = " << joint.first.first << ", " << joint.first.second << ", " << joint.second;
		return buffer.str();
	}

	// pure virtual base class, low level data without range checking so only use derived classes
	class RobotJointsState {
	public:

		static const uint16_t count = 17;
		robotLowLevelCommand getStartCommand(robotType type);

	protected:

		RobotJointsState(uint8_t *data) { setData(data); }

		virtual void virtfunction() = 0;
		void setData(uint8_t *data) { this->data = data; }
		void echo();
		void set(uint16_t offset, uint8_t b);
		void setLowLevelCommand(robotLowLevelCommand cmd) { set(extValBytesOffset, cmd); };
		void setDelta(uint8_t value = 128) { set(deltaValBytesOffset, value); }
		void setButton(uint8_t value = 0) { set(buttonByteOffset, value); }

		void setLowLevelX(int x, int magicNumer) { set(xHighByteOffset, xLowByteOffset, x + magicNumer); } // no validation at this level use with care
		void setLowLevelY(int y) { set(yHighByteOffset, yLowByteOffset, y); }
		void setLowLevelZ(int z) { set(zHighByteOffset, zLowByteOffset, z); }
		void setLowLevelWristAngle(int a) { set(wristAngleHighByteOffset, wristAngleLowByteOffset, a + 90); };
		void setLowLevelWristRotate(int a) { set(wristRotateHighByteOffset, wristRotateLowByteOffset, a); };
		void setLowLevelGripper(int distance) { set(gripperHighByteOffset, gripperLowByteOffset, distance); };

		int getX() { return get(xHighByteOffset, xLowByteOffset); }
		int getY() { return get(yHighByteOffset, yLowByteOffset); }
		int getZ() { return get(zHighByteOffset, zLowByteOffset); }
		int getWristAngle() { return get(wristAngleHighByteOffset, wristAngleLowByteOffset); }
		int getWristRotate() { return get(wristRotateHighByteOffset, wristRotateLowByteOffset); }
		int getGripper() { return get(gripperHighByteOffset, gripperLowByteOffset); }

		uint8_t *getData();

	private:
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

		uint8_t getChkSum(); // hide to prevent using data out side of this object as much as possible

		uint8_t lowByte(uint16_t a) { return a % 256; }
		uint8_t highByte(uint16_t a) { return (a / 256) % 256; }
		uint8_t *data = nullptr; // data to send
	};

	class RobotValueRanges {
	public:

		std::map<SpecificJoint, int> minValue;
		std::map<SpecificJoint, int> maxValue;
		std::map<SpecificJoint, int> defaultValue;
	};

	// stores only valid values for specific joints, does validation, defaults and other things, but no high end logic around motion
	class RobotJoints : public RobotJointsState {
	public:
		// constructor required
		RobotJoints(const robotType& typeOfRobot) : RobotJointsState(nullptr) { this->typeOfRobot = typeOfRobot; };
		RobotJoints(uint8_t* data, const robotType& typeOfRobot) : RobotJointsState(data) { this->typeOfRobot = typeOfRobot; }
		RobotJoints(uint8_t* data) : RobotJointsState(data) { typeOfRobot = createUndefinedRobotType(); }

		void setDefault(SpecificJoint joint, int value);
		void setMin(SpecificJoint joint, int value);
		void setMax(SpecificJoint joint, int value);

		void setX(int x);
		void setY(int y);
		void setZ(int z);
		void setWristAngle(int a);
		void setWristRotate(int a);
		void setGripper(int distance);

		void oneTimeSetup();

		bool inRange(robotArmJointType type, int value);

		int getDefaultValue(robotArmJointType type);

		// ranges based on device
		int getMin(robotArmJointType type);
		int getMax(robotArmJointType type);
		int getMid(robotArmJointType type);
		int getDeltaDefault() { return deltaDefault; }
		bool isCartesion() { return (typeOfRobot.first == IKM_IK3D_CARTESIAN || typeOfRobot.first == IKM_IK3D_CARTESIAN_90); }
		bool isCylindrical() { return (typeOfRobot.first == IKM_CYLINDRICAL || typeOfRobot.first == IKM_CYLINDRICAL_90); }
		int addMagicNumber() { return isCylindrical() ? 0 : 512; }
		bool is90() { return typeOfRobot.first == IKM_IK3D_CARTESIAN_90 || typeOfRobot.first == IKM_CYLINDRICAL_90; }

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
		int deltaDefault = 255;

		void set(SpecificJoint type, int min, int max, int defaultvalue);
		robotType typeOfRobot;// required
		void virtfunction() {};

	};

}