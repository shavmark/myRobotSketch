#include "ofutils.h"
#include "trossenrobots.h"

namespace RobotArtists {

	RobotValueRanges RobotJoints::hardwareRanges;

	int RobotJointsState::get(uint16_t high, uint16_t low) {
		int number = -1;
		if (data) {
			int number = bytes_to_u16(data[high], data[low]);
		}
		return number;
	}

	void RobotJointsState::set(uint16_t high, uint16_t low, int val) {
		TraceBaseClass() << "set " << val << std::endl;
		set(high, highByte(val));
		set(low, lowByte(val));
	}
	void RobotJoints::setX(int x) {
		TraceBaseClass() << "try to set x=" << x << std::endl;
		if (inRange(X, x)) {
			setLowLevelX(x, addMagicNumber());
		}
	}
	void RobotJoints::setY(int y) {
		TraceBaseClass() << "try to set y=" << y << std::endl;
		if (inRange(Y, y)) {
			setLowLevelY(y);
		}
	}
	void RobotJoints::setZ(int z) {
		TraceBaseClass() << "try to set z=" << z << std::endl;
		if (inRange(Z, z)) {
			setLowLevelZ(z);
		}
	}
	void RobotJoints::setWristAngle(int a) {
		TraceBaseClass() << "try to set setWristAngle=" << a << std::endl;
		if (inRange(wristAngle, a)) {
			setLowLevelWristAngle(a);
		}
	}
	void RobotJoints::setWristRotate(int a) {
		TraceBaseClass() << "try to set setWristRotate=" << a << std::endl;
		if (inRange(wristRotate, a)) {
			setLowLevelWristRotate(a);
		}
	}
	void RobotJoints::setGripper(int distance) {
		TraceBaseClass() << "try to set setGripper=" << distance << std::endl;
		if (inRange(Gripper, distance)) {
			setLowLevelGripper(distance);
		}
	}
	void RobotJoints::setMin(SpecificJoint joint, int value) {
		if (userDefinedRanges) {
			if (value < hardwareRanges.maxValue[joint] && value > hardwareRanges.minValue[joint]) {
				userDefinedRanges->minValue[joint] = value;
			}
			else {
				TraceBaseClass(ErrorLog) << "minValue out of range " << value << " range is " << hardwareRanges.minValue[joint] << ", " << hardwareRanges.maxValue[joint] << std::endl;
			}
		}
	}
	void RobotJoints::setMax(SpecificJoint joint, int value) {
		if (userDefinedRanges) {
			if (value < hardwareRanges.maxValue[joint] && value > hardwareRanges.minValue[joint]) {
				userDefinedRanges->maxValue[joint] = value;
			}
			else {
				TraceBaseClass(ErrorLog) << "maxValue out of range " << value << " range is " << hardwareRanges.minValue[joint] << ", " << hardwareRanges.maxValue[joint] << std::endl;
			}
		}
	}
	// return core data making sure its set properly
	uint8_t *RobotJointsState::getData() {
		set(headerByteOffset, 255);
		getChkSum();
		return data;
	}
	int RobotJoints::getMin(robotArmJointType type) {
		if (userDefinedRanges && userDefinedRanges->minValue.find(SpecificJoint(typeOfRobot, type)) != userDefinedRanges->minValue.end()) {
			return userDefinedRanges->minValue[SpecificJoint(typeOfRobot, type)];
		}
		return hardwareRanges.minValue[SpecificJoint(typeOfRobot, type)];
	}
	int RobotJoints::getMid(robotArmJointType type) {
		return (getMax(type) - getMin(type)) / 2; //bugbug works for robot types? 
	}

	int RobotJoints::getMax(robotArmJointType type) {
		if (userDefinedRanges && userDefinedRanges->maxValue.find(SpecificJoint(typeOfRobot, type)) != userDefinedRanges->maxValue.end()) {
			return userDefinedRanges->maxValue[SpecificJoint(typeOfRobot, type)];
		}
		return hardwareRanges.maxValue[SpecificJoint(typeOfRobot, type)];
	}

	bool RobotJoints::inRange(robotArmJointType type, int value) {
		if (value > getMax(type) || value < getMin(type)) {
			TraceBaseClass(ErrorLog) << "out of range " << value << " (" << getMin(type) << ", " << getMax(type) << ")" << std::endl;
			return false;
		}
		return true;
	}
	// needs to only be called one time -- uses static data to save time/space. backhoe not supported
	void RobotJoints::oneTimeSetup() {

		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, InterbotiXPhantomXReactorArm), X), -300, 300, 0);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, InterbotiXPhantomXReactorArm), X), -300, 300, 0);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, InterbotiXPhantomXReactorArm), Y), 50, 350, 235);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, InterbotiXPhantomXReactorArm), Y), 20, 150, 140);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, InterbotiXPhantomXReactorArm), Z), 20, 250, 210);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, InterbotiXPhantomXReactorArm), Z), 10, 150, 30);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, InterbotiXPhantomXReactorArm), wristAngle), -30, 30, 0);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, InterbotiXPhantomXReactorArm), wristAngle), -90, -45, -90);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, InterbotiXPhantomXReactorArm), wristRotate), 0, 1023, 512);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, InterbotiXPhantomXReactorArm), wristRotate), 0, 1023, 512);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, InterbotiXPhantomXReactorArm), Gripper), 0, 512, 512);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, InterbotiXPhantomXReactorArm), Gripper), 0, 512, 512);


		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, InterbotiXPhantomXReactorArm), X), 0, 1023, 512);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, InterbotiXPhantomXReactorArm), X), 0, 1023, 512);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, InterbotiXPhantomXReactorArm), Y), 50, 350, 235);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, InterbotiXPhantomXReactorArm), Y), 20, 150, 140);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, InterbotiXPhantomXReactorArm), Z), 20, 250, 210);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, InterbotiXPhantomXReactorArm), Z), 10, 150, 30);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, InterbotiXPhantomXReactorArm), wristAngle), -30, 30, 0);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, InterbotiXPhantomXReactorArm), wristAngle), -90, -45, -90);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, InterbotiXPhantomXReactorArm), wristRotate), 0, 1023, 512);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, InterbotiXPhantomXReactorArm), wristRotate), 0, 1023, 512);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, InterbotiXPhantomXReactorArm), Gripper), 0, 512, 512);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, InterbotiXPhantomXReactorArm), Gripper), 0, 512, 512);

		// mark end of list for debugging
		set(SpecificJoint(createUndefinedRobotType(), JointNotDefined), 0, 0, 0); // should not be set to this while running, only during object init

		return;

	}

	robotLowLevelCommand RobotJointsState::getStartCommand(robotType type) {
		if (type.first == IKM_IK3D_CARTESIAN) {
			return setArm3DCartesianStraightWristAndGoHome; // bugbug support all types once the basics are working
		}
		if (type.first == IKM_IK3D_CARTESIAN_90) {
			return setArm3DCartesian90DegreeWristAndGoHome; // bugbug support all types once the basics are working
		}
		if (type.first == IKM_CYLINDRICAL_90) {
			return setArm3DCylindrical90DegreeWristAndGoHome; // bugbug support all types once the basics are working
		}
		if (type.first == IKM_CYLINDRICAL) {
			return setArm3DCylindricalStraightWristAndGoHome; // bugbug support all types once the basics are working
		}
		return unKnownCommand;//bugbug support all types once the basics are working
	}
	void RobotJointsState::set(uint16_t offset, uint8_t b) {
		TraceBaseClass() << "set data[" << offset << "] = " << (uint16_t)b << std::endl;
		if (data) {
			data[offset] = b;
		}
	}
	// "home" and set data matching state
	void RobotJoints::setDefaultState() {
		TraceBaseClass() << "setDefaults";
		setX(getDefaultValue(X));
		setY(getDefaultValue(Y));
		setZ(getDefaultValue(Z));
		setWristAngle(getDefaultValue(wristAngle));
		setWristRotate(getDefaultValue(wristRotate));
		setGripper(getDefaultValue(Gripper));
		setDelta(getDeltaDefault());
		setLowLevelCommand(NoArmCommand);
		setButton();
	}
	// will block until arm is ready
	robotType RobotJoints::setStartState() {
		TraceBaseClass() << "setStartState " << typeOfRobot.first << " " << typeOfRobot.second << std::endl;
		setLowLevelCommand(getStartCommand(typeOfRobot));
		return typeOfRobot;
	}

	void RobotJointsState::echo() {
		if (!data) {
			TraceBaseClass() << "no data to echo" << std::endl;
			return;
		}
#define ECHO(a)TraceBaseClass() << "echo[" << a << "] = "  << std::hex << (unsigned int)data[a] << "h "  <<  std::dec <<(unsigned int)data[a] << "d "<< #a << std::endl;

		ECHO(headerByteOffset)
			ECHO(xLowByteOffset)
			ECHO(yHighByteOffset)
			ECHO(yLowByteOffset)
			ECHO(zHighByteOffset)
			ECHO(zLowByteOffset)
			ECHO(wristAngleHighByteOffset)
			ECHO(wristAngleLowByteOffset)
			ECHO(wristRotateHighByteOffset)
			ECHO(wristRotateLowByteOffset)
			ECHO(gripperHighByteOffset)
			ECHO(gripperLowByteOffset)
			ECHO(deltaValBytesOffset)
			ECHO(buttonByteOffset)
			ECHO(extValBytesOffset)
			ECHO(checksum)
	}

	uint8_t RobotJointsState::getChkSum() {
		if (data) {
			uint16_t sum = 0;
			for (int i = xHighByteOffset; i <= extValBytesOffset; ++i) {
				sum += data[i];
			}
			uint16_t invertedChecksum = sum % 256;//isolate the lowest byte 8 or 16?

			data[checksum] = 255 - invertedChecksum; //invert value to get file checksum
			return data[checksum];

		}
		return 0;
	}
	// user defined can never be greater than hardware min/max
	void RobotJoints::setUserDefinedRanges(SpecificJoint joint, RobotValueRanges *userDefinedRanges) {

		if (userDefinedRanges->minValue[joint] < hardwareRanges.minValue[joint] || userDefinedRanges->maxValue[joint] > hardwareRanges.maxValue[joint]) {
			TraceBaseClass(ErrorLog) << "RobotJoints::setUserDefinedRanges value out of range ignored " << std::endl;
			return;
		}
		this->userDefinedRanges = userDefinedRanges;
	}

	void RobotJoints::set(SpecificJoint type, int minVal, int maxVal, int defaultVal) {
		TraceBaseClass() << "RobotJoints::set" << echoJointType(type) << " - min = " << minVal << " max = " << maxVal << " default = " << defaultVal << std::endl;

		hardwareRanges.minValue[type] = minVal;
		hardwareRanges.maxValue[type] = maxVal;
		hardwareRanges.defaultValue[type] = defaultVal;
	}

	int RobotJoints::getDefaultValue(robotArmJointType type) {
		if (userDefinedRanges && userDefinedRanges->defaultValue.find(SpecificJoint(typeOfRobot, type)) != userDefinedRanges->defaultValue.end()) {
			return userDefinedRanges->defaultValue[SpecificJoint(typeOfRobot, type)];
		}
		return  hardwareRanges.defaultValue[SpecificJoint(typeOfRobot, type)];
	}
	void RobotJoints::setDefault(SpecificJoint joint, int value) {
		if (userDefinedRanges) {
			if (value < hardwareRanges.maxValue[joint] && value > hardwareRanges.minValue[joint]) {
				userDefinedRanges->defaultValue[joint] = value;
			}
			else {
				TraceBaseClass(ErrorLog) << "defaultValue out of range " << value << " range is " << hardwareRanges.minValue[joint] << ", " << hardwareRanges.maxValue[joint] << std::endl;
			}
		}
	}
}