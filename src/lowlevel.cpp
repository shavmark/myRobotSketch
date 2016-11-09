/*
lowlevel.cpp - openframeworks based classes for managing  robots
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
#include "ofApp.h"
#include "ofutils.h"
#include "lowlevel.h"
//http://biorobots.case.edu/wp-content/uploads/2014/12/IntroductiontoDynamixelMotorControlUsingtheArbotiX20141112-1.pdf
namespace RobotArtists {

	RobotValueRanges ofRobotJoints::hardwareRanges; // just need to set once

	int ofRobotSerial::readLine(uint8_t* bytes, int bytesMax)	{
		if (!bytes) {
			return 0;
		}
		// null data in case none is found we always return null termanted data
		memset(bytes, 0, bytesMax);
		int i = 0;
		for (; i < bytesMax; ++i) {
			if (readAllBytes(&bytes[i], 1) == 1) {
				if (bytes[i] == '\r') {
					i++;
					readAllBytes(&bytes[i], 1);// get other eol marker
					i++;
					break;
				}
			}
			else {
				break; // data is messed up, try to move on
			}
		}
		
		return i;
	}
	int ofRobotSerial::readBytesWithoutWaitingForData(uint8_t *bytes, int bytesMax) {
		int result = 0;
		if (available() > 0) {
			if ((result = readBytes(bytes, bytesMax)) == OF_SERIAL_ERROR) {
				ofRobotTrace(ErrorLog) << "serial failed" << std::endl;
				return 0;
			}
			while (result == OF_SERIAL_NO_DATA) {
				result = readBytes(bytes, bytesMax);
				if (result == OF_SERIAL_ERROR) {
					ofRobotTrace(ErrorLog) << "serial failed" << std::endl;
					return 0;
				}
				if (result != OF_SERIAL_NO_DATA) {
					return result;
				}
			}
		}
		return result;
	}
	int ofRobotSerial::readAllBytes(uint8_t *bytes, int bytesRequired) {
		int readIn = 0;

		if (bytes) {
			memset(bytes, 0, bytesRequired); // keep data  clean
			int tries = 0;
			int bytesRemaining = bytesRequired;
			// loop until we've read everything
			while (bytesRemaining > 0) {
				// check for data
				if (available() > 0) {
					// try to read - note offset into the bytes[] array, this is so
					// that we don't overwrite the bytes we already have
					int bytesArrayOffset = bytesRequired - bytesRemaining;
					int result = readBytes(&bytes[bytesArrayOffset], bytesRemaining);

					// check for error code
					if (result == OF_SERIAL_ERROR) {
						// something bad happened
						ofRobotTrace(ErrorLog) << "unrecoverable error reading from serial" << std::endl;
						// bail out
						break;
					}
					else if (result == OF_SERIAL_NO_DATA) {
						// nothing was read, try again
					}
					else {
						// we read some data!
						readIn += result;
						bytesRemaining -= result;
					}
				}
				if (tries++ > maxRetries) {
					ofRobotTrace() << "data not found" << std::endl;
					break;
				}
				ofSleepMillis(waitsleeptime); // else wait a bit more
			}
		}
		return readIn;
	}

	// return true if ID packet is value
	bool ofTrosseRobotSerial::idPacket(uint8_t *bytes, int size) {
		if (size == 5 && bytes[3] == 0) {
			uint8_t chksum = (unsigned char)(255 - (bytes[1] + bytes[2] + 0) % 256);
			return chksum == bytes[4];
		}
		return false;
	}

	// bool readOnly -- just read serial do not send request
	robotType ofTrosseRobotSerial::ArmIDResponsePacket(uint8_t *bytes, int count) {
		if (!idPacket(bytes, count)) {
			return robotType(IKM_NOT_DEFINED, unknownRobotType);
		}
		if (bytes != nullptr) {
			robotArmMode armMode = (robotArmMode)bytes[2];
			switch (armMode) {
			case IKM_IK3D_CARTESIAN:
				ofRobotTrace() << "arm mode IKM_IK3D_CARTESIAN" << std::endl;
				break;
			case IKM_IK3D_CARTESIAN_90:
				ofRobotTrace() << "arm mode IKM_IK3D_CARTESIAN_90" << std::endl;
				break;
			case IKM_CYLINDRICAL:
				ofRobotTrace() << "arm mode IKM_CYLINDRICAL" << std::endl;
				break;
			case IKM_CYLINDRICAL_90:
				ofRobotTrace() << "arm mode IKM_CYLINDRICAL_90" << std::endl;
				break;
			default:
				ofRobotTrace() << "arm mode IKM_BACKHOE mode?" << std::endl;
				break;
			}
			RobotTypeID id = unknownRobotType;
			
			switch (bytes[1]) {
			case PINCHER_ARMID:
				id = PhantomXPincherArm;
				ofRobotTrace() << "InterbotiXPhantomXPincherArm" << std::endl;
				break;
			case REACTOR_ARMID:
				id = PhantomXReactorArm;
				ofRobotTrace() << "InterbotiXPhantomXReactorArm" << std::endl;
				break;
			case WIDOWX:
				id = WidowX;
				ofRobotTrace() << "WidowX" << std::endl;
				break;
			}
			return robotType(armMode, id);
		}
		return robotType(IKM_NOT_DEFINED, unknownRobotType);
	}

	bool ofRobotSerial::waitForSerial(int retries) {
		for (int i = 0; i < retries; ++i) {
			ofRobotTrace() << "check serial data (try #/) = " << i << "/" << retries << std::endl;
			if (available() > 0) {
				ofRobotTrace() << "data found" << std::endl;
				return true;
			}
			ofSleepMillis(1000);
		}
		return false;
	}



	robotType ofTrosseRobotSerial::waitForRobot(string& name, int retries) {
		ofRobotTrace() << "wait for mr robot ... " << std::endl;

		robotType type = createUndefinedRobotType();
		uint8_t bytes[500];

		if (waitForSerial(retries)) {
			// somethis is out there, see if we can ID it

			int readin = readAllBytes(bytes, 5);
			if (readin == 5) {
				type = ArmIDResponsePacket(bytes, 5);
				if (type.first == IKM_NOT_DEFINED) {
					ofRobotTrace(ErrorLog) << "invalid robot type" << std::endl;
					return type;
				}
			}
			else {
				bytes[readin] = 0;
				uint16_t i = *bytes;
				ofRobotTrace(ErrorLog) << "invalid robot sign on:" << i << std::endl;
				return type;
			}

			// get sign on echo from device
			readin = readLine(bytes, sizeof bytes);
			ofRobotTrace() << bytes << std::endl;

			readin = readLine(bytes, sizeof bytes); // make this the name

			name.reserve(readin); // skip end of line markers
			for (int i = 0; i < readin-2; ++i) {
				name += bytes[i];
			}
			ofRobotTrace() << "robot name " << name << std::endl;

			getPose(); // pose for command and pose for start up must be read in

		}
		return type;
	}
	// echo, ignoring null bytes
	const string ofTrosseRobotSerial::dataName(int id) {
		switch (id) {
		case RobotState::headerByteOffset:
			return " headerByteOffset";
		case RobotState::xHighByteOffset:
			return " xHighByteOffset";
		case RobotState::xLowByteOffset:
			return " xLowByteOffset";
		case RobotState::yHighByteOffset:
			return " yHighByteOffset";
		case RobotState::yLowByteOffset:
			return " yLowByteOffset";
		case RobotState::zHighByteOffset:
			return " zHighByteOffset";
		case RobotState::zLowByteOffset:
			return " zLowByteOffset";
		case RobotState::wristAngleHighByteOffset:
			return " wristAngleHighByteOffset";
		case RobotState::wristAngleLowByteOffset:
			return " wristAngleLowByteOffset";
		case RobotState::wristRotateHighByteOffset:
			return " wristRotateHighByteOffset";
		case RobotState::wristRotateLowByteOffset:
			return " wristRotateLowByteOffset";
		case RobotState::gripperHighByteOffset:
			return " gripperHighByteOffset";
		case RobotState::gripperLowByteOffset:
			return " gripperLowByteOffset";
		case RobotState::deltaValBytesOffset:
			return " deltaValBytesOffset";
		case RobotState::buttonByteOffset:
			return " buttonByteOffset";
		case RobotState::extValBytesOffset:
			return " extValBytesOffset";
		case RobotState::checksum:
			return " checksum";
		}
		return "???";
	}
	void ofTrosseRobotSerial::echoRawBytes(uint8_t *bytes, int count) {
		std::stringstream buffer;
		for (int i = 0; i < count; ++i) {
			buffer << " bytes[" << i << "] = " << (int)bytes[i] << dataName(i) << std::endl; // echo in one line
		}
		ofRobotTrace() << buffer.str() << std::endl;
	}


	int ofRobotSerial::write(uint8_t* data, int count) {
		// from http://learn.trossenrobotics.com/arbotix/arbotix-communication-controllers/31-arm-link-reference.html

		//If you are sending packets at an interval, do not send them faster than 30hz(one packet every 33ms).
		// no need to hurry packets so just want the minimum amount no matter what
		ofSleepMillis(100); // 100 ms seems ok, to much less and we start to overrun bugbug is this true?  
		echoRawBytes(data, count);
		int sent = writeBytes(data, count);

		ofRobotTrace() << "write sent = " << sent << std::endl;

		getPose(); // pose is sent all the time  by the micro code for at least trossen robots

		return sent;

	}

	// read pose from robot after every move and setup, just report on it or ignore it
	void ofTrosseRobotSerial::getPose() {
		uint8_t data[500];
		if (readLine(data, sizeof data) > 0) {
			ofRobotTrace() << "current pos vals = " << data << std::endl;
		}
	}

	int ofTrosseRobotSerial::getServoRegister(TrossenServoIDs id, AXRegisters registerNumber, int length) {
		// send and read data
		RobotState interface;
		interface.setLowLevelCommand(getServoRegisterCommand);
		interface.setLowLevelX(id); // servo 
		interface.setLowLevelY(registerNumber);
		interface.setLowLevelZ(length);
		
		int sent = writePose(interface.getPose());
		ofSleepMillis(33);

		uint8_t data[500]; // could be lots of sperius data out there, unlikely but if it occurs we want to echo it
		memset(data, 0, sizeof data);
		uint16_t val = 0;
		int readin = readAllBytes(data, 5);
		if (readin == 5 && data[0] == 255 && data[1] == getServoRegisterCommand) {
			uint8_t high = data[2];
			uint8_t low = data[3];
			if (data[4] == interface.calcChkSum(data, 1, 3)) {
				val = bytes_to_u16(high, low);
				ofRobotTrace() << "servo " << id << " registerNumber " << registerNumber << " value " << val << std::endl;
				return val;
			}
		}
		ofRobotTrace(ErrorLog) << "reportServoRegister fails" << std::endl;
		// see what data is out there
		readLine(&data[readin], sizeof data - readin);
		ofRobotTrace(ErrorLog) << "spurious data " << data << std::endl;
		return 0;
	}

	int ofRobotSerial::writePose(Pose&pose) {
		return write(pose.get(), pose.size());
	}

	// length == 2 for ax12SetRegister2
	void ofTrosseRobotSerial::setServoRegister(TrossenServoIDs id, AXRegisters registerNumber, int length, int dataToSend) {
		RobotState interface;
		interface.setLowLevelCommand(setServoRegisterCommand);
		interface.setLowLevelX(id); // servo 
		interface.setLowLevelY(registerNumber);
		interface.setLowLevelZ(length);
		interface.setLowLevelWristAngle(dataToSend, 0);
		flush();   // reset
		int sent = writePose(interface.getPose());
		ofSleepMillis(33);
		uint8_t data[5];
		int readin = readAllBytes(data, 5);
		if (readin == 5 && data[0] == 255 && data[1] == setServoRegisterCommand) {
			uint8_t high = data[2];
			uint8_t low = data[3];
			if (data[4] == interface.calcChkSum(data, 1, 3)) {
				uint16_t val = bytes_to_u16(high, low);
				if (val == dataToSend){
					ofRobotTrace() << "servo " << id << " registerNumber set " << registerNumber << " value " << val << std::endl;
				}
				else {
					ofRobotTrace(ErrorLog) << "servo regiser send fails" << id << " registerNumber " << registerNumber << " value " << val << std::endl;
				}
			}
		}
	}

	int RobotState::get(uint16_t high, uint16_t low) {
		return bytes_to_u16(pose[high], pose[low]);
	}

	void RobotState::set(uint16_t high, uint16_t low, int val) {
		ofRobotTrace() << "set " << val << std::endl;
		set(high, highByte(val));
		set(low, lowByte(val));
	}
	void ofRobotJoints::setX(int x) {
		ofRobotTrace() << "try to set x=" << x << std::endl;
		if (inRange(X, x)) {
			setLowLevelX(x, addMagicNumber());
		}
	}
	void ofRobotJoints::setY(int y) {
		ofRobotTrace() << "try to set y=" << y << std::endl;
		if (inRange(Y, y)) {
			setLowLevelY(y);
		}
	}
	void ofRobotJoints::setZ(int z) {
		ofRobotTrace() << "try to set z=" << z << std::endl;
		if (inRange(Z, z)) {
			setLowLevelZ(z);
		}
	}
	void ofRobotJoints::setWristAngle(int a) {
		ofRobotTrace() << "try to set setWristAngle=" << a << std::endl;
		if (inRange(wristAngle, a)) {
			setLowLevelWristAngle(a);
		}
	}
	void ofRobotJoints::setWristRotate(int a) {
		ofRobotTrace() << "try to set setWristRotate=" << a << std::endl;
		if (inRange(wristRotate, a)) {
			setLowLevelWristRotate(a);
		}
	}
	void ofRobotJoints::setGripper(int distance) {
		ofRobotTrace() << "try to set setGripper=" << distance << std::endl;
		if (inRange(Gripper, distance)) {
			setLowLevelGripper(distance);
		}
	}
	void ofRobotJoints::setMin(SpecificJoint joint, int value) {
		if (userDefinedRanges) {
			if (value < hardwareRanges.maxValue[joint] && value > hardwareRanges.minValue[joint]) {
				userDefinedRanges->minValue[joint] = value;
			}
			else {
				ofRobotTrace(ErrorLog) << "minValue out of range " << value << " range is " << hardwareRanges.minValue[joint] << ", " << hardwareRanges.maxValue[joint] << std::endl;
			}
		}
	}
	void ofRobotJoints::setMax(SpecificJoint joint, int value) {
		if (userDefinedRanges) {
			if (value < hardwareRanges.maxValue[joint] && value > hardwareRanges.minValue[joint]) {
				userDefinedRanges->maxValue[joint] = value;
			}
			else {
				ofRobotTrace(ErrorLog) << "maxValue out of range " << value << " range is " << hardwareRanges.minValue[joint] << ", " << hardwareRanges.maxValue[joint] << std::endl;
			}
		}
	}
	// return core data making sure its set properly
	uint8_t *RobotState::getPoseData() {
		set(headerByteOffset, 255);
		getChkSum();
		return pose.get();
	}
	int ofRobotJoints::getMin(robotArmJointType type) {
		if (userDefinedRanges && userDefinedRanges->minValue.find(SpecificJoint(typeOfRobot, type)) != userDefinedRanges->minValue.end()) {
			return userDefinedRanges->minValue[SpecificJoint(typeOfRobot, type)];
		}
		return hardwareRanges.minValue[SpecificJoint(typeOfRobot, type)];
	}
	int ofRobotJoints::getMid(robotArmJointType type) {
		return (getMax(type) - getMin(type)) / 2; //bugbug works for robot types? 
	}

	int ofRobotJoints::getMax(robotArmJointType type) {
		if (userDefinedRanges && userDefinedRanges->maxValue.find(SpecificJoint(typeOfRobot, type)) != userDefinedRanges->maxValue.end()) {
			return userDefinedRanges->maxValue[SpecificJoint(typeOfRobot, type)];
		}
		return hardwareRanges.maxValue[SpecificJoint(typeOfRobot, type)];
	}

	bool ofRobotJoints::inRange(robotArmJointType type, int value) {
		if (value > getMax(type) || value < getMin(type)) {
			ofRobotTrace(ErrorLog) << "out of range " << value << " (" << getMin(type) << ", " << getMax(type) << ")" << std::endl;
			return false;
		}
		return true;
	}
	// needs to only be called one time -- uses static data to save time/space. backhoe not supported
	void ofRobotJoints::oneTimeSetup() {

		// these values come from http://learn.trossenrobotics.com/arbotix/arbotix-communication-controllers/31-arm-link-reference.html#limits for pincher and there
		// is also a page for the Reactor.  Any ranges can be set as default ranges.

		// Reactor

		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, PhantomXReactorArm), X), -300, 300, 0);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, PhantomXReactorArm), X), -300, 300, 0);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, PhantomXReactorArm), Y), 50, 350, 235);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, PhantomXReactorArm), Y), 20, 150, 140);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, PhantomXReactorArm), Z), 20, 250, 210);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, PhantomXReactorArm), Z), 10, 150, 30);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, PhantomXReactorArm), wristAngle), -30, 30, 0);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, PhantomXReactorArm), wristAngle), -90, -45, -90);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, PhantomXReactorArm), wristRotate), 0, 1023, 512);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, PhantomXReactorArm), wristRotate), 0, 1023, 512);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, PhantomXReactorArm), Gripper), 0, 512, 512);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, PhantomXReactorArm), Gripper), 0, 512, 512);


		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, PhantomXReactorArm), X), 0, 1023, 512);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, PhantomXReactorArm), X), 0, 1023, 512);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, PhantomXReactorArm), Y), 50, 350, 235);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, PhantomXReactorArm), Y), 20, 150, 140);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, PhantomXReactorArm), Z), 20, 250, 210);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, PhantomXReactorArm), Z), 10, 150, 30);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, PhantomXReactorArm), wristAngle), -30, 30, 0);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, PhantomXReactorArm), wristAngle), -90, -45, -90);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, PhantomXReactorArm), wristRotate), 0, 1023, 512);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, PhantomXReactorArm), wristRotate), 0, 1023, 512);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, PhantomXReactorArm), Gripper), 0, 512, 512);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, PhantomXReactorArm), Gripper), 0, 512, 512);

		// Pincher
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, PhantomXPincherArm), X), -200, 200, 0);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, PhantomXPincherArm), X), -200, 200, 0);

		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, PhantomXPincherArm), Y), 20, 240, 170);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, PhantomXPincherArm), Y), 20, 150, 140);

		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, PhantomXPincherArm), Z), 20, 250, 210);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, PhantomXPincherArm), Z), 10, 150, 30);

		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, PhantomXPincherArm), wristAngle), -30, 30, 0);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, PhantomXPincherArm), wristAngle), -90, -45, -90);

		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, PhantomXPincherArm), wristRotate), 0, 1023, 512);
				set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, PhantomXPincherArm), wristRotate), 0, 1023, 512);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, PhantomXPincherArm), Gripper), 10, 512, 512);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, PhantomXPincherArm), Gripper), 10, 512, 512);


		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, PhantomXPincherArm), X), 0, 1023, 512);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, PhantomXPincherArm), X), 0, 1023, 512);

		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, PhantomXPincherArm), Y), 20, 240, 170);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, PhantomXPincherArm), Y), 20, 150, 140);

		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, PhantomXPincherArm), Z), 20, 250, 210);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, PhantomXPincherArm), Z), 10, 150, 30);

		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, PhantomXPincherArm), wristAngle), -30, 30, 0);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, PhantomXPincherArm), wristAngle), -90, -45, -90);

		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, PhantomXPincherArm), wristRotate), 0, 1023, 512);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, PhantomXPincherArm), wristRotate), 0, 1023, 512);

		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, PhantomXPincherArm), Gripper), 10, 512, 512);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, PhantomXPincherArm), Gripper), 10, 512, 512);

		// mark end of list for debugging
		set(SpecificJoint(createUndefinedRobotType(), JointNotDefined), 0, 0, 0); // should not be set to this while running, only during object init

		return;

	}

	robotLowLevelCommandTrossen RobotState::getStartCommand(robotArmMode mode) {
		if (mode == IKM_IK3D_CARTESIAN) {
			return setArm3DCartesianStraightWristAndGoHomeCommand; 
		}
		if (mode == IKM_IK3D_CARTESIAN_90) {
			return setArm3DCartesian90DegreeWristAndGoHomeCommand; 
		}
		if (mode == IKM_CYLINDRICAL_90) {
			return setArm3DCylindrical90DegreeWristAndGoHomeCommand; 
		}
		if (mode == IKM_CYLINDRICAL) {
			return setArm3DCylindricalStraightWristAndGoHomeCommand;
		}
		return unKnownCommand;
	}
	void RobotState::set(uint16_t offset, uint8_t b) {
		ofRobotTrace() << "set data[" << offset << "] = " << (uint16_t)b << std::endl;
		pose.set(offset, b);
	}
	// "home" and set data matching state
	void ofRobotJoints::setDefaultState() {
		ofRobotTrace() << "setDefaults";
		setX(getDefaultValue(X));
		setY(getDefaultValue(Y));
		setZ(getDefaultValue(Z));
		setWristAngle(getDefaultValue(wristAngle));
		setWristRotate(getDefaultValue(wristRotate));
		setGripper(getDefaultValue(Gripper));
		setDelta();
		setLowLevelCommand(NoArmCommand);
		setButton();
	}
	void ofRobotJoints::setMode(robotArmMode mode) {
		typeOfRobot.first = mode;
	}
	// will block until arm is ready
	robotType ofRobotJoints::setStartState(robotArmMode mode) {
		setMode(mode);
		ofRobotTrace() << "setStartState(mode, type) " << getMode() << " " << getType() << std::endl;
		setLowLevelCommand(getStartCommand(getMode()));
		return typeOfRobot;
	}

	void RobotState::echoRawData() {

#define ECHO(a)ofRobotTrace() << "echo[" << a << "] = "  << std::hex << (unsigned int)pose[a] << "h "  <<  std::dec <<(unsigned int)pose[a] << "d "<< #a << std::endl;

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
	uint8_t RobotState::calcChkSum(uint8_t *pose, int start, int end) {
		if (pose) {
			uint16_t sum = 0;
			for (int i = start; i <= end; ++i) {
				sum += pose[i];
			}
			uint16_t invertedChecksum = sum % 256;//isolate the lowest byte 8 or 16?

			return 255 - invertedChecksum; //invert value to get file checksum
		}
		return 0;
	}
	uint8_t RobotState::getChkSum() {
		uint16_t sum = 0;
		for (int i = xHighByteOffset; i <= extValBytesOffset; ++i) {
			sum += pose[i];
		}
		uint16_t invertedChecksum = sum % 256;//isolate the lowest byte 8 or 16?

		pose.set(checksum,  255 - invertedChecksum); //invert value to get file checksum
		return pose[checksum];
	}
	// user defined can never be greater than hardware min/max
	void ofRobotJoints::setUserDefinedRanges(SpecificJoint joint, shared_ptr<RobotValueRanges>) {

		if (userDefinedRanges) {
			if (userDefinedRanges->minValue[joint] < hardwareRanges.minValue[joint] || userDefinedRanges->maxValue[joint] > hardwareRanges.maxValue[joint]) {
				ofRobotTrace(ErrorLog) << "RobotJoints::setUserDefinedRanges value out of range ignored " << std::endl;
				return;
			}
		}
		this->userDefinedRanges = userDefinedRanges;
	}

	void ofRobotJoints::set(SpecificJoint type, int minVal, int maxVal, int defaultVal) {
		ofRobotTrace() << "RobotJoints::set" << echoJointType(type) << " - min = " << minVal << " max = " << maxVal << " default = " << defaultVal << std::endl;

		hardwareRanges.minValue[type] = minVal;
		hardwareRanges.maxValue[type] = maxVal;
		hardwareRanges.defaultValue[type] = defaultVal;
	}

	int ofRobotJoints::getDefaultValue(robotArmJointType type) {
		if (userDefinedRanges && userDefinedRanges->defaultValue.find(SpecificJoint(typeOfRobot, type)) != userDefinedRanges->defaultValue.end()) {
			return userDefinedRanges->defaultValue[SpecificJoint(typeOfRobot, type)];
		}
		return  hardwareRanges.defaultValue[SpecificJoint(typeOfRobot, type)];
	}
	void ofRobotJoints::setDefault(SpecificJoint joint, int value) {
		if (userDefinedRanges) {
			if (value < hardwareRanges.maxValue[joint] && value > hardwareRanges.minValue[joint]) {
				userDefinedRanges->defaultValue[joint] = value;
			}
			else {
				ofRobotTrace(ErrorLog) << "defaultValue out of range " << value << " range is " << hardwareRanges.minValue[joint] << ", " << hardwareRanges.maxValue[joint] << std::endl;
			}
		}
	}
}