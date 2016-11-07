/*
trossenrobots.cpp - openframeworks based classes for managing trossen robots
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
#include "trossenrobots.h"
//http://biorobots.case.edu/wp-content/uploads/2014/12/IntroductiontoDynamixelMotorControlUsingtheArbotiX20141112-1.pdf
namespace RobotArtists {

	RobotValueRanges RobotJoints::hardwareRanges; // just need to set once

	int ofRobotSerial::readLine(uint8_t* bytes, int bytesMax)	{
		if (!bytes) {
			return 0;
		}
		// null data in case none is found we always return null termanted data
		memset(bytes, 0, bytesMax);
		int i = 0;
		for (; i < bytesMax; ++i) {
			if (readBytesInOneShot(&bytes[i], 1) == 1) {
				if (bytes[i] == '\r') {
					i++;
					readBytesInOneShot(&bytes[i], 1);// get other eol marker
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
	int ofRobotSerial::readBytesInOneShot(uint8_t *bytes, int bytesMax) {
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
			*bytes = 0;// null out to show data read
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
			}
		}
		return readIn;
	}

	// return true if ID packet is value
	bool ofRobotSerial::idPacket(uint8_t *bytes, int size) {
		if (size == 5 && bytes[3] == 0) {
			uint8_t chksum = (unsigned char)(255 - (bytes[1] + bytes[2] + 0) % 256);
			return chksum == bytes[4];
		}
		return false;
		/* from device:
		void IDPacket() {
			Serial.write(0xFF);
			Serial.write((unsigned char)ARMID);
			Serial.write((unsigned char)g_bIKMode);
			Serial.write((unsigned char)0);
			Serial.write((unsigned char)(255 - (ARMID + g_bIKMode + 0) % 256));

		}
		*/
	}

	// bool readOnly -- just read serial do not send request
	robotType ofRobotSerial::ArmIDResponsePacket(uint8_t *bytes, int count) {
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


	robotType ofRobotSerial::waitForRobot(string& name, int retries) {
		ofRobotTrace() << "wait for mr robot ... " << std::endl;

		robotType type = createUndefinedRobotType();
		uint8_t bytes[31];

		if (waitForSerial(retries)) {
			// somethis is out there, see if we can ID it

			int readin = readBytesInOneShot(bytes, 5);
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

			getPose(); // see if a pose is out there bugbug clean all this pose junk up
		}
		return type;
	}

	void ofRobotSerial::write(uint8_t* data, int count) {
		// from http://learn.trossenrobotics.com/arbotix/arbotix-communication-controllers/31-arm-link-reference.html

		//If you are sending packets at an interval, do not send them faster than 30hz(one packet every 33ms).
		// no need to hurry packets so just want the minimum amount no matter what
		ofSleepMillis(100); // 100 ms seems ok, to much less and we start to overrun bugbug is this true?  
		echoRawBytes(data, count);
		int sent = writeBytes(data, count);

		ofRobotTrace() << "write sent = " << sent << std::endl;

		getPose(); // pose is sent all the time  by the micro code

	}

	// read pose from robot after every move and setup, just report on it or ignore it
	void ofRobotSerial::getPose() {
		uint8_t data[100];
		readLine(data, sizeof data);
		flush(); // could be a few of these out there
		ofRobotTrace() << "vals = " << data << std::endl;
	}

	int ofRobotSerial::getServoRegister(ServoIDs id, Registers registerNumber, int length) {
		// send and read data
		uint8_t pose[RobotState::count];
		memset(pose, 0, sizeof pose);
		RobotCommandInterface interface(pose);
		interface.setLowLevelCommand(getServoRegisterCommand);
		interface.setLowLevelX(id); // servo 
		interface.setLowLevelY(registerNumber);
		interface.setLowLevelZ(length);
		flush();   // reset
		int sent = writeBytes(interface.getPose(), RobotState::count);
		ofSleepMillis(33);
		uint8_t data[5];
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
		uint8_t moredata[500];
		readLine(moredata, 500);
		ofRobotTrace(ErrorLog) << "spurious data " << data << " ** and **" << moredata << std::endl;
		return 0;
	}
	// length == 2 for ax12SetRegister2
	void ofRobotSerial::setServoRegister(ServoIDs id, Registers registerNumber, int length, int dataToSend) {
		uint8_t pose[RobotState::count];
		memset(pose, 0, sizeof pose);

		RobotCommandInterface interface(pose);
		interface.setLowLevelCommand(setServoRegisterCommand);
		interface.setLowLevelX(id); // servo 
		interface.setLowLevelY(registerNumber);
		interface.setLowLevelZ(length);
		interface.setLowLevelWristAngle(dataToSend, 0);
		flush();   // reset
		int sent = writeBytes(interface.getPose(), RobotState::count);
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


		 //else if (armlink.ext == 0x82) {  //130
			// SetServoRegister(armlink.ext, armlink.Xaxis, armlink.Yaxis, armlink.Zaxis, armlink.W_ang);
		 //}
		/*
		void SetServoRegister(unsigned char command, int id, int registerNumber, int length, int data)
{

  unsigned char registerHigh;
  unsigned char registerLow;
  
  if(length == 1)
  {
    ax12SetRegister(id, registerNumber, data);
  }
  else if (length == 2)
  {
    ax12SetRegister2(id, registerNumber, data);
  }
  


  registerHigh = ((data & 0xFF00) >> 8);
  registerLow = (data & 0x00FF);
  Serial.write(0xff);
  Serial.write(command); // should this be an error bit?
  Serial.write(registerHigh);
  Serial.write(registerLow);
  Serial.write((unsigned char)(255 - (command+registerHigh+registerLow)%256));

}

*/
		// write
		//uint8_t registerHigh;
		//uint8_t registerLow;
	}


	int RobotState::get(uint16_t high, uint16_t low) {
		int number = -1;
		if (pose) {
			int number = bytes_to_u16(pose[high], pose[low]);
		}
		return number;
	}

	void RobotState::set(uint16_t high, uint16_t low, int val) {
		ofRobotTrace() << "set " << val << std::endl;
		set(high, highByte(val));
		set(low, lowByte(val));
	}
	void RobotJoints::setX(int x) {
		ofRobotTrace() << "try to set x=" << x << std::endl;
		if (inRange(X, x)) {
			setLowLevelX(x, addMagicNumber());
		}
	}
	void RobotJoints::setY(int y) {
		ofRobotTrace() << "try to set y=" << y << std::endl;
		if (inRange(Y, y)) {
			setLowLevelY(y);
		}
	}
	void RobotJoints::setZ(int z) {
		ofRobotTrace() << "try to set z=" << z << std::endl;
		if (inRange(Z, z)) {
			setLowLevelZ(z);
		}
	}
	void RobotJoints::setWristAngle(int a) {
		ofRobotTrace() << "try to set setWristAngle=" << a << std::endl;
		if (inRange(wristAngle, a)) {
			setLowLevelWristAngle(a);
		}
	}
	void RobotJoints::setWristRotate(int a) {
		ofRobotTrace() << "try to set setWristRotate=" << a << std::endl;
		if (inRange(wristRotate, a)) {
			setLowLevelWristRotate(a);
		}
	}
	void RobotJoints::setGripper(int distance) {
		ofRobotTrace() << "try to set setGripper=" << distance << std::endl;
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
				ofRobotTrace(ErrorLog) << "minValue out of range " << value << " range is " << hardwareRanges.minValue[joint] << ", " << hardwareRanges.maxValue[joint] << std::endl;
			}
		}
	}
	void RobotJoints::setMax(SpecificJoint joint, int value) {
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
	uint8_t *RobotState::getPose() {
		set(headerByteOffset, 255);
		getChkSum();
		return pose;
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
			ofRobotTrace(ErrorLog) << "out of range " << value << " (" << getMin(type) << ", " << getMax(type) << ")" << std::endl;
			return false;
		}
		return true;
	}
	// needs to only be called one time -- uses static data to save time/space. backhoe not supported
	void RobotJoints::oneTimeSetup() {

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

	robotLowLevelCommand RobotState::getStartCommand(robotType type) {
		if (type.first == IKM_IK3D_CARTESIAN) {
			return setArm3DCartesianStraightWristAndGoHomeCommand; 
		}
		if (type.first == IKM_IK3D_CARTESIAN_90) {
			return setArm3DCartesian90DegreeWristAndGoHomeCommand; 
		}
		if (type.first == IKM_CYLINDRICAL_90) {
			return setArm3DCylindrical90DegreeWristAndGoHomeCommand; 
		}
		if (type.first == IKM_CYLINDRICAL) {
			return setArm3DCylindricalStraightWristAndGoHomeCommand;
		}
		return unKnownCommand;
	}
	void RobotState::set(uint16_t offset, uint8_t b) {
		ofRobotTrace() << "set data[" << offset << "] = " << (uint16_t)b << std::endl;
		if (pose) {
			pose[offset] = b;
		}
	}
	// "home" and set data matching state
	void RobotJoints::setDefaultState() {
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
	// will block until arm is ready
	robotType RobotJoints::setStartState() {
		ofRobotTrace() << "setStartState " << typeOfRobot.first << " " << typeOfRobot.second << std::endl;
		setLowLevelCommand(getStartCommand(typeOfRobot));
		return typeOfRobot;
	}

	void RobotState::echoRawData() {
		if (!pose) {
			ofRobotTrace() << "no pose to echo" << std::endl;
			return;
		}
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
		if (pose) {
			uint16_t sum = 0;
			for (int i = xHighByteOffset; i <= extValBytesOffset; ++i) {
				sum += pose[i];
			}
			uint16_t invertedChecksum = sum % 256;//isolate the lowest byte 8 or 16?

			pose[checksum] = 255 - invertedChecksum; //invert value to get file checksum
			return pose[checksum];

		}
		return 0;
	}
	// user defined can never be greater than hardware min/max
	void RobotJoints::setUserDefinedRanges(SpecificJoint joint, shared_ptr<RobotValueRanges>) {

		if (userDefinedRanges) {
			if (userDefinedRanges->minValue[joint] < hardwareRanges.minValue[joint] || userDefinedRanges->maxValue[joint] > hardwareRanges.maxValue[joint]) {
				ofRobotTrace(ErrorLog) << "RobotJoints::setUserDefinedRanges value out of range ignored " << std::endl;
				return;
			}
		}
		this->userDefinedRanges = userDefinedRanges;
	}

	void RobotJoints::set(SpecificJoint type, int minVal, int maxVal, int defaultVal) {
		ofRobotTrace() << "RobotJoints::set" << echoJointType(type) << " - min = " << minVal << " max = " << maxVal << " default = " << defaultVal << std::endl;

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
				ofRobotTrace(ErrorLog) << "defaultValue out of range " << value << " range is " << hardwareRanges.minValue[joint] << ", " << hardwareRanges.maxValue[joint] << std::endl;
			}
		}
	}
}