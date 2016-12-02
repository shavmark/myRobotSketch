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

	RobotValueRanges ofTrRobotArmInternals::hardwareRanges; // just need to set once
													
	string Pose::dataName(int id) {
		switch (id) {
		case headerByteOffset:
			return " headerByteOffset";
		case xHighByteOffset:
			return " xHighByteOffset";
		case xLowByteOffset:
			return " xLowByteOffset";
		case yHighByteOffset:
			return " yHighByteOffset";
		case yLowByteOffset:
			return " yLowByteOffset";
		case zHighByteOffset:
			return " zHighByteOffset";
		case zLowByteOffset:
			return " zLowByteOffset";
		case wristAngleHighByteOffset:
			return " wristAngleHighByteOffset";
		case wristAngleLowByteOffset:
			return " wristAngleLowByteOffset";
		case wristRotateHighByteOffset:
			return " wristRotateHighByteOffset";
		case wristRotateLowByteOffset:
			return " wristRotateLowByteOffset";
		case gripperHighByteOffset:
			return " gripperHighByteOffset";
		case gripperLowByteOffset:
			return " gripperLowByteOffset";
		case deltaValBytesOffset:
			return " deltaValBytesOffset";
		case buttonByteOffset:
			return " buttonByteOffset";
		case extValBytesOffset:
			return " extValBytesOffset";
		case trChecksum:
			return " checksum";
		}
		return "???";
	}

	int ofRobotSerial::readLine(string &s) {
		uint8_t bytes[512]; // max size of a line, beyond this things get ignored
		int c;
		s.clear();
		if ((c=readLine(bytes, sizeof bytes)) > 0) {
			for (int i = 0; i < c; ++i) {
				s += bytes[i];
			}
		}
		return c;
	}
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
	bool ofRobotSerial::idPacket(uint8_t *bytes, int size) {
		ofRobotTrace() << "validate packet" << std::endl;
		if (bytes[0] == 0xee || bytes[0] == 0xff) {
			uint8_t chksum = getChkSum(bytes, 1, size - 2);
			if (chksum == bytes[size-1]) {
				return true;
			}
		}
		for (int i = 0; i < size; ++i) {
			ofRobotTrace(ErrorLog) << "invalid packet[" << i << "]" << (int)bytes[i] << std::endl;
		}
		return false;
	}

	// bool readOnly -- just read serial do not send request
	robotType ofRobotSerial::IDResponsePacket(uint8_t *bytes, int count) {
		if (!idPacket(bytes, count)) {
			return robotType(IKM_NOT_DEFINED, unknownRobotType);
		}
		if (bytes != nullptr) {
			robotMode armMode = (robotMode)bytes[2];
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
			case IKM_MAKERBOTXY:
				ofRobotTrace() << "IKM_MAKERBOTXY" << std::endl;
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
			case MAKERBOT_ID:
				id = MakerBotXY;
				ofRobotTrace() << "MakerBotXY" << std::endl;
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

	robotType ofRobotSerial::waitForRobot(string& name, int retries, int packetsize) {
		ofRobotTrace() << "wait for mr robot ... " << std::endl;

		robotType type = createUndefinedRobotType();
		uint8_t bytes[500];
		
		if (waitForSerial(retries)) {
			// somethis is out there, see if we can ID it

			int readin = readAllBytes(bytes, packetsize);
			if (readin == packetsize) {
				type = IDResponsePacket(bytes, packetsize);
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
		}
		return type;
	}
	void Pose::setup() {
		set(headerByteOffset, 255);
	}
	vector <ofSerialDeviceInfo>& ofRobotSerial::getDevices() {
		buildDeviceList();
		return devices;
	}


	void SerialData::trace() {
		std::stringstream buffer;
		for (int i = 0; i < size(); ++i) {
			buffer << " bytes[" << i << "] = " << (int)at(i) << dataName(i) << std::endl; // echo in one line
		}
		ofRobotTrace() << buffer.str() << std::endl;
	}
	void xyDataToSend::init() {
		steppers[0].resize(3);
		steppers[1].resize(3);
	}
	xyDataToSend::xyDataToSend(XYCommands cmd, const ofVec2f& point) {
		init();
		setCommand(cmd, point);
	}
	xyDataToSend::xyDataToSend() {
		init();
		setCommand(IDstepperX, NoXYCommand);
		setCommand(IDstepperY, NoXYCommand);
	}
	xyDataToSend::xyDataToSend(Steppers stepperID, XYCommands cmd, int i) {
		init();
		setCommand(stepperID, cmd, i);
	}
	xyDataToSend::xyDataToSend(Steppers stepperID, XYCommands cmd)  {
		init();
		setCommand(stepperID, cmd);
	}
	// send two commands, to X and to Y
	void xyDataToSend::setCommand(XYCommands cmd, const ofVec2f& point) {
		setCommand(IDstepperX, cmd, point.x);
		setCommand(IDstepperY, cmd, point.y);
	}

	void xyDataToSend::setCommand(Steppers stepperID, XYCommands cmd, int i) {
		parameters[stepperID] = ofToString(i);
		setCommand(stepperID, cmd);
	}
	void xyDataToSend::setCommand(Steppers stepperID, XYCommands cmd, float f) {
		parameters[stepperID] = ofToString(f);
		setCommand(stepperID, cmd);
	}

	void xyDataToSend::setCommand(Steppers stepperID, XYCommands cmd) {
		ofRobotTrace() << "stepperID = " << stepperID << "cmd = " << cmd << std::endl;
		steppers[stepperID].set(0, 0xee); 
		steppers[stepperID].set(1, stepperID); 
		steppers[stepperID].set(2, cmd);
		ofRobotTrace() << "parameters [stepperID] " << parameters[stepperID] << stepperID << std::endl;
	}

	int ofRobotSerial::write(uint8_t* data, size_t count) {
		// from http://learn.trossenrobotics.com/arbotix/arbotix-communication-controllers/31-arm-link-reference.html
		if (count <= 0) {
			return 0;
		}
		//If you are sending packets at an interval, do not send them faster than 30hz(one packet every 33ms).
		// no need to hurry packets so just want the minimum amount no matter what
		ofSleepMillis(100); // 100 ms seems ok, to much less and we start to overrun bugbug is this true?  
		int sent = writeBytes(data, (int)count);//bugbug of function should e size_t

		ofRobotTrace() << "write sent = " << sent << std::endl;

		return sent;
	}

	// read pose from robot after every move and setup, just report on it or ignore it
	void ofTrRobotArmInternals::readResults() {
		ofRobotTrace() << "read pose " << std::endl;
		uint8_t data[500];
		if (getDriver()->readLine(data, sizeof data) > 0) {
			ofRobotTrace() << "current pos vals = " << data << std::endl;
		}
	}

	int ofTrRobotArmInternals::getServoRegister(TrossenServoIDs id, AXRegisters registerNumber, int length) {
		// send and read data
		Pose pose;
		pose.setLowLevelCommand(getServoRegisterCommand);
		pose.setLowLevelX(id); // servo 
		pose.setLowLevelY(registerNumber);
		pose.setLowLevelZ(length);
		
		int sent = getDriver()->write(&pose);
		ofSleepMillis(33);

		uint8_t data[500]; // could be lots of sperius data out there, unlikely but if it occurs we want to echo it
		memset(data, 0, sizeof data);
		uint16_t val = 0;
		int readin = getDriver()->readAllBytes(data, 5);
		if (readin == 5 && data[0] == 255 && data[1] == getServoRegisterCommand) {
			uint8_t high = data[2];
			uint8_t low = data[3];
			if (data[4] == getChkSum(data, 1, 3)) {
				val = pose.bytes_to_u16(high, low);
				ofRobotTrace() << "servo " << id << " registerNumber " << registerNumber << " value " << val << std::endl;
				return val;
			}
		}
		ofRobotTrace(ErrorLog) << "reportServoRegister fails" << std::endl;
		// see what data is out there
		getDriver()->readLine(&data[readin], sizeof data - readin);
		ofRobotTrace(ErrorLog) << "spurious data " << data << std::endl;
		return 0;
	}

	int ofRobotSerial::write(SerialData*serial) {
		if (serial) {
			serial->update();
			serial->trace();
			return write(serial->data(), serial->size());
		}
		return 0;
	}

	string xyDataToSend::dataName(int i) {
		switch (i) {
		case 0:
			return " signature ";
		case 1:
			return " stepper index ";
		case 2:
			return " cmd ";
		}
		return "???";
	}
	string xyGetdata::dataName(int i) {
		switch (i) {
		case 0:
			return " signature ";
		case 1:
			return " data1 ";
		case 2:
			return " data2  ";
		case 3:
			return " cmd ";
		case 4:
			return " chksum ";
		}
		return "???";
	}
	// length == 2 for ax12SetRegister2
	void ofTrRobotArmInternals::setServoRegister(TrossenServoIDs id, AXRegisters registerNumber, int length, int dataToSend) {
		setLowLevelCommand(setServoRegisterCommand);
		setLowLevelX(id); // servo 
		setLowLevelY(registerNumber);
		setLowLevelZ(length);
		setLowLevelWristAngle(dataToSend, 0);
		
		getDriver()->flush();   // reset
		int sent = getDriver()->write(this);
		ofSleepMillis(33);
		uint8_t data[5];
		int readin = getDriver()->readAllBytes(data, 5);
		if (readin == 5 && data[0] == 255 && data[1] == setServoRegisterCommand) {
			uint8_t high = data[2];
			uint8_t low = data[3];
			if (data[4] == getChkSum(data, 1, 3)) {
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
	void xyRobot::draw() {
		ofRobotTrace() << "draw xyRobot" << name << std::endl;

		if (driver) {
			for (auto& a : vectorOfCommands) {
				sendToRobot(a.getData(IDstepperX)); 
				driver->write(a.getParameter(IDstepperX));
				readResults(IDstepperX);

				sendToRobot(a.getData(IDstepperY));
				driver->write(a.getParameter(IDstepperY));
				readResults(IDstepperY);
			}
		}
		vectorOfCommands.clear();
	}
	// just echo results
	bool xyRobot::readResults(Steppers stepper) {
		
		ofRobotTrace() << "read xyRobot " << std::endl;
		SerialData data(2); // results header
		string s;

		if (getDriver()->readAllBytes(data.data(), data.size()) == data.size()) {
			if (data[0] != 0xee) {
				// all commands need this header
				ofRobotTrace() << "unknown readResults " << data.data() << std::endl;
				return false;
			}
			else if (data[1] == SignOn) {
				getDriver()->readLine(s); // just read the x, y max info, left the rest to be compatable with other cards
				maxPositions[IDstepperX] = ofToInt(s);
				getDriver()->readLine(s); 
				maxPositions[IDstepperY] = ofToInt(s);
				return true;
			}
			else if (data[1] == GetCurrentPosition) {
				if (getDriver()->readLine(s) > 0) {
					currentPositions[stepper] = ofToInt(s);
					return true;
				}
			}
			else {
				// no matter the input command ACK always echos
				ofRobotTrace() << "xyRobot ACK for " << (int)data[1] << std::endl;
				// readline for commands that send lines getDriver()->readLine(s);
			}
		}
		return false;
	}
	void xyRobot::add(XYCommands cmd, const ofVec2f& point) {
		ofVec2f absolutePoint;
		absolutePoint.x = (int)(maxPositions[IDstepperX] * point.x);
		absolutePoint.y = (int)(maxPositions[IDstepperY] * point.y);
		add(xyDataToSend(cmd, absolutePoint));
		return;
	}
	int SerialData::get(int high, int low) {
		return bytes_to_u16(at(high), at(low));
	}

	void SerialData::set(uint16_t high, uint16_t low, int val) {
		ofRobotTrace() << "set " << val << std::endl;
		set(high, highByte(val));
		set(low, lowByte(val));
	}
	void ofTrRobotArmInternals::setX(int x) {
		ofRobotTrace() << "try to set x=" << x << std::endl;
		if (inRange(ArmX, x)) {
			setLowLevelX(x, addMagicNumber());
		}
	}
	void ofTrRobotArmInternals::setY(int y) {
		ofRobotTrace() << "try to set y=" << y << std::endl;
		if (inRange(ArmY, y)) {
			setLowLevelY(y);
		}
	}
	void ofTrRobotArmInternals::setZ(int z) {
		ofRobotTrace() << "try to set z=" << z << std::endl;
		if (inRange(ArmZ, z)) {
			setLowLevelZ(z);
		}
	}
	void ofTrRobotArmInternals::setWristAngle(int a) {
		ofRobotTrace() << "try to set setWristAngle=" << a << std::endl;
		if (inRange(wristAngle, a)) {
			setLowLevelWristAngle(a);
		}
	}
	void ofTrRobotArmInternals::setWristRotate(int a) {
		ofRobotTrace() << "try to set setWristRotate=" << a << std::endl;
		if (inRange(wristRotate, a)) {
			setLowLevelWristRotate(a);
		}
	}
	void ofTrRobotArmInternals::setGripper(int distance) {
		ofRobotTrace() << "try to set setGripper=" << distance << std::endl;
		if (inRange(ArmGripper, distance)) {
			setLowLevelGripper(distance);
		}
	}
	void ofTrRobotArmInternals::setMin(SpecificJoint joint, int value) {
		if (userDefinedRanges) {
			if (value < hardwareRanges.maxValue[joint] && value > hardwareRanges.minValue[joint]) {
				userDefinedRanges->minValue[joint] = value;
			}
			else {
				ofRobotTrace(ErrorLog) << "minValue out of range " << value << " range is " << hardwareRanges.minValue[joint] << ", " << hardwareRanges.maxValue[joint] << std::endl;
			}
		}
	}
	void ofTrRobotArmInternals::setMax(SpecificJoint joint, int value) {
		if (userDefinedRanges) {
			if (value < hardwareRanges.maxValue[joint] && value > hardwareRanges.minValue[joint]) {
				userDefinedRanges->maxValue[joint] = value;
			}
			else {
				ofRobotTrace(ErrorLog) << "maxValue out of range " << value << " range is " << hardwareRanges.minValue[joint] << ", " << hardwareRanges.maxValue[joint] << std::endl;
			}
		}
	}
	int ofTrRobotArmInternals::getMin(robotArmJointType type) {
		if (userDefinedRanges && userDefinedRanges->minValue.find(SpecificJoint(info.getType(), type)) != userDefinedRanges->minValue.end()) {
			return userDefinedRanges->minValue[SpecificJoint(info.getType(), type)];
		}
		return hardwareRanges.minValue[SpecificJoint(info.getType(), type)];
	}
	int ofTrRobotArmInternals::getMid(robotArmJointType type) {
		return (getMax(type) - getMin(type)) / 2; //bugbug works for robot types? 
	}

	int ofTrRobotArmInternals::getMax(robotArmJointType type) {
		if (userDefinedRanges && userDefinedRanges->maxValue.find(SpecificJoint(info.getType(), type)) != userDefinedRanges->maxValue.end()) {
			return userDefinedRanges->maxValue[SpecificJoint(info.getType(), type)];
		}
		return hardwareRanges.maxValue[SpecificJoint(info.getType(), type)];
	}

	bool ofTrRobotArmInternals::inRange(robotArmJointType type, int value) {
		if (value > getMax(type) || value < getMin(type)) {
			ofRobotTrace(ErrorLog) << "out of range " << value << " (" << getMin(type) << ", " << getMax(type) << ")" << std::endl;
			return false;
		}
		return true;
	}
	// needs to only be called one time -- uses static data to save time/space. backhoe not supported
	void ofTrRobotArmInternals::oneTimeSetup() {

		// these values come from http://learn.trossenrobotics.com/arbotix/arbotix-communication-controllers/31-arm-link-reference.html#limits for pincher and there
		// is also a page for the Reactor.  Any ranges can be set as default ranges.

		// Reactor

		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, PhantomXReactorArm), ArmX), -300, 300, 0);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, PhantomXReactorArm), ArmX), -300, 300, 0);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, PhantomXReactorArm), ArmY), 50, 350, 235);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, PhantomXReactorArm), ArmY), 20, 150, 140);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, PhantomXReactorArm), ArmZ), 20, 250, 210);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, PhantomXReactorArm), ArmZ), 10, 150, 30);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, PhantomXReactorArm), wristAngle), -30, 30, 0);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, PhantomXReactorArm), wristAngle), -90, -45, -90);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, PhantomXReactorArm), wristRotate), 0, 1023, 512);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, PhantomXReactorArm), wristRotate), 0, 1023, 512);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, PhantomXReactorArm), ArmGripper), 0, 512, 512);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, PhantomXReactorArm), ArmGripper), 0, 512, 512);


		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, PhantomXReactorArm), ArmX), 0, 1023, 512);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, PhantomXReactorArm), ArmX), 0, 1023, 512);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, PhantomXReactorArm), ArmY), 50, 350, 235);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, PhantomXReactorArm), ArmY), 20, 150, 140);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, PhantomXReactorArm), ArmZ), 20, 250, 210);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, PhantomXReactorArm), ArmZ), 10, 150, 30);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, PhantomXReactorArm), wristAngle), -30, 30, 0);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, PhantomXReactorArm), wristAngle), -90, -45, -90);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, PhantomXReactorArm), wristRotate), 0, 1023, 512);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, PhantomXReactorArm), wristRotate), 0, 1023, 512);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, PhantomXReactorArm), ArmGripper), 0, 512, 512);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, PhantomXReactorArm), ArmGripper), 0, 512, 512);

		// Pincher
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, PhantomXPincherArm), ArmX), -200, 200, 0);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, PhantomXPincherArm), ArmX), -200, 200, 0);

		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, PhantomXPincherArm), ArmY), 20, 240, 170);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, PhantomXPincherArm), ArmY), 20, 150, 140);

		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, PhantomXPincherArm), ArmZ), 20, 250, 210);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, PhantomXPincherArm), ArmZ), 10, 150, 30);

		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, PhantomXPincherArm), wristAngle), -30, 30, 0);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, PhantomXPincherArm), wristAngle), -90, -45, -90);

		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, PhantomXPincherArm), wristRotate), 0, 1023, 512);
				set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, PhantomXPincherArm), wristRotate), 0, 1023, 512);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, PhantomXPincherArm), ArmGripper), 10, 512, 512);
		set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, PhantomXPincherArm), ArmGripper), 10, 512, 512);


		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, PhantomXPincherArm), ArmX), 0, 1023, 512);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, PhantomXPincherArm), ArmX), 0, 1023, 512);

		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, PhantomXPincherArm), ArmY), 20, 240, 170);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, PhantomXPincherArm), ArmY), 20, 150, 140);

		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, PhantomXPincherArm), ArmZ), 20, 250, 210);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, PhantomXPincherArm), ArmZ), 10, 150, 30);

		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, PhantomXPincherArm), wristAngle), -30, 30, 0);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, PhantomXPincherArm), wristAngle), -90, -45, -90);

		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, PhantomXPincherArm), wristRotate), 0, 1023, 512);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, PhantomXPincherArm), wristRotate), 0, 1023, 512);

		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, PhantomXPincherArm), ArmGripper), 10, 512, 512);
		set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, PhantomXPincherArm), ArmGripper), 10, 512, 512);

		// mark end of list for debugging
		set(SpecificJoint(createUndefinedRobotType(), JointNotDefined), 0, 0, 0); // should not be set to this while running, only during object init

		return;

	}

	robotLowLevelCommands getStartCommand(robotMode mode) {
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
	void SerialData::set(uint16_t offset, uint8_t b) {
		ofRobotTrace() << "set SerialData[" << offset << "] = " << (uint16_t)b << std::endl;
		at(offset) = b;
	}

	const string BotInfo::trace() {
		std::ostringstream message;
		message << "ArmInfo(mode, type) " << getMode() << " " << getTypeID();
		return message.str();
	}

	// "home" and set data matching state
	void ofTrRobotArmInternals::setDefaultState() {
		ofRobotTrace() << "setDefaults";
		setX(getDefaultValue(ArmX));
		setY(getDefaultValue(ArmY));
		setZ(getDefaultValue(ArmZ));
		setWristAngle(getDefaultValue(wristAngle));
		setWristRotate(getDefaultValue(wristRotate));
		setGripper(getDefaultValue(ArmGripper));
		setDelta();
		setLowLevelCommand(noCommand());
		setButton();
	}
	// will block until arm is ready
	robotType ofTrRobotArmInternals::setStartState(robotMode mode) {
		info.setMode(mode);
		ofRobotTrace() << "setStartState(mode, type) " << info.trace() << std::endl;
		setLowLevelCommand(getStartCommand(info.getMode()));
		return info.getType();
	}

	void Pose::trace() {

#define ECHO(a)ofRobotTrace() << "echo[" << a << "] = "  << std::hex << (unsigned int)at(a) << "h "  <<  std::dec <<(unsigned int)at(a) << "d "<< #a << std::endl;

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
			ECHO(trChecksum)
	}
	
	uint8_t getChkSum(uint8_t*data, int start, int end) {
		uint16_t sum = 0;
		for (int i = start; i <= end; ++i) {
			sum += data[i];
		}
		uint16_t invertedChecksum = sum % 256;//isolate the lowest byte 8 or 16?
		uint8_t cksum = 255 - invertedChecksum;
		return cksum;
	}
	void SerialData::setChkSum(int index) {
		at(index) = getChkSum(data());
	}
	// user defined can never be greater than hardware min/max
	void ofTrRobotArmInternals::setUserDefinedRanges(SpecificJoint joint, shared_ptr<RobotValueRanges>) {

		if (userDefinedRanges) {
			if (userDefinedRanges->minValue[joint] < hardwareRanges.minValue[joint] || userDefinedRanges->maxValue[joint] > hardwareRanges.maxValue[joint]) {
				ofRobotTrace(ErrorLog) << "RobotJoints::setUserDefinedRanges value out of range ignored " << std::endl;
				return;
			}
		}
		this->userDefinedRanges = userDefinedRanges;
	}

	void ofTrRobotArmInternals::set(SpecificJoint type, int minVal, int maxVal, int defaultVal) {
		ofRobotTrace() << "RobotJoints::set" << echoJointType(type) << " - min = " << minVal << " max = " << maxVal << " default = " << defaultVal << std::endl;

		hardwareRanges.minValue[type] = minVal;
		hardwareRanges.maxValue[type] = maxVal;
		hardwareRanges.defaultValue[type] = defaultVal;
	}

	int ofTrRobotArmInternals::getDefaultValue(robotArmJointType type) {
		if (userDefinedRanges && userDefinedRanges->defaultValue.find(SpecificJoint(info.getType(), type)) != userDefinedRanges->defaultValue.end()) {
			return userDefinedRanges->defaultValue[SpecificJoint(info.getType(), type)];
		}
		return  hardwareRanges.defaultValue[SpecificJoint(info.getType(), type)];
	}
	void ofTrRobotArmInternals::setDefault(SpecificJoint joint, int value) {
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