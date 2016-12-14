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

//http://www.roborealm.com/help/API.php

//http://biorobots.case.edu/wp-content/uploads/2014/12/IntroductiontoDynamixelMotorControlUsingtheArbotiX20141112-1.pdf
namespace RobotArtists {

	RobotValueRanges ofTrRobotArmInternals::hardwareRanges; // just need to set once
													
	string Pose::dataName(int id) {
		switch (id) {
		case headerByteOffset:
			return " headerByte";
		case xHighByteOffset:
			return " xHighByte";
		case xLowByteOffset:
			return " xLowByte";
		case yHighByteOffset:
			return " yHighByte";
		case yLowByteOffset:
			return " yLowByte";
		case zHighByteOffset:
			return " zHighByte";
		case zLowByteOffset:
			return " zLowByte";
		case wristAngleHighByteOffset:
			return " wristAngleHighByte";
		case wristAngleLowByteOffset:
			return " wristAngleLowByte";
		case wristRotateHighByteOffset:
			return " wristRotateHighByte";
		case wristRotateLowByteOffset:
			return " wristRotateLowByte";
		case gripperHighByteOffset:
			return " gripperHighByte";
		case gripperLowByteOffset:
			return " gripperLowByte";
		case deltaValBytesOffset:
			return " deltaValBytes";
		case buttonByteOffset:
			return " buttonByte";
		case extValBytesOffset:
			return " extValBytes";
		case trChecksum:
			return " checksum";
		}
		return "???";
	}
	size_t ofRobotSerial::readInt() {
		string s;
		readLine(s);
		return ofToInt(s);
	}
	float ofRobotSerial::readFloat() {
		string s;
		readLine(s);
		return ofToFloat(s);
	}
	size_t ofRobotSerial::writeLn(const string&str) { 
		size_t size = write((uint8_t*)str.c_str(), str.size()); 
		write((uint8_t*)"\r", 1);
		return size; 
	}
	size_t ofRobotSerial::write(const ofVec2f& motion) {
		size_t count = write(motion.x);
		count += write(motion.y);
		return count;
	}
	size_t ofRobotSerial::readLine(string &s) {
		uint8_t bytes[512]; // max size of a line, beyond this things get ignored
		size_t c;
		s.clear();
		if ((c=readLine(bytes, sizeof bytes)) > 0) {
			for (int i = 0; i < c; ++i) {
				s += bytes[i];
			}
		}
		return c;
	}
	size_t ofRobotSerial::readLine(uint8_t* bytes, size_t bytesMax)	{
		if (!bytes) {
			return 0;
		}
		// null data in case none is found we always return null termanted data
		memset(bytes, 0, bytesMax);
		size_t i = 0;
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
	size_t ofRobotSerial::readBytesWithoutWaitingForData(uint8_t *bytes, size_t bytesMax) {
		size_t result = 0;
		if (available() > 0) {
			if ((result = readBytesSizeT(bytes, bytesMax)) == OF_SERIAL_ERROR) {
				ofRobotTrace(ErrorLog) << "serial failed" << std::endl;
				return 0;
			}
			while (result == OF_SERIAL_NO_DATA) {
				result = readBytesSizeT(bytes, bytesMax);
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
	size_t ofRobotSerial::readAllBytes(uint8_t *bytes, size_t bytesRequired) {
		size_t readIn = 0;

		if (bytes) {
			memset(bytes, 0, bytesRequired); // keep data  clean
			int tries = 0;
			size_t bytesRemaining = bytesRequired;
			// loop until we've read everything
			while (bytesRemaining > 0) {
				// check for data
				if (available() > 0) {
					// try to read - note offset into the bytes[] array, this is so
					// that we don't overwrite the bytes we already have
					size_t bytesArrayOffset = bytesRequired - bytesRemaining;
					size_t result = readBytesSizeT(&bytes[bytesArrayOffset], bytesRemaining);
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
	size_t iRobot::sendToRobot(SerialData *serial) {
		size_t size = 0;
		if (driver) {
			size = driver->write(serial);
			ofSleepMillis(50);
		}
		return size;
	}


	// return true if ID packet is value
	bool ofRobotSerial::idPacket(uint8_t *bytes, size_t size) {
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
	robotType ofRobotSerial::IDResponsePacket(uint8_t *bytes, size_t count) {
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
	void xyDataToSend::move(const xyMotion& to) {
		add(to);
	}
	float getPt(float n1, float n2, float perc) {
		float diff = n2 - n1;

		return n1 + (diff * perc);
	}
	void xyDataToSend::elipse() {
		float height = 0.1f;
		float width = 0.1f;
		for (float y = -height; y <= height; y++) {
			for (long x = -width; x <= width; x++) {
				if (x*x*height*height + y*y*width*width <= height*height*width*width) {
					move(xyMotion(x, y)); 
				}
			}
		}

	}
	// fun curve
	void xyDataToSend::bezier() {
		ofVec2f one(0.01f, 0.01f), two(0.04f, 0.02f), three(0.03f, 0.02f);
		for (float i = 0; i < 1; i += 0.01) {
			// The Green Line
			float xa = getPt(one.x, two.x, i);
			float ya = getPt(one.y, two.y, i);
			float xb = getPt(two.x, three.x, i);
			float yb = getPt(two.y, three.y, i);

			// The Black Dot
			float x = getPt(xa, xb, i);
			float y = getPt(ya, yb, i);
			move(xyMotion(x,y));
		}
	}
	/*A point at angle theta on the circle whose centre is (x0,y0) 
	 and whose radius is r is (x0 + r cos theta, y0 + r sin theta). 
	 Now choose theta values evenly spaced between 0 and 2pi.*/
	void xyDataToSend::circle() {
		float radius = 0.1;
		ofVec2f center;//0,0
		ofVec2f start(radius, 0);
		ofVec2f val;
		int count = 100;
		double slice = 2 * PI / count;
		for (int i = 0; i <= count; i++)		{
			double angle = slice * i;
			float x = (center.x + radius * cos(angle));
			float y = (center.y + radius * sin(angle));
			// move to x,y from current location
			ofVec2f p = ofVec2f(x, y)-start;
			move(xyMotion(p));
			start = ofVec2f(x, y);
		}
	}
	//bugbug draw a few basic shapes then make them functions vs one big function
	void xyDataToSend::macro(const XYCommands& command) {
		float a = 0.1f;
		float b = 0.1f;
		float c = sqrt(a*a + b*b);

		switch (command) {
		case xyRectangle:
			move(xyMotion(0.1f, 0.0f));
			move(xyMotion(0.0f, 0.1f));
			move(xyMotion(-0.1f, 0.0f));
			move(xyMotion(0.0f, -0.1f));
			break;
		case xyTriangle:
			// assume a = 0.1, b = 0.1, c = 0.1
			move(xyMotion(b, a)); // hypotenuse
			move(xyMotion(-b, 0.0f));
			move(xyMotion(0.0f, -a));
			break;
		case xyLine:
			move(xyMotion(a, b));
			move(xyMotion(-a, -b)); // go back just to test
			break;
		case xyCircle:
			circle();
			break;
		case xyBezier:
			bezier();
			break;
		default:
			break;
		}
		trace();

	}

	robotType ofRobotSerial::waitForRobot(string& name, int retries, size_t packetsize) {
		ofRobotTrace() << "wait for mr robot ... " << std::endl;

		robotType type = createUndefinedRobotType();
		uint8_t bytes[500];
		
		if (waitForSerial(retries)) {
			// somethis is out there, see if we can ID it

			size_t readin = readAllBytes(bytes, packetsize);
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
	// one command, two values
	xyDataToSend::xyDataToSend(XYCommands cmd, const xyMotion& point):SerialData(2) {
		setCommand(cmd);
		parameters.push_back(point);
	}

	void xyDataToSend::trace() {
		ofRobotTrace("xyDataToSend") << "cmd = " << (int)getCommand() << std::endl;
		for (auto&a : parameters) {
			a.trace();
		}
	}
	void xyDataToSend::setCommand(XYCommands cmd) {
		set(0, 0xee);
		set(1, cmd);
		trace();
	}

	size_t ofRobotSerial::write(uint8_t* data, size_t count) {
		// from http://learn.trossenrobotics.com/arbotix/arbotix-communication-controllers/31-arm-link-reference.html
		if (count <= 0) {
			return 0;
		}
		//If you are sending packets at an interval, do not send them faster than 30hz(one packet every 33ms).
		// no need to hurry packets so just want the minimum amount no matter what
		ofSleepMillis(100); // 100 ms seems ok, to much less and we start to overrun bugbug is this true?  
		size_t sent = writeBytes(data, (int)count);//bugbug of function should e size_t

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
		
		size_t sent = getDriver()->write(&pose);
		ofSleepMillis(33);

		uint8_t data[500]; // could be lots of sperius data out there, unlikely but if it occurs we want to echo it
		memset(data, 0, sizeof data);
		uint16_t val = 0;
		size_t readin = getDriver()->readAllBytes(data, 5);
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

	size_t ofRobotSerial::write(SerialData*serial) {
		if (serial) {
			serial->update();
			for (int i = 0; i < serial->size(); ++i) {
				ofRobotTrace() << " bytes[" << i << "] = " << (int)serial->at(i) << serial->dataName(i) << std::endl; 
			}
			return write(serial->data(), serial->size());
		}
		return 0;
	}

	
	// length == 2 for ax12SetRegister2
	void ofTrRobotArmInternals::setServoRegister(TrossenServoIDs id, AXRegisters registerNumber, int length, int dataToSend) {
		setLowLevelCommand(setServoRegisterCommand);
		setLowLevelX(id); // servo 
		setLowLevelY(registerNumber);
		setLowLevelZ(length);
		setLowLevelWristAngle(dataToSend, 0);
		
		getDriver()->flush();   // reset
		size_t sent = getDriver()->write(this);
		ofSleepMillis(33);
		uint8_t data[5];
		size_t readin = getDriver()->readAllBytes(data, 5);
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
	int SerialData::get(int high, int low) {
		return bytes_to_u16(at(high), at(low));
	}

	void SerialData::set(uint16_t high, uint16_t low, int val) {
		ofRobotTrace() << "set " << val << std::endl;
		set(high, highByte(val));
		set(low, lowByte(val));
	}
	// set if in valid range
	void ofTrRobotArmInternals::setJointValue(robotArmJointType type, int val) {
		ofRobotTrace() << "try to set val=" << val << " type: " << type << std::endl;
		if (inRange(type, val)) {
			switch (type) {
			case ArmX:
				setLowLevelX(val, addMagicNumber());
				return;
			case ArmY:
				setLowLevelY(val);
				return;
			case ArmZ:
				setLowLevelZ(val);
				return;
			case wristAngle:
				setLowLevelWristAngle(val);
				return;
			case wristRotate:
				setLowLevelWristRotate(val);
				return;
			case ArmGripper:
				setLowLevelGripper(val);
				return;
			}
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
		datasetup = true;
	}

	const string BotInfo::trace() {
		std::ostringstream message;
		message << "ArmInfo(mode, type) " << getMode() << " " << getTypeID();
		return message.str();
	}

	// "home" and set data matching state
	void ofTrRobotArmInternals::setDefaultState() {
		ofRobotTrace() << "setDefaults";
		setJointValue(ArmX, getDefaultValue(ArmX));
		setJointValue(ArmY, getDefaultValue(ArmY));
		setJointValue(ArmZ, getDefaultValue(ArmZ));
		setJointValue(wristAngle, getDefaultValue(wristAngle));
		setJointValue(wristRotate, getDefaultValue(wristRotate));
		setJointValue(ArmGripper, getDefaultValue(ArmGripper));
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
	
	uint8_t getChkSum(uint8_t*data, size_t start, size_t end) {
		uint16_t sum = 0;
		for (size_t i = start; i <= end; ++i) {
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