/*
ofRobots.cpp - openframeworks based classes for managing robots
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
#include <algorithm> 
#define _USE_MATH_DEFINES
#include <math.h>

namespace RobotArtists {

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

	// echo, ignoring null bytes
	void ofRobotSerial::echoRawBytes(uint8_t *bytes, int count) {
		std::stringstream buffer;
		for (int i = 0; i < count; ++i) {
			buffer << " bytes[" << i << "] = " << (int)bytes[i]; // echo in one line
		}
		ofRobotTrace() << buffer.str() << std::endl;
	}

	// get pose data from serial port bugbug decode this
	void ofRobotSerial::readPose() {
		if (available() == 0) {
			return;
		}

		ofRobotTrace() << "RobotSerial::readPose" << std::endl;

		uint8_t *bytes = new uint8_t[1000];//bugbug clean this up
		int i = 0;
		for (; i < 1000; ++i) {
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
		bytes[i] = 0;
		if (i == 5) { // input data is fixed size format etc, just trace stuff
			ArmIDResponsePacket(bytes);
		}
		else if (i == 10) {
			echoRawBytes(bytes, 10);
			ArmIDResponsePacket(bytes); // 2 signs ons come back some times, likely a timing issue?
			ArmIDResponsePacket(bytes); // first 2 bytes are sign on type
		}
		else {
			ofRobotTrace() << bytes << std::endl;
		}
		delete bytes;

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

	// bool readOnly -- just read serial do not send request
	robotType ofRobotSerial::ArmIDResponsePacket(uint8_t *bytes) {
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
			case 1:
				id = InterbotiXPhantomXPincherArm;
				ofRobotTrace() << "InterbotiXPhantomXPincherArm" << std::endl;
				break;
			case 2:
				id = InterbotiXPhantomXReactorArm;
				ofRobotTrace() << "InterbotiXPhantomXReactorArm" << std::endl;
				break;
			}
			return robotType(armMode, id);
		}
		return robotType(IKM_NOT_DEFINED, unknownRobotType);
	}


	robotType ofRobotSerial::waitForRobot(int retries) {
		ofRobotTrace() << "wait for mr robot ... aka " << std::endl;

		robotType type = createUndefinedRobotType();

		if (waitForSerial(retries)) {
			// somethis is out there, see if we can ID it
			uint8_t bytes[31];

			int readin = readBytesInOneShot(bytes, 5);
			if (readin == 5) {
				type = ArmIDResponsePacket(bytes);
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
			readin = readBytesInOneShot(bytes, 30);
			bytes[readin] = 0;
			ofRobotTrace() << bytes << std::endl;
		}
		return type;
	}

	void ofRobotSerial::write(uint8_t* data, int count) {
		// from http://learn.trossenrobotics.com/arbotix/arbotix-communication-controllers/31-arm-link-reference.html

		//If you are sending packets at an interval, do not send them faster than 30hz(one packet every 33ms).
		// no need to hurry packets so just want the minimum amount no matter what
		ofSleepMillis(100); // 100 ms seems ok, to much less and we start to overrun bugbug is this true?  
		int sent = writeBytes(data, count);

		ofRobotTrace() << "write sent = " << sent << std::endl;

		readPose(); // pose is sent all the time
		readPose(); // how often are two sent?

	}
	void ofRobotState::echo() const {
		ofRobotTrace() << "WristAngle=" << (set[0] ? ofToString(getWristAngle()) : "<not set>") << std::endl;
		ofRobotTrace() << "WristRotatation=" << (set[1] ? ofToString(getWristRotation()) : "<not set>") << std::endl;
		ofRobotTrace() << "Gripper=" << (set[2] ? ofToString(getGripper()) : "<not set>") << std::endl;
	}
	void ofRobotPosition::echo() const {
		ofRobotTrace() << "x=" << (set[0] ? ofToString(x) : "<not set>") << std::endl;
		ofRobotTrace() << "y=" << (set[1] ? ofToString(y) : "<not set>") << std::endl;
		ofRobotTrace() << "z=" << (set[2] ? ofToString(z) : "<not set>") << std::endl;
	}
	// can be +//
	bool ofRobotPosition::validRange(float f) {
		if (abs(f) >= 0.0f && abs(f) <= 1.0f) {
			return true;
		}
		return false;
	}

	void ofRobotPosition::setPercents(float xPercent, float yPercent, float zPercent) {
		if (xPercent != NoRobotValue && validRange(xPercent)) {
			x = xPercent;
			set[0] = true;
		}
		else {
			set[0] = false;
		}
		if (yPercent != NoRobotValue && validRange(yPercent)) {
			y = yPercent;
			set[1] = true;
		}
		else {
			set[1] = false;
		}
		if (zPercent != NoRobotValue && validRange(zPercent)) {
			z = zPercent;
			set[2] = true;
		}
		else {
			set[2] = false;
		}
	}

	void ofRobotCommands::echo() const {
		for (const auto& cmd : cmdVector) {
			cmd.echo();
		}
	}

	// x - wrist angle, y is wrist rotate, z is gripper (using ofVec3f so its features can be leverage)
	void ofRobotCommands::setState(ofRobotState statePercent) {
		if (isCylindrical()) {
			if (statePercent.set[0]) {
				setWristAngle(getMin(wristAngle) + (statePercent.getWristAngle() * (getMax(wristAngle) - getMin(wristAngle))));
			}
			if (statePercent.set[1]) {
				setWristRotate(getMin(wristRotate) + (statePercent.getWristRotation() * (getMax(wristRotate) - getMin(wristRotate))));
			}
			if (statePercent.set[2]) {
				setGripper(getMin(Gripper) + (statePercent.getGripper() * (getMax(Gripper) - getMin(Gripper))));
			}
		}
	}

	//+/- .001 to 1.000, 0 means ignore 
	void ofRobotCommands::setPoint(ofRobotPosition ptPercent) {
		// only Cylindrical supported by this function, mainly the setx one
		if (isCylindrical()) {
			//ofMap
			if (ptPercent.set[0]) {
				setX(getMin(X) + (ptPercent.getX() * (getMax(X) - getMin(X))));
			}
			if (ptPercent.set[1]) {
				setY(getMin(Y) + (ptPercent.getY() * (getMax(Y) - getMin(Y))));
			}
			if (ptPercent.set[2]) {
				setZ(getMin(Z) + (ptPercent.getZ() * (getMax(Z) - getMin(Z))));
			}
		}
		else {
			ofRobotTrace(ErrorLog) << "setPoint not supported" << std::endl;
		}
	}

	// add ranges checking
	void ofRobotCommands::add(const ofRobotCommand& cmd) {
		cmdVector.push_back(cmd);
	}

	void ofRobotCommands::sanityTestHighLevel() {
		TraceBaseClass() << "high level sanityTest" << std::endl;
		reset();

		// add one command with main points/states using various techniques
		ofRobotCommand cmd(0.3f, 0.6f, NoRobotValue, 1.0f, -1.0f, 0.5f);
		cmd.add(ofRobotCommand::setCommand(ofRobotPosition(0.3f, 0.6f), ofRobotState(1.0f, -1.0f, 0.5f)));// need to be percents!!
		cmd.add(ofRobotCommand::setCommand(ofRobotPosition(NoRobotValue, NoRobotValue, 0.3f)));// need to be percents!!
		cmd.add(ofRobotCommand::setCommand(ofRobotPosition(NoRobotValue, NoRobotValue, 1.0f)));// need to be percents!!

		// add a new command, either way works
		add(ofRobotCommand(1000)); // sleep

		setLowLevelCommand(NoArmCommand);
		setDelta(255);
		setButton();
	}
	void ofRobotCommands::send(ofRobotSerial* serial) {

		echo();
		if (serial) {
			serial->write(getData(), count);
		}
	}

	// set basic data that moves a little bit after starting up. does low level writes only. Does not call reset() or any high level function
	void ofRobotCommands::sanityTestLowLevel() {
		ofRobotTrace() << "low level sanityTest" << std::endl;
		setGripper(ofRandom(255));
		setLowLevelCommand(NoArmCommand);
		setDelta(255);
		setX(100); // absolution position vs. percentages
		setY(100);
		setZ(100);
		setWristAngle(30);
		setWristRotate(120);
		setGripper(0);
		setButton();
		send(&robot->serial);
	}


	ofRobotCommands::ofRobotCommands(ofRobot *robot) :RobotJoints(robot->data, robot->type) {
		this->robot = robot;
		if (robot) {
			//typedef pair<robotType, robotArmJointType> SpecificJoint
			setUserDefinedRanges(SpecificJoint(robot->type, X), robot->userDefinedRanges);
		}
	}
	void ofRobotCommands::reset() { // setup can be ignored for a reset is not required
		setStartState();
		send(&robot->serial); // send the mode, also resets the robot
		setDefaultState();
		clear(cmdVector);
	}

	//ofPushMatrix();
	//ofTranslate(400, 300);
	//ofPopMatrix();

	// draw circle at current positon
	void ofRobotCommand::drawCircle()
	{
		// do a move like OF does so drawing always starts at current
		float slice = 2 * M_PI / 10;
		for (int i = 0; i < 10; i++) {
			float angle = slice * i;
			float newX = floatdata * cos(angle);
			float newY = floatdata * sin(angle);
			ofRobotTrace() << newX << " " << newY << std::endl; 
			add(setCommand(ofRobotPosition(newX, newY), RobotJointsState::slowestDelta)); 
			//add(ofRobotCommand(newX, newY)); // need to be percents!! bugbug make this a json player, then the creators of json are the engine
		}
		int i = 0;
		/*
		double slice = 2 * M_PI / points;
		for (int i = 0; i < points; i++)	{
			double angle = slice * i;
			float newX = (float)(center.x + radius * cos(angle));
			float newY = (float)(center.y + radius * sin(angle));
			add(ofRobotCommand(newX, newY)); // need to be percents!! bugbug make this a json player, then the creators of json are the engine
		}
		*/
	}

	void ofRobotCommands::move(const ofRobotPosition& pos) {
		setPoint(pos);
		
		// set Z to max, then restore z
		pushMatrix();
		ofRobotPosition newPos = pos;
		newPos.setPercents(NoRobotValue, NoRobotValue, getMin(Z));
		send(&robot->serial);
		popMatrix();

		send(&robot->serial); // restore to normal z

	}
	void ofRobotCommands::testdata() {
		getData(); // do check sum
		echoRawData();
		echo(); // echo object data
	}
	void ofRobotCommands::setTuple(ofRobotCommand::robotCommandRequest request) {
		setPoint(std::get<0>(request));
		setState(std::get<1>(request));
		setDelta(std::get<2>(request));
	}

	void ofRobotCommands::draw() {

		if (robot) {
			
			vector< ofRobotCommand >::iterator it = cmdVector.begin();
			while (it != cmdVector.end()) {
				switch (it->commandType()) {
				case ofRobotCommand::HighLevelTest:
					sanityTestHighLevel(); // populates cmdVector
					break;
				case ofRobotCommand::LowLevelTest:
					sanityTestLowLevel();
					return; // low level, nothing else to do
				case ofRobotCommand::Push:
					pushMatrix();
					break;
				case ofRobotCommand::Pop:
					popMatrix();
					break;
				case ofRobotCommand::Circle:
					it->drawCircle(); // populates vectorData
					break;
				case ofRobotCommand::Sleep:
					sleep(it->getFloatData()); // sleep at command level
					break;
				case ofRobotCommand::Translate:
					if (it->vectorData.size() == 0) {
						ofRobotTrace(ErrorLog) << "ofRobotCommand::Translate requires data" << std::endl;
					}
					else {
						move(std::get<0>(it->vectorData[0])); // Translate requires data
															  //bugbug test before sending
															  //testdata();
					}
					break;
				}

				// draw out all the points & states, sleeping as needed
				for (const auto& a : it->vectorData) {
					setTuple(a);
					//bugbug test before sending 
					//testdata();
					send(&robot->serial);
				}
				if (it->OKToDelete()) {
					it = cmdVector.erase(it);
				}
				else {
					++it;
				}
			}
		}
	}

	void ofRobotCommand::echo() const {
		for (const auto& a : vectorData) {
			std::get<0>(a).echo();
			std::get<1>(a).echo();
		}
	}

	void ofRobot::setup() {

		commands = make_shared<ofRobotCommands>(this);

	}
	void ofRobot::update() {
	}

	void ofRobot::echo() {
		if (commands) {
			commands->echo();
		}
	}

	void ofRobot::draw() {
		if (pause) {
			return;
		}
		if (commands) {
			commands->draw();
		}
	}
	shared_ptr<ofRobot> ofRobotFamly::getRobot(int index, RobotTypeID id) {
		ofRobotTrace() << "getRobot" << std::endl;
		int count = 0;
		for (int i = 0; i < robots.size(); ++i) {
			if (id == AllRobotTypes || id == robots[i]->getTypeID()) {
				if (count == index) {
					return robots[i];
				}
				++count;
			}
		}
		return nullptr;
	}
	void ofRobotFamly::update() {
		ofRobotTrace() << "update robots" << std::endl;
		for (const auto robot : robots) {
			if (idToUse == AllRobotTypes || idToUse == robot->getTypeID()) {
				robot->update();
			}
		}
	}

	void ofRobotFamly::draw() {
		ofRobotTrace() << "draw robots" << std::endl;
		for (const auto robot : robots) {
			if (idToUse == AllRobotTypes || idToUse == robot->getTypeID()) {
				robot->draw();
			}
		}
	}

	void ofRobotFamly::setup() {

		// do one time setup
		RobotJoints::oneTimeSetup(); // do one time setup of static data

		ofRobotSerial serial;
		serial.listDevices(); // let viewer see thats out there

		for (auto&device : serial.getDevices()) {
			if (device.getDeviceID() == 0) {
				continue;//bugbug skipping com1, not sure whats on it
			}
			ofLogNotice("FindAllRobots") << "[" << device.getDeviceID() << "] = " << device.getDeviceName().c_str();
			if (!serial.setup(device.getDeviceName(), 38400)) {
				ofRobotTrace(FatalErrorLog) << "FindAllRobots" << std::endl;
				return;
			}
			// port found, see what may be on it
			// start with default mode
			uint64_t t1 = ofGetElapsedTimeMillis();
			robotType robotType;
			if (!robotTypeIsError(robotType = serial.waitForRobot(25))) {
				uint64_t t2 = ofGetElapsedTimeMillis();
				int gone = t2 - t1;

				// one of InterbotiXPhantomXReactorArm, InterbotiXPhantomXPincherArm, unknownRobotType 
				std::ostringstream name;
				switch (robotType.second) {
				case InterbotiXPhantomXReactorArm:
					// add robot here
					name << "Reactor " << device.getDeviceID();
					break;
				case InterbotiXPhantomXPincherArm:
					name << "Pincher " << device.getDeviceID();
					break;
				case unknownRobotType:
					ofRobotTrace() << "unknown robot type " << std::endl;
					return;
				}
				shared_ptr<ofRobot> robot = make_shared<ofRobot>(name.str(), robotType, serial);
				if (robot) {
					robot->setup();
					robots.push_back(robot);
					ofRobotTrace() << "install duration in milliseconds " << gone << " for " << name.str() << std::endl;
				}
			}
			else {
				ofRobotTrace() << "no robot at " << device.getDeviceName() << std::endl;
			}
		}

	}

}
