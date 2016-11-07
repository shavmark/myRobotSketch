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
			buffer << " bytes[" << i << "] = " << (int)bytes[i] << std::endl; // echo in one line
		}
		ofRobotTrace() << buffer.str() << std::endl;
	}

	void ofRobotState::echo() {
		ofRobotTrace() << "WristAngle=" << (set[0] ? ofToString(getWristAngle()) : "<not set>") << std::endl;
		ofRobotTrace() << "WristRotatation=" << (set[1] ? ofToString(getWristRotation()) : "<not set>") << std::endl;
		ofRobotTrace() << "Gripper=" << (set[2] ? ofToString(getGripper()) : "<not set>") << std::endl;
	}
	void ofRobotPosition::echo()  {
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

	void ofRobotCommands::echo()  {
		for ( auto& cmd : vectorOfRobotCommands) {
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
		vectorOfRobotCommands.push_back(cmd);
	}

	void ofRobotCommands::sanityTestHighLevel() {
		ofRobotTrace() << "high level sanityTest" << std::endl;
		reset();
		ofRobotCommand cmd(0.3f, 0.6f, NoRobotValue, 1.0f, -1.0f, 0.5f);
		// add one command with main points/states using various techniques
		cmd.add(ofRobotPosition(0.3f, 0.6f), ofRobotState(1.0f, -1.0f, 0.5f));// need to be percents!!
		cmd.add(ofRobotPosition(NoRobotValue, NoRobotValue, 0.3f));// need to be percents!!
		cmd.add(ofRobotPosition(NoRobotValue, NoRobotValue, 1.0f));// need to be percents!!
		//typedef std::tuple<ofRobotPosition, ofRobotState, RobotArmDelta> robotCommandRequest;

		// add a new command, either way works
		add(ofRobotCommand(Sleep, 1000)); // sleep

		add(cmd);

		///setLowLevelCommand(NoArmCommand);
		//setDelta();
		//setButton();
	}
	void ofRobotCommands::sendData(RobotCommandData&data) {
		set(data);
		sendToRobot(&robot->serial);
	}
	void ofRobotCommands::sendResults(vector<RobotCommandData>& results) {
		for (auto& a : results) {
			sendData(a);
		}
	}


	void ofRobotJoints::sendToRobot(ofRobotSerial* serial) {
		echo();
		if (serial) {
			serial->write(getPose(), count);
		}
	}

	// set basic data that moves a little bit after starting up. does low level writes only. Does not call reset() or any high level function
	void ofRobotCommands::sanityTestLowLevel() {
		ofRobotTrace() << "low level sanityTest" << std::endl;
		reset();
		sendToRobot(&robot->serial);
		setDelta(254);
		setX(100); // absolution position vs. percentages
		setY(100);
		//setZ(50);
		setWristAngle(30);
		//setWristRotate(120);
		setGripper(10);
		setButton();
		sendToRobot(&robot->serial);
		setGripper(100);
		sendToRobot(&robot->serial);
		setGripper(511);
		sendToRobot(&robot->serial);
		sleep(1000);
		setX(200);
		sendToRobot(&robot->serial);
		setX(0);
		sendToRobot(&robot->serial);
	}


	ofRobotCommands::ofRobotCommands(ofRobot *robot) :ofRobotJoints(robot->pose, robot->type) {
		this->robot = robot;
		if (robot) {
			//typedef pair<robotType, robotArmJointType> SpecificJoint
			setUserDefinedRanges(SpecificJoint(robot->type, X), robot->userDefinedRanges);
		}
	}
	void ofRobotCommands::reset() { // setup can be ignored for a reset is not required
		setStartState();
		sendToRobot(&robot->serial); // send the mode, also resets the robot
		setDefaultState();
		clear(vectorOfRobotCommands);
	}

	//ofPushMatrix();
	//ofTranslate(400, 300);
	//ofPopMatrix();

	// draw circle at current positon
	void ofRobotCommands::drawCircle(float r)
	{
		// do a move like OF does so drawing always starts at current
		float slice = 2 * M_PI / 10;
		for (int i = 0; i < 10; i++) {
			float angle = slice * i;
			float newX = r * cos(angle);
			float newY = r * sin(angle);
			ofRobotTrace() << newX << " " << newY << std::endl; 
			ofRobotCommand cmd(RobotCommandData(ofRobotPosition(newX, newY), ofRobotState(), RobotBaseClass::minDelta()));
			add(cmd);
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
		sendToRobot(&robot->serial);
		popMatrix();

		sendToRobot(&robot->serial); // restore to normal z

	}
	void ofRobotCommands::testdata() {
		getPose(); // do check sum
		echoRawData();
		echo(); // echo object data
	}
	void ofRobotCommands::set(RobotCommandData& request) {
		setPoint(request.getPoint());
		setState(request.getState());
		setDelta(request.getDelta());
	}

	void ofRobotCommands::update() {
		
		
	}

	// drawing occurs here as its tied to the robot directly
	void ofRobotCommands::draw() {

		if (robot) {
			
			vector< ofRobotCommand >::iterator it = vectorOfRobotCommands.begin();
			// for all commands
			while (it != vectorOfRobotCommands.end()) {
				clear(results);
				switch (it->cmd) {
				case HighLevelTest:
					sanityTestHighLevel(); 
					break;
				case LowLevelTest:
					sanityTestLowLevel();
					break;
				case UserDefinded:
					// no processing needed, just send it on
					sendResults(it->vectorOfCommandData);
					break;
				case Circle:
					for (auto& a : it->vectorOfCommandData) {
						drawCircle(a.float1); // populates vectorData
					}
					break;
				case Sleep:
					for (auto& a : it->vectorOfCommandData) {
						sleep(a.int1); // populates vectorData
					}
					break;
				case Translate:
					sendResults(it->vectorOfCommandData);
					break;
				case Push:
					pushMatrix();
					break;
				case Pop:
					popMatrix();
					break;
				}
				
				sendResults(results);
				if (it->OKToDelete()) {
					it = vectorOfRobotCommands.erase(it);
				}
				else {
					++it;
				}
			}
		}
	}

	void ofRobotCommand::echo()  {
		for ( auto& a : vectorOfCommandData) {
			a.getPoint().echo();
			a.getState().echo();
		}
	}

	void ofRobot::validate() {

		ofRobotTrace() << "valiate robot" << std::endl;
		
		for (int i = FIRST_SERVO; i < servorCount; ++i) {
			for (int j = 1; j <= 5; ++j) { // flash a bit
				serial.setLED(static_cast<ServoIDs>(i), 1);
				int led = serial.getLED(static_cast<ServoIDs>(i));
				if (led != 1) {
					ofRobotTrace(WarningLog) << "reg set fails" << std::endl; // may occur during startup
				}
				ofSleepMillis(100);
				serial.setLED(static_cast<ServoIDs>(i), 0);
				led = serial.getLED(static_cast<ServoIDs>(i));
				if (led != 0) {
					ofRobotTrace(WarningLog) << "reg set fails" << std::endl;
				}
			}
			for (int i = FIRST_SERVO; i < servorCount; ++i) {
				ofRobotTrace() << "servo " << i;

				int pos = serial.getPosition(static_cast<ServoIDs>(i));
				ofRobotTrace() << " pos " << pos;

				int tmp = serial.getTempature(static_cast<ServoIDs>(i));
				ofRobotTrace() << " temp. " << tmp;

				float v = serial.getVoltage(static_cast<ServoIDs>(i))/10;
				ofRobotTrace() << " voltage " << v << std::endl;
			}
		}
		
		
	}

	void ofRobot::setup(int deviceID) {
		commands = make_shared<ofRobotCommands>(this);
		memset(pose, 0, sizeof(pose));
		
		if (!serial.setup(deviceID, 38400)) {
			ofRobotTrace(FatalErrorLog) << "serial fails to setup" << std::endl;
			return;
		}

		ofRobotTrace() << "setup robot" << std::endl;

		// setup the robot
		switch (getTypeID()) {
		case PhantomXReactorArm:
			servorCount = REACTOR_SERVO_COUNT;
			break;
		case PhantomXPincherArm:
			servorCount = PINCHER_SERVO_COUNT;
			break;
		case WidowX:
			servorCount = WIDOWX_SERVO_COUNT;
			break;
		}

		validate(); // validate roboth

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
			ofRobotTrace("FindAllRobots") << "[" << device.getDeviceID() << "] = " << device.getDeviceName().c_str();
			ofRobotSerial testSerial;// need to close this and free port once set up
			if (!testSerial.setup(device.getDeviceName(), 38400)) {
				ofRobotTrace(FatalErrorLog) << "FindAllRobots" << std::endl;
				return;
			}
			// port found, see what may be on it
			// start with default mode
			uint64_t t1 = ofGetElapsedTimeMillis();
			robotType robotType;
			string robotName;
			if (!robotTypeIsError(robotType = testSerial.waitForRobot(robotName, 25))) {
				uint64_t t2 = ofGetElapsedTimeMillis();
				int gone = t2 - t1;

				// one of InterbotiXPhantomXReactorArm, InterbotiXPhantomXPincherArm, unknownRobotType 
				switch (robotType.second) {
				case PhantomXReactorArm:
					// add robot here
					ofRobotTrace() << "Reactor, " << robotName << " on " << device.getDeviceName();
					break;
				case PhantomXPincherArm:
					ofRobotTrace() << "Pincher, " << robotName << " on " << device.getDeviceName();
					break;
				case unknownRobotType:
					ofRobotTrace() << "unknown robot type " << std::endl;
					return;
				}
				testSerial.close();
				shared_ptr<ofRobot> robot = make_shared<ofRobot>(robotName, robotType);
				if (robot) {
					robot->setup(device.getDeviceID());
					robots.push_back(robot);
					ofRobotTrace() << "install duration in milliseconds " << gone << " for " << robotName << std::endl;
				}
			}
			else {
				ofRobotTrace() << "no robot at " << device.getDeviceName() << std::endl;
			}
		}

	}

}
