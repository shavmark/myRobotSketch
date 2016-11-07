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

	//+/- 0 to 1.000
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

	void ofRobotCommands::sanityTestHighLevel(vector<ofRobotCommand>&commands) {
		ofRobotTrace() << "high level sanityTest" << std::endl;
		//IKM_CYLINDRICAL
		reset(IKM_CYLINDRICAL);
		ofRobotCommand cmd(0.3f, 0.6f, NoRobotValue, 1.0f, -1.0f, 0.5f);
		// add one command with main points/states using various techniques
		cmd.add(ofRobotPosition(0.3f, 0.6f), ofRobotState(1.0f, -1.0f, 0.5f));// need to be percents!!
		cmd.add(ofRobotPosition(NoRobotValue, NoRobotValue, 0.3f));// need to be percents!!
		cmd.add(ofRobotPosition(NoRobotValue, NoRobotValue, 1.0f));// need to be percents!!
		//typedef std::tuple<ofRobotPosition, ofRobotState, RobotArmDelta> robotCommandRequest;

		// add a new command, either way works
		commands.push_back(cmd);
		commands.push_back(addSleep(1000));

		///setLowLevelCommand(NoArmCommand);
		//setDelta();
		//setButton();
	}
	void ofRobotCommands::sendData(vector<RobotCommandData>&data) {
		for (auto& a : data) {
			set(a);
			sendToRobot(&robot->serial);
		}
	}
	void ofRobotCommands::sendResults(vector<ofRobotCommand>& results) {
		for (auto& a : results) {
			sendData(a.vectorOfCommandData);
		}
	}

	void ofRobotJoints::sendToRobot(ofRobotSerial* serial) {
		echo();
		if (serial) {
			serial->write(getPoseData(), count);
		}
	}

	// set basic data that moves a little bit after starting up. does low level writes only. Does not call reset() or any high level function
	void ofRobotCommands::sanityTestLowLevel() {
		ofRobotTrace() << "low level sanityTest" << std::endl;
		reset(IKM_CYLINDRICAL);
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
	void ofRobotCommands::reset(robotArmMode mode) { // setup can be ignored for a reset is not required
		setStartState(mode);
		sendToRobot(&robot->serial); // send the mode, also resets the robot
		setDefaultState();
		clear(vectorOfRobotCommands);
	}

	//ofPushMatrix();
	//ofTranslate(400, 300);
	//ofPopMatrix();

	// draw circle at current positon
	void ofRobotCommands::drawCircle(vector<ofRobotCommand>&commands, float r)
	{
		// do a move like OF does so drawing always starts at current
		float slice = 2 * M_PI / 10;
		for (int i = 0; i < 10; i++) {
			float angle = slice * i;
			float newX = r * cos(angle);
			float newY = r * sin(angle);
			ofRobotTrace() << newX << " " << newY << std::endl; 
			ofRobotCommand cmd(RobotCommandData(ofRobotPosition(newX, newY), ofRobotState(), RobotBaseClass::maxDelta()));
			commands.push_back(cmd);
		}
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
		getPoseData(); // do check sum
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
				vector<ofRobotCommand> results; // results of executing commands
				switch (it->cmd) {
				case HighLevelTest:
					sanityTestHighLevel(results);
					sendResults(results);
					break;
				case LowLevelTest:
					sanityTestLowLevel();
					break;
				case UserDefinded:
					// no processing needed, just send it on
					sendData(it->vectorOfCommandData);
					break;
				case Circle:
					for (auto& a : it->vectorOfCommandData) {
						drawCircle(results, a.float1); // populates vectorData
					}
					sendResults(results);
					break;
				case Sleep:
					for (auto& a : it->vectorOfCommandData) {
						sleep(a.int1); // populates vectorData
					}
					break;
				case Translate:
					sendData(it->vectorOfCommandData);
					break;
				case Push:
					pushMatrix();
					break;
				case Pop:
					popMatrix();
					break;
				}
				
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
		return;// do not bother when debugging

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

	void ofRobot::setup() {
		commands = make_shared<ofRobotCommands>(this);
		memset(pose, 0, sizeof(pose));
		
		ofRobotTrace() << "setup robot " << name << " type ";

		// setup the robot
		switch (getTypeID()) {
		case PhantomXReactorArm:
			servorCount = REACTOR_SERVO_COUNT;
			ofRobotTrace() << "Reactor";
			break;
		case PhantomXPincherArm:
			servorCount = PINCHER_SERVO_COUNT;
			ofRobotTrace() << "Pincher";
			break;
		case WidowX:
			servorCount = WIDOWX_SERVO_COUNT;
			ofRobotTrace() << "WidowX";
			break;
		}

		ofRobotTrace() << " on " << serial.deviceName << std::endl;

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
		if (robots.size() == 0) {
			ofRobotTrace() << "update robots no data" << std::endl;
			ofSleepMillis(500);
		}
		else {
			ofRobotTrace() << "update robots" << std::endl;
		}
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
			shared_ptr<ofRobot> robot = make_shared<ofRobot>();
			if (!robot) {
				return; // something is really wrong
			}
			if (!robot->serial.setup(device.getDeviceName(), 38400)) {
				ofRobotTrace(FatalErrorLog) << "FindAllRobots" << std::endl;
				return;
			}
			robot->serial.deviceName = device.getDeviceName();
			// port found, see what may be on it
			// start with default mode
			uint64_t t1 = ofGetElapsedTimeMillis();
			robotType robotType;
			string robotName;
			if (!robotTypeIsError(robotType = robot->serial.waitForRobot(robotName, 25))) {
				uint64_t t2 = ofGetElapsedTimeMillis();
				int gone = t2 - t1;
				robot->setName(robotName);
				robot->setType(robotType);
				robot->setup();
				robots.push_back(robot);
				ofRobotTrace() << "install duration in milliseconds " << gone << " for " << robotName << std::endl;
			}
			else {
				// robot object will delete itself if no robot is found
				ofRobotTrace() << "no robot at " << device.getDeviceName() << std::endl;
			}
		}

	}

}
