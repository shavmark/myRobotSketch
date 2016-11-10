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
		ofRobotTrace() << "WristAngle=" << (valueIsSet(getWristAngle()) ? ofToString(getWristAngle()) : "<not set>") << std::endl;
		ofRobotTrace() << "WristRotatation=" << (valueIsSet(getWristRotation()) ? ofToString(getWristRotation()) : "<not set>") << std::endl;
		ofRobotTrace() << "Gripper=" << (valueIsSet(getGripper()) ? ofToString(getGripper()) : "<not set>") << std::endl;
	}
	
	void ofRobotPosition::echo()  {
		ofRobotTrace() << "x=" << (valueIsSet(x) ? ofToString(x) : "<not set>") << std::endl;
		ofRobotTrace() << "y=" << (valueIsSet(y) ? ofToString(y) : "<not set>") << std::endl;
		ofRobotTrace() << "z=" << (valueIsSet(z) ? ofToString(z) : "<not set>") << std::endl;
	}

	// can be +//
	bool ofRobotPosition::validRange(float f) {
		if (abs(f) >= 0.0f && abs(f) <= 1.0f) {
			return true;
		}
		return false;
	}

	void ofRobotPosition::setPercents(float xPercent, float yPercent, float zPercent) {
		if (xPercent == NoRobotValue || validRange(xPercent)) {
			x = xPercent;
		}
		if (yPercent == NoRobotValue || validRange(yPercent)) {
			y = yPercent;
		}
		if (zPercent == NoRobotValue || validRange(zPercent)) {
			z = zPercent;
		}
	}

	void ofRobotCommands::echo()  {
		for ( auto& cmd : vectorOfRobotCommands) {
			cmd.echo();
		}
	}

	// x - wrist angle, y is wrist rotate, z is gripper (using ofVec3f so its features can be leverage)
	void ofRobotCommands::setState(ofRobotState statePercent) {
		if (valueIsSet(statePercent.getWristAngle())) {
			setWristAngle(getMin(wristAngle) + (statePercent.getWristAngle() * (getMax(wristAngle) - getMin(wristAngle))));
		}
		if (valueIsSet(statePercent.getWristRotation())) {
			setWristRotate(getMin(wristRotate) + (statePercent.getWristRotation() * (getMax(wristRotate) - getMin(wristRotate))));
		}
		if (valueIsSet(statePercent.getGripper())) {
			setGripper(getMin(Gripper) + (statePercent.getGripper() * (getMax(Gripper) - getMin(Gripper))));
		}
	}

	//+/- 0 to 1.000
	void ofRobotCommands::setPoint(ofRobotPosition ptPercent) {
		// only Cylindrical supported by this function, mainly the setx one
		if (info.isCylindrical()) {
			//ofMap
			if (valueIsSet(ptPercent.getX())) {
				setX(getMin(X) + (ptPercent.getX() * (getMax(X) - getMin(X))));
			}
			if (valueIsSet(ptPercent.getY())) {
				setY(getMin(Y) + (ptPercent.getY() * (getMax(Y) - getMin(Y))));
			}
			if (valueIsSet(ptPercent.getZ())) {
				setZ(getMin(Z) + (ptPercent.getZ() * (getMax(Z) - getMin(Z))));
			}
		}
		else {
			ofRobotTrace(ErrorLog) << "setPoint not supported for non Cylindrical" << std::endl;
		}
	}

	// various tests
	void ofRobotCommands::sanityTestHighLevel(vector<ofRobotCommand>&commands) {
		ofRobotTrace() << "high level sanityTest" << std::endl;

		//ofRobotCommand cmd(0.3f, 0.6f, NoRobotValue, 1.0f, -1.0f, 0.5f);
		ofRobotCommand cmd(0.0f);
		cmd.addWristAngle(0.0f);
		cmd.addWristAngle(1.0f);
		commands.push_back(cmd); // saves a copy
		cmd.reset();
		cmd.add(1.0f);
		// add a new command, either way works
		commands.push_back(cmd);
		commands.push_back(ofRobotCommand::getSleep(1000));
		ofRobotCommand cmd2(NoRobotValue, 0.10f);
		cmd2.add(NoRobotValue, 1.0f);
		commands.push_back(cmd2);
		commands.push_back(ofRobotCommand::getSleep(1000)); 
		ofRobotCommand cmd3(NoRobotValue, NoRobotValue, 0.0f);
		cmd3.add(NoRobotValue, NoRobotValue, 1.0f);
		commands.push_back(cmd3);
	}

	void ofRobotJoints::sendToRobot(ofRobotSerial* serial) {
		
		if (serial) {
			serial->writePose(&pose);
		}
	}

	// set basic data that moves a little bit after starting up. does low level writes only. Does not call reset() or any high level function
	void ofRobotCommands::sanityTestLowLevel() {
		ofRobotTrace() << "low level sanityTest" << std::endl;
		sendToRobot(&robot->serial);
		pose.setDelta(254);
		setX(100); // absolution position vs. percentages
		setY(100);
		//setZ(50);
		setWristAngle(30);
		//setWristRotate(120);
		setGripper(10);
		pose.setButton();
		sendToRobot(&robot->serial);
		setGripper(100);
		sendToRobot(&robot->serial);
		setGripper(511);
		sendToRobot(&robot->serial);
		ofSleepMillis(1000);
		setX(200);
		sendToRobot(&robot->serial);
		setX(0);
		sendToRobot(&robot->serial);
	}


	void ofRobotCommands::setup(ofRobot *robot, robotArmMode mode) { 
		this->robot = robot;
		setStartState(mode);
		if (robot) {
			info.setType(robot->info.getType());
			setUserDefinedRanges(SpecificJoint(robot->info.getType(), X), robot->userDefinedRanges);
			sendToRobot(&robot->serial); // send the mode, also resets the robot
		}
		setDefaultState();
		clear(vectorOfRobotCommands);
	}

	//ofPushMatrix();
	//ofTranslate(400, 300);
	//ofPopMatrix();

	void ofRobotCommands::penUp(vector<ofRobotCommand>&commands) {
		ofRobotCommand cmd(UserDefinded);
		
		cmd.addZ(pose.getZ()+0.1); // bugbug what if its max,I guess it just ignores the request
		commands.push_back(cmd);
	}

	void ofRobotCommands::penDown(vector<ofRobotCommand>&commands) {
		ofRobotCommand cmd(UserDefinded);
		cmd.addZ(pose.getZ() - 0.1);
		commands.push_back(cmd);
	}

	// draw optimized line from current location
	void ofRobotCommands::line(vector<ofRobotCommand>&commands, const ofRobotPosition& to)	{
		// drop pen and move
		penDown(commands);
		move(commands, to);
		penUp(commands);
		/*
		float dx = to.x - currentPosition.x; // delta between x and y
		float dy = to.x - currentPosition.y;
		float c = sqrt(dx*dx + dy*dy); // a squared etc

		dx /= c; // normalize
		dy /= c;
		float distance = 2; // min distance to move
		int newx = (int)(currentPosition.x + dx * (c + distance));
		int newy = (int)(currentPosition.y + dy * (c + distance));
		*/
	}

	// move arm
	void ofRobotCommands::move(vector<ofRobotCommand>&commands,  const ofRobotPosition& to) {
		// move arm
		ofRobotCommand cmd(RobotMoveTo, RobotCommandData(to));
		commands.push_back(cmd);
	}
	
	ofRobotPosition& ofRobotPosition::operator=(const ofRobotPosition&newpos) {
		if (valueIsSet(newpos.x)) {
			x = newpos.x;
		}
		if (valueIsSet(newpos.y)) {
			y = newpos.y;
		}
		if (valueIsSet(newpos.z)) {
			z = newpos.z;
		}
		return *this;
	}

	// create circle data
	void ofRobotCommands::circle(vector<ofRobotCommand>&commands, float r)
	{
		// do a move like OF does so drawing always starts at current
		float slice = 2 * M_PI / 10;
		for (int i = 0; i < 10; i++) {
			float angle = slice * i;
			float newX = r * cos(angle);
			float newY = r * sin(angle);
			ofRobotTrace() << newX << " " << newY << std::endl; 
			ofRobotCommand cmd(RobotCommandData(ofRobotPosition(newX, newY)));
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
		pose.trace();
		echo(); // echo object data
	}

	void ofRobotCommands::set(RobotCommandData& request) {
		setPoint(request.getPoint());
		setState(request.getState());
		pose.setDelta(request.getDelta());
	}

	void ofRobotCommands::sendData(vector<RobotCommandData>&data) {
		for (auto& a : data) {
			set(a);
			sendToRobot(&robot->serial);
		}
	}

	// send and delete (if requested) commands created as a result of using built in functions such as drawCircle
	void ofRobotCommands::sendExpandedResults(vector<ofRobotCommand>& results) {
		for (auto& result : results) {
			if (result.getCommand() == Sleep) {
				for (auto& a : result.getVector()) {
					ofSleepMillis(a.int1);
				}
			}
			else {
				sendData(result.getVector());
			}
		}
	}

	void ofRobotCommands::update() {
		
		clear(expandedResults);

		// expand data as needed by all commands
		for (auto& cmd : vectorOfRobotCommands) {
			switch (cmd.getCommand()) {
			case HighLevelTest:
				sanityTestHighLevel(expandedResults);
				break;
			case RobotCircle:
				for (auto& a : cmd.getVector()) {
					circle(expandedResults, a.float1); 
				}
				break;
			case RobotLineTo:
				for (auto& a : cmd.getVector()) {
					line(expandedResults, a.position); 
				}
				break;
			case RobotMoveTo:
				for (auto& a : cmd.getVector()) {
					move(expandedResults, a.position);
				}
				break;
			}
		}
	}

	// drawing occurs here as its tied to the robot directly
	void ofRobotCommands::draw() {

		if (robot) {
			
			vector< ofRobotCommand >::iterator it = vectorOfRobotCommands.begin();
			while (it != vectorOfRobotCommands.end()) {

				switch (it->getCommand()) {
				case LowLevelTest:
					sanityTestLowLevel(); // special case, does not use data or other objects, used to test/debug
					break;
				case UserDefinded:
					// no processing needed, just send it on. Low level, no commands parsed such as sleep, assuming that is done else where
					sendData(it->getVector());
					break;
				case Translate://bugbug make a mov, ie that lifts the brush for example
					sendData(it->getVector());
					break;
				case Push:
					pushMatrix();
					break;
				case Pop:
					popMatrix();
					break;
				}

				if (expandedResults.size() > 0) {
					sendExpandedResults(expandedResults);
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
		
		for (int i = FIRST_SERVO; i < servoCount; ++i) {
			for (int j = 1; j <= 5; ++j) { // flash a bit
				serial.setLED(static_cast<TrossenServoIDs>(i), 1);
				int led = serial.getLED(static_cast<TrossenServoIDs>(i));
				if (led != 1) {
					ofRobotTrace(WarningLog) << "reg set fails" << std::endl; // may occur during startup
				}
				ofSleepMillis(100);
				serial.setLED(static_cast<TrossenServoIDs>(i), 0);
				led = serial.getLED(static_cast<TrossenServoIDs>(i));
				if (led != 0) {
					ofRobotTrace(WarningLog) << "reg set fails" << std::endl;
				}
			}
			for (int i = FIRST_SERVO; i < servoCount; ++i) {
				ofRobotTrace() << "servo " << i;

				int pos = serial.getPosition(static_cast<TrossenServoIDs>(i));
				ofRobotTrace() << " pos " << pos;

				int tmp = serial.getTempature(static_cast<TrossenServoIDs>(i));
				ofRobotTrace() << " temp. " << tmp;

				float v = serial.getVoltage(static_cast<TrossenServoIDs>(i))/10;
				ofRobotTrace() << " voltage " << v << std::endl;
			}
		}
	}

	void ofRobot::setup() {

		ofRobotTrace() << "setup robot " << name << " type ";

		info.setMode(IKM_CYLINDRICAL);
		commands.setup(this, IKM_CYLINDRICAL);

		// setup the robot
		switch (info.getTypeID()) {
		case PhantomXReactorArm:
			servoCount = REACTOR_SERVO_COUNT;
			ofRobotTrace() << "Reactor";
			break;
		case PhantomXPincherArm:
			servoCount = PINCHER_SERVO_COUNT;
			ofRobotTrace() << "Pincher";
			break;
		case WidowX:
			servoCount = WIDOWX_SERVO_COUNT;
			ofRobotTrace() << "WidowX";
			break;
		}

		ofRobotTrace() << " on " << serial.deviceName << std::endl;

		validate(); // validate robot

	}

	void ofRobot::update() {
		commands.update();
	}

	void ofRobot::echo() {
		commands.echo();
	}

	void ofRobot::draw() {
		if (pause) {
			return;
		}
		commands.draw();
	}

	shared_ptr<ofRobot> ofRobotFamly::getRobot(int index, RobotTypeID id) {
		ofRobotTrace() << "getRobot" << std::endl;
		int count = 0;
		for (int i = 0; i < robots.size(); ++i) {
			if (id == AllRobotTypes || id == robots[i]->info.getTypeID()) {
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
			if (idToUse == AllRobotTypes || idToUse == robot->info.getTypeID()) {
				robot->update();
			}
		}
	}

	void ofRobotFamly::draw() {
		ofRobotTrace() << "draw robots" << std::endl;
		for (const auto robot : robots) {
			if (idToUse == AllRobotTypes || idToUse == robot->info.getTypeID()) {
				robot->draw();
			}
		}
	}

	void ofRobotFamly::setup() {

		// do one time setup
		ofRobotJoints::oneTimeSetup(); // do one time setup of static data

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
				robot->info.setType(robotType);
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
