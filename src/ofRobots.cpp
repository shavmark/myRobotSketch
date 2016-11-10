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

	void ofRobotArmState::trace() {
		ofRobotTrace() << "WristAngle=" << (valueIsSet(getWristAngle()) ? ofToString(getWristAngle()) : "<not set>") << std::endl;
		ofRobotTrace() << "WristRotatation=" << (valueIsSet(getWristRotation()) ? ofToString(getWristRotation()) : "<not set>") << std::endl;
		ofRobotTrace() << "Gripper=" << (valueIsSet(getGripper()) ? ofToString(getGripper()) : "<not set>") << std::endl;
	}
	
	void ofRobotPosition::trace()  {
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

	void ofTrRobotArm::trace()  {
		for ( auto& cmd : vectorOfCommands) {
			cmd.trace();
		}
	}

	// x - wrist angle, y is wrist rotate, z is gripper (using ofVec3f so its features can be leverage)
	void ofTrRobotArm::setState(ofRobotArmState statePercent) {
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
	void ofTrRobotArm::setPoint(ofRobotPosition ptPercent) {
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
	void ofTrRobotArm::sanityTestHighLevel(vector<ofRobotArmCommand>&commands) {
		ofRobotTrace() << "high level sanityTest" << std::endl;

		//ofRobotCommand cmd(0.3f, 0.6f, NoRobotValue, 1.0f, -1.0f, 0.5f);
		ofRobotArmCommand cmd(0.0f);
		//cmd.addWristAngle(0.0f);
		//cmd.addWristAngle(1.0f);
		commands.push_back(cmd); // saves a copy
		ofRobotArmCommand cmd2(1.0f);
		commands.push_back(cmd2);
		return;
		/* add a new command, either way works
		commands.push_back(cmd);
		commands.push_back(ofRobotArmCommand::getSleep(1000));
		ofRobotArmCommand cmd3(NoRobotValue, 0.10f);
		cmd3.add(NoRobotValue, 1.0f);
		commands.push_back(cmd3);
		commands.push_back(ofRobotArmCommand::getSleep(1000)); 
		ofRobotArmCommand cmd3(NoRobotValue, NoRobotValue, 0.0f);
		cmd3.add(NoRobotValue, NoRobotValue, 1.0f);
		commands.push_back(cmd3);
		*/
	}

	void ofTrRobotArmInternals::sendToRobot(ofRobotSerial* serial) {
		
		if (serial) {
			serial->writePose(&pose);
		}
	}

	// set basic data that moves a little bit after starting up. does low level writes only. Does not call reset() or any high level function
	void ofTrRobotArm::sanityTestLowLevel() {
		ofRobotTrace() << "low level sanityTest" << std::endl;
		sendToRobot(&serial);
		pose.setDelta(254);
		setX(100); // absolution position vs. percentages
		setY(100);
		//setZ(50);
		setWristAngle(30);
		//setWristRotate(120);
		setGripper(10);
		pose.setButton();
		sendToRobot(&serial);
		setGripper(100);
		sendToRobot(&serial);
		setGripper(511);
		sendToRobot(&serial);
		ofSleepMillis(1000);
		setX(200);
		sendToRobot(&serial);
		setX(0);
		sendToRobot(&serial);
	}

	void ofTrRobotArm::popMatrix() {
		if (stack.size() == 0) {
			ofRobotTrace(ErrorLog) << "popMatrix on empty stack" << std::endl;
		}
		else {
			setPose(stack.top());
			stack.pop();
		}
	}
	void ofTrRobotArm::setup(robotArmMode mode) { 
		ofRobotTrace() << "setup ofTrRobotArm " << name << " type " << info.trace();
		
		setStartState(mode);
		setUserDefinedRanges(SpecificJoint(info.getType(), X), userDefinedRanges);
		sendToRobot(&serial); // send the mode, also resets the robot
		setDefaultState();

		clear(vectorOfCommands);
	
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

	//ofPushMatrix();
	//ofTranslate(400, 300);
	//ofPopMatrix();

	void ofTrRobotArm::penUpMacro(vector<ofRobotArmCommand>&commands) {
		ofRobotArmCommand cmd(UserDefinded);
		
		cmd.addZ(pose.getZ()+0.1); // bugbug what if its max,I guess it just ignores the request
		commands.push_back(cmd);
	}

	void ofTrRobotArm::penDownMacro(vector<ofRobotArmCommand>&commands) {
		ofRobotArmCommand cmd(UserDefinded);
		cmd.addZ(pose.getZ() - 0.1);
		commands.push_back(cmd);
	}

	// draw optimized line from current location
	void ofTrRobotArm::lineMacro(vector<ofRobotArmCommand>&commands, const ofRobotPosition& to)	{
		// drop pen and move
		penDownMacro(commands);
		moveMacro(commands, to);
		penUpMacro(commands);
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
	void ofTrRobotArm::moveMacro(vector<ofRobotArmCommand>&commands,  const ofRobotPosition& to) {
		// move arm
		ofRobotArmCommand cmd(RobotMoveTo, RobotArmCommandData(to));
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
	void ofTrRobotArm::circleMacro(vector<ofRobotArmCommand>&commands, float r)
	{
		// do a move like OF does so drawing always starts at current
		float slice = 2 * M_PI / 10;
		for (int i = 0; i < 10; i++) {
			float angle = slice * i;
			float newX = r * cos(angle);
			float newY = r * sin(angle);
			ofRobotTrace() << newX << " " << newY << std::endl; 
			ofRobotArmCommand cmd(RobotArmCommandData(ofRobotPosition(newX, newY)));
			commands.push_back(cmd);
		}
	}

	void ofTrRobotArm::testdata() {
		pose.trace();
		trace(); // echo object data
	}

	void ofTrRobotArm::set(RobotArmCommandData& request) {
		setPoint(request.getPoint());
		setState(request.getState());
		pose.setDelta(request.getDelta());
	}

	void ofTrRobotArm::sendData(vector<RobotArmCommandData>&data) {
		for (auto& a : data) {
			set(a);
			sendToRobot(&serial);
		}
	}

	// send and delete (if requested) commands created as a result of using built in functions such as drawCircle
	void ofTrRobotArm::sendExpandedResults(vector<ofRobotArmCommand>& results) {
		for (auto& result : results) {
			if (result.getCommand() == Sleep) {
				for (auto& a : result.getVectorOfParameters()) {
					ofSleepMillis(a.int1);
				}
			}
			else {
				sendData(result.getVectorOfParameters());
			}
		}
	}

	void ofTrRobotArm::update() {
		
	}

	// drawing occurs here as its tied to the robot directly
	void ofTrRobotArm::draw() {

		vector< ofRobotArmCommand >::iterator it = vectorOfCommands.begin();
		while (it != vectorOfCommands.end()) {
			vector<ofRobotArmCommand> expandedResults;

			switch (it->getCommand()) {
			case RegressionTest:
				sanityTestHighLevel(expandedResults);
				break;
			case RobotCircle:
				for (auto& a : it->getVectorOfParameters()) {
					circleMacro(expandedResults, a.float1);
				}
				break;
			case RobotLineTo:
				for (auto& a : it->getVectorOfParameters()) {
					lineMacro(expandedResults, a.position);
				}
				break;
			case RobotMoveTo:
				for (auto& a : it->getVectorOfParameters()) {
					moveMacro(expandedResults, a.position);
				}
				break;
			case LowLevelTest:
				sanityTestLowLevel(); // special case, does not use data or other objects, used to test/debug
				break;
			case UserDefinded:
				// no processing needed, just send it on. Low level, no commands parsed such as sleep, assuming that is done else where
				sendData(it->getVectorOfParameters());
				break;
			case Translate:
				sendData(it->getVectorOfParameters());
				break;
			case Push:
				pushMatrix();
				break;
			case Pop:
				popMatrix();
				break;
			}

			sendExpandedResults(expandedResults);

			if (it->OKToDelete()) {
				it = vectorOfCommands.erase(it);
			}
			else {
				++it;
			}
		}
	}

	void ofRobotArmCommand::trace()  {
		for ( auto& a : vectorOfCommandData) {
			a.getPoint().trace();
			a.getState().trace();
		}
	}

	void ofTrRobotArm::validate() {
		return;// do not bother when debugging

		ofRobotTrace() << "valiate ofTrRobotArm" << std::endl;
		
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

	void ofRobot::trace() {
		ofRobotTrace() << "trace robot " << name << std::endl;
		for (auto arm : arms) {
			arm->trace();
		}
	}

	void ofRobot::draw() {
		ofRobotTrace() << "draw robot" << name << std::endl;
		for (auto arm : arms) {
			arm->draw();
		}
	}
	void ofRobot::update() {
		ofRobotTrace() << "update robot" << name << std::endl;
		for (auto arm : arms) {
			arm->update();
		}
	}

	void ofRobot::setup() {
		
		ofRobotTrace() << "setup robot" << name << std::endl;
		
		// do one time setup
		ofTrRobotArmInternals::oneTimeSetup(); // do one time setup of static data

		ofRobotSerial serial;
		serial.listDevices(); // let viewer see thats out there

		for (auto&device : serial.getDevices()) {
			if (device.getDeviceID() == 0) {
				continue;//bugbug skipping com1, not sure whats on it
			}
			ofRobotTrace("FindAllArms") << "[" << device.getDeviceID() << "] = " << device.getDeviceName().c_str();
			shared_ptr<ofTrRobotArm> arm = make_shared<ofTrRobotArm>();
			if (!arm) {
				return; // something is really wrong
			}
			if (!arm->serial.setup(device.getDeviceName(), 38400)) {
				ofRobotTrace(FatalErrorLog) << "FindAllArms" << std::endl;
				return;
			}
			arm->serial.deviceName = device.getDeviceName();
			// port found, see what may be on it
			// start with default mode
			uint64_t t1 = ofGetElapsedTimeMillis();
			robotType robotType;
			string robotName;
			if (!robotTypeIsError(robotType = arm->serial.waitForRobotArm(robotName, 25))) {
				uint64_t t2 = ofGetElapsedTimeMillis();
				int gone = t2 - t1;
				arm->setName(robotName);
				arm->setType(robotType);
				arm->setup(IKM_CYLINDRICAL);
				arms.push_back(arm);
				ofRobotTrace() << "install duration in milliseconds " << gone << " for " << robotName << std::endl;
			}
			else {
				// robot object will delete itself if no robot is found
				ofRobotTrace() << "no robot at " << device.getDeviceName() << std::endl;
			}
		}
	}
}
