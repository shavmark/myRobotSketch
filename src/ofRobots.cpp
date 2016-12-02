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
			setGripper(getMin(ArmGripper) + (statePercent.getGripper() * (getMax(ArmGripper) - getMin(ArmGripper))));
		}
	}

	//+/- 0 to 1.000
	void ofTrRobotArm::setPoint(ofRobotPosition ptPercent) {
		// only Cylindrical supported by this function, mainly the setx one
		if (info.isCylindrical()) {
			//ofMap
			if (valueIsSet(ptPercent.getX())) {
				setX(getMin(ArmX) + (ptPercent.getX() * (getMax(ArmX) - getMin(ArmX))));
			}
			if (valueIsSet(ptPercent.getY())) {
				setY(getMin(ArmY) + (ptPercent.getY() * (getMax(ArmY) - getMin(ArmY))));
			}
			if (valueIsSet(ptPercent.getZ())) {
				setZ(getMin(ArmZ) + (ptPercent.getZ() * (getMax(ArmZ) - getMin(ArmZ))));
			}
		}
		else {
			ofRobotTrace(ErrorLog) << "setPoint not supported for non Cylindrical" << std::endl;
		}
	}

	ofRobotArmCommand::ofRobotArmCommand(robotArmJointType type, float value, uint8_t delta) {
		//enum robotArmJointType { ArmX, ArmY, ArmZ, wristAngle, wristRotate, ArmGripper, JointNotDefined };
		set(UserDefinded);
		addParameter(type, value, delta);
	}

	void ofRobotArmCommand::addParameter(robotArmJointType type, float value, uint8_t delta) {
		switch (type) {
		case ArmX:
			addParameter(value, NoRobotValue, NoRobotValue, NoRobotValue, NoRobotValue, NoRobotValue, delta);
			return;
		case ArmY:
			addParameter(NoRobotValue, value, NoRobotValue, NoRobotValue, NoRobotValue, NoRobotValue, delta);
			return;
		case ArmZ:
			addParameter(NoRobotValue, NoRobotValue, value, NoRobotValue, NoRobotValue, NoRobotValue, delta);
			return;
		case wristAngle:
			addParameter(NoRobotValue, NoRobotValue, NoRobotValue, value, NoRobotValue, NoRobotValue, delta);
			return;
		case wristRotate:
			addParameter(NoRobotValue, NoRobotValue, NoRobotValue, NoRobotValue, value, NoRobotValue, delta);
			return;
		case ArmGripper:
			addParameter(NoRobotValue, NoRobotValue, NoRobotValue, NoRobotValue, NoRobotValue, value, delta);
			return;
		}
	}
	void ofRobotArmCommand::setup(const RobotCommand&cmd, const RobotArmCommandData& data) {
		ClearVector(vectorOfCommandData);
		set(cmd); 
		addParameter(data); 
	}

	// various tests
	void ofTrRobotArm::regressionTest(vector<ofRobotArmCommand>&commands) {
		ofRobotTrace() << "high level sanityTest" << std::endl;

		ofRobotArmCommand sleep(Sleep, 5000);
		sleep.addParameter(2000); // example of 2nd parameter, would add 2000 seconds of sleep as an example

		commands.push_back(ofRobotArmCommand(0.0f));
		commands.push_back(sleep);
		commands.push_back(ofRobotArmCommand(1.0f));
		commands.push_back(sleep);
		commands.push_back(ofRobotArmCommand(0.5f, 0.2f));
		commands.push_back(ofRobotArmCommand(ArmY, 1.0f, 25)); // fast
		commands.push_back(ofRobotArmCommand(ArmY, 0.5f));
		commands.push_back(ofRobotArmCommand(ArmZ, 0.2f));
		commands.push_back(ofRobotArmCommand(ArmZ, 0.8f));
		return;
		/* add a new command, either way works
		cmd.addWristAngle(0.0f);
		cmd.addWristAngle(1.0f);
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

	// set basic data that moves a little bit after starting up. does low level writes only. Does not call reset() or any high level function
	void ofTrRobotArm::sanityTestLowLevel() {
		ofRobotTrace() << "low level sanityTest" << std::endl;
		sendToRobot(this);
		setDelta(254);
		setX(100); // absolution position vs. percentages
		setY(100);
		//setZ(50);
		setWristAngle(30);
		//setWristRotate(120);
		setGripper(10);
		setButton();
		sendToRobot(this);
		setGripper(100);
		sendToRobot(this);
		setGripper(511);
		sendToRobot(this);
		ofSleepMillis(1000);
		setX(200);
		sendToRobot(this);
		setX(0);
		sendToRobot(this);
	}

	void ofTrRobotArm::popMatrix() {
		if (stack.size() == 0) {
			ofRobotTrace(ErrorLog) << "popMatrix on empty stack" << std::endl;
		}
		else {
			//bugbug figure out stack or drop it setPose(stack.top());
			stack.pop();
		}
	}
	void ofTrRobotArm::setup(robotMode mode) { 
		ofRobotTrace() << "setup ofTrRobotArm " << name << " type " << info.trace();
		
		setStartState(mode);
		setUserDefinedRanges(SpecificJoint(info.getType(), ArmX), userDefinedRanges);
		sendToRobot(this); // send the mode, also resets the robot
		setDefaultState();

		ClearVector(vectorOfCommands);
	
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

		ofRobotTrace() << " on " << getDriver()->deviceName << std::endl;

		validate(); // validate robot

	}

	//ofPushMatrix();
	//ofTranslate(400, 300);
	//ofPopMatrix();

	void ofTrRobotArm::penUpMacro(vector<ofRobotArmCommand>&commands, uint8_t delta) {
		ofRobotArmCommand cmd(UserDefinded, delta);
		
		cmd.addParameter(ArmZ, getZ()+0.1, delta); // bugbug what if its max,I guess it just ignores the request
		commands.push_back(cmd);
	}

	void ofTrRobotArm::penDownMacro(vector<ofRobotArmCommand>&commands, uint8_t delta) {
		ofRobotArmCommand cmd(UserDefinded, delta);
		cmd.addParameter(ArmZ, getZ() - 0.1, delta);
		commands.push_back(cmd);
	}

	// draw optimized line from current location
	void ofTrRobotArm::lineMacro(vector<ofRobotArmCommand>&commands, const ofRobotPosition& to, uint8_t delta)	{
		// drop pen and move
		penDownMacro(commands, delta);
		moveMacro(commands, to, delta);
		penUpMacro(commands, delta);
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
	void ofTrRobotArm::moveMacro(vector<ofRobotArmCommand>&commands,  const ofRobotPosition& to, uint8_t delta) {
		// move arm
		commands.push_back(ofRobotArmCommand(RobotMoveTo, RobotArmCommandData(to, delta)));
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
	void ofTrRobotArm::circleMacro(vector<ofRobotArmCommand>&commands, float r, uint8_t delta)
	{
		// do a move like OF does so drawing always starts at current
		float slice = 2 * M_PI / 10;
		for (int i = 0; i < 10; i++) {
			float angle = slice * i;
			float newX = r * cos(angle);
			float newY = r * sin(angle);
			ofRobotTrace() << newX << " " << newY << std::endl; 
			ofRobotArmCommand cmd(RobotArmCommandData(ofRobotPosition(newX, newY), delta));
			commands.push_back(cmd);
		}
	}

	void ofTrRobotArm::testdata() {
		trace(); // echo object data
	}

	void ofTrRobotArm::set(RobotArmCommandData& request) {
		setPoint(request.getPoint());
		setState(request.getState());
		setDelta(request.getDelta());
	}

	void ofTrRobotArm::sendData(vector<RobotArmCommandData>&data) {
		for (auto& a : data) {
			set(a);
			sendToRobot(this);

		}
	}

	// send and delete (if requested) commands created as a result of using built in functions such as drawCircle
	void ofTrRobotArm::sendExpandedResults(vector<ofRobotArmCommand>& results) {
		for (auto& result : results) {
			if (result.getCommand() == Sleep) {
				for (auto& a : result.vectorOfCommandData) {
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
				regressionTest(expandedResults);
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
				setLED(static_cast<TrossenServoIDs>(i), 1);
				int led = getLED(static_cast<TrossenServoIDs>(i));
				if (led != 1) {
					ofRobotTrace(WarningLog) << "reg set fails" << std::endl; // may occur during startup
				}
				ofSleepMillis(100);
				setLED(static_cast<TrossenServoIDs>(i), 0);
				led = getLED(static_cast<TrossenServoIDs>(i));
				if (led != 0) {
					ofRobotTrace(WarningLog) << "reg set fails" << std::endl;
				}
			}
			for (int i = FIRST_SERVO; i < servoCount; ++i) {
				ofRobotTrace() << "servo " << i;

				int pos = getPosition(static_cast<TrossenServoIDs>(i));
				ofRobotTrace() << " pos " << pos;

				int tmp = getTempature(static_cast<TrossenServoIDs>(i));
				ofRobotTrace() << " temp. " << tmp;

				float v = getVoltage(static_cast<TrossenServoIDs>(i))/10;
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

		for (auto bot : makerbots) {
			bot->draw();
		}
		
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
			ofRobotTrace("FindAllMyElements") << "[" << device.getDeviceID() << "] = " << device.getDeviceName().c_str();
			int baudrate = 19200;
			ofRobotTrace("FindAllMyElements") << "baud rate " << baudrate << device.getDeviceName().c_str();

			shared_ptr<ofRobotSerial> serialdriver = make_shared<ofRobotSerial>();
			if (!serialdriver->setup(device.getDeviceName(), baudrate)) { // MAKE SURE BAUD RATES ARE THE SAME
				ofRobotTrace(FatalErrorLog) << device.getDeviceName() << std::endl;
				return;
			}

			shared_ptr<xyRobot> maker = make_shared<xyRobot>(serialdriver); // move the driver over
																				// port found, see what may be on it
			// start with default mode
			shared_ptr<ofTrRobotArm> arm = make_shared<ofTrRobotArm>();
			if (!arm) {
				return; // something is really wrong
			}
			robotType robotType;
			string robotName;
			maker->add(xyDataToSend(IDstepperX, SignOn)); // some drivers require a sign on
			maker->draw();

			if (!robotTypeIsError(robotType = serialdriver->waitForRobot(robotName, 25, 5))) {
			
				switch (robotType.second) {
				case PhantomXReactorArm:
				case PhantomXPincherArm:
				case WidowX:
					arm->setSerial(serialdriver);
					arm->setName(robotName);
					arm->setType(robotType);
					arm->setup(IKM_CYLINDRICAL);
					arms.push_back(arm);
					break;
				case MakerBotXY:
					maker->setName(robotName);
					maker->setType(robotType);
					maker->add(xyDataToSend(IDstepperX, GetState));  
					maker->add(xyDataToSend(IDstepperY, GetState));  
					maker->draw();
					makerbots.push_back(maker);
					break;
				}
			}
		}
	}
}
