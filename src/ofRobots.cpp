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

	void ofRobotPosition::trace() {
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

	void ofTrRobotArm::trace() {
		for (auto& cmd : vectorOfCommands) {
			cmd.trace();
		}
	}

	// x - wrist angle, y is wrist rotate, z is gripper (using ofVec3f so its features can be leverage)
	void ofTrRobotArm::setState(ofRobotArmState statePercent) {
		if (valueIsSet(statePercent.getWristAngle())) {
			setJointValue(wristAngle, getMin(wristAngle) + (statePercent.getWristAngle() * (getMax(wristAngle) - getMin(wristAngle))));
		}
		if (valueIsSet(statePercent.getWristRotation())) {
			setJointValue(wristRotate, getMin(wristRotate) + (statePercent.getWristRotation() * (getMax(wristRotate) - getMin(wristRotate))));
		}
		if (valueIsSet(statePercent.getGripper())) {
			setJointValue(ArmGripper, getMin(ArmGripper) + (statePercent.getGripper() * (getMax(ArmGripper) - getMin(ArmGripper))));
		}
	}

	//+/- 0 to 1.000
	void ofTrRobotArm::setPoint(ofRobotPosition ptPercent) {
		// only Cylindrical supported by this function, mainly the setx one
		if (info.isCylindrical()) {
			//ofMap
			if (valueIsSet(ptPercent.getX())) {
				setJointValue(ArmX, getMin(ArmX) + (ptPercent.getX() * (getMax(ArmX) - getMin(ArmX))));
			}
			if (valueIsSet(ptPercent.getY())) {
				setJointValue(ArmY, getMin(ArmY) + (ptPercent.getY() * (getMax(ArmY) - getMin(ArmY))));
			}
			if (valueIsSet(ptPercent.getZ())) {
				setJointValue(ArmZ, getMin(ArmZ) + (ptPercent.getZ() * (getMax(ArmZ) - getMin(ArmZ))));
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
	// parameters are values of 0.0 to 1.0 at this level, lower level code translates them to robot specific values
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

	void ofTrRobotArm::penPose(vector<ofRobotArmCommand>&commands, vector<RobotArmCommandData>&data)
	{
		// next: is there a way to wait for a command to complete?
		commands.push_back(ofRobotArmCommand(ArmZ, 0.0f));
		commands.push_back(ofRobotArmCommand(ArmGripper, 0.0f, 10)); // give it some animation
		commands.push_back(ofRobotArmCommand(ArmGripper, 1.0f, 10));
		commands.push_back(ofRobotArmCommand(ArmGripper, 0.1f, 20));
	}

	// various tests
	void ofTrRobotArm::regressionTest(vector<ofRobotArmCommand>&commands, vector<RobotArmCommandData>&data) {
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
	void ofTrRobotArm::sanityTestLowLevel(vector<ofRobotArmCommand>&commands, vector<RobotArmCommandData>&data) {
		ofRobotTrace() << "low level sanityTest" << std::endl;
		sendToRobot(this);
		setDelta(254);

		setJointValue(ArmX, 100); // absolution position vs. percentages
		setJointValue(ArmY, 100);
		//setZ(50);
		setJointValue(wristAngle, 30);
		//setWristRotate(120);
		setJointValue(ArmGripper, 10);
		setButton();
		sendToRobot(this);
		setJointValue(ArmGripper, 100);
		sendToRobot(this);
		setJointValue(ArmGripper, 511);
		sendToRobot(this);
		ofSleepMillis(1000);
		setJointValue(ArmX, 200);
		sendToRobot(this);
		setJointValue(ArmX, 0);
		sendToRobot(this);
	}

	void ofTrRobotArm::setup(robotMode mode) {
		ofRobotTrace() << "setup ofTrRobotArm " << getName() << " type " << info.trace();

		setStartState(mode);
		setUserDefinedRanges(SpecificJoint(info.getType(), ArmX), userDefinedRanges);
		sendToRobot(this); // send the mode, also resets the robot
		setDefaultState();

		funcMap[PenPose] = &ofTrRobotArm::penPose; //and so forth
		funcMap[RegressionTest] = &ofTrRobotArm::regressionTest;
		funcMap[LowLevelTest] = &ofTrRobotArm::sanityTestLowLevel;
		funcMap[RobotCircle] = &ofTrRobotArm::circleMacro;
		funcMap[RobotLineTo] = &ofTrRobotArm::lineMacro;
		funcMap[Translate] = &ofTrRobotArm::justSendItNowMacro;
		funcMap[RobotMoveTo] = &ofTrRobotArm::moveMacro;
		funcMap[UserDefinded] = &ofTrRobotArm::justSendItNowMacro;

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


	// draw optimized line from current location
	void ofTrRobotArm::lineMacro(vector<ofRobotArmCommand>&commands, vector<RobotArmCommandData>&data) {
		for (auto& a : data) {
			//bugbug code tghis
		}

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
	void ofTrRobotArm::moveMacro(vector<ofRobotArmCommand>&commands, vector<RobotArmCommandData>&data) {

		// move arm
		for (auto&a : data) {
			commands.push_back(ofRobotArmCommand(RobotMoveTo, a));
		}
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
	void ofTrRobotArm::circleMacro(vector<ofRobotArmCommand>&commands, vector<RobotArmCommandData>&data) {
		for (auto& a : data) {
			float slice = 2 * M_PI / 10;
			float r = a.float1;
			for (int i = 0; i < 10; i++) {
				float angle = slice * i;
				float newX = r * cos(angle);
				float newY = r * sin(angle);
				ofRobotTrace() << newX << " " << newY << std::endl;
				ofRobotArmCommand cmd(RobotArmCommandData(ofRobotPosition(newX, newY))); //bugbug figure out how to pass delta
				commands.push_back(cmd);
			}
		}
		// do a move like OF does so drawing always starts at current
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
			if (sendToRobot(this)) {
				readResults();
			}
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
		Pose::update();
	}
	//parameters compatable with function call table
	void ofTrRobotArm::justSendItNowMacro(vector<ofRobotArmCommand>&commands, vector<RobotArmCommandData>&data) {
		sendData(data);
	}

	//pArmfunction
	// drawing occurs here as its tied to the robot directly
	void ofTrRobotArm::draw() {

		auto  it = vectorOfCommands.begin();
		while (it != vectorOfCommands.end()) {
			// execute if from table, otherwise see if its a special case
			auto itr = funcMap.find(it->getCommand());
			if (itr != funcMap.end()) {
				vector<ofRobotArmCommand> expandedResults;
				//C++ is so awesome...
				(this->*funcMap[it->getCommand()])(expandedResults, it->getVectorOfParameters());
				sendExpandedResults(expandedResults);
			}


			if (it->OKToDelete()) {
				it = vectorOfCommands.erase(it);
			}
			else {
				++it;
			}
		}
	}

	void ofRobotArmCommand::trace() {
		for (auto& a : vectorOfCommandData) {
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

				float v = getVoltage(static_cast<TrossenServoIDs>(i)) / 10;
				ofRobotTrace() << " voltage " << v << std::endl;
			}
		}
	}

	void ofMasterRobot::trace() const {
		ofRobotTrace() << "trace robot " << getName() << std::endl;
		for (const auto& bot : robotHelpers) {
			bot->trace();
		}
	}

	void ofMasterRobot::draw() {
		for (auto& bot : robotHelpers) {
			bot->draw();
		}
	}
	void ofMasterRobot::update() {
		for (auto& bot : robotHelpers) {
			bot->update();
		}
	}

	void xyRobot::setup() {
	}

	void xyRobot::update(xyDataToSend& data) {
		sendit(data);
	}
	shared_ptr<ofTrRobotArm>ofMasterRobot::getTrossen(int index) {
		if (index < robotHelpers.size()) {
			return std::dynamic_pointer_cast<ofTrRobotArm>(robotHelpers[index]);
		}
		return nullptr;
	}
	shared_ptr<xyRobot>ofMasterRobot::getXyRobot(int index) {
		if (index < robotHelpers.size()) {
			return std::dynamic_pointer_cast<xyRobot>(robotHelpers[index]);
		}
		return nullptr;
	}

	// external setup, tied to command line
	void ofMasterRobot::setup(const vector<string>& arguments) {
		// example -trossen 1
		for (int i = 1; i < arguments.size(); ++i) {
			int port = -1; // unknown
			int baudrate = 38400;
			RobotBrand id = RobotArtists::Trossen;
			if (arguments.at(i) != "-trossen") {
				baudrate = 19200;
				id = RobotArtists::MakeBlock;
			}
			++i;
			if (i < arguments.size()) {
				// something is out there, see if its a parameter or parameter data (ie -troseen 1 or -trossen -makeblock)
				// at least one char must be out there if its a string from the command line
				if (arguments.at(i)[0] == '-') {
					--i; // its a new parameter, put it back 
				}
				else {
					port = ofToInt(arguments.at(i));
				}
			}

			if (setup(id, port, baudrate)) {
				if (getTrossen(0)) {
					ofRobotArmCommand cmd(PenPose);
					getTrossen(0)->add(ofRobotArmCommand(PenPose));
				}
				if (getXyRobot(0)) {
					xyDataToSend data(xyPolyLine);
					data.macro(xyBezier);
					getXyRobot(0)->add(data);

					//bugbug figure out rotate, size, push/pop the canvas like OF does

					return;


				}
			}
		}
	}


	// setup
	bool ofMasterRobot::setup(RobotBrand id, int port, int baudrate) {

		ofRobotTrace() << "setup robot" << getName() << std::endl;

		// do one time setup
		if (id == Trossen) {
			ofTrRobotArmInternals::oneTimeSetup(); // do one time setup of static data
		}

		ofRobotSerial serial;
		serial.listDevices(); // let viewer see thats out there

		for (auto&device : serial.getDevices()) {
			if (device.getDeviceID() == 0 || (port != -1 && device.getDeviceID() != port)) {
				continue;//bugbug skipping com1, not sure whats on it, or unwanted port if port is known
			}
			shared_ptr<ofRobotSerial> serialdriver = make_shared<ofRobotSerial>();
			robotType robotType;
			string robotName;
			if (id == Trossen) {
				if (!serialdriver->setup(device.getDeviceName(), baudrate)) { // MAKE SURE BAUD RATES ARE THE SAME
					ofRobotTrace(FatalErrorLog) << device.getDeviceName() << std::endl;
					return false;
				}
				// if no mem at this point let it just crash
				// try trossen first
				if (!robotTypeIsError(robotType = serialdriver->waitForRobot(robotName, 10, 5))) {
					switch (robotType.second) {
					case PhantomXReactorArm:
					case PhantomXPincherArm:
					case WidowX:
						shared_ptr<ofTrRobotArm> arm = make_shared<ofTrRobotArm>();
						arm->setSerial(serialdriver);
						arm->setName(robotName);
						arm->setType(robotType);
						arm->setup(IKM_CYLINDRICAL);
						robotHelpers.push_back(arm);
						break;
					}
				}
				if (port > -1) {
					// port is known, only one robut, return now
					return robotType != createUndefinedRobotType();
				}
			}
			else {
				if (!serialdriver->setup(device.getDeviceName(), baudrate)) { // MAKE SURE BAUD RATES ARE THE SAME
					ofRobotTrace(FatalErrorLog) << device.getDeviceName() << std::endl;
					return false;
				}
				shared_ptr<xyRobot> maker = make_shared<xyRobot>(serialdriver);
				maker->update(xyDataToSend(SignOn));// some drivers require a sign on
				if (!robotTypeIsError(robotType = serialdriver->waitForRobot(robotName, 10, 5))) {
					switch (robotType.second) {
					case MakerBotXY:
						maker->setName(robotName);
						maker->setType(robotType);
						maker->setup();
						//maker->center();//bugbug too slow while debugging
						robotHelpers.push_back(maker);
						break;
					}
					if (port > -1) {
						// port is known, only one robut, return now
						return robotType != createUndefinedRobotType();
					}
				}
			}
		}
		return false;
	}
	//bugbug need a fill command, setcolor, setpixel? and then tie xy to robot arm  , const ofColor& color
	void xyRobot::rotate(const xyMotion& center, float angle, xyMotion& point) {
		if (!angle) {
			return;
		}
		float s = sin(angle);
		float c = cos(angle);

		// translate point back to origin:
		point.x -= center.x;
		point.y -= center.y;

		// rotate point
		float xnew = point.x * c - point.y * s;
		float ynew = point.x * s + point.y * c;

		// translate point back:
		point.x = xnew + center.x;
		point.y = ynew + center.y;
	}
	// create a line to function then use it here, that way driver just needs to do the basics
	void xyRobot::rectangleMacro(const xyMotion& point2, const xyMotion& point3, const xyMotion& point4, float angle) {
	}

	void xyRobot::add(XYCommands cmd, const vector<xyMotion>& points) {
		xyDataToSend data(cmd);
		for (const auto& point : points) {
			add(data);
		}
	}
	void xyRobot::add(XYCommands cmd, const xyMotion& point) {
		xyDataToSend data(cmd);
		add(data);
	}

	void xyRobot::sendit(xyDataToSend&data) {
		if (driver && data.isSetup()) {
			ofRobotTrace("xyRobot::sendit") << "cmd:" << (int)data.getCommand() << std::endl;
			sendToRobot(&data);
			int32_t c = (int32_t)data.getParameters().size(); // only send less than 200 parameters
			if (c > 0) {
				driver->write(c);
				string fastwrite;
				for (const auto& parm : data.getParameters()) {
					parm.trace();
					fastwrite += ofToString(parm.x);
					fastwrite += ",";
					fastwrite += ofToString(parm.y);
					fastwrite += ",";
				}
				size_t count = driver->write((uint8_t*)fastwrite.c_str(), fastwrite.size());
			}
			readResults(data.getCommand()); // waits for results bugbug put in a thread?
		}
	}
	void xyRobot::draw() {
		ofRobotTrace() << "draw xyRobot" << getName() << std::endl;

		if (driver) {
			int count = 0;
			for (auto& a : vectorOfCommands) {
				sendit(a);
			}
		}
		vectorOfCommands.clear();
	}
	// process results as needed
	bool xyRobot::readResults(int8_t cmd) {
		if (cmd == SignOn) { // signon data is read by the common sign on code
			return true;
		}
		ofRobotTrace() << "read xyRobot " << std::endl;
		SerialData data(2); // results header

		//bugbug put in thread or such?
		while (getDriver()->available() == 0)
			;

		while (getDriver()->available() > 0) {
			if (getDriver()->readAllBytes(data.data(), data.size()) == data.size()) {
				if (data[0] != 0xee) {
					// all commands need this header
					ofRobotTrace() << "unknown readResults " << data.data() << std::endl;
					return false;
				}
				else if (data[1] == Trace) {
					int32_t count = getDriver()->readInt32();
					int32_t number = getDriver()->readInt32();
					int32_t x = getDriver()->readInt32();
					int32_t y = getDriver()->readInt32();
					int i = 0; // place to set a break point
				}
				else {
					ofRobotTrace() << "xyRobot ACK for " << (int)data[1] << std::endl;
					if (cmd != data[1]) {
						ofRobotTrace() << "invalid ACK expected " << cmd << std::endl;
					}
					return true; //bugbug too much data to echo once things are working
				}
			}
		}
		return false;
	}

}