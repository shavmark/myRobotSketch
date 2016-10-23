#include "ofApp.h"
#include <algorithm> 


void ofRobotState::echo() const {
	ofRobotTrace() << "WristAngle=" << (set[0] ? ofToString(getWristAngle()) : "<not set>") << std::endl;
	ofRobotTrace() << "WristRotatation=" << (set[1] ? ofToString(getWristRotation()) : "<not set>") << std::endl;
	ofRobotTrace() << "Gripper=" << (set[2] ? ofToString(getGripper()) : "<not set>") << std::endl;
}
// RobotPositions can only be 0.0 to +/- 1.0 (0 to +/- 100%)
void ofRobotCommand::init(const ofRobotPosition& pointPercent, const ofRobotState& settingsPercent, int millisSleep, bool deleteWhenDone) {
	this->pointPercent = pointPercent;
	this->settingsPercent = settingsPercent;
	this->deleteWhenDone = deleteWhenDone;
	this->millisSleep = millisSleep;
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
		ofLogError("Command::setPoint") << "setPoint not supported" << std::endl;
	}
}

// add ranges checking
void ofRobotCommands::add(const ofRobotCommand& cmd, BuiltInCommandNames name) {
	cmdVector.push_back(cmd);
	this->name = name;
}

void ofRobotCommands::sizing() {
	RobotTrace() << "sizing" << std::endl;
	reset();

	for (float f = 0.0f; f < 1.0f; f += 0.01) {
		add(ofRobotCommand(f)); // need to be percents!!
		add(ofRobotCommand(5000));// sleep
	}

	add(ofRobotCommand(0.5f, 0.0f)); // need to be percents!!
	add(ofRobotCommand(5000));

	for (float f = 0.0f; f < 1.0f; f += 0.01) {
		add(ofRobotCommand(NoRobotValue, f)); // need to be percents!!
		add(ofRobotCommand(5000));
	}

	//z not sized

	setLowLevelCommand(NoArmCommand);
	setDelta(255);
}
void ofRobotCommands::sanityTestHighLevel() {
	RobotTrace() << "high level sanityTest" << std::endl;
	reset();
	add(ofRobotCommand(0.3f, 0.6f, NoRobotValue, 1.0f, -1.0f, 0.5f)); // need to be percents!!
	add(ofRobotCommand(NoRobotValue, NoRobotValue, 0.3f)); // too close could hit the bottom
	add(ofRobotCommand(NoRobotValue, NoRobotValue, 1.0f));
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
	RobotTrace() << "low level sanityTest" << std::endl;
	setX(300); // absolution position vs. percentages
	setY(150);
	setZ(150);
	setWristAngle(30);
	setWristRotate(120);
	setGripper(0);
	setLowLevelCommand(NoArmCommand);
	setDelta(255);
	setButton();
}

shared_ptr<ofRobotCommands> ofRobot::add(ofRobotCommands::BuiltInCommandNames name) {
	shared_ptr<ofRobotCommands> p = make_shared<ofRobotCommands>(this, name);
	if (p) {
		cmds.push_back(p);
	}
	return p;
}

ofRobotCommands::ofRobotCommands(ofRobot *robot, BuiltInCommandNames name) :RobotJoints(robot->data, robot->type) {
	this->robot = robot;
	if (robot) {
		//typedef pair<robotType, robotArmJointType> SpecificJoint
		setUserDefinedRanges(SpecificJoint(robot->getType(), X), &robot->userDefinedRanges);
	}
	this->name = name;
}
void ofRobotCommands::reset() { // setup can be ignored for a reset is not required
	setStartState();
	send(&robot->serial); // send the mode, also resets the robot
	setDefaultState();
	clear(cmdVector);
}
void ofRobotCommands::draw() {

	if (robot) {
		switch (name) {
		case ofRobotCommands::HighLevelTest:
			sanityTestHighLevel(); // will populate cmdVector
			break;
		case ofRobotCommands::LowLevelTest:
			sanityTestLowLevel();
			break;
		case ofRobotCommands::Sizing:
			sizing();
			break;
		}

		vector< ofRobotCommand >::iterator it = cmdVector.begin();
		while (it != cmdVector.end()) {
			//{UserDefined, LowLevelTest, HighLevelTest, Sizing

			setPoint(it->pointPercent);
			setState(it->settingsPercent);
			it->sleep(); // sleep if requested
			send(&robot->serial);// move
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
	pointPercent.echo();
	settingsPercent.echo();
}

void ofRobot::setup() {

	clear(cmds);

	serial.listDevices(); // let viewer see thats out there

	serial.setup(1, 38400);//bugbug get from xml

						   // start with default mode
	robotType defaultType;
	if (!robotTypeIsError(defaultType = serial.waitForRobot())) {
		RobotTrace() << "robot setup complete" << std::endl;
	}

	RobotTrace() << "use IKM_CYLINDRICAL" << std::endl;
	type = robotType(IKM_CYLINDRICAL, defaultType.second);

	// do one time setup
	RobotJoints jv(data, type);
	jv.oneTimeSetup(); // do one time setup of static data

}
void ofRobot::update() {
}

void ofRobot::echo() {
	for (auto& cmd : cmds) {
		cmd->echo();
	}
}

void ofRobot::draw() {
	if (pause) {
		return;
	}
	for (auto& cmd : cmds) {
		cmd->draw();
	}
}

void ofDrawingRobot::setup() {

	ofRobot::setup(); // set base class first

					  //bugbug swing arm x and y, space means break and you get size that way. use a menu and xml for this
					  // set ranges so percents work against these. leave Y as is bugbug figure this all out
	RobotJoints jv(getType());

	// only set what we care about, let the rest stay at default. userDefinedRanges is a sparse array of sorts
	jv.setMin(createJoint(X, getType().first, getType().second), jv.getMid(X) - 300);
	jv.setMax(createJoint(X, getType().first, getType().second), jv.getMid(X) + 300);

}


