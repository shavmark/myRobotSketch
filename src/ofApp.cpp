#include "ofApp.h"
#include "trossenrobots.h"
#include <algorithm> 

template<typename T>void clear(vector< T > vect) {
	vector< T >::iterator it = vect.begin();
	while (it != vect.end()) {
		it = vect.erase(it);
	}
}

// echo, ignoring null bytes
void ofRobotSerial::echoRawBytes(uint8_t *bytes, int count) {
	std::stringstream buffer;
	for (int i = 0; i < count; ++i) {
		buffer << " bytes[" << i << "] = " << (int)bytes[i]; // echo in one line
	}
	getTracer() << buffer.str() << std::endl;
}

// get pose data from serial port bugbug decode this
void ofRobotSerial::readPose() {
	if (available() == 0) {
		return;
	}

	getTracer() << "RobotSerial::readPose" << std::endl;

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
	else if (i == 10){
		echoRawBytes(bytes, 10);
		ArmIDResponsePacket(bytes); // 2 signs ons come back some times, likely a timing issue?
		ArmIDResponsePacket(bytes); // first 2 bytes are sign on type
	}
	else {
		ofLogNotice() << bytes;
	}
	delete bytes;

}
int ofRobotSerial::readBytesInOneShot(uint8_t *bytes, int bytesMax) {
	int result = 0;
	if (available() > 0) {
		if ((result = readBytes(bytes, bytesMax)) == OF_SERIAL_ERROR) {
			getErrorTracer() << "serial failed" << std::endl;
			return 0;
		}
		while (result == OF_SERIAL_NO_DATA) {
			result = readBytes(bytes, bytesMax);
			if (result == OF_SERIAL_ERROR) {
				getErrorTracer() << "serial failed" << std::endl;
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
				int result = readBytes(&bytes[bytesArrayOffset],	bytesRemaining);

				// check for error code
				if (result == OF_SERIAL_ERROR) {
					// something bad happened
					getErrorTracer() << "unrecoverable error reading from serial" << std::endl;
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
			getTracer() << "arm mode IKM_IK3D_CARTESIAN" << std::endl;
			break;
		case IKM_IK3D_CARTESIAN_90:
			getTracer() << "arm mode IKM_IK3D_CARTESIAN_90" << std::endl;
			break;
		case IKM_CYLINDRICAL:
			getTracer() << "arm mode IKM_CYLINDRICAL" << std::endl;
			break;
		case IKM_CYLINDRICAL_90:
			getTracer() << "arm mode IKM_CYLINDRICAL_90" << std::endl;
			break;
		default:
			getTracer() << "arm mode IKM_BACKHOE mode?" << std::endl;
			break;
		}
		RobotTypeID id= unknownRobotType;
		switch (bytes[1]) {
		case 2:
			id = InterbotiXPhantomXReactorArm;
			getTracer() << "InterbotiXPhantomXReactorArm" << std::endl;
			break;
		}
		return robotType(armMode, id);
	}
	return robotType(IKM_NOT_DEFINED, unknownRobotType);
}


robotType ofRobotSerial::waitForRobot() {
	ofRobotTrace() << "wait for mr robot..." << std::endl;
	
	waitForSerial();

	robotType type = createUndefinedRobotType();
	uint8_t bytes[31];

	int readin = readBytesInOneShot(bytes, 5);
	if (readin == 5) {
		type = ArmIDResponsePacket(bytes);
		if (type.first == IKM_NOT_DEFINED) {
			getErrorTracer() << "invalid robot type" << std::endl;
			return type;
		}
	}
	else {
		getErrorTracer() << "invalid robot sign on" << std::endl;
		return type;
	}

	// get sign on echo from device
	readin = readBytesInOneShot(bytes, 30);
	bytes[readin] = 0;
	getTracer() << bytes << std::endl;
	return type;
}
void ofRobot::setup() {

	clear(cmds);

	serial.listDevices(); // let viewer see thats out there

	serial.setup(1, 38400);//bugbug get from xml

	// start with default mode
	robotType defaultType;
	if (!robotTypeIsError(defaultType = serial.waitForRobot())) {
		getTracer() << "robot setup complete" << std::endl;
	}

	getTracer() << "use IKM_CYLINDRICAL" << std::endl;
	type = robotType(IKM_CYLINDRICAL, defaultType.second);

	// do one time setup
	RobotJoints jv(data, type, &getTracer());
	jv.oneTimeSetup(); // do one time setup of static data
					   
}
void ofRobot::update() {
}

void ofRobotCommands::echo() const {
	for (const auto& cmd :cmdVector) {
		cmd.echo();
	}
}
void RobotCommand::echo() const {
	pointPercent.echo();
	settingsPercent.echo();
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
	RobotJoints jv(getType(), &getTracer());

	// only set what we care about, let the rest stay at default. userDefinedRanges is a sparse array of sorts
	jv.setMin(createJoint(X, getType().first, getType().second), jv.getMid(X) - 300);
	jv.setMax(createJoint(X, getType().first, getType().second), jv.getMid(X) + 300);

}

void ofRobotSerial::write(uint8_t* data, int count) {
	// from http://learn.trossenrobotics.com/arbotix/arbotix-communication-controllers/31-arm-link-reference.html
		
	//If you are sending packets at an interval, do not send them faster than 30hz(one packet every 33ms).
	// no need to hurry packets so just want the minimum amount no matter what
	ofSleepMillis(100); // 100 ms seems ok, to much less and we start to overrun bugbug is this true?  
	int sent = writeBytes(data, count);

	getTracer() << "write sent = " << sent << std::endl;

	readPose(); // pose is sent all the time
	readPose(); // how often are two sent?

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

void ofRobotState::echo() const {
	ofRobotTrace() << "WristAngle=" << (set[0] ? ofToString(getWristAngle()) : "<not set>") << std::endl;
	ofRobotTrace() << "WristRotatation=" << (set[1] ? ofToString(getWristRotation()) : "<not set>") << std::endl;
	ofRobotTrace() << "Gripper=" << (set[2] ? ofToString(getGripper()) : "<not set>") << std::endl;
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
// RobotPositions can only be 0.0 to +/- 1.0 (0 to +/- 100%)
void RobotCommand::init(const ofRobotPosition& pointPercent, const ofRobotState& settingsPercent, int millisSleep, bool deleteWhenDone) {
	this->pointPercent = pointPercent;
	this->settingsPercent = settingsPercent;
	this->deleteWhenDone = deleteWhenDone;
	this->millisSleep = millisSleep;
}

// add ranges checking
void ofRobotCommands::add(const RobotCommand& cmd, BuiltInCommandNames name) { 
	cmdVector.push_back(cmd); 
	this->name = name; 
}

void ofRobotCommands::sizing() {
	getTracer() << "sizing" << std::endl;
	reset();

	for (float f = 0.0f; f < 1.0f; f += 0.01) {
		add(RobotCommand(f)); // need to be percents!!
		add(RobotCommand(5000));// sleep
	}

	add(RobotCommand(0.5f, 0.0f)); // need to be percents!!
	add(RobotCommand(5000));

	for (float f = 0.0f; f < 1.0f; f += 0.01) {
		add(RobotCommand(NoRobotValue, f)); // need to be percents!!
		add(RobotCommand(5000));
	}

	//z not sized

	setLowLevelCommand(NoArmCommand);
	setDelta(255);
}
void ofRobotCommands::sanityTestHighLevel() {
	getTracer() << "high level sanityTest" << std::endl;
	reset();
	add(RobotCommand(0.3f, 0.6f, NoRobotValue, 1.0f, -1.0f, 0.5f)); // need to be percents!!
	add(RobotCommand(NoRobotValue, NoRobotValue, 0.3f)); // too close could hit the bottom
	add(RobotCommand(NoRobotValue, NoRobotValue, 1.0f));
	add(RobotCommand(1000)); // sleep
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
	getTracer() << "low level sanityTest" << std::endl;
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
	shared_ptr<ofRobotCommands> p = make_shared<ofRobotCommands>(this, name, &getTracer());
	if (p) {
		cmds.push_back(p);
	}
	return p;
}

ofRobotCommands::ofRobotCommands(ofRobot *robot, BuiltInCommandNames name, RobotTrace *tracer) :RobotJoints(robot->data, robot->type, tracer) {
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

		vector< RobotCommand >::iterator it = cmdVector.begin();
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

//--------------------------------------------------------------
void ofApp::setup(){
	robot.setup();
}

//--------------------------------------------------------------
void ofApp::update(){
	
	robot.update();

	shared_ptr<ofRobotCommands> cmd = robot.add(ofRobotCommands::HighLevelTest);

	/*
	shared_ptr<RobotCommands> cmd = robot.createAndAdd<RobotCommands>(RobotCommand());
	cmd->reset(); // home the device
	cmd->setup(0.0001, 0, 0.0, 0.0, 0.0); // far left

	shared_ptr<rectangleCommand> cmd2 = robot.createCommand<rectangleCommand>();
	cmd2->reset(); // home the device
	cmd2->millisSleep = 10000;// wait before moving
	cmd2->setup(1.0, 0, 0.0, 0.0, 0.0); // far right
	robot.add(cmd2);

	shared_ptr<rectangleCommand> cmd3 = robot.createCommand<rectangleCommand>();
	cmd3->reset(); // home the device
	cmd3->millisSleep = 10000;// wait before moving
	cmd3->setup(0.0, 1.0, 0.0, 0.0, 0.0); 
	robot.add(cmd3);

	shared_ptr<rectangleCommand> cmd4 = robot.createCommand<rectangleCommand>();
	cmd4->reset(); // home the device
	cmd4->millisSleep = 10000;// wait before moving
	cmd4->setup(0.0, 0.0001, 0.0, 0.0, 0.0);
	robot.add(cmd4);

	//motion->setup(); // start a new motion bugbug outside of testing  like now this is only done one time
	//path.push(motion);
	//shared_ptr<RobotMotion> joints = make_shared<RobotMotion>(serial, data);
	//joints->setup(SignOnDance);
	//path.push(joints);
	/*
	return;
	data.setCommand(EnableArmMovement);
	data.setY(50);
	path.push(data);
	data.reset();
	data.setY(350);
	path.push(data);
	return;
	data.reset();
	data.setX(10); // 300 max 60 units covers about 3", 20 units is .75" so given the arch its not just inches
	data.setY(350);
	path.push(data);
	data.reset();
	data.setZ(250);
	data.setX(-10); // -300 max
	path.push(data);
	data.reset();
	data.setCommand(DelayArm, 1000);
	path.push(data);

	loc.reset();
	loc.moveYout(260);
	loc.moveZup(250);
	path.push(loc);
	loc.reset();
	loc.moveXleft(-212);
	path.push(loc);
	loc.reset();
	loc.moveXleft(200);
	loc.moveYout(200);
	path.push(loc);
	*/

}

//--------------------------------------------------------------
void ofApp::draw(){
	robot.draw();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if (key == ' ') {
		robot.setPause();//bugbug need a thread/semaphore to do this
		robot.echo();
		robot.setPause(false);
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
