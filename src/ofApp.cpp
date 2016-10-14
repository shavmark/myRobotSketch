#include "ofApp.h"
#include <algorithm> 

// shared across all robots and joints
map<valueType, uint16_t> RobotJoints::minValue;
map<valueType, uint16_t> RobotJoints::maxValue;
map<valueType, uint16_t> RobotJoints::defaultValue;
uint16_t RobotJoints::deltaDefault = 255;

void RobotJoints::set(valueType type, uint16_t min, uint16_t max, uint16_t defaultvalue) {
	minValue[type] = min;
	maxValue[type] = max;
	defaultValue[type] = defaultvalue;
}

RobotJoints::RobotJoints(shared_ptr<uint8_t> data, robotArmMode mode) : RobotJointsState(data){
	armMode = mode;
}

void RobotJoints::setX(uint16_t x) { 
	if (inRange(X, x)) {
		setLowLevelX(x);
	}
}
void RobotJoints::setY(uint16_t y) { 
	if (inRange(Y, y)) {
		setLowLevelY(y);
	}
}
void RobotJoints::setZ(uint16_t z) { 
	if (inRange(Z, z)) {
		setLowLevelZ(z);
	}
}
void RobotJoints::setWristAngle(uint16_t a) { 
	if (inRange(wristAngle, a)) {
		setLowLevelWristAngle(a);
	}
}
void RobotJoints::setWristRotate(uint16_t a) { 
	if (inRange(wristRotate, a)) {
		setLowLevelWristRotate(a);
	}
}
void RobotJoints::setGripper(uint16_t distance) { 
	if (inRange(Gripper, distance)) {
		setLowLevelGripper(distance);
	}
}

void RobotJointsState::draw(shared_ptr<RobotSerial> serial) {
	set(0, 255); // make sure its set
	getChkSum();
	echo();
	serial->write(data, count);
}

bool RobotJoints::inRange(robotArmJointType type, uint16_t value) {
	if (value > maxValue[valueType(armMode, type)] || value < minValue[valueType(armMode, type)]) {
		ofLogError() << "out of range " << value << " (" << minValue[valueType(armMode, type)] << ", " << maxValue[valueType(armMode, type)] << ")";
		return false;
	}
	return true;
}
// instance setup
void RobotJoints::setup() {
	setDefaults();
}
// needs to only be called one time -- uses static data to save time/space
shared_ptr<uint8_t> RobotJoints::oneTimeSetup() {
	set(valueType(IKM_IK3D_CARTESIAN, X), -300, 300, 0);
	set(valueType(IKM_IK3D_CARTESIAN_90, X), -300, 300, 0);
	set(valueType(IKM_CYLINDRICAL, X), 0, 1023, 512);
	set(valueType(IKM_CYLINDRICAL_90, X), 0, 1023, 512);
	set(valueType(IKM_BACKHOE, X), 0, 0, 512);

	set(valueType(IKM_IK3D_CARTESIAN, Y), 50, 350, 235);
	set(valueType(IKM_IK3D_CARTESIAN_90, Y), 20, 150, 140);
	set(valueType(IKM_CYLINDRICAL, Y), 50, 350, 235);
	set(valueType(IKM_CYLINDRICAL_90, Y), 20, 150, 140);
	set(valueType(IKM_BACKHOE, Y), 0, 0, 512);

	set(valueType(IKM_IK3D_CARTESIAN, Z), 20, 250, 210);
	set(valueType(IKM_IK3D_CARTESIAN_90, Z), 10, 150, 30);
	set(valueType(IKM_CYLINDRICAL, Z), 20, 250, 210);
	set(valueType(IKM_CYLINDRICAL_90, Z), 10, 150, 30);
	set(valueType(IKM_BACKHOE, Z), 0, 0, 512);

	set(valueType(IKM_IK3D_CARTESIAN, wristAngle), -30, 30, 0);
	set(valueType(IKM_IK3D_CARTESIAN_90, wristAngle), -90, -45, -90);
	set(valueType(IKM_CYLINDRICAL, wristAngle), 30, 30, 0);
	set(valueType(IKM_CYLINDRICAL_90, wristAngle), -90, -45, -90);

	set(valueType(IKM_IK3D_CARTESIAN, wristRotate), 0, 1023, 512);
	set(valueType(IKM_IK3D_CARTESIAN_90, wristRotate), 0, 1023, 512);
	set(valueType(IKM_CYLINDRICAL, wristRotate), 0, 1023, 512);
	set(valueType(IKM_CYLINDRICAL_90, wristRotate), 0, 1023, 512);

	set(valueType(IKM_IK3D_CARTESIAN, Gripper), 0, 512, 512);
	set(valueType(IKM_IK3D_CARTESIAN_90, Gripper), 0, 512, 512);
	set(valueType(IKM_CYLINDRICAL, Gripper), 0, 512, 512);
	set(valueType(IKM_CYLINDRICAL_90, Gripper), 0, 512, 512);

	set(valueType(IKM_NOT_DEFINED, JointNotDefined), 0, 0, 0); // should not be set to this while running, only during object init

	return allocateData();

}

// get pose data from serial port bugbug decode this
void RobotSerial::readPose() {
	if (available() == 0) {
		return;
	}
	int size = available();
	uint8_t *bytes = new uint8_t[size+1];
	int i = 0;
	for (; true; ++i) {
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
	else {
		ofLogNotice() << bytes;
	}
	delete bytes;

}
int RobotSerial::readBytesInOneShot(uint8_t *bytes, int bytesMax) {
	int result = 0;
	if (available() > 0) {
		if ((result = readBytes(bytes, bytesMax)) == OF_SERIAL_ERROR) {
			ofLogError() << "serial failed";
			return 0;
		}
		while (result == OF_SERIAL_NO_DATA) {
			result = readBytes(bytes, bytesMax);
			if (result == OF_SERIAL_ERROR) {
				ofLogError() << "serial failed";
				return 0;
			}
			if (result != OF_SERIAL_NO_DATA) {
				return result;
			}
		}
	}
	return result;
}
int RobotSerial::readBytes(uint8_t *bytes, int bytesRequired) {
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
					ofLog(OF_LOG_ERROR, "unrecoverable error reading from serial");
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
robotType RobotSerial::ArmIDResponsePacket(uint8_t *bytes) {
	if (bytes != nullptr) {
		robotArmMode armMode = (robotArmMode)bytes[2];
		switch (armMode) {
		case IKM_IK3D_CARTESIAN:
			ofLogNotice() << "arm mode IKM_IK3D_CARTESIAN";
			break;
		case IKM_IK3D_CARTESIAN_90:
			ofLogNotice() << "arm mode IKM_IK3D_CARTESIAN_90";
			break;
		case IKM_CYLINDRICAL:
			ofLogNotice() << "arm mode IKM_CYLINDRICAL";
			break;
		case IKM_CYLINDRICAL_90:
			ofLogNotice() << "arm mode IKM_CYLINDRICAL_90";
			break;
		default:
			ofLogNotice() << "arm mode IKM_BACKHOE mode?";
			break;
		}
		RobotTypeID id= unknownRobotType;
		switch (bytes[1]) {
		case 2:
			id = InterbotiXPhantomXReactorArm;
			ofLogNotice() << "InterbotiXPhantomXReactorArm";
			break;
		}
		return robotType(armMode, id);
	}
	return robotType(IKM_NOT_DEFINED, unknownRobotType);
}


robotType RobotSerial::waitForRobot() {
	ofLogNotice() << "wait for robot...";
	
	waitForSerial();

	robotType type = robotType(IKM_NOT_DEFINED, unknownRobotType);
	unsigned char bytes[31];

	int readin = readBytesInOneShot(bytes, 5);
	if (readin == 5) {
		type = ArmIDResponsePacket(bytes);
		if (type.first == IKM_NOT_DEFINED) {
			ofLogError() << "invalid robot type";
		}
	}
	else {
		ofLogError() << "invalid robot sign on";
	}

	// get sign on echo from device
	readin = readBytesInOneShot(bytes, 30);
	bytes[readin] = 0;
	ofLogNotice() << bytes;
	return type;
}
void Robot::setup() {
	
	if (!(serial = make_shared<RobotSerial>())) {
		ofLogFatalError() << "memory";
		return;
	}
	RobotJoints jv(data);
	data = jv.oneTimeSetup(); // do one time setup of static data
	
	serial->listDevices();
	serial->setup(1, 38400);//bugbug get from xml 
	serial->waitForRobot();

	jv.setStartState(); // go  to a known state
	jv.draw(serial);
	
}
void Robot::update() {
	shared_ptr<RobotMotion> joints = make_shared<RobotMotion>(serial, data);
	joints->setup(SignOnDance);
	path.push(joints);
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

void segment(int a, int b) {

}
void line(int a, int b) {

}
void RobotMotion::draw() {
	if (getCommand(nullptr) == NoRobotHighLevelCommand) {
		RobotJointsState::draw(serial);
	}
	else {	// process commands
		int64_t value;
		robotCommand cmd = getCommand(&value);
		switch (cmd) {
		case SignOnDance:
			dance();
			break;
		case CenterArm:
			center();
			break;
		case DelayArm:
			ofSleepMillis(value);
			break;
		}
	}

}
void Robot::draw() {
	
	while (!path.empty()) {
		shared_ptr<RobotMotion> joints;
		
		path.front()->draw();
		path.pop();

	}
}
void RobotJointsState::set(uint16_t offset, uint8_t b) { 
	ofLogNotice() << "data.get()[" << offset << "] = " << b; 
	data.get()[offset] = b; 
}


void RobotJointsState::echo() {
	for (int i = 0; i < count; ++i) {
		ofLogNotice() << "echo[" << i << "] = " << std::hex << "0x" << (unsigned int)data.get()[i];
	}
}

uint8_t RobotJointsState::getChkSum() {
	uint16_t sum = 0;
	for (int i = xHighByteOffset; i <= extValBytesOffset; ++i) {
		sum += data.get()[i];
	}
	uint8_t invertedChecksum = sum % 256;//isolate the lowest byte 8 or 16?

	data.get()[checksum] = 255 - invertedChecksum; //invert value to get file checksum

	return data.get()[checksum];
}

void RobotSerial::write(shared_ptr<uint8_t> data, int count) {
	// from http://learn.trossenrobotics.com/arbotix/arbotix-communication-controllers/31-arm-link-reference.html
		
	//If you are sending packets at an interval, do not send them faster than 30hz(one packet every 33ms).
	// no need to hurry packets so just want the minimum amount no matter what
	ofSleepMillis(500);
	int sent = writeBytes(data.get(), count);

	ofLogNotice() << "write sent = " << sent;

	readPose(); // pose is sent all the time
	readPose(); // how often are two sent?

}

void RobotMotion::center() {
	ofLogNotice() << "center"; //bugbug left off here

							   //moveXleft(JointValue(valueType(armMode, X)).getMax() / 2);
							   //moveYout(JointValue(valueType(armMode, Y)).getMax() / 2);
}

void RobotMotion::dance() {
	// spin nose
	/*
	set(wristRotateHighByteOffset, wristRotateLowByteOffset, JointValue(valueType(armMode, wristRotate)).getMax());
	set(zHighByteOffset, zLowByteOffset, JointValue(valueType(armMode, Z)).getMax());
	sendNow();
	set(wristRotateHighByteOffset, wristRotateLowByteOffset, JointValue(valueType(armMode, wristRotate)).getMin());
	set(zHighByteOffset, zLowByteOffset, JointValue(valueType(armMode, Z)).getDefaultValue());
	sendNow();
	set(xHighByteOffset, xLowByteOffset, JointValue(valueType(armMode, X)).getMax());  // go right
	sendNow();
	set(xHighByteOffset, xLowByteOffset, JointValue(valueType(armMode, X)).getMin());  // go go left
	sendNow();
	set(xHighByteOffset, xLowByteOffset, JointValue(valueType(armMode, X)).getMax()/2);  // center X
	sendNow();
	*/
}

// set basic data that moves a little bit after starting up
void RobotMotion::sanityTest() {
	ofLogNotice() << "sanityTest";
	setStartState();
	draw();
	setX(512);
	setY(150);
	setZ(150);
	setWristAngle(90);
	setWristRotate(512);
	setGripper(0);
	setLowLevelCommand(NoArmCommand);
	setDelta(128);
	setButton();
	draw();
}

void RobotMotion::centerAllServos() {
	ofLogNotice() << "centerAllServos";
	setStartState(IKM_BACKHOE, setArmBackhoeJointAndGoHome);
	setX(512);
	setY(512);
	setZ(512);
	setWristAngle(512);
	setWristRotate(512);
	setGripper(512);
	setDelta(128);
	draw();
}
// "home" and set data matching state
void RobotJoints::setDefaults() {
	ofLogNotice() << "setDefaults";
	setX(getDefaultValue(X)+512);
	setY(getDefaultValue(Y));
	setZ(getDefaultValue(Z));
	setWristAngle(getDefaultValue(wristAngle));
	setWristRotate(getDefaultValue(wristRotate));
	setGripper(getDefaultValue(Gripper));
	setLowLevelCommand(NoArmCommand);
	setDelta(getDeltaDefault());
	setButton();
}
void Robot::reset() { 
	while (!path.empty()) { 
		path.pop(); 
	} 
};
// will block until arm is ready
void RobotJoints::setStartState(robotArmMode mode, robotLowLevelCommand cmd) {
	ofLogNotice() << "setStartState " << mode << " " << cmd;
	armMode = mode;
	setLowLevelCommand(cmd);
}

//--------------------------------------------------------------
void ofApp::setup(){
	robot.setup();
}

//--------------------------------------------------------------
void ofApp::update(){
	robot.update();
}

//--------------------------------------------------------------
void ofApp::draw(){
	robot.draw();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

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
