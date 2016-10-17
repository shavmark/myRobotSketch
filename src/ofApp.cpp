#include "ofApp.h"
#include <algorithm> 
// https://msdn.microsoft.com/en-us/library/windows/desktop/aa387285(v=vs.85).aspx
//https://github.com/sparkle-project/Sparkle
// shared across all robots and joints
map<SpecificJoint, int> RobotJoints::minValue;
map<SpecificJoint, int> RobotJoints::maxValue;
map<SpecificJoint, int> RobotJoints::defaultValue;
int RobotJoints::deltaDefault = 255;

// tracing helper
string echoJointType(SpecificJoint joint) {
	std::stringstream buffer;
	buffer << "SpecificJoint<robotType, robotArmJointType> = " << joint.first.first << ", " << joint.first.second << ", " << joint.second;
	return buffer.str();
}

void RobotJoints::set(SpecificJoint type, int minVal, int maxVal, int defaultVal) {
	ofLogNotice() << "RobotJoints::set" << echoJointType(type) << " - min = " << minVal << " max = " << maxVal << " default = " << defaultVal;

	minValue[type] = minVal;
	maxValue[type] = maxVal;
	defaultValue[type] = defaultVal;
}

RobotJoints::RobotJoints(uint8_t* data, const robotType& typeOfRobot) : RobotJointsState(data){
	this->typeOfRobot = typeOfRobot;
}

void RobotJoints::setX(int x) {
	ofLogNotice() << "try to set x=" << x;
	if (inRange(X, x)) {
		setLowLevelX(x);
	}
}
void RobotJoints::setY(int y) {
	ofLogNotice() << "try to set y=" << y;
	if (inRange(Y, y)) {
		setLowLevelY(y);
	}
}
void RobotJoints::setZ(int z) {
	ofLogNotice() << "try to set z=" << z;
	if (inRange(Z, z)) {
		setLowLevelZ(z);
	}
}
void RobotJoints::setWristAngle(int a) {
	ofLogNotice() << "try to set setWristAngle=" << a;
	if (inRange(wristAngle, a)) {
		setLowLevelWristAngle(a);
	}
}
void RobotJoints::setWristRotate(int a) {
	ofLogNotice() << "try to set setWristRotate=" << a;
	if (inRange(wristRotate, a)) {
		setLowLevelWristRotate(a);
	}
}
void RobotJoints::setGripper(int distance) {
	ofLogNotice() << "try to set setGripper=" << distance;
	if (inRange(Gripper, distance)) {
		setLowLevelGripper(distance);
	}
}
void RobotJointsState::set(uint16_t high, uint16_t low, int val) {
	ofLogNotice() << "set " << val;
	set(high, highByte(val));
	set(low, lowByte(val));
}

void RobotJointsState::send(RobotSerial* serial) {
	set(headerByteOffset, 255); // make sure its set
	getChkSum();
	echo();
	if (serial) {
		serial->write(data, count);
	}
}

robotLowLevelCommand RobotJointsState::getStartCommand(robotType type) {
	if (type.first == IKM_IK3D_CARTESIAN) {
		return setArm3DCartesianStraightWristAndGoHome; // bugbug support all types once the basics are working
	}
	if (type.first == IKM_IK3D_CARTESIAN_90) {
		return setArm3DCartesian90DegreeWristAndGoHome; // bugbug support all types once the basics are working
	}
	if (type.first == IKM_CYLINDRICAL_90) {
		return setArm3DCylindrical90DegreeWristAndGoHome; // bugbug support all types once the basics are working
	}
	if (type.first == IKM_CYLINDRICAL) {
		return setArm3DCylindricalStraightWristAndGoHome; // bugbug support all types once the basics are working
	}
	return unKnownCommand;//bugbug support all types once the basics are working
}
bool RobotJoints::inRange(robotArmJointType type, int value) {
	if (value > maxValue[SpecificJoint(typeOfRobot, type)] || value < minValue[SpecificJoint(typeOfRobot, type)]) {
		ofLogError() << "out of range " << value << " (" << minValue[SpecificJoint(typeOfRobot, type)] << ", " << maxValue[SpecificJoint(typeOfRobot, type)] << ")";
		return false;
	}
	return true;
}
// needs to only be called one time -- uses static data to save time/space. backhoe not supported
void RobotJoints::oneTimeSetup() {
	//createRobotType(robotArmMode mode, RobotTypeID id)
	set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, InterbotiXPhantomXReactorArm), X), -300, 300, 0);
	set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, InterbotiXPhantomXReactorArm), X), -300, 300, 0);
	set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, InterbotiXPhantomXReactorArm), Y), 50, 350, 235);
	set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, InterbotiXPhantomXReactorArm), Y), 20, 150, 140);
	set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, InterbotiXPhantomXReactorArm), Z), 20, 250, 210);
	set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, InterbotiXPhantomXReactorArm), Z), 10, 150, 30);
	set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, InterbotiXPhantomXReactorArm), wristAngle), -30, 30, 0);
	set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, InterbotiXPhantomXReactorArm), wristAngle), -90, -45, -90);
	set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, InterbotiXPhantomXReactorArm), wristRotate), 0, 1023, 512);
	set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, InterbotiXPhantomXReactorArm), wristRotate), 0, 1023, 512);
	set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, InterbotiXPhantomXReactorArm), Gripper), 0, 512, 512);
	set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, InterbotiXPhantomXReactorArm), Gripper), 0, 512, 512);


	set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, InterbotiXPhantomXReactorArm), X), 0, 1023, 512);
	set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, InterbotiXPhantomXReactorArm), X), 0, 1023, 512);
	set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, InterbotiXPhantomXReactorArm), Y), 50, 350, 235);
	set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, InterbotiXPhantomXReactorArm), Y), 20, 150, 140);
	set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, InterbotiXPhantomXReactorArm), Z), 20, 250, 210);
	set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, InterbotiXPhantomXReactorArm), Z), 10, 150, 30);
	set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, InterbotiXPhantomXReactorArm), wristAngle), -30, 30, 0);
	set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, InterbotiXPhantomXReactorArm), wristAngle), -90, -45, -90);
	set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, InterbotiXPhantomXReactorArm), wristRotate), 0, 1023, 512);
	set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, InterbotiXPhantomXReactorArm), wristRotate), 0, 1023, 512);
	set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, InterbotiXPhantomXReactorArm), Gripper), 0, 512, 512);
	set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, InterbotiXPhantomXReactorArm), Gripper), 0, 512, 512);

	// mark end of list for debugging
	set(SpecificJoint(createUndefinedRobotType(), JointNotDefined), 0, 0, 0); // should not be set to this while running, only during object init

	return;

}

// echo, ignoring null bytes
void echoRawBytes(uint8_t *bytes, int count) {
	std::stringstream buffer;
	for (int i = 0; i < count; ++i) {
		buffer << " bytes[" << i << "] = " << (int)bytes[i]; // echo in one line
	}
	ofLogNotice() << buffer.str();
}
// get pose data from serial port bugbug decode this
void RobotSerial::readPose() {
	if (available() == 0) {
		return;
	}

	ofLogNotice() << "RobotSerial::readPose";

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
int RobotSerial::readAllBytes(uint8_t *bytes, int bytesRequired) {
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
	ofLogNotice() << "wait for mr robot...";
	
	waitForSerial();

	robotType type = createUndefinedRobotType();
	uint8_t bytes[31];

	int readin = readBytesInOneShot(bytes, 5);
	if (readin == 5) {
		type = ArmIDResponsePacket(bytes);
		if (type.first == IKM_NOT_DEFINED) {
			ofLogError() << "invalid robot type";
			return type;
		}
	}
	else {
		ofLogError() << "invalid robot sign on";
		return type;
	}

	// get sign on echo from device
	readin = readBytesInOneShot(bytes, 30);
	bytes[readin] = 0;
	ofLogNotice() << bytes;
	return type;
}
void Robot::setup() {

	while (!path.empty()) {
		path.pop(); // clean any old stuff out
	}

	// do one time setup
	RobotJoints jv(data);
	jv.oneTimeSetup(); // do one time setup of static data

	serial.listDevices(); // let viewer see thats out there

	serial.setup(1, 38400);//bugbug get from xml

	if (!robotTypeIsError(serial.waitForRobot())) {
		ofLogNotice() << "robot setup complete";
	}

}
void Robot::update() {
}

void Robot::draw() {
	
	while (!path.empty()) {
		path.front()->draw();
		if (path.front()->deleteWhenDone) {
			path.pop();
		}
	}
}
void RobotJointsState::set(uint16_t offset, uint8_t b) { 
	ofLogNotice() << "set data[" << offset << "] = " << (uint16_t)b;

	data[offset] = b;
}


void RobotJointsState::echo() {
#define ECHO(a)ofLogNotice() << "echo[" << a << "] = "  << std::hex << (unsigned int)data[a] << "h "  << (unsigned int)data[a] << "d "<< #a;
	ECHO(headerByteOffset)
	ECHO(xLowByteOffset)
	ECHO(yHighByteOffset)
	ECHO(yLowByteOffset)
	ECHO(zHighByteOffset)
	ECHO(zLowByteOffset)
	ECHO(wristAngleHighByteOffset)
	ECHO(wristAngleLowByteOffset)
	ECHO(wristRotateHighByteOffset)
	ECHO(wristRotateLowByteOffset)
	ECHO(gripperHighByteOffset)
	ECHO(gripperLowByteOffset)
	ECHO(deltaValBytesOffset)
	ECHO(buttonByteOffset)
	ECHO(extValBytesOffset)
	ECHO(checksum)
}

uint8_t RobotJointsState::getChkSum() {
	uint16_t sum = 0;
	for (int i = xHighByteOffset; i <= extValBytesOffset; ++i) {
		sum += data[i];
	}
	uint16_t invertedChecksum = sum % 256;//isolate the lowest byte 8 or 16?

	data[checksum] = 255 - invertedChecksum; //invert value to get file checksum

	return data[checksum];
}

void RobotSerial::write(uint8_t* data, int count) {
	// from http://learn.trossenrobotics.com/arbotix/arbotix-communication-controllers/31-arm-link-reference.html
		
	//If you are sending packets at an interval, do not send them faster than 30hz(one packet every 33ms).
	// no need to hurry packets so just want the minimum amount no matter what
	ofSleepMillis(500);
	int sent = writeBytes(data, count);

	ofLogNotice() << "write sent = " << sent;

	readPose(); // pose is sent all the time
	readPose(); // how often are two sent?

}
void Command::setPoint(ofPoint pt) {
	if (pt.x) {
		setX(pt.x);
	}
	if (pt.y) {
		setY(pt.y);
	}
	if (pt.z) {
		setZ(pt.z);
	}
}
void danceCommand::draw() {
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

void sanityTestCommand::draw() {
	ofLogNotice() << "sanityTest";
	setX(300);
	setY(150);
	setZ(150);
	setWristAngle(30);
	setWristRotate(120);
	setGripper(0);
	setLowLevelCommand(NoArmCommand);
	setDelta(255);
	setButton();
	send(&robot->serial);
}

// "home" and set data matching state
void RobotJoints::setDefaultState() {
	ofLogNotice() << "setDefaults";
	setX(getDefaultValue(X));
	setY(getDefaultValue(Y));
	setZ(getDefaultValue(Z));
	setWristAngle(getDefaultValue(wristAngle));
	setWristRotate(getDefaultValue(wristRotate));
	setGripper(getDefaultValue(Gripper));
	setDelta(getDeltaDefault());
	setLowLevelCommand(NoArmCommand);
	setButton();
}
// will block until arm is ready
robotType RobotJoints::setStartState() {
	ofLogNotice() << "setStartState " << typeOfRobot.first << " " << typeOfRobot.second;
	setLowLevelCommand(getStartCommand(typeOfRobot));
	return typeOfRobot;
}

//--------------------------------------------------------------
void ofApp::setup(){
	robot.setup();
}

//--------------------------------------------------------------
void ofApp::update(){
	
	robot.update();
	shared_ptr<Command> cmd = robot.createCommand<Command>();
	cmd->reset();
	//cmd->addPoint(ofPoint(300, 0, 0));
	cmd->addPoint(ofPoint(0, 100, 0));
	cmd->addPoint(ofPoint(100, 150, 0));
	robot.add(cmd);

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
