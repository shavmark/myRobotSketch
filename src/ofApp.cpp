#include "ofApp.h"
#include <algorithm> 

void JointValue::set(valueType type, uint16_t min, uint16_t max, uint16_t defaultvalue) {
	minValue[type] = min;
	maxValue[type] = max;
	defaultValue[type] = defaultvalue;
}
map<valueType, uint16_t> JointValue::minValue;
map<valueType, uint16_t> JointValue::maxValue;
map<valueType, uint16_t> JointValue::defaultValue;
uint16_t JointValue::deltaDefault= 255;

JointValue::JointValue(valueType type, uint16_t value) {
	reset();
	set(value);
	this->type = type;
}
JointValue::JointValue(valueType type) {
	reset();
	this->type = type;
}
JointValue::JointValue() {
	this->type = valueType(IKM_NOT_DEFINED, JointNotDefined);
}
void JointValue::set(uint16_t value) {
	if (inRange(value)) {
		this->value = value;
		valueSet = true;
	}
}
uint16_t JointValue::getValue() {
	if (isSet()) {
		return value;
	}
	// else return value, do a fail safe check here too
	if (defaultValue[type] < -300) {
		ofLogError() << "invalid data";
		return 0;// fail safe
	}
	return defaultValue[type];
}

bool JointValue::inRange(uint16_t value) {
	if (value > maxValue[type] || value < minValue[type]) {
		ofLogError() << "out of range " << value << " (" << minValue[type] << ", " << maxValue[type] << ")";
		return false;
	}
	return true;
}

// needs to only be called one time -- uses static data to save time/space
void JointValue::setup() { 
	reset();

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

	set(valueType(IKM_IK3D_CARTESIAN, gripper), 0, 512, 512);
	set(valueType(IKM_IK3D_CARTESIAN_90, gripper), 0, 512, 512);
	set(valueType(IKM_CYLINDRICAL, gripper), 0, 512, 512);
	set(valueType(IKM_CYLINDRICAL_90, gripper), 0, 512, 512);

	set(valueType(IKM_NOT_DEFINED, JointNotDefined), 0, 0, 0); // should not be set to this while running, only during object init
	
}

void JointValue::reset() {
	valueSet = false;
}
// get pose data from serial port bugbug decode this
void RobotState::readPose() {
	if (serial.available() == 0) {
		return;
	}
	int size = serial.available();
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
int RobotState::readBytesInOneShot(uint8_t *bytes, int bytesMax) {
	int result = 0;
	if (serial.available() > 0) {
		if ((result = serial.readBytes(bytes, bytesMax)) == OF_SERIAL_ERROR) {
			ofLogError() << "serial failed";
			return 0;
		}
		while (result == OF_SERIAL_NO_DATA) {
			result = serial.readBytes(bytes, bytesMax);
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
int RobotState::readBytes(uint8_t *bytes, int bytesRequired) {
	int readIn = 0;
	if (bytes) {
		*bytes = 0;// null out to show data read
		int bytesRemaining = bytesRequired;
		// loop until we've read everything
		while (bytesRemaining > 0) {
			// check for data
			if (serial.available() > 0) {
				// try to read - note offset into the bytes[] array, this is so
				// that we don't overwrite the bytes we already have
				int bytesArrayOffset = bytesRequired - bytesRemaining;
				int result = serial.readBytes(&bytes[bytesArrayOffset],	bytesRemaining);

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
RobotCommandsAndData::RobotCommandsAndData() {
	reset();
}

// bool readOnly -- just read serial do not send request
bool RobotState::ArmIDResponsePacket(uint8_t *bytes) {
	if (bytes != nullptr) {
		armMode = (robotArmMode)bytes[2];
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
		switch (bytes[1]) {
		case 2:
			id = InterbotiXPhantomXReactorArm;
			ofLogNotice() << "InterbotiXPhantomXReactorArm";
			break;
		}
		return true;
	}
	return false;
}
void RobotCommandsAndData::reset() {
	setCommand(NoArmCommand);
	for (auto &location : locations) {
		location.second.reset();
	}
}
void RobotState::setup() {
	
	JointValue jv;
	jv.setup(); // do one time setup
	data[0] = 255;
	serial.listDevices();
	serial.setup(1, 38400);//bugbug get from xml 

	ofLogNotice() << "wait for robot...";

	waitForSerial();
	
	unsigned char bytes[31];
	int readin = readBytesInOneShot(bytes, 5);
	if (readin == 5) {
		ArmIDResponsePacket(bytes);
	}

	// get sign on echo from device
	readin = readBytesInOneShot(bytes, 30);
	bytes[readin] = 0;
	ofLogNotice() << bytes;
	set3DCartesianStraightWristAndGoHome(); // go  to a known state
	setDefaults();
}
void RobotState::update() {
	RobotCommandsAndData data;
	data.setCommand(SignOnDance);
	pushAndClearCommand(data);
	return;
	data.setCommand(EnableArmMovement);
	data.setYout(50);
	path.push(data);
	data.reset();
	data.setYout(350);
	path.push(data);
	return;
	data.reset();
	data.setXleft(10); // 300 max 60 units covers about 3", 20 units is .75" so given the arch its not just inches
	data.setYout(350);
	path.push(data);
	data.reset();
	data.setZup(250);
	data.setXleft(-10); // -300 max
	path.push(data);
	data.reset();
	data.setCommand(DelayArm, 1000);
	path.push(data);
	/*
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

void RobotState::dance() {
	// spin nose
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

}
void RobotState::draw() {
	
	while (!path.empty()) {
		RobotCommandsAndData data;
		
		data = path.front();
		path.pop();

		if (data.getCommand(nullptr) == NoArmCommand) {
			moveXleft(data.get(robotArmJointType::X));
			moveYout(data.get(robotArmJointType::Y));
			moveZup(data.get(robotArmJointType::Z));
			sendNow();
		}
		else {	// process commands
			int64_t value;
			robotCommand cmd = data.getCommand(&value);
			switch (cmd) {
			case HomeArm:
				home();
				break; 
			case SignOnDance:
				dance();
				break;
			case CenterArm:
				center();
				break;
			case DelayArm:
				ofSleepMillis(value);
				break;
			case setArm3DCylindricalStraightWristAndGoHome:
				setStateAndGoHome("set3DCylindricalStraightWristAndGoHome", IKM_CYLINDRICAL, 48);
				break;
			case setArm3DCylindrical90DegreeWristAndGoHome:
				setStateAndGoHome("set3DCylindrical90DegreeWristAndGoHome", IKM_CYLINDRICAL_90, 56);
				break;
			case setArm3DCartesianStraightWristAndGoHome:
				setStateAndGoHome("set3DCartesianStraightWristAndGoHome", IKM_IK3D_CARTESIAN, 32);
				break;
			case setArm3DCartesian90DegreeWristAndGoHome:
				setStateAndGoHome("set3DCartesian90DegreeWristAndGoHome", IKM_IK3D_CARTESIAN_90, 40);
				break;
			case setArmBackhoeJointAndGoHome:
				setBackhoeJointAndGoHome();
				break;
			case EnableArmMovement:
				enableMoveArm(); 
				break;
				
			}
		}
	}
}
void RobotState::sendNow() {
	setSend();
	write();

}
void RobotState::send(const string &s, uint8_t cmd) {
	ofLogNotice() << s;
	set(extValBytesOffset, cmd);
	sendNow(); 
}
void RobotState::center() {
	ofLogNotice() << "center"; //bugbug left off here

	moveXleft(JointValue(valueType(armMode, X)).getMax() / 2);
	moveYout(JointValue(valueType(armMode, Y)).getMax() / 2);
}

void RobotState::echo() {
	for (int i = 0; i < count; ++i) {
		ofLogNotice() << "echo[" << i << "] = " << std::hex << "0x" << (unsigned int)data[i];
	}
}

void RobotState::write() {
	if (sendData) {
		// from http://learn.trossenrobotics.com/arbotix/arbotix-communication-controllers/31-arm-link-reference.html
		uint16_t sum = 0;
		for (int i = xHighByteOffset; i <= extValBytesOffset; ++i) {
			sum += data[i];
		}
		uint8_t invertedChecksum = sum % 256;//isolate the lowest byte 8 or 16?

		data[checksum] = 255 - invertedChecksum; //invert value to get file checksum

		echo();
		
		//If you are sending packets at an interval, do not send them faster than 30hz(one packet every 33ms).
		 // no need to hurry packets so just want the minimum amount no matter what
		int sent = serial.writeBytes(data, count);

		ofLogNotice() << "draw, count = " << count << ", sent = " << sent;

		readPose(); // pose is sent all the time
		readPose(); // how often are two sent?

		// once it draws set to clean
		setSend(false);
	}
}

void RobotState::moveXleft(JointValue& x, bool send) {
	if (x.isSet()) {
		//The parameters X and WristAngle both can have negative values.However all of the values transmitted via the 
		///ArmControl serial packet are unsigned bytes that are converted into unsigned integers - that is, they can only have positive values.
		//To compensate for this fact, X and WristAngle are transmitted as offset values.
		//Add 512 to X to obtain the value to transmit over Arm Link.
		ofLogNotice() << "moveXLeft " << x.getValue() + 512;
		set(xHighByteOffset, xLowByteOffset, x.getValue() + 512);
		if (send) {
			sendNow();
		}

	}
	
	//backhoe not supported for X
}
void RobotState::moveYout(JointValue& y, bool send) {
	if (y.isSet()) {
		ofLogNotice() << "moveYout " << y.getValue();
		set(yHighByteOffset, yLowByteOffset, y.getValue());
		if (send) {
			sendNow();
		}
	}
}
void RobotState::moveZup(JointValue& z, bool send) {
	if (z.isSet()) {
		ofLogNotice() << "moveZup " << z.getValue();
		set(zHighByteOffset, zLowByteOffset, z.getValue());
		if (send) {
			sendNow();
		}
	}
}

void RobotState::setWristAngledown(JointValue& a, bool send) {
	if (a.isSet()) {
		ofLogNotice() << "setWristAngledown " << a.getValue() + 90;
		//The parameters X and WristAngle both can have negative values.However all of the values 
		//transmitted via the ArmControl serial packet are unsigned bytes that are converted into unsigned integers - that is, they can only have positive values.
		//To compensate for this fact, X and WristAngle are transmitted as offset values.
		//Add 90 to Wrist Angle to obtain the value to transmit over Arm Link. 
		set(wristAngleHighByteOffset, wristAngleLowByteOffset, a.getValue() + 90);
		if (send) {
			sendNow();
		}
	}
}
void RobotState::setWristRotate(JointValue& a, bool send) {
	if (a.isSet()) {
		ofLogNotice() << "setWristRotate " << a.getValue();
		set(wristRotateHighByteOffset, wristRotateLowByteOffset, a.getValue());
		if (send) {
			sendNow();
		}
	}
}
// 0 close, 512 fully open
void RobotState::openGripper(JointValue& distance, bool send) {
	if (distance.isSet()) {
		ofLogNotice() << "openGripper " << distance.getValue();
		set(gripperHighByteOffset, gripperLowByteOffset, distance.getValue());
		if (send) {
			sendNow();
		}
	}
}
void RobotState::centerAllServos() {
	ofLogNotice() << "centerAllServos";
	setBackhoeJointAndGoHome();
	moveXleft(JointValue(valueType(IKM_BACKHOE, X), 512));
	moveYout(512);
	moveZup(512);
	setWristAngledown(512);
	setWristRotate(512);
	openGripper(512);
	setSpeed(128);
	sendNow();
}
// "home" and set data matching state
void RobotState::setDefaults() {
	ofLogNotice() << "setDefaults";
	set(xHighByteOffset, xLowByteOffset, JointValue(valueType(armMode, X)).getDefaultValue()+512);
	set(yHighByteOffset, yLowByteOffset, JointValue(valueType(armMode, Y)).getDefaultValue()); //150
	set(zHighByteOffset, zLowByteOffset, JointValue(valueType(armMode, Z)).getDefaultValue()); //150
	set(wristAngleHighByteOffset, wristAngleLowByteOffset, JointValue(valueType(armMode, wristAngle)).getDefaultValue() + 90);
	set(wristRotateHighByteOffset, wristRotateLowByteOffset, JointValue(valueType(armMode, wristRotate)).getDefaultValue()); // 512
	set(gripperHighByteOffset, gripperLowByteOffset, JointValue(valueType(armMode, gripper)).getDefaultValue()); //0
	set(buttonByteOffset, 0);
	set(deltaValBytesOffset, JointValue::getDelta());
	set(extValBytesOffset, 0);

}
// set basic data that moves a little bit after starting up
void RobotState::sanityTest() {
	ofLogNotice() << "sanityTest";
	set(xHighByteOffset, xLowByteOffset, 512);
	set(yHighByteOffset, yLowByteOffset, 150);
	set(zHighByteOffset, zLowByteOffset, 150);
	set(wristAngleHighByteOffset, wristAngleLowByteOffset, 90);
	set(wristRotateHighByteOffset, wristRotateLowByteOffset, 512);
	set(gripperHighByteOffset, gripperLowByteOffset, 0);
	set(buttonByteOffset, 0);
	set(deltaValBytesOffset, 128);
	set(extValBytesOffset, 0);
}
void RobotState::reset() { 
	while (!path.empty()) { 
		path.pop(); 
	} 
};
// will block until arm is ready
void RobotState::setStateAndGoHome(const string& s, robotArmMode mode, uint8_t cmd) {
	reset();
	ofLogNotice() << s;
	armMode = mode;
	set(extValBytesOffset, cmd);//bugbug what should the data be?
	sendNow();
	
}
void RobotState::set(uint16_t high, uint16_t low, uint16_t val) {
	ofLogNotice() << "RobotState::set (" << high << "," << low << ")" << val;
	set(high, highByte(val));
	set(low, lowByte(val));
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
