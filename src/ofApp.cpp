#include "ofApp.h"
#include <algorithm> 
void JointValue::set(valueType type, int32_t min, int32_t max, int32_t defaultvalue) {
	minValue[type] = min;
	maxValue[type] = max;
	defaultValue[type] = defaultvalue;
}
map<valueType, int32_t> JointValue::minValue;
map<valueType, int32_t> JointValue::maxValue;
map<valueType, int32_t> JointValue::defaultValue;
int32_t JointValue::deltaDefault;

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

	deltaDefault = 255;
}

void JointValue::reset() {
	valueSet = false;
}
int RobotState::readBytes(unsigned char *bytes, int bytesRequired) {
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
RobotCommands::RobotCommands() {
	reset();
}

bool RobotState::ArmIDResponsePacket() {
	// see what occured
	set(extValBytesOffset, 112);
	sendNow();
	unsigned char bytes[5];
	if (readBytes(bytes, 5) == 5) {
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
void RobotCommands::reset() {
	setNoCommand();
	locations[robotArmJointType::X].reset();
	locations[robotArmJointType::Y].reset();
	locations[robotArmJointType::Z].reset();
	locations[robotArmJointType::wristAngle].reset();
	locations[robotArmJointType::wristRotate].reset();
	locations[robotArmJointType::delta].reset();
}
void RobotState::setup() {
	serial.listDevices();
	serial.setup(1, 38400);//bugbug get from xml 
	data[0] = 0xFF; // header, byte 0
	setSpeed(255);
	set3DCartesianStraightWristAndGoHome();
	sendNow();
	ArmIDResponsePacket();
	setDefaults();
}
void RobotState::update() {
	RobotCommands loc;
	
	loc.setHome();
	loc.reset();
	enableMoveArm();
	loc.moveYout(50);
	path.push(loc);
	loc.reset();
	loc.moveYout(350);
	path.push(loc);
	return;
	loc.reset();
	loc.moveXleft(10); // 300 max 60 units covers about 3", 20 units is .75" so given the arch its not just inches
	loc.moveYout(350);
	path.push(loc);
	loc.reset();
	loc.moveZup(250);
	loc.moveXleft(-10); // -300 max
	path.push(loc);
	loc.reset();
	loc.setDelay(1000);
	path.push(loc);
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
void RobotState::draw() {
	
	while (!path.empty()) {
		RobotCommands loc;
		
		loc = path.front();
		path.pop();

		if (loc.getCommand(nullptr) == RobotCommands::None) {
			moveXleft(loc.get(robotArmJointType::X));
			moveYout(loc.get(robotArmJointType::Y));
			moveZup(loc.get(robotArmJointType::Z));
			sendNow();
		}
		else {	// process commands
			int64_t value;
			if (loc.getCommand(nullptr) == RobotCommands::Home) {
				home();
			}
			if (loc.getCommand(nullptr) == RobotCommands::Center) {
				center();
			}
			else if (loc.getCommand(&value) == RobotCommands::Delay) {
				ofSleepMillis(value);
			}
		}
		
	}
	
}
void RobotState::sendNow() {
	setSend();
	write();
}
void RobotState::home() {
	ofLogNotice() << "home";
	if (is90()) {
		set(extValBytesOffset, 88);
	}
	else {
		set(extValBytesOffset, 80);
	}
	sendNow();
}
void RobotState::center() {
	ofLogNotice() << "center";
	if (is90()) {
		set(extValBytesOffset, 88);
	}
	else {
		set(extValBytesOffset, 80);
	}
	sendNow();
}
void RobotState::sleepArm() {
	ofLogNotice() << "sleepArm";
	set(extValBytesOffset, 96);
	sendNow();
}
void RobotState::emergencyStop() {
	ofLogNotice() << "emergencyStop";
	set(extValBytesOffset, 17);
	sendNow();
}
void RobotState::enableMoveArm() {
	set(extValBytesOffset, 0);
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
		uint16_t invertedChecksum = sum % 256;//isolate the lowest byte

		data[checksum] = 255 - (sum % 256); //invert value to get file checksum

		echo();
		
		//If you are sending packets at an interval, do not send them faster than 30hz(one packet every 33ms).
		ofSleepMillis(33); // no need to hurry packets so just want the minimum amount no matter what
		int sent = serial.writeBytes(data, count);

		ofLogNotice() << "draw, count = " << count << ", sent = " << sent;

		// once it draws set to clean
		setSend(false);
	}
}
// 10 super fast, 255 slow
void RobotState::setSpeed(uint8_t speed) {
	set(deltaValBytesOffset, min(speed, (uint8_t)254));
}

void RobotState::moveXleft(JointValue& x, bool send) {
	//The parameters X and WristAngle both can have negative values.However all of the values transmitted via the 
	///ArmControl serial packet are unsigned bytes that are converted into unsigned integers - that is, they can only have positive values.
	//To compensate for this fact, X and WristAngle are transmitted as offset values.
	//Add 512 to X to obtain the value to transmit over Arm Link.
	ofLogNotice() << "moveXLeft " << x + 512;
	set(xHighByteOffset, xLowByteOffset, x+512);
	if (send) {
		sendNow();
	}
	
	//backhoe not supported for X
}
void RobotState::moveYout(JointValue& y, bool send) {
	ofLogNotice() << "moveYout " << y.getValue();
	set(yHighByteOffset, yLowByteOffset, y.getValue());
	if (send) {
		sendNow();
	}
}
void RobotState::moveZup(JointValue& z, bool send) {
	ofLogNotice() << "moveZup " << z.getValue();
	set(zHighByteOffset, zLowByteOffset, z.getValue());
	if (send) {
		sendNow();
	}
}

void RobotState::setWristAngledown(JointValue& a, bool send) {
	ofLogNotice() << "setWristAngledown " << a.getValue()+90;
	//The parameters X and WristAngle both can have negative values.However all of the values 
	//transmitted via the ArmControl serial packet are unsigned bytes that are converted into unsigned integers - that is, they can only have positive values.
	//To compensate for this fact, X and WristAngle are transmitted as offset values.
	//Add 90 to Wrist Angle to obtain the value to transmit over Arm Link. 
	set(wristAngleHighByteOffset, wristAngleLowByteOffset, a + 90);
	if (send) {
		sendNow();
	}
}
void RobotState::setWristRotate(JointValue& a, bool send) {
	ofLogNotice() << "setWristRotate " << a.getValue();
	set(wristRotateHighByteOffset, wristRotateLowByteOffset, a.getValue());
	if (send) {
		sendNow();
	}
}
// 0 close, 512 fully open
void RobotState::openGripper(JointValue& distance, bool send) {
	ofLogNotice() << "openGripper " << distance.getValue();
	set(gripperHighByteOffset, gripperLowByteOffset, distance.getValue());
	if (send) {
		sendNow();
	}
}
void RobotState::centerAllServos() {
	ofLogNotice() << "centerAllServos";
	setBackhoeJointAndGoHome();
	moveXleft(JointValue(valueType(IKM_BACKHOE, X), 512));
	moveYout(512);
	moveZup(512);
	setWristAngledown(getDefault(-90,0));
	setWristRotate(512);
	openGripper(512);
	setSpeed(128);

}
void RobotState::setDefaults() {
	ofLogNotice() << "setDefaults";
	// default to IKM_IK3D_CARTESIAN
	moveXleft(JointValue(valueType(IKM_IK3D_CARTESIAN, X)));
	moveYout(JointValue(valueType(IKM_IK3D_CARTESIAN, Y)));
	moveZup(JointValue(valueType(IKM_IK3D_CARTESIAN, Z)));
	setWristAngledown(JointValue(valueType(IKM_IK3D_CARTESIAN, wristAngle)));
	setWristRotate(JointValue(valueType(IKM_IK3D_CARTESIAN, wristRotate)));
	openGripper(JointValue(valueType(IKM_IK3D_CARTESIAN, gripper)));
	setSpeed(); // slow is 255, 10 is super fast
	set(buttonByteOffset, 0);
	set(extValBytesOffset, 0);
	enableMoveArm();
}
void RobotState::set3DCylindricalStraightWristAndGoHome() {
	ofLogNotice() << "set3DCylindricalStraightWristAndGoHome";
	armMode = IKM_CYLINDRICAL;
	set(extValBytesOffset, 48);
}

void RobotState::set3DCartesian90DegreeWristAndGoHome() {
	ofLogNotice() << "set3DCartesian90DegreeWristAndGoHome";
	armMode = IKM_IK3D_CARTESIAN_90;
	set(extValBytesOffset, 40);
}
//Set 3D Cartesian mode / straight wrist and go to home
void RobotState::set3DCartesianStraightWristAndGoHome() {
	ofLogNotice() << "set3DCartesianStraightWristAndGoHome";
	armMode = IKM_IK3D_CARTESIAN;
	set(extValBytesOffset, 32);
}
void RobotState::set3DCylindrical90DegreeWristAndGoHome() {
	ofLogNotice() << "set3DCylindrical90DegreeWristAndGoHome";
	armMode = IKM_CYLINDRICAL_90;
	set(extValBytesOffset, 56);
}
//backhoe not fully supported
void RobotState::setBackhoeJointAndGoHome() {
	ofLogNotice() << "setBackhoeJointAndGoHome";
	armMode = IKM_BACKHOE;
	set(extValBytesOffset, 64);
}

void RobotState::set(uint16_t high, uint16_t low, uint16_t val) {
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
