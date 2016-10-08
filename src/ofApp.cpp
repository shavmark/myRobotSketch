#include "ofApp.h"
#include <algorithm> 
void RobotMotionData::setup() {
	serial.listDevices();
	serial.setup(1, 38400);//bugbug get from xml 
	data[0] = 0xFF; // header, byte 0
	set3DCartesianStraightWristAndGoHome();
	sendNow();//bugbug make list of items to draw with delays etc
	home();
	setDefaults();
	moveArm();
	moveYout(200);
	echo();
	//sendNow(); 
	moveZup(200);
	echo();
	sendNow();
}
void RobotMotionData::sendNow() {
	setSend();
	draw();
}
void RobotMotionData::home() {
	set(extValBytesOffset, 80);
	sendNow();
}
void RobotMotionData::home90() {
	set(extValBytesOffset, 88);
	sendNow();
}
void RobotMotionData::sleepArm() {
	set(extValBytesOffset, 96);
	sendNow();
}
void RobotMotionData::emergencyStop() {
	set(extValBytesOffset, 17);
	sendNow();
}
void RobotMotionData::moveArm() {
	set(extValBytesOffset, 0);
}

void RobotMotionData::echo() {
	for (int i = 0; i < count; ++i) {
		ofLogNotice() << "echo[" << i << "] = " << std::hex << "0x" << (unsigned int)data[i];
	}
}

void RobotMotionData::draw() {
	if (sendData) {
		// from http://learn.trossenrobotics.com/arbotix/arbotix-communication-controllers/31-arm-link-reference.html
		uint16_t sum = 0;
		for (int i = xHighByteOffset; i <= extValBytesOffset; ++i) {
			sum += data[i];
		}
		uint16_t invertedChecksum = sum % 256;//isolate the lowest byte

		data[checksum] = 255 - (sum % 256); //invert value to get file checksum
		int sent = serial.writeBytes(data, count);

		ofLogNotice() << "draw, count = " << count << ", sent = " << sent;

		// once it draws set to clean
		setSend(false);
	}
}
// 10 super fast, 255 slow
void RobotMotionData::setSpeed(uint8_t speed) {
	set(deltaValBytesOffset, min(speed, (uint8_t)254));
}
bool RobotMotionData::inRange(int low90, int high90, int low, int high, int value) {
	// bugbug only Cartesian Positioning  supported right now
	if (in90) {
		if (value > high90 || value < low90) {
			ofLogError() << "out of range" << value;
			return false;
		}
	}
	else {
		if (value > high || value < low) {
			ofLogError() << "out of range" << value;
			return false;
		}
	}
	return true;
}

void RobotMotionData::moveXLeft(int x) {
	ofLogNotice() << "moveXLeft " << x;
	//The parameters X and WristAngle both can have negative values.However all of the values transmitted via the 
	///ArmControl serial packet are unsigned bytes that are converted into unsigned integers - that is, they can only have positive values.
	//To compensate for this fact, X and WristAngle are transmitted as offset values.
	//Add 512 to X to obtain the value to transmit over Arm Link.
	if (armMode == Cartesian) {
		if (inRange(-300, 300, -300, 300, x)) {
			set(xHighByteOffset, xLowByteOffset, 512+x);
		}
	}
	else if (armMode == Cylindrical) {
		if (inRange(0, 1023, 0, 1023, x)) {
			set(xHighByteOffset, xLowByteOffset, 512 + x);
		}
	}
	//backhoe not supported for X
}
void RobotMotionData::moveYout(uint16_t y) {
	ofLogNotice() << "moveYout " << y;
	if (inRange(20, 150, 50, 350, y)) {
		set(yHighByteOffset, yLowByteOffset, y);
	}
}
void RobotMotionData::moveZup(uint16_t z) {
	ofLogNotice() << "moveZup " << z;
	if (inRange(10, 150, 20, 250,  z)) {
		set(zHighByteOffset, zLowByteOffset, z);
	}
}

//The parameters X and WristAngle both can have negative values.However all of the values 
//transmitted via the ArmControl serial packet are unsigned bytes that are converted into unsigned integers - that is, they can only have positive values.
//To compensate for this fact, X and WristAngle are transmitted as offset values.
//Add 90 to Wrist Angle to obtain the value to transmit over Arm Link. 
void RobotMotionData::setWristAngledown(int a) {
	ofLogNotice() << "setWristAngledown " << a;
	if (inRange(-90, -45, -30, 30, a)) {
		set(wristAngleHighByteOffset, wristAngleLowByteOffset, 90+a);
	}
}
void RobotMotionData::setWristRotate(int a) {
	ofLogNotice() << "setWristRotate " << a;
	if (inRange(0, 1023, 0, 1023, a)) {
		set(wristRotateHighByteOffset, wristRotateLowByteOffset, 512);
	}
}
// 0 close, 512 fully open
void RobotMotionData::openGripper(uint16_t distance) {
	ofLogNotice() << "openGripper " << distance;
	set(gripperHighByteOffset, gripperLowByteOffset, 0);
}
void RobotMotionData::centerAllServos() {
	setBackhoeJointAndGoHome();
	moveXLeft(512);
	moveYout(512);
	moveZup(512);
	setWristAngledown(512);
	setWristRotate(512);
	openGripper(512);
	setSpeed(128);

}
void RobotMotionData::setDefaults() {
	ofLogNotice() << "setDefaults";
	moveArm();
	moveXLeft();
	moveYout();
	moveZup();
	setWristAngledown();
	setWristRotate();
	openGripper();
	setSpeed(); // slow is 255, 10 is super fast
	set(buttonByteOffset, 0);
	set(extValBytesOffset, 0);
}
void RobotMotionData::set3DCylindricalStraightWristAndGoHome() {
	ofLogNotice() << "set3DCylindricalStraightWristAndGoHome";
	in90 = false;
	armMode = Cylindrical;
	set(extValBytesOffset, 48);
}

void RobotMotionData::set3DCartesian90DegreeWristAndGoHome() {
	ofLogNotice() << "set3DCartesian90DegreeWristAndGoHome";
	in90 = true;
	armMode = Cartesian;
	set(extValBytesOffset, 40);
}
//Set 3D Cartesian mode / straight wrist and go to home
void RobotMotionData::set3DCartesianStraightWristAndGoHome() {
	ofLogNotice() << "set3DCartesianStraightWristAndGoHome";
	in90 = false;
	armMode = Cartesian;
	set(extValBytesOffset, 32);
}
void RobotMotionData::set3DCylindrical90DegreeWristAndGoHome() {
	ofLogNotice() << "set3DCylindrical90DegreeWristAndGoHome";
	in90 = true;
	armMode = Cylindrical;
	set(extValBytesOffset, 56);
}
//backhoe not fully supported
void RobotMotionData::setBackhoeJointAndGoHome() {
	ofLogNotice() << "setBackhoeJointAndGoHome";
	armMode = Backhoe;
	set(extValBytesOffset, 64);
}

void RobotMotionData::set(uint16_t high, uint16_t low, uint16_t val) {
	set(high, highByte(val));
	set(low, lowByte(val));
}


//--------------------------------------------------------------
void ofApp::setup(){
	robot.setup();
}

//--------------------------------------------------------------
void ofApp::update(){

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
