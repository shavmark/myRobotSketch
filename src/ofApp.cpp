#include "ofApp.h"
#include <algorithm> 

RobotLocation::RobotLocation() {
	x.second = false;
	y.second = false;
	z.second = false;
	wristAngle.second = false;
	wristRotate.second = false;
	gripper.second = false;
	distance.second = false;
}
void RobotState::setup() {
	serial.listDevices();
	serial.setup(1, 38400);//bugbug get from xml 
	data[0] = 0xFF; // header, byte 0
	setSpeed(255);
	set3DCartesianStraightWristAndGoHome();
	sendNow();//bugbug make list of items to draw with delays etc
	setDefaults();
	moveArm();
	RobotLocation loc;
	loc.moveYout(260);
	loc.moveZup(250);
	path.push_back(loc);
	loc.moveXleft(-212);
	path.push_back(loc);
	path.push_back(loc);
	loc.moveXleft(200);
	while(1) {
		//bugbug make helper function
		for (int i = 0; i < path.size(); ++i) {
			if (path[i].x.second) {
				moveXleft(path[i].x.first);
			}
			if (path[i].y.second) {
				moveYout(path[i].y.first);
			}
			if (path[i].z.second) {
				moveZup(path[i].z.first);
			}
			sendNow();
		}
		Sleep(1000);
	}
}
void RobotState::sendNow() {
	setSend();
	write();
}
void RobotState::home() {
	set(extValBytesOffset, 80);
	sendNow();
}
void RobotState::home90() {
	set(extValBytesOffset, 88);
	sendNow();
}
void RobotState::sleepArm() {
	set(extValBytesOffset, 96);
	sendNow();
}
void RobotState::emergencyStop() {
	set(extValBytesOffset, 17);
	sendNow();
}
void RobotState::moveArm() {
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
bool RobotState::inRange(int32_t low90, int32_t high90, int32_t low, int32_t high, int32_t value) {
	// bugbug only Cartesian Positioning  supported right now
	if (in90) {
		if (value > high90 || value < low90) {
			ofLogError() << "out of range " << value << " (" << low90 << ", " << high90 << ")";
			return false;
		}
	}
	else {
		if (value > high || value < low) {
			ofLogError() << "out of range " << value << " (" << low90 << ", " << high90 << ")";
			return false;
		}
	}
	return true;
}

void RobotState::moveXleft(int32_t x, bool send) {
	ofLogNotice() << "moveXLeft " << x;
	//The parameters X and WristAngle both can have negative values.However all of the values transmitted via the 
	///ArmControl serial packet are unsigned bytes that are converted into unsigned integers - that is, they can only have positive values.
	//To compensate for this fact, X and WristAngle are transmitted as offset values.
	//Add 512 to X to obtain the value to transmit over Arm Link.
	if (armMode == Cartesian) {
		if (!inRange(-300, 300, -300, 300, x)) {
			return;
		}
	}
	else if (armMode == Cylindrical) {
		if (!inRange(0, 1023, 0, 1023, x)) {
			return;
		}
	}
	set(xHighByteOffset, xLowByteOffset, 512 + x);
	if (send) {
		sendNow();
	}
	//backhoe not supported for X
}
void RobotState::moveYout(uint16_t y, bool send) {
	ofLogNotice() << "moveYout " << y;
	if (inRange(20, 150, 50, 350, y)) {
		set(yHighByteOffset, yLowByteOffset, y);
		if (send) {
			sendNow();
		}
	}
}
void RobotState::moveZup(uint16_t z, bool send) {
	ofLogNotice() << "moveZup " << z;
	if (inRange(10, 150, 20, 250,  z)) {
		set(zHighByteOffset, zLowByteOffset, z);
		if (send) {
			sendNow();
		}
	}
}
int RobotState::getDefault(int32_t int90, int32_t intStraight) {
	if (in90) {
		return int90;
	}
	return intStraight;
}
int RobotState::getDefault(int32_t cart90, int32_t cart, int32_t cyn90, int32_t cyn) {
	if (armMode == Cartesian) {
		if (in90) {
			return cart90;
		}
		return cart;
	}
	if (armMode == Cylindrical) {
		if (in90) {
			return cyn90;
		}
		return cyn;
	}
}

//The parameters X and WristAngle both can have negative values.However all of the values 
//transmitted via the ArmControl serial packet are unsigned bytes that are converted into unsigned integers - that is, they can only have positive values.
//To compensate for this fact, X and WristAngle are transmitted as offset values.
//Add 90 to Wrist Angle to obtain the value to transmit over Arm Link. 
void RobotState::setWristAngledown(int32_t a, bool send) {
	ofLogNotice() << "setWristAngledown " << a;
	if (inRange(-90, -45, -30, 30, a)) {
		set(wristAngleHighByteOffset, wristAngleLowByteOffset, 90+a);
		if (send) {
			sendNow();
		}
	}
}
void RobotState::setWristRotate(int32_t a, bool send) {
	ofLogNotice() << "setWristRotate " << a;
	if (inRange(0, 1023, 0, 1023, a)) {
		set(wristRotateHighByteOffset, wristRotateLowByteOffset, 512);
		if (send) {
			sendNow();
		}
	}
}
// 0 close, 512 fully open
void RobotState::openGripper(uint16_t distance, bool send) {
	ofLogNotice() << "openGripper " << distance;
	set(gripperHighByteOffset, gripperLowByteOffset, 0);
	if (send) {
		sendNow();
	}
}
void RobotState::centerAllServos() {
	setBackhoeJointAndGoHome();
	moveXleft(512);
	moveYout(512);
	moveZup(512);
	setWristAngledown(getDefault(-90,0));
	setWristRotate(512);
	openGripper(512);
	setSpeed(128);

}
void RobotState::setDefaults() {
	ofLogNotice() << "setDefaults";
	moveArm();
	moveXleft(getDefault(0,0,512,512));
	moveYout(getDefault(140, 235, 140, 235));
	moveZup(getDefault(30, 210, 30, 210));
	setWristAngledown(getDefault(-90, 0));
	setWristRotate(getDefault(515, 512));
	openGripper();
	setSpeed(); // slow is 255, 10 is super fast
	set(buttonByteOffset, 0);
	set(extValBytesOffset, 0);
}
void RobotState::set3DCylindricalStraightWristAndGoHome() {
	ofLogNotice() << "set3DCylindricalStraightWristAndGoHome";
	in90 = false;
	armMode = Cylindrical;
	set(extValBytesOffset, 48);
}

void RobotState::set3DCartesian90DegreeWristAndGoHome() {
	ofLogNotice() << "set3DCartesian90DegreeWristAndGoHome";
	in90 = true;
	armMode = Cartesian;
	set(extValBytesOffset, 40);
}
//Set 3D Cartesian mode / straight wrist and go to home
void RobotState::set3DCartesianStraightWristAndGoHome() {
	ofLogNotice() << "set3DCartesianStraightWristAndGoHome";
	in90 = false;
	armMode = Cartesian;
	set(extValBytesOffset, 32);
}
void RobotState::set3DCylindrical90DegreeWristAndGoHome() {
	ofLogNotice() << "set3DCylindrical90DegreeWristAndGoHome";
	in90 = true;
	armMode = Cylindrical;
	set(extValBytesOffset, 56);
}
//backhoe not fully supported
void RobotState::setBackhoeJointAndGoHome() {
	ofLogNotice() << "setBackhoeJointAndGoHome";
	armMode = Backhoe;
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

}

//--------------------------------------------------------------
void ofApp::draw(){
	
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
