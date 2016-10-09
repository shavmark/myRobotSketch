#include "ofApp.h"
#include <algorithm> 

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
RobotLocation::RobotLocation() {
	reset();
}

bool RobotState::ArmIDResponsePacket() {
	// see what occured
	set(extValBytesOffset, 112);
	sendNow();
	unsigned char bytes[5];
	if (readBytes(bytes, 5) == 5) {
		armMode = (enum mode)bytes[2];
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
		case IKM_BACKHOE:
			ofLogNotice() << "arm mode IKM_BACKHOE";
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
void RobotLocation::reset() {
	setNoCommand();
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
	sendNow();
	ArmIDResponsePacket();
	setDefaults();
}
void RobotState::update() {
	RobotLocation loc;
	
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
		RobotLocation loc;
		
		loc = path.front();
		path.pop();

		if (loc.cmd.first == RobotLocation::None) {
			if (loc.x.second) {
				moveXleft(loc.x.first);
			}
			if (loc.y.second) {
				moveYout(loc.y.first);
			}
			if (loc.z.second) {
				moveZup(loc.z.first);
			}
			sendNow();
		}
		else {	// process commands
			if (loc.cmd.first == RobotLocation::Home) {
				home();
			}
			else if (loc.cmd.first == RobotLocation::Delay) {
				ofSleepMillis(loc.cmd.second);
			}
		}
		
	}


}
void RobotState::sendNow() {
	setSend();
	write();
}
void RobotState::home() {
	if (armMode == IKM_IK3D_CARTESIAN_90 || armMode == IKM_CYLINDRICAL_90) {
		set(extValBytesOffset, 88);
	}
	else {
		set(extValBytesOffset, 80);
	}
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
bool RobotState::inRange(int32_t low90, int32_t high90, int32_t low, int32_t high, int32_t value) {
	// bugbug only Cartesian Positioning  supported right now
	
	if (armMode == IKM_IK3D_CARTESIAN_90 || armMode == IKM_CYLINDRICAL_90){
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
	ofLogNotice() << "moveXLeft " << x+512;
	//The parameters X and WristAngle both can have negative values.However all of the values transmitted via the 
	///ArmControl serial packet are unsigned bytes that are converted into unsigned integers - that is, they can only have positive values.
	//To compensate for this fact, X and WristAngle are transmitted as offset values.
	//Add 512 to X to obtain the value to transmit over Arm Link.
	if (armMode == IKM_IK3D_CARTESIAN) {
		if (!inRange(-300, 300, -300, 300, x)) {
			return;
		}
	}
	else if (armMode == IKM_CYLINDRICAL) {
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
	if (armMode == IKM_IK3D_CARTESIAN_90 || armMode == IKM_CYLINDRICAL_90) {
		return int90;
	}
	return intStraight;
}
int RobotState::getDefault(int32_t cart90, int32_t cart, int32_t cyn90, int32_t cyn) {
	switch (armMode) {
	case IKM_IK3D_CARTESIAN:
		return cart;
	case IKM_IK3D_CARTESIAN_90:
		return cart90;
	case IKM_CYLINDRICAL:
		return cyn;
	case IKM_CYLINDRICAL_90:
		return cyn90;
	case IKM_BACKHOE:
		break;
	}
	return -1;
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
	ofLogNotice() << "centerAllServos";
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
	enableMoveArm();
	moveXleft(getDefault(0,0,512,512));
	moveYout(getDefault(140, 235, 140, 235));
	moveZup(getDefault(30, 210, 30, 210));
	setWristAngledown(getDefault(-90, 0));
	setWristRotate(getDefault(515, 512));
	openGripper();
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
