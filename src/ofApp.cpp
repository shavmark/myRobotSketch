#include "ofApp.h"

void RobotMotionData::setup() {
	serial.listDevices();
	serial.setup(1, 38400);//bugbug get from xml 
	data[0] = 0xFF; // header, byte 0
	set3DCartesianStraightWristAndGoHome();
	setSend();
	draw();//bugbug make list of items to draw with delays etc
	home();
	setSend();
	draw();
	setY(200);
	echo();
	setSend();
	draw();
}

void RobotMotionData::echo() {
	for (int i = 0; i < count; ++i) {
		ofLogNotice() << "echo[" << i << "] = " << std::hex << "0x" << (unsigned int)data[i];
	}
}

void RobotMotionData::draw() {
	if (sendData) {
		// from http://learn.trossenrobotics.com/arbotix/arbotix-communication-controllers/31-arm-link-reference.html
		uint16_t sum=0;
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
	set(deltaValBytesOffset, speed);
}
void RobotMotionData::setX(uint16_t x) {
	ofLogNotice() << "setX " << x;
	set(xHighByteOffset, xLowByteOffset, x);
}
void RobotMotionData::setY(uint16_t y) {
	ofLogNotice() << "setY " << y;
	set(yHighByteOffset, yLowByteOffset, y);
}
// home 0xff 0x2 0x0 0x0 0x96 0x0 0x96 0x0 0x5a 0x2 0x0 0x1 0x0 0x80 0x0 0x0 0xf4
void RobotMotionData::home() {
	ofLogNotice() << "home";

	setX();
	setY();
	set(zHighByteOffset, zLowByteOffset, 150);
	set(wristAngleHighByteOffset, wristAngleLowByteOffset, 90);
	set(wristRotateHighByteOffset, wristRotateLowByteOffset, 512);
	set(gripperHighByteOffset, gripperLowByteOffset, 0);
	setSpeed(); // slow is 255, 10 is super fast
	set(buttonByteOffset, 0);
	set(extValBytesOffset, 0);
}
void RobotMotionData::set3DCartesian90DegreeWristAndGoHome() {
	ofLogNotice() << "set3DCartesian90DegreeWristAndGoHome";
	set(extValBytesOffset, 40);
}
//Set 3D Cartesian mode / straight wrist and go to home
void RobotMotionData::set3DCartesianStraightWristAndGoHome() {
	ofLogNotice() << "set3DCartesianStraightWristAndGoHome";
	set(extValBytesOffset, 32);
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
