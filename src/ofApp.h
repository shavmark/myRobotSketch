#pragma once

#include "ofMain.h"

class RobotMotionData {
public:

	void setup();
	
	void draw();

	// home 0xff 0x2 0x0 0x0 0x96 0x0 0x96 0x0 0x5a 0x2 0x0 0x1 0x0 0x80 0x0 0x0 0xf4
	void home();
	//Set 3D Cartesian mode / straight wrist and go to home
	void set3DCartesianStraightWristAndGoHome();
	void set3DCartesian90DegreeWristAndGoHome();
	void setX(uint16_t x= 512);
	void setY(uint16_t y = 150);
	void echo();
	void setSpeed(uint8_t speed = 128);
private:
	// offsets
	static const uint16_t xHighByteOffset = 1;
	static const uint16_t xLowByteOffset = 2;
	static const uint16_t yHighByteOffset = 3;
	static const uint16_t yLowByteOffset = 4;
	static const uint16_t zHighByteOffset = 5;
	static const uint16_t zLowByteOffset = 6;
	static const uint16_t wristAngleHighByteOffset = 7;
	static const uint16_t wristAngleLowByteOffset = 8;
	static const uint16_t wristRotateHighByteOffset = 9;
	static const uint16_t wristRotateLowByteOffset = 10;
	static const uint16_t gripperHighByteOffset = 11;
	static const uint16_t gripperLowByteOffset = 12;
	static const uint16_t deltaValBytesOffset = 13;
	static const uint16_t buttonByteOffset = 14;
	static const uint16_t extValBytesOffset = 15;
	static const uint16_t checksum = 16;
	static const uint16_t count = 17;
	void set(uint16_t high, uint16_t low, uint16_t val);
	void set(uint16_t offset, uint8_t b) { data[offset] = b; }
	void setSend(bool b = true) { sendData = b; }
	uint8_t lowByte(uint16_t a) { return a % 256; }
	uint8_t highByte(uint16_t a) { return (a / 256) % 256; }

	uint8_t data[count];
	ofSerial serial;

	bool sendData = false; // only send data once

};
class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);

		RobotMotionData robot;
		
};
