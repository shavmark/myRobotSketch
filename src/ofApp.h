#pragma once

#include "ofMain.h"

class RobotMotionData {
public:

	//http://learn.trossenrobotics.com/arbotix/arbotix-communication-controllers/31-arm-link-reference.html
	void setup();
	void draw();

	void home();
	void home90();
	void moveArm();

	// home 0xff 0x2 0x0 0x0 0x96 0x0 0x96 0x0 0x5a 0x2 0x0 0x1 0x0 0x80 0x0 0x0 0xf4
	void setDefaults();
	//Set 3D Cartesian mode / straight wrist and go to home
	void set3DCartesianStraightWristAndGoHome();
	void set3DCartesian90DegreeWristAndGoHome();
	void set3DCylindricalStraightWristAndGoHome();
	void set3DCylindrical90DegreeWristAndGoHome();
	void setBackhoeJointAndGoHome();
	void centerAllServos();
	void emergencyStop();
	void sleepArm();

	void moveXLeft(int x= 0);
	void moveYout(uint16_t y = 235);
	void moveZup(uint16_t z = 210);
	void setWristAngledown(int a = 60);
	void setWristRotate(int a= 512);
	void openGripper(uint16_t distance = 512);
	void setSpeed(uint8_t speed = 128);
	void echo();
	void sendNow();
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
	static const uint16_t buttonByteOffset = 14;//bugbug not supported
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
	bool in90 = false; // arm 90 degrees down
	enum mode { Cartesian, Cylindrical, Backhoe	};
	mode armMode;
	bool inRange(int low90, int high90, int low, int high, int value);
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
