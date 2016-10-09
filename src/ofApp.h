#pragma once

#include "ofMain.h"

class RobotLocation {
public:
	RobotLocation();
	void moveXleft(int32_t x) { this->x.first = x; this->x.second = true; }
	void moveYout(uint16_t y) { this->y.first = y; this->y.second = true; }
	void moveZup(uint16_t z) { this->z.first = z; this->z.second = true; }
	void setWristAngledown(int a) { this->wristAngle.first = a; this->wristAngle.second = true;  }
	void setWristRotate(int32_t a) { this->wristRotate.first = a; this->wristRotate.second = true;	}
	void openGripper(uint16_t distance = 512) { this->distance.first = distance; this->distance.second = true;	}
	void reset();

	enum command {
		None, Home, Delay
	};

	void setHome() {
		cmd = pair<command, int64_t>(Home, 0);
	}
	void setDelay(int64_t duration) {
		cmd = pair<command, int64_t>(Delay, duration);
	}
	void setNoCommand() {
		cmd = pair<command, int64_t>(None, 0);
	}

	pair<command, int64_t> cmd;
	pair<int32_t, bool> x;
	pair<int32_t, bool> y;
	pair<int32_t, bool> z;
	pair<int32_t, bool> wristAngle;
	pair<int32_t, bool> wristRotate;
	pair<int32_t, bool> gripper;
	pair<int32_t, bool> distance;

};
class RobotState {
public:
	// from firmware
	enum mode {
		IKM_IK3D_CARTESIAN, IKM_IK3D_CARTESIAN_90, IKM_CYLINDRICAL, IKM_CYLINDRICAL_90, IKM_BACKHOE
	} ;
	enum ID {
		InterbotiXPhantomXReactorArm
	};
	//http://learn.trossenrobotics.com/arbotix/arbotix-communication-controllers/31-arm-link-reference.html
	void setup();
	void update();
	void draw();

	void home();
	void enableMoveArm();

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
	void setSpeed(uint8_t speed = 128);
	void echo();
	void sendNow();
	queue <RobotLocation> path; // bugug add access wraper, use shared_ptr

protected:
	
	void moveXleft(int32_t x, bool send = false);
	void moveYout(uint16_t y, bool send = false);
	void moveZup(uint16_t z, bool send = false);
	void setWristAngledown(int32_t a, bool send = false);
	void setWristRotate(int32_t a, bool send = false);
	void openGripper(uint16_t distance = 512, bool send = false);
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
	int getDefault(int32_t cart90, int32_t cart, int32_t cyn90, int32_t cyn);
	int getDefault(int32_t int90, int32_t intStraight);
	bool sendData = false; // only send data once
	mode armMode;
	ID id;
	bool inRange(int32_t low90, int32_t high90, int32_t low, int32_t high, int32_t value);
	void write();
	bool ArmIDResponsePacket();
private:

	int readBytes(unsigned char *bytes, int bytesRequired = 5);
	uint8_t data[count]; // data to send
	ofSerial serial;
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

		RobotState robot;
		
};
