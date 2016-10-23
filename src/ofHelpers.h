#pragma once
#include "robot.h"
// basic OF related classes

class ofRobotTrace : public RobotTrace {
public:
	virtual void sendline() {
		if (isError) {
			ofLogError() << message.str(); //bugbug support all log types from OF
		}
		else {
			ofLogNotice() << message.str();
		}
	}
	virtual void send() {}
};

class ofRobotSerial : public ofSerial {
public:
	ofRobotSerial() {}
	void waitForSerial() { while (1) if (available() > 0) { return; } }
	void clearSerial() { flush(); }
	int readAllBytes(uint8_t* bytes, int bytesRequired = 5);
	int readBytesInOneShot(uint8_t* bytes, int bytesMax = 100);
	void readPose();
	void write(uint8_t* data, int count);
	robotType waitForRobot();

protected:
	void echoRawBytes(uint8_t *bytes, int count);
	robotType ArmIDResponsePacket(uint8_t *bytes);
};

class ofRobotVoice {
public:
	void draw() {}//bugbug enumerate and say, bring in SAPI 11 or such
	void add(const string& say) { thingsToSay.push_back(say); }
	vector<string> thingsToSay;
};

// positions are defined as % change of all range of joint, from the current position
// RobotPositions can only be 0.0 to +/- 1.0 (0 to +/- 100%)
class ofRobotPosition : protected ofPoint {
public:
	//=FLT_MAX means not set
#define NoRobotValue FLT_MAX
	ofRobotPosition(float xPercent = NoRobotValue, float yPercent = NoRobotValue, float zPercent = NoRobotValue) { setPercents(xPercent, yPercent, zPercent); }
	void setPercents(float xPercent = NoRobotValue, float yPercent = NoRobotValue, float zPercent = NoRobotValue);
	virtual void echo() const;
	float getX()const { return x; }
	float getY() const { return y; }
	float getZ() const { return z; } // want to make sure x is read only

	bool set[3];

protected:
	bool validRange(float f);
};

class ofRobotState : public ofRobotPosition {
public:
	ofRobotState(float wristAngle = FLT_MAX, float wristRotate = FLT_MAX, float gripper = FLT_MAX) :ofRobotPosition(wristAngle, wristRotate, gripper) {  }

	float getWristAngle()const { return getPtr()[0]; }
	float getWristRotation() const { return getPtr()[1]; }
	float getGripper() const { return getPtr()[2]; }

	void echo() const;

};

