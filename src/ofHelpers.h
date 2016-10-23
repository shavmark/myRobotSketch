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



