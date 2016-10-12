#pragma once

#include "ofMain.h"

// from firmware
enum robotArmMode {
	//IKM_BACKHOE not 100% supported
	IKM_IK3D_CARTESIAN, IKM_IK3D_CARTESIAN_90, IKM_CYLINDRICAL, IKM_CYLINDRICAL_90, IKM_BACKHOE, IKM_NOT_DEFINED
};
enum robotArmJointType {
	X, Y, Z, wristAngle, wristRotate, gripper, delta, JointNotDefined
};

typedef pair<robotArmMode, robotArmJointType> valueType;

class JointValue {
public:

	JointValue(valueType type, int32_t value);
	JointValue(valueType type);
	JointValue();
	void reset();
	void setup();
	bool inRange(int32_t value);
	int32_t operator=(int32_t value) {set(value);return value;}
	int32_t operator+(int32_t valueIn) {	return getValue() + valueIn;}
	void set(int32_t value);
	bool isSet() { return valueSet; }
	int32_t getDefaultValue() {	return defaultValue[type];}
	int32_t getValue();

protected:
	static map<valueType, int32_t> minValue;
	static map<valueType, int32_t> maxValue;
	static map<valueType, int32_t> defaultValue;
	static int32_t deltaDefault;

private:
	int32_t value;
	void set(valueType type, int32_t min, int32_t max, int32_t defaultvalue);
	bool valueSet; // value not yet set
	valueType type;
};

class RobotCommands {
public:
	enum command {
		None, Home, Delay, Center
	};

	RobotCommands();

	void moveXleft(int32_t x) { locations[X] = x; }
	void moveYout(int32_t y) { locations[Y] = y; }
	void moveZup(int32_t z) { locations[Z] = z; }
	void setWristAngledown(int a) { locations[wristAngle] = a; }
	void setWristRotate(int32_t a) { locations[wristRotate] = a; }
	void openGripper(uint16_t distance = 512) { locations[gripper] = distance; }
	void reset();

	void setHome() {	cmd = pair<command, int64_t>(Home, 0);}
	void setCenter() {	cmd = pair<command, int64_t>(Center, 0);}
	void setDelay(int64_t duration) {	cmd = pair<command, int64_t>(Delay, duration);}
	void setNoCommand() {	cmd = pair<command, int64_t>(None, 0);	}
	command getCommand(int64_t* value) { if (value) *value = cmd.second; return cmd.first; }
	JointValue& get(robotArmJointType jointType) { return locations[jointType]; }
protected:
	pair<command, int64_t> cmd;
	map<robotArmJointType, JointValue> locations;

};
class RobotState {
public:
	enum ID {
		// only 1 supported
		InterbotiXPhantomXReactorArm
	};
	std::list<JointValue> values;

	//http://learn.trossenrobotics.com/arbotix/arbotix-communication-controllers/31-arm-link-reference.html
	void setup();
	void update();
	void draw();
	void reset() { while (!path.empty()) { path.pop(); } memset(data, 0, sizeof data); };

	void home();
	void center();
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
	queue <RobotCommands> path; // bugug add access wraper, use shared_ptr

protected:
	
	void moveXleft(JointValue& x, bool send = false);
	void moveXleft(int32_t x, bool send = false) { moveXleft(JointValue(valueType(armMode, X), x), send); };

	void moveYout(JointValue& y, bool send = false);
	void moveYout(int32_t y, bool send = false) { moveYout(JointValue(valueType(armMode, Y), y), send); };

	void moveZup(JointValue& z, bool send = false);
	void moveZup(int32_t z, bool send = false) { moveZup(JointValue(valueType(armMode, Z), z), send); };


	void setWristAngledown(JointValue& a, bool send = false);
	void setWristAngledown(int32_t a, bool send = false) { setWristAngledown(JointValue(valueType(armMode, wristAngle), a), send); };

	void setWristRotate(JointValue& a, bool send = false);
	void setWristRotate(int32_t a, bool send = false) { setWristRotate(JointValue(valueType(armMode, wristRotate), a), send); };

	void openGripper(JointValue& distance, bool send = false);
	void openGripper(int32_t distance, bool send = false) { openGripper(JointValue(valueType(armMode, gripper), distance), send); };


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
	bool sendData = false; // only send data once
	ID id;
	void write();
	bool ArmIDResponsePacket();
	robotArmMode armMode;
	bool is90() { return armMode == IKM_IK3D_CARTESIAN_90 || armMode == IKM_CYLINDRICAL_90; }

private:
	uint8_t lowByte(uint16_t a) { return a % 256; }
	uint8_t highByte(uint16_t a) { return (a / 256) % 256; }
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
