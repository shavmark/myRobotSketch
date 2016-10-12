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
enum robotCommand {
	NoArmCommand, EnableArmMovement, SignOnDance, HomeArm, DelayArm, CenterArm, setArm3DCylindricalStraightWristAndGoHome, setArm3DCartesian90DegreeWristAndGoHome, setArm3DCartesianStraightWristAndGoHome, setArm3DCylindrical90DegreeWristAndGoHome, setArmBackhoeJointAndGoHome
};
enum RobotTypeID {
	// only 1 supported
	InterbotiXPhantomXReactorArm, unknownRobotType
};
typedef pair<robotArmMode, robotArmJointType> valueType;

// stores only valid values for specific joints
class JointValue {
public:

	JointValue(valueType type, uint16_t value);
	JointValue(valueType type);
	JointValue();
	void reset();
	void setup();
	bool inRange(uint16_t value);
	int32_t operator=(uint16_t value) {set(value);return value;}
	void set(uint16_t value);
	bool isSet() { return valueSet; }
	uint16_t getDefaultValue() {	return defaultValue[type];}
	uint16_t getValue();
	uint16_t getMin() { return minValue[type]; }
	uint16_t getMax() { return maxValue[type]; }

protected:
	static map<valueType, uint16_t> minValue;
	static map<valueType, uint16_t> maxValue;
	static map<valueType, uint16_t> defaultValue;
	static uint16_t deltaDefault;

private:
	int32_t value=0; // default to a non fatal value 
	void set(valueType type, uint16_t min, uint16_t max, uint16_t defaultvalue);
	bool valueSet; // value not yet set
	valueType type;
};

// drives the robot, data can come from any source 
class RobotCommandsAndData {
public:

	RobotCommandsAndData();

	void setXleft(int32_t x) { locations[X] = x; }
	void setYout(int32_t y) { locations[Y] = y; }
	void setZup(int32_t z) { locations[Z] = z; }
	void setWristAngledown(int a) { locations[wristAngle] = a; }
	void setWristRotate(int32_t a) { locations[wristRotate] = a; }
	void openGripper(uint16_t distance = 512) { locations[gripper] = distance; }
	void reset();

	void setCommand(robotCommand command, int64_t value=0) { cmd = pair<robotCommand, int64_t>(command, value); }
	robotCommand getCommand(int64_t* value) { if (value) *value = cmd.second; return cmd.first; }
	JointValue& get(robotArmJointType jointType) { return locations[jointType]; }
protected:
	pair<robotCommand, int64_t> cmd;
	map<robotArmJointType, JointValue> locations;

};

// talks to the robot and keeps its state
class RobotState {
public:
	
	queue <RobotCommandsAndData> path; // bugug add access wraper, use shared_ptr

	//http://learn.trossenrobotics.com/arbotix/arbotix-communication-controllers/31-arm-link-reference.html
	void setup();
	void update();
	void draw();
	void reset();
	void home() {	send("home", is90() ? 88 : 80);}
	void sleepArm() {	send("sleepArm", 96);}
	void emergencyStop() {	send("emergencyStop", 17);	}
	void enableMoveArm() {		set(extValBytesOffset, 0);	}
	// 10 super fast, 255 slow
	void setSpeed(uint8_t speed = 128) {	set(deltaValBytesOffset, min(speed, (uint8_t)254));	}
	void setDefaults();

	void center();
	void dance();

	// home 0xff 0x2 0x0 0x0 0x96 0x0 0x96 0x0 0x5a 0x2 0x0 0x1 0x0 0x80 0x0 0x0 0xf4
	//Set 3D Cartesian mode / straight wrist and go to home
	void RobotState::set3DCylindricalStraightWristAndGoHome() {
		setStateAndGoHome("set3DCylindricalStraightWristAndGoHome", IKM_CYLINDRICAL, 48);
	}
	void RobotState::set3DCartesian90DegreeWristAndGoHome() {
		setStateAndGoHome("set3DCartesian90DegreeWristAndGoHome", IKM_IK3D_CARTESIAN_90, 40);
	}
	//Set 3D Cartesian mode / straight wrist and go to home
	void RobotState::set3DCartesianStraightWristAndGoHome() {
		setStateAndGoHome("set3DCartesianStraightWristAndGoHome", IKM_IK3D_CARTESIAN, 32);
	}
	void RobotState::set3DCylindrical90DegreeWristAndGoHome() {
		setStateAndGoHome("set3DCylindrical90DegreeWristAndGoHome", IKM_CYLINDRICAL_90, 56);
	}
	//backhoe not fully supported
	void RobotState::setBackhoeJointAndGoHome() {
		setStateAndGoHome("setBackhoeJointAndGoHome", IKM_BACKHOE, 64);
	}

	void centerAllServos();

protected:
	// push a copy of data then reset existing data so it can be used again
	void pushAndClearCommand(RobotCommandsAndData &data) { path.push(data); data.reset();}
	void waitForSerial() {	while(1) if (serial.available() > 0) { return; }	}
	void clearSerial() { serial.flush(); }
	void echo();
	void sendNow();
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
	RobotTypeID id;
	void write();
	bool ArmIDResponsePacket(uint8_t *bytes);
	robotArmMode armMode;
	bool is90() { return armMode == IKM_IK3D_CARTESIAN_90 || armMode == IKM_CYLINDRICAL_90; }

private:
	uint8_t lowByte(uint16_t a) { return a % 256; }
	uint8_t highByte(uint16_t a) { return (a / 256) % 256; }
	int readBytes(uint8_t *bytes, int bytesRequired = 5);
	int readBytesInOneShot(uint8_t *bytes, int bytesMax = 100);
	uint8_t data[count]; // data to send
	ofSerial serial;
	void setStateAndGoHome(const string& s, robotArmMode mode, uint8_t cmd);
	void send(const string &s, uint8_t cmd);
	void readPose();
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
