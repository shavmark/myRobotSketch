#pragma once

#include "ofMain.h"

// from firmware
enum robotArmMode {
	//IKM_BACKHOE not 100% supported
	IKM_IK3D_CARTESIAN, IKM_IK3D_CARTESIAN_90, IKM_CYLINDRICAL, IKM_CYLINDRICAL_90, IKM_BACKHOE, IKM_NOT_DEFINED
};
enum robotArmJointType {
	X, Y, Z, wristAngle, wristRotate, Gripper, JointNotDefined
};
enum robotCommand {
	NoArmCommand, EnableArmMovement, SignOnDance, HomeArm, DelayArm, CenterArm, setArm3DCylindricalStraightWristAndGoHome, setArm3DCartesian90DegreeWristAndGoHome, setArm3DCartesianStraightWristAndGoHome, setArm3DCylindrical90DegreeWristAndGoHome, setArmBackhoeJointAndGoHome
};
enum RobotTypeID {
	// only 1 supported
	InterbotiXPhantomXReactorArm, unknownRobotType
};
typedef pair<robotArmMode, robotArmJointType> valueType;

class RobotSerial : public ofSerial {
public:
	void waitForSerial() { while (1) if (available() > 0) { return; } }
	void clearSerial() { flush(); }
};


// one instance for robot
class RobotJointsState {

public:

	void setData(shared_ptr<uint8_t> data) { this->data = data; set(0, 255); };
	void setSerial(shared_ptr<RobotSerial> serial) { this->serial = serial; }
	shared_ptr<uint8_t>getSharedData() { return data; }
	uint8_t* getData() { return data.get(); }

	void set(uint16_t high, uint16_t low, uint16_t val){
		set(high, highByte(val));
		set(low, lowByte(val));
	}
	void set(uint16_t offset, uint8_t b) { data.get()[offset] = b; }
	void setEnableMoveArm() { set(extValBytesOffset, 0); }
	void setSpeed(uint8_t speed = 128) { set(deltaValBytesOffset, speed); }

	// only one data set per robot
	static shared_ptr<uint8_t> allocateData() { return make_shared<uint8_t>(count); }

protected:

	void echo();
	void sendNow();
	void send(const string &s, uint8_t cmd);
	void setSend(bool b = true) { sendData = b; }
	bool sendData = false; // only send data once
	int readBytes(uint8_t *bytes, int bytesRequired = 5);
	int readBytesInOneShot(uint8_t *bytes, int bytesMax = 100);
	void readPose();

	// offsets bugbug store with JointValue, set at init time via constructor but still const
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

	shared_ptr<uint8_t> data; // data to send
	uint8_t lowByte(uint16_t a) { return a % 256; }
	uint8_t highByte(uint16_t a) { return (a / 256) % 256; }
	void write();
	shared_ptr<RobotSerial> serial = nullptr;
};

// stores only valid values for specific joints
class RobotJoints : public RobotJointsState {
public:
	// constructor required
	RobotJoints(shared_ptr<uint8_t> data, robotArmMode mode);
	RobotJoints(shared_ptr<uint8_t> data) { setData(data); } 

	void setX(uint16_t x) { set(X, x); };
	void setY(uint16_t y) { set(Y, y); };
	void setZ(uint16_t z) { set(Z, z); };
	void setWristAngle(uint16_t a) { set(wristAngle, a); };
	void setWristRotate(uint16_t a) { set(wristRotate, a); };
	void setGripper(uint16_t distance) { set(wristRotate, Gripper); };
	void home() { send("home", is90() ? 88 : 80); }
	void sleepArm() { send("sleepArm", 96); }
	void emergencyStop() { send("emergencyStop", 17); }
	void draw() {
		sendNow();
	}
	void reset();
	void setup();
	bool inRange(robotArmJointType type, uint16_t value);
	void set(robotArmJointType type, uint16_t value);
	bool isSet() { return valueSet; }
	uint16_t getDefaultValue(robotArmJointType type) {	return defaultValue[valueType(armMode, type)];}
	uint16_t getValue(robotArmJointType type);
	uint16_t getMin(robotArmJointType type) { return minValue[valueType(armMode, type)]; }
	uint16_t getMax(robotArmJointType type) { return maxValue[valueType(armMode, type)]; }
	static uint16_t getDelta() { return deltaDefault; }
	void	 setDelta(uint16_t value) { deltaDefault = value; }
	pair<robotCommand, int64_t> cmd;
	void setCommand(robotCommand command, int64_t value = 0) { cmd = pair<robotCommand, int64_t>(command, value); }
	robotCommand getCommand(int64_t* value) { if (value) *value = cmd.second; return cmd.first; }
	bool is90() { return armMode == IKM_IK3D_CARTESIAN_90 || armMode == IKM_CYLINDRICAL_90; }
	bool ArmIDResponsePacket(uint8_t *bytes);

protected:
	static map<valueType, uint16_t> minValue;
	static map<valueType, uint16_t> maxValue;
	static map<valueType, uint16_t> defaultValue;
	static uint16_t deltaDefault; // manage speed
private:
	int32_t value=0; // default to a non fatal value 
	void set(valueType type, uint16_t min, uint16_t max, uint16_t defaultvalue);
	bool valueSet=false; // value not yet set
	robotArmMode armMode = IKM_IK3D_CARTESIAN;
	RobotTypeID id;

};

// smallest unit of movement, move from a to b
class Segment {
public:
	void setup() {};
	void draw() {
		// go to a, if a is not set just go to b
		// draw b
	};

private:
	RobotJoints a;
	RobotJoints b;
};

// can take any shape, much like a line a human would draw, just connects the dots so to speak
class Line {
public:
	void setup() {};
	void draw() {
		for (auto& segment : line) {
			segment.draw();
		}
	};

	vector <Segment> line; 
};

// talks to the robot and keeps its state
class RobotState {
public:
	
	queue <shared_ptr<RobotJoints>> path;

	//http://learn.trossenrobotics.com/arbotix/arbotix-communication-controllers/31-arm-link-reference.html
	void setup();
	void update();
	void draw();
	void reset();
	void waitForRobot();
	void center();
	void dance();
	void setDefaults();

	//Set 3D Cartesian mode / straight wrist and go to home
	void set3DCylindricalStraightWristAndGoHome() {
		setStateAndGoHome("set3DCylindricalStraightWristAndGoHome", IKM_CYLINDRICAL, 48);
	}
	void set3DCartesian90DegreeWristAndGoHome() {
		setStateAndGoHome("set3DCartesian90DegreeWristAndGoHome", IKM_IK3D_CARTESIAN_90, 40);
	}
	void set3DCartesianStraightWristAndGoHome() {
		setStateAndGoHome("set3DCartesianStraightWristAndGoHome", IKM_IK3D_CARTESIAN, 32);
	}
	void set3DCylindrical90DegreeWristAndGoHome() {
		setStateAndGoHome("set3DCylindrical90DegreeWristAndGoHome", IKM_CYLINDRICAL_90, 56);
	}
	//backhoe not fully supported
	void setBackhoeJointAndGoHome() {
		setStateAndGoHome("setBackhoeJointAndGoHome", IKM_BACKHOE, 64);
	}
	void sanityTest();
	void centerAllServos();

protected:

private:
	void setStateAndGoHome(const string& s, robotArmMode mode, uint8_t cmd);
	
	shared_ptr<RobotSerial> serial=nullptr; // how we talk to each other
	shared_ptr<uint8_t> data=nullptr;// one data instance per robot
};

// the robot itself
class Robot : RobotState {
public:
	Robot() {
		
	}

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
