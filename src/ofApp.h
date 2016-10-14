#pragma once

#include "ofMain.h"

//http://learn.trossenrobotics.com/arbotix/arbotix-communication-controllers/31-arm-link-reference.html

// from firmware
enum robotArmMode {
	//IKM_BACKHOE not 100% supported
	IKM_IK3D_CARTESIAN, IKM_IK3D_CARTESIAN_90, IKM_CYLINDRICAL, IKM_CYLINDRICAL_90, IKM_BACKHOE, IKM_NOT_DEFINED
};
enum robotArmJointType {
	X, Y, Z, wristAngle, wristRotate, Gripper, JointNotDefined
};
// low level commands
enum robotLowLevelCommand : uint8_t {
	NoArmCommand=0, EmergencyStop= 17, SleepArm= 96, HomeArm=80, HomeArm90=88, setArm3DCylindricalStraightWristAndGoHome= 48, 
	setArm3DCartesian90DegreeWristAndGoHome= 40, setArm3DCartesianStraightWristAndGoHome=32, setArm3DCylindrical90DegreeWristAndGoHome=56, setArmBackhoeJointAndGoHome=64
};
// high level commands
enum robotCommand {
	NoRobotHighLevelCommand, SignOnDance, DelayArm, CenterArm
};
enum RobotTypeID {
	// only 1 supported
	InterbotiXPhantomXReactorArm, unknownRobotType
};
typedef pair<robotArmMode, robotArmJointType> valueType;
typedef pair<robotArmMode, RobotTypeID> robotType;

class RobotSerial : public ofSerial {
public:
	void waitForSerial() { while (1) if (available() > 0) { return; } }
	void clearSerial() { flush(); }
	int readBytes(uint8_t *bytes, int bytesRequired = 5);
	int readBytesInOneShot(uint8_t *bytes, int bytesMax = 100);
	void readPose();
	void write(shared_ptr<uint8_t> data, int count);
	robotType waitForRobot();
protected:
	robotType ArmIDResponsePacket(uint8_t *bytes);
};


// one instance for robot, low level data 
class RobotJointsState {

public:
	void draw(shared_ptr<RobotSerial> serial) {
		getChkSum();
		echo();
		serial->write(data, count);
	}
	void setData(shared_ptr<uint8_t> data) { this->data = data; set(0, 255); };
	shared_ptr<uint8_t>getSharedData() { return data; }
	uint8_t* getData() { return data.get(); }

	void set(uint16_t high, uint16_t low, uint16_t val){
		set(high, highByte(val));
		set(low, lowByte(val));
	}
	void set(uint16_t offset, uint8_t b);
	void setLowLevelCommand(robotLowLevelCommand cmd) { set(extValBytesOffset, cmd); };
	void setDelta(uint8_t value=128) { set(deltaValBytesOffset, value); }
	void setButton(uint8_t value=0) { set(buttonByteOffset, value); }

	// only one data set per robot
	static shared_ptr<uint8_t> allocateData() { return make_shared<uint8_t>(count); }

protected:
	void echo();
	uint8_t getChkSum();
	void setLowLevelX(uint16_t x) { set(xHighByteOffset, xLowByteOffset, x + 512); } // no validation at this level use with care
	void setLowLevelY(uint16_t y) { set(yHighByteOffset, yLowByteOffset, y); }
	void setLowLevelZ(uint16_t z) { set(zHighByteOffset, zLowByteOffset, z); }
	void setLowLevelWristAngle(uint16_t a) { set(wristAngleHighByteOffset, wristAngleLowByteOffset, a+90); };
	void setLowLevelWristRotate(uint16_t a) { set(wristRotateHighByteOffset, wristRotateLowByteOffset, a); };
	void setLowLevelGripper(uint16_t distance) { set(gripperHighByteOffset, gripperLowByteOffset, distance); };

	// offsets bugbug store with JointValue, set at init time via constructor but still const
private:
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

	uint8_t lowByte(uint16_t a) { return a % 256; }
	uint8_t highByte(uint16_t a) { return (a / 256) % 256; }
	shared_ptr<uint8_t> data; // data to send
};

// stores only valid values for specific joints, does validation, defaults and other things, but no high end logic around motion
class RobotJoints : public RobotJointsState {
public:
	// constructor required
	RobotJoints(shared_ptr<uint8_t> data, robotArmMode mode);
	RobotJoints(shared_ptr<uint8_t> data) { setData(data); } 

	void setX(uint16_t x);
	void setY(uint16_t y);
	void setZ(uint16_t z);
	void setWristAngle(uint16_t a);
	void setWristRotate(uint16_t a);
	void setGripper(uint16_t distance);
	
	void setup();
	void oneTimeSetup();

	bool inRange(robotArmJointType type, uint16_t value);

	uint16_t getDefaultValue(robotArmJointType type) {	return defaultValue[valueType(armMode, type)];}
	uint16_t getMin(robotArmJointType type) { return minValue[valueType(armMode, type)]; }
	uint16_t getMax(robotArmJointType type) { return maxValue[valueType(armMode, type)]; }
	
	static uint8_t getDeltaDefault() { return deltaDefault; }

	void setCommand(robotCommand command, int64_t value = 0) { cmd = pair<robotCommand, int64_t>(command, value); }
	robotCommand getCommand(int64_t* value) { if (value) *value = cmd.second; return cmd.first; }
	bool is90() { return armMode == IKM_IK3D_CARTESIAN_90 || armMode == IKM_CYLINDRICAL_90; }

	//Set 3D Cartesian mode / straight wrist and go to home etc
	void setStartState(robotArmMode mode= IKM_IK3D_CARTESIAN, robotLowLevelCommand cmd= setArm3DCartesianStraightWristAndGoHome);
	void setDefaults();

protected:
	pair<robotCommand, int64_t> cmd;
	static map<valueType, uint16_t> minValue;
	static map<valueType, uint16_t> maxValue;
	static map<valueType, uint16_t> defaultValue;
	static uint16_t deltaDefault; // manage speed

private:
	void set(valueType type, uint16_t min, uint16_t max, uint16_t defaultvalue);
	robotArmMode armMode = IKM_IK3D_CARTESIAN;
	RobotTypeID id;
};

class RobotMotion : protected RobotJoints {
public:
	RobotMotion(shared_ptr<RobotSerial> serial, shared_ptr<uint8_t> data, robotArmMode mode) :RobotJoints(data, mode) { this->serial = serial; };
	RobotMotion(shared_ptr<RobotSerial> serial, shared_ptr<uint8_t> data) :RobotJoints(data) { this->serial = serial; }

	void draw();
	void setup() { RobotJoints::setup(); }
	void setup(robotCommand cmd, int64_t value = 0) { RobotJoints::setup(); RobotJoints::setCommand(cmd, value);	}

	// commands
	void center();
	void dance();
	void sanityTest();
	void centerAllServos();
private:
	shared_ptr<RobotSerial> serial;
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

// the robot itself
class Robot  {
public:
	void draw();
	void setup();
	void update();
	void reset();
private:
	queue <shared_ptr<RobotMotion>> path; // move to robot, move all other stuff out of here, up or down
	shared_ptr<RobotSerial> serial; // talking to the robot
	shared_ptr<uint8_t> data = nullptr;// one data instance per robot
	robotType type;
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

		Robot robot;
		
};
