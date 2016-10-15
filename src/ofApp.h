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
	int readAllBytes(uint8_t *bytes, int bytesRequired = 5);
	int readBytesInOneShot(uint8_t *bytes, int bytesMax = 100);
	void readPose();
	void write(shared_ptr<uint8_t> data, int count);
	robotType waitForRobot();
protected:
	robotType ArmIDResponsePacket(uint8_t *bytes);
};

// one instance for robot, low level data without range checking so only use derived classes
class RobotJointsState {

public:
	RobotJointsState(shared_ptr<uint8_t> data) { this->data = data; }
	void send(shared_ptr<RobotSerial> serial);

	// only one data set per robot
	static shared_ptr<uint8_t> allocateData() { return make_shared<uint8_t>(count); }
	virtual void virtfunction() = 0;
protected:
	void setData(shared_ptr<uint8_t> data) { this->data = data; }
	void echo();
	void set(uint16_t high, uint16_t low, uint16_t val) {
		set(high, highByte(val));
		set(low, lowByte(val));
	}
	void set(uint16_t offset, uint8_t b);
	void setLowLevelCommand(robotLowLevelCommand cmd) { set(extValBytesOffset, cmd); };
	void setDelta(uint8_t value = 128) { set(deltaValBytesOffset, value); }
	void setButton(uint8_t value = 0) { set(buttonByteOffset, value); }

	void setLowLevelX(int x) { set(xHighByteOffset, xLowByteOffset, x + 512); } // no validation at this level use with care
	void setLowLevelY(int y) { set(yHighByteOffset, yLowByteOffset, y); }
	void setLowLevelZ(int z) { set(zHighByteOffset, zLowByteOffset, z); }
	void setLowLevelWristAngle(int a) { set(wristAngleHighByteOffset, wristAngleLowByteOffset, a+90); };
	void setLowLevelWristRotate(int a) { set(wristRotateHighByteOffset, wristRotateLowByteOffset, a); };
	void setLowLevelGripper(int distance) { set(gripperHighByteOffset, gripperLowByteOffset, distance); };

	// offsets bugbug store with JointValue, set at init time via constructor but still const
private:
	uint8_t getChkSum();
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
	shared_ptr<uint8_t> data=nullptr; // data to send
};

// stores only valid values for specific joints, does validation, defaults and other things, but no high end logic around motion
class RobotJoints : public RobotJointsState {
public:
	// constructor required
	RobotJoints(shared_ptr<uint8_t> data, robotArmMode mode);
	RobotJoints(shared_ptr<uint8_t> data) : RobotJointsState(data) {}
	RobotJoints() : RobotJointsState(nullptr) {}

	void setX(int x);
	void setY(int y);
	void setZ(int z);
	void setWristAngle(int a);
	void setWristRotate(int a);
	void setGripper(int distance);

	void oneTimeSetup(shared_ptr<uint8_t>data);

	bool inRange(robotArmJointType type, int value);

	int getDefaultValue(robotArmJointType type) {	return defaultValue[valueType(armMode, type)];}
	int getMin(robotArmJointType type) { return minValue[valueType(armMode, type)]; }
	int getMax(robotArmJointType type) { return maxValue[valueType(armMode, type)]; }
	
	static uint8_t getDeltaDefault() { return deltaDefault; }

	bool is90() { return armMode == IKM_IK3D_CARTESIAN_90 || armMode == IKM_CYLINDRICAL_90; }

	//Set 3D Cartesian mode / straight wrist and go to home etc
	robotArmMode setStartState(robotArmMode mode= IKM_IK3D_CARTESIAN, robotLowLevelCommand cmd= setArm3DCartesianStraightWristAndGoHome);
	void setSharedMemoryToDefaultState();

protected:
	static map<valueType, int> minValue;
	static map<valueType, int> maxValue;
	static map<valueType, int> defaultValue;
	static uint16_t deltaDefault; // manage speed

private:
	void set(valueType type, int min, int max, int defaultvalue);
	robotArmMode armMode = IKM_IK3D_CARTESIAN;
	RobotTypeID id;
	void virtfunction() {};

};

class Command : protected RobotJoints {
public:
	Command(shared_ptr<uint8_t> data, robotArmMode mode):RobotJoints(data, mode){ }

	// move or draw based on the value in moveOrDraw
	virtual void draw(shared_ptr<RobotSerial> serial) {
		sleep(); // sleep if requested
		//bugbug move arm up from paper if !moveOrDraw
		for (const auto& point : points) {
			setPoint(point);
			send(serial);// move
		}
	}

	void addPoint(const ofPoint& addPt) { points.push_back(addPt); }

	void sleep() { if (millisSleep > -1) ofSleepMillis(500); }

	int millisSleep = -1;// no sleep by default
	bool moveOrDraw = true; // false means draw
	bool deleteWhenDone = true; // false to repeat command per every draw occurance
protected:
	void setPoint(ofPoint pt);
	vector<ofPoint> points; // one more more points

private:
};
// example
class servosCenterCommand : public Command {
public:
	servosCenterCommand(shared_ptr<uint8_t> data, robotArmMode mode) :Command(data, mode) { }
	void draw(shared_ptr<RobotSerial> serial);
};
class sanityTestCommand : public Command {
public:
	sanityTestCommand(shared_ptr<uint8_t> data, robotArmMode mode) :Command(data, mode) { }
	void draw(shared_ptr<RobotSerial> serial);
};
// just for fun
class danceCommand : public Command {
public:
	danceCommand(shared_ptr<uint8_t> data, robotArmMode mode) :Command(data, mode) { }
	void draw(shared_ptr<RobotSerial> serial);
};

// the robot itself
class Robot {
public:
	void setup();
	void update();
	void draw();

	template<typename T> shared_ptr<T> createCommand() {return make_shared<T>(data, mode);};
	void add(shared_ptr<Command>cmd) { path.push(cmd); }

private:
	queue <shared_ptr<Command>> path; // move to robot, move all other stuff out of here, up or down
	shared_ptr<RobotSerial> serial; // talking to the robot
	shared_ptr<uint8_t> data = nullptr;// one data instance per robot
	robotType type;
	robotArmMode mode;
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
