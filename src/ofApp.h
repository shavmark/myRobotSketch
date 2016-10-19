#pragma once

#include "ofMain.h"

// lib is not dev'd or tested for multi threading yet

//http://learn.trossenrobotics.com/arbotix/arbotix-communication-controllers/31-arm-link-reference.html

// from firmware IKM_BACKHOE not 100% supported
enum robotArmMode {	IKM_IK3D_CARTESIAN, IKM_IK3D_CARTESIAN_90, IKM_CYLINDRICAL, IKM_CYLINDRICAL_90, IKM_BACKHOE, IKM_NOT_DEFINED};
enum robotArmJointType {X, Y, Z, wristAngle, wristRotate, Gripper, JointNotDefined};
// only 1 supported
enum RobotTypeID {InterbotiXPhantomXReactorArm, unknownRobotType};

typedef pair<robotArmMode, RobotTypeID> robotType;
typedef pair<robotType, robotArmJointType> SpecificJoint; // backhoe gets different handling, see is spec. Its not fully supported here

inline robotType createRobotType(robotArmMode mode, RobotTypeID id) {
	return robotType(mode, id);
}
inline robotType createDefaultRobotType(RobotTypeID id) {
	return robotType(IKM_CYLINDRICAL, id);
}
inline robotType createUndefinedRobotType() {
	return createRobotType(IKM_NOT_DEFINED, unknownRobotType);
}
inline bool robotTypeIsError(robotType type) {
	return type == createUndefinedRobotType();
}
inline SpecificJoint createJoint(robotArmJointType joint, robotArmMode mode, RobotTypeID id) {
	return SpecificJoint(createRobotType(mode, id), joint);
}
// low level commands
enum robotLowLevelCommand : uint8_t {
	unKnownCommand = 255, NoArmCommand = 0, EmergencyStop = 17, SleepArm = 96, HomeArm = 80, HomeArm90 = 88, setArm3DCylindricalStraightWristAndGoHome = 48,
	setArm3DCartesian90DegreeWristAndGoHome = 40, setArm3DCartesianStraightWristAndGoHome = 32, setArm3DCylindrical90DegreeWristAndGoHome = 56, setArmBackhoeJointAndGoHome = 64
};

class RobotSerial : public ofSerial {
public:
	void waitForSerial() { while (1) if (available() > 0) { return; } }
	void clearSerial() { flush(); }
	int readAllBytes(uint8_t* bytes, int bytesRequired = 5);
	int readBytesInOneShot(uint8_t* bytes, int bytesMax = 100);
	void readPose();
	void write(uint8_t* data, int count);
	robotType waitForRobot();

protected:
	robotType ArmIDResponsePacket(uint8_t *bytes);
};

// pure virtual base class, low level data without range checking so only use derived classes
class RobotJointsState {

public:

	static const uint16_t count = 17;
	void send(RobotSerial* serial);
	robotLowLevelCommand getStartCommand(robotType type);

protected:

	RobotJointsState(uint8_t *data=nullptr) { setData(data); }
	
	virtual void virtfunction() = 0;
	void setData(uint8_t *data) { this->data = data; }
	void echo();
	void set(uint16_t offset, uint8_t b);
	void setLowLevelCommand(robotLowLevelCommand cmd) { set(extValBytesOffset, cmd); };
	void setDelta(uint8_t value = 128) { set(deltaValBytesOffset, value); }
	void setButton(uint8_t value = 0) { set(buttonByteOffset, value); }

	void setLowLevelX(int x, int magicNumer) { set(xHighByteOffset, xLowByteOffset, x + magicNumer); } // no validation at this level use with care
	void setLowLevelY(int y) { set(yHighByteOffset, yLowByteOffset, y); }
	void setLowLevelZ(int z) { set(zHighByteOffset, zLowByteOffset, z); }
	void setLowLevelWristAngle(int a) { set(wristAngleHighByteOffset, wristAngleLowByteOffset, a+90); };
	void setLowLevelWristRotate(int a) { set(wristRotateHighByteOffset, wristRotateLowByteOffset, a); };
	void setLowLevelGripper(int distance) { set(gripperHighByteOffset, gripperLowByteOffset, distance); };

	int getX() { return get(xHighByteOffset, xLowByteOffset); }
	int getY() { return get(yHighByteOffset, yLowByteOffset); }
	int getZ() { return get(zHighByteOffset, zLowByteOffset); }
	int getWristAngle() { return get(wristAngleHighByteOffset, wristAngleLowByteOffset); }
	int getWristRotate() { return get(wristRotateHighByteOffset, wristRotateLowByteOffset); }
	int getGripper() { return get(gripperHighByteOffset, gripperLowByteOffset); }

private:
	void set(uint16_t high, uint16_t low, int val);
	int get(uint16_t high, uint16_t low);
	uint8_t getChkSum();
	static const uint16_t headerByteOffset = 0;
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

	uint8_t lowByte(uint16_t a) { return a % 256; }
	uint8_t highByte(uint16_t a) { return (a / 256) % 256; }
	uint8_t *data=nullptr; // data to send
};

class RobotValueRanges {
public:

	void setDefault(SpecificJoint joint, int value, int min = MAXINT, int max = MININT);
	void setMin(SpecificJoint joint, int value, int min = MAXINT, int max = MININT);
	void setMax(SpecificJoint joint, int value, int min = MAXINT, int max = MININT);

	map<SpecificJoint, int> minValue;
	map<SpecificJoint, int> maxValue;
	map<SpecificJoint, int> defaultValue;
};

// stores only valid values for specific joints, does validation, defaults and other things, but no high end logic around motion
class RobotJoints : public RobotJointsState {
public:
	// constructor required
	RobotJoints(const robotType& typeOfRobot) : RobotJointsState() { this->typeOfRobot = typeOfRobot; };
	RobotJoints(uint8_t* data, const robotType& typeOfRobot) : RobotJointsState(data) {	this->typeOfRobot = typeOfRobot;}
	RobotJoints(uint8_t* data) : RobotJointsState(data) { typeOfRobot = createUndefinedRobotType(); }
	
	void setX(int x);
	void setY(int y);
	void setZ(int z);
	void setWristAngle(int a);
	void setWristRotate(int a);
	void setGripper(int distance);

	void oneTimeSetup();

	bool inRange(robotArmJointType type, int value);

	int getDefaultValue(robotArmJointType type);

	// ranges based on device
	int getMin(robotArmJointType type);
	int getMax(robotArmJointType type);
	int getMid(robotArmJointType type);
	int getDeltaDefault() { return deltaDefault; }
	bool isCartesion() { return (typeOfRobot.first == IKM_IK3D_CARTESIAN || typeOfRobot.first == IKM_IK3D_CARTESIAN_90); }
	bool isCylindrical() { return (typeOfRobot.first == IKM_CYLINDRICAL || typeOfRobot.first == IKM_CYLINDRICAL_90); }
	int addMagicNumber() { return isCylindrical() ? 0 : 512; }
	bool is90() { return typeOfRobot.first == IKM_IK3D_CARTESIAN_90 || typeOfRobot.first == IKM_CYLINDRICAL_90; }

	//Set 3D Cartesian mode / straight wrist and go to home etc
	robotType setStartState();
	robotType getRobotType() { return typeOfRobot; }
	void setDefaultState();
	void setUserDefinedRanges(SpecificJoint joint, RobotValueRanges *userDefinedRanges);
protected:

private:
	// user defined ranges
	static RobotValueRanges hardwareRanges;
	RobotValueRanges *userDefinedRanges=nullptr;
	int deltaDefault = 255;

	void set(SpecificJoint type, int min, int max, int defaultvalue);
	robotType typeOfRobot;// required
	void virtfunction() {};

};

// positions are defined as % change of all range of joint, from the current position
// RobotPositions can only be 0.0 to +/- 1.0 (0 to +/- 100%)
class RobotPosition : protected ofPoint {
public:
	//=FLT_MAX means not set
	#define NoRobotValue FLT_MAX
	RobotPosition(float xPercent = NoRobotValue, float yPercent = NoRobotValue, float zPercent = NoRobotValue) { setPercents(xPercent, yPercent, zPercent); }
	void setPercents(float xPercent = NoRobotValue, float yPercent = NoRobotValue, float zPercent = NoRobotValue);
	virtual void echo() const;
	float getX()const { return x; }
	float getY() const { return y; }
	float getZ() const { return z; } // want to make sure x is read only

	bool set[3];

protected:
	bool validRange(float f);
};

class RobotState : public RobotPosition {
public:
	RobotState(float wristAngle = FLT_MAX, float wristRotate = FLT_MAX, float gripper = FLT_MAX) :RobotPosition(wristAngle, wristRotate, gripper) {  }

	float getWristAngle()const { return getPtr()[0]; }
	float getWristRotation() const { return getPtr()[1]; }
	float getGripper() const { return getPtr()[2]; }

	void echo() const;

};

class RobotCommand {
public:
	// commands and only be 0.0 to +/- 1.0 (0 to +/- 100%)
	RobotCommand(float xPercent=0, float yPercent = 0, float zPercent = 0, float wristAnglePercent = 0, float wristRotatePercent = 0, float gripperPercent = 0, int millisSleep = -1, bool deleteWhenDone = true) {
		init(RobotPosition(xPercent, yPercent, zPercent), RobotState(wristAnglePercent, wristRotatePercent, gripperPercent), millisSleep, deleteWhenDone);
	}
	// object based
	RobotCommand(const RobotPosition& pointPercent, const RobotState& settingsPercent = RobotState(), int millisSleep = -1, bool deleteWhenDone = true) {
		init(pointPercent, settingsPercent, millisSleep, deleteWhenDone);
	}
	// make a sleep only command
	RobotCommand(int millisSleep = -1, bool deleteWhenDone = true) {
		init(RobotPosition(), RobotState(), millisSleep, deleteWhenDone);
	}

	void sleep() const { if (millisSleep > -1) ofSleepMillis(millisSleep); }
	bool OKToDelete() { return deleteWhenDone; }
	void echo() const ;

	RobotPosition pointPercent;
	RobotState settingsPercent;

private:
	void init(const RobotPosition& pointPercent = RobotPosition(), const RobotState& settingsPercent = RobotState(), int millisSleep = -1, bool deleteWhenDone = true);
	bool deleteWhenDone; // false to repeat command per every draw occurance
	int millisSleep;// no sleep by default

};

class Robot;

class RobotCommands : protected RobotJoints {
public:
	
	enum BuiltInCommandNames {UserDefined, LowLevelTest, HighLevelTest, Sizing};// command and basic commands.  Derive object or create functions to create more commands

	// passed robot cannot go away while this object exists
	RobotCommands(Robot *robot, BuiltInCommandNames = UserDefined);
	RobotCommands(BuiltInCommandNames name = UserDefined) :RobotJoints(nullptr) { this->name = name; }

	void echo() const; // echos positions

	// put command data in a known state
	void reset();

	// move or draw based on the value in moveOrDraw
	virtual void draw();
	void setFillMode(int mode) { fillmode = mode; }
	void add(const RobotCommand& cmd, BuiltInCommandNames name = UserDefined);
	bool moveOrDraw = true; // false means draw
	int fillmode = 0;

protected:
	BuiltInCommandNames getName() { return name; };
	void setPoint(RobotPosition pt);
	void setState(RobotState pt);
	vector<RobotCommand> cmdVector; // one more more points
	Robot *robot = nullptr; // owner

private:
	void sanityTestLowLevel(); // built in commands
	void sanityTestHighLevel();
	void sizeDrawing() {} // bugbug to do
	BuiltInCommandNames name;
};

// the robot itself
class Robot {
public:
	friend class RobotCommands;
	void setup();
	void update();
	void draw();
	void echo(); // echos positions
	void setPause(bool pause = true) { this->pause = pause; }//bugbug go to threads

	shared_ptr<RobotCommands> add(RobotCommands::BuiltInCommandNames name = RobotCommands::UserDefined);

	robotType& getType() { return type; }
	RobotSerial& getSerial() { return serial; }

protected:
	RobotValueRanges userDefinedRanges;

private:
	RobotSerial serial; // talking to the robot
	uint8_t data[RobotJointsState::count];// one data instance per robot
	robotType type;
	vector<shared_ptr<RobotCommands>> cmds;
	bool pause = false;
};

class DrawingRobot : public Robot {
public:
	void setup();
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

		DrawingRobot robot;
		
};
