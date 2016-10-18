#pragma once

#include "ofMain.h"

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

	RobotJointsState(uint8_t *data) { setData(data); }
	
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

	// offsets bugbug store with JointValue, set at init time via constructor but still const
private:
	void set(uint16_t high, uint16_t low, int val);
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

// stores only valid values for specific joints, does validation, defaults and other things, but no high end logic around motion
class RobotJoints : public RobotJointsState {
public:
	// constructor required
	RobotJoints(uint8_t* data, const robotType& typeOfRobot);
	RobotJoints(uint8_t* data) : RobotJointsState(data) { typeOfRobot = createUndefinedRobotType(); }
	
	void setX(int x);
	void setY(int y);
	void setZ(int z);
	void setWristAngle(int a);
	void setWristRotate(int a);
	void setGripper(int distance);

	void oneTimeSetup();

	bool inRange(robotArmJointType type, int value);

	int getDefaultValue(robotArmJointType type) { return  defaultValue[SpecificJoint(typeOfRobot, type)]; }
	int getMin(robotArmJointType type) { return minValue[SpecificJoint(typeOfRobot, type)]; }
	int getMax(robotArmJointType type) { return maxValue[SpecificJoint(typeOfRobot, type)]; }
	//26.5" x max

	static int getDeltaDefault() { return deltaDefault; }
	bool isCartesion() { return (typeOfRobot.first == IKM_IK3D_CARTESIAN || typeOfRobot.first == IKM_IK3D_CARTESIAN_90); }
	bool isCylindrical() { return (typeOfRobot.first == IKM_CYLINDRICAL || typeOfRobot.first == IKM_CYLINDRICAL_90); }
	int addMagicNumber() { return isCylindrical() ? 0 : 512; }
	bool is90() { return typeOfRobot.first == IKM_IK3D_CARTESIAN_90 || typeOfRobot.first == IKM_CYLINDRICAL_90; }

	//Set 3D Cartesian mode / straight wrist and go to home etc
	robotType setStartState();
	robotType getRobotType() { return typeOfRobot; }
	void setDefaultState();

protected:
	static map<SpecificJoint, int> minValue;
	static map<SpecificJoint, int> maxValue;
	static map<SpecificJoint, int> defaultValue;
	static int deltaDefault; // manage speed

private:
	void set(SpecificJoint type, int min, int max, int defaultvalue);
	robotType typeOfRobot;// required
	void virtfunction() {};

};
class Command;//forward reference
// the robot itself
class Robot {
public:
	friend class Command;
	void setup();
	void update();
	void draw();

	template<typename T> shared_ptr<T> createCommand() { return make_shared<T>(*this); };
	void add(shared_ptr<Command>cmd) { path.push(cmd); }

	RobotSerial serial; // talking to the robot

private:

	uint8_t data[RobotJointsState::count];// one data instance per robot
	robotType type;
	queue <shared_ptr<Command>> path; // move to robot, move all other stuff out of here, up or down
};

class Command : protected RobotJoints {
public:
	// passed robot cannot go away while this object exists bugbug should this be a shared pointer?
	Command(Robot &robot) :RobotJoints(robot.data, robot.type) { this->robot = &robot;  }

	class CommandInfo {
	public:
		CommandInfo(const ofPoint& point, const ofPoint& settings) { this->point = point; this->settings = settings; }

		ofPoint point;
		ofPoint settings;
	};
	// put command in a known state
	void reset() { // setup can be ignored for a reset is not required
		setStartState();
		send(&robot->serial); // send the mode, also resets the robot
		setDefaultState();
	}
	// move or draw based on the value in moveOrDraw
	virtual void draw() {
		sleep(); // sleep if requested
		if (robot) {
			//bugbug move arm up from paper if !moveOrDraw
			for (const auto& info : infoVector) {
				setPoint(info.point);
				setState(info.settings);
				send(&robot->serial);// move
			}
		}
	}
	
	void setFillMode(int mode) { fillmode = mode; }
	void addPointAndState(const ofPoint& point, const ofPoint& state = ofPoint()) { infoVector.push_back(CommandInfo(point, state)); }

	void sleep() { if (millisSleep > -1) ofSleepMillis(500); }

	int millisSleep = -1;// no sleep by default
	bool moveOrDraw = true; // false means draw
	bool deleteWhenDone = true; // false to repeat command per every draw occurance
	int fillmode = 0;
protected:
	void setPoint(ofPoint pt);
	void setState(ofVec3f pt);
	vector<CommandInfo> infoVector; // one more more points
	Robot *robot = nullptr; // owner
private:
};
// examples
class sanityTestCommand : public Command {
public:
	sanityTestCommand(Robot &robot) :Command(robot) { }
	void draw();
};
// just for fun
class danceCommand : public Command {
public://
	danceCommand(Robot &robot) :Command(robot) { }
	void draw();
};

class rectangleCommand : public Command {
public://
	rectangleCommand(Robot &robot) :Command(robot) { }
	void draw();
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
