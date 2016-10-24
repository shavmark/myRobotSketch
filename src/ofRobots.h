#pragma once
#include <stack>

// robots

namespace RobotArtists {

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
	class ofRobotPosition : public ofPoint {
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

	class ofRobotCommand {
	public:

		typedef std::pair<ofRobotPosition, ofRobotState> robotCommandState;

		// commands and only be 0.0 to +/- 1.0 (0 to +/- 100%)
		ofRobotCommand(float xPercent, float yPercent = NoRobotValue, float zPercent = NoRobotValue, float wristAnglePercent = NoRobotValue, float wristRotatePercent = NoRobotValue, float gripperPercent = NoRobotValue) {
			add(robotCommandState(ofRobotPosition(xPercent, yPercent, zPercent), ofRobotState(wristAnglePercent, wristRotatePercent, gripperPercent)));
		}
		// object based
		ofRobotCommand(const robotCommandState&state) {
			add(state);
		}
		ofRobotCommand(int millisSleep) { setSleep(millisSleep); }

		void add(const robotCommandState& data) { vectorData.push_back(data); }

		void addSay(const string& say) { voice.add(say); }

		void SetDeleteWhenDone(bool b = true) { deleteWhenDone = b; }
		void setSleep(int value) { millisSleep = value; }
		void sleep() const { if (millisSleep > -1) ofSleepMillis(millisSleep); }
		bool OKToDelete() { return deleteWhenDone; }

		void echo() const;

		vector<robotCommandState> vectorData;

	private:
		bool deleteWhenDone = true; // false to repeat command per every draw occurance
		int millisSleep = -1;// no sleep by default
		ofRobotVoice voice;

	};

	class ofRobot;

	class ofRobotCommands : protected RobotJoints {
	public:
		ofRobotCommands() {}
		enum BuiltInCommandNames { Reset, UserDefined, LowLevelTest, HighLevelTest, Translate, Push, Pop, Sleep };// command and basic commands.  Derive object or create functions to create more commands

	    // passed robot cannot go away while this object exists
		ofRobotCommands(ofRobot *robot);

		void echo() const; // echos positions

		void send(ofRobotSerial* serial);

		// put command data in a known state
		void reset();

		// move or draw based on the value in moveOrDraw
		virtual void draw();
		void setFillMode(int mode) { fillmode = mode; }
		void add(const ofRobotCommand& cmd);
		bool moveOrDraw = true; // false means draw
		int fillmode = 0;
		BuiltInCommandNames getName() { return name; };

	protected:
		void translate(float x, float y) {currentPosition.setPercents(x, y);}
		void pushMatrix() { stack.push(currentPosition); }
		void popMatrix() { 
			if (stack.size() == 0) {
				ofRobotTrace(ErrorLog) << "popMatrix on empty stack" << std::endl;
			}
			else {
				currentPosition = stack.top();
				stack.pop();
			}
		}
		void setPoint(ofRobotPosition pt);
		void setState(ofRobotState pt);
		vector<ofRobotCommand> cmdVector; // one more more points
		ofRobot *robot = nullptr; // owner

	private:
		BuiltInCommandNames name;

		ofRobotPosition currentPosition; // default to 0,0,0
		stack<ofRobotPosition> stack; // bugbug once working likely to include colors, brush size etc
									  // built in commands
		void sanityTestLowLevel();
		void sanityTestHighLevel();
		void userDefined();
		void DrawCircle(double radius);

	};

	// the robot itself
	class ofRobot {
	public:
		
		friend class ofRobotCommands;
		void setup();
		void update();
		void draw();
		void echo(); // echos positions
		void setPause(bool pause = true) { this->pause = pause; }//bugbug go to threads

		ofRobotCommands commands;

		robotType& getType() { return type; }

	protected:
		shared_ptr<RobotValueRanges> userDefinedRanges=nullptr; // none set by default

	private:
		ofRobotSerial serial; // talking to the robot
		uint8_t data[RobotJointsState::count];// one data instance per robot
		robotType type;
		bool pause = false;
	};

}