#pragma once

// robots

namespace RobotArtists {

	class ofRobotTrace : public TraceBaseClass {
	public:
		ofRobotTrace(TraceType type = TraceLog) : TraceBaseClass(type) {}
		virtual void sendErrorline() {
			ofLogError() << formatMessage(); //bugbug support all log types from OF
		}
		virtual void sendLogline() {
			ofLogNotice() << formatMessage();
		}
	};
	class ofRobotTraceNetwork : public TraceBaseClass {
	public:
		ofRobotTraceNetwork(TraceType type = TraceLog) : TraceBaseClass(type) {}
		virtual void sendErrorline() {
			ofLogError() << formatMessage(); //bugbug support all log types from OF
		}
		virtual void sendLogline() {
			ofLogNotice() << formatMessage();
		}
	};

	class ofRobotSay : public TraceBaseClass {
	public:
		//bugbug send to say code via network
		virtual void sendErrorline() {
			ofLogError() << formatMessage(); //bugbug support all log types from OF
		}
		virtual void sendLogline() {
			ofLogNotice() << formatMessage();
		}
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



	class ofRobotCommand {
	public:
		// commands and only be 0.0 to +/- 1.0 (0 to +/- 100%)
		ofRobotCommand(float xPercent, float yPercent = NoRobotValue, float zPercent = NoRobotValue, float wristAnglePercent = NoRobotValue, float wristRotatePercent = NoRobotValue, float gripperPercent = NoRobotValue, int millisSleep = -1, bool deleteWhenDone = true) {
			init(ofRobotPosition(xPercent, yPercent, zPercent), ofRobotState(wristAnglePercent, wristRotatePercent, gripperPercent), millisSleep, deleteWhenDone);
		}
		// object based
		ofRobotCommand(const ofRobotPosition& pointPercent, const ofRobotState& settingsPercent = ofRobotState(), int millisSleep = -1, bool deleteWhenDone = true) {
			init(pointPercent, settingsPercent, millisSleep, deleteWhenDone);
		}
		// make a sleep only command
		ofRobotCommand(int millisSleep = -1, bool deleteWhenDone = true) {
			init(ofRobotPosition(), ofRobotState(), millisSleep, deleteWhenDone);
		}

		void addSay(const string& say) { voice.add(say); }
		void sleep() const { if (millisSleep > -1) ofSleepMillis(millisSleep); }
		bool OKToDelete() { return deleteWhenDone; }
		void echo() const;

		ofRobotPosition pointPercent;
		ofRobotState settingsPercent;

	private:
		void init(const ofRobotPosition& pointPercent = ofRobotPosition(), const ofRobotState& settingsPercent = ofRobotState(), int millisSleep = -1, bool deleteWhenDone = true);
		bool deleteWhenDone; // false to repeat command per every draw occurance
		int millisSleep;// no sleep by default
		ofRobotVoice voice;

	};

	class ofRobot;

	class ofRobotCommands : protected RobotJoints {
	public:

		enum BuiltInCommandNames { UserDefined, LowLevelTest, HighLevelTest, Sizing };// command and basic commands.  Derive object or create functions to create more commands

																					  // passed robot cannot go away while this object exists
		ofRobotCommands(ofRobot *robot, BuiltInCommandNames);
		ofRobotCommands(BuiltInCommandNames name) :RobotJoints(nullptr) { this->name = name; }

		void echo() const; // echos positions

		void send(ofRobotSerial* serial);

		// put command data in a known state
		void reset();

		// move or draw based on the value in moveOrDraw
		virtual void draw();
		void setFillMode(int mode) { fillmode = mode; }
		void add(const ofRobotCommand& cmd, BuiltInCommandNames name = UserDefined);
		bool moveOrDraw = true; // false means draw
		int fillmode = 0;

	protected:
		BuiltInCommandNames getName() { return name; };
		void setPoint(ofRobotPosition pt);
		void setState(ofRobotState pt);
		vector<ofRobotCommand> cmdVector; // one more more points
		ofRobot *robot = nullptr; // owner

	private:
		void sanityTestLowLevel(); // built in commands
		void sanityTestHighLevel();
		void sizing();
		BuiltInCommandNames name;
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

		shared_ptr<ofRobotCommands> add(ofRobotCommands::BuiltInCommandNames name = ofRobotCommands::UserDefined);

		robotType& getType() { return type; }

	protected:
		RobotValueRanges userDefinedRanges;

	private:
		ofRobotSerial serial; // talking to the robot
		uint8_t data[RobotJointsState::count];// one data instance per robot
		robotType type;
		vector<shared_ptr<ofRobotCommands>> cmds;
		bool pause = false;
	};

	class ofDrawingRobot : public ofRobot {
	public:
		void setup();
	};

}