/*
ofRobots.h - openframeworks based classes for managing robots
Copyright (c) 2016 Mark J Shavlik.  All right reserved.This file is part of myRobotSketch.

myRobotSketch is free software : you can redistribute it and / or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

myRobotSketch is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with myRobotSketch.If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once
#include <stack>
#include <tuple>
// robots

namespace RobotArtists {

	class ofRobotSerial : public ofSerial {
	public:
		ofRobotSerial() {}

		bool waitForSerial(int retries);
		void clearSerial() { flush(); }
		int readAllBytes(uint8_t* bytes, int bytesRequired = 5);
		int readBytesInOneShot(uint8_t* bytes, int bytesMax = 100);
		void readPose();
		void write(uint8_t* data, int count);
		robotType waitForRobot(int retries);
		vector <ofSerialDeviceInfo>& getDevices() {
			buildDeviceList();
			return devices;
		}
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
		virtual void echo();
		float getX() { return x; }
		float getY() { return y; }
		float getZ() { return z; } // want to make sure we do not access data directly so we can range check

		bool set[3];

	protected:
		bool validRange(float f);
	};

	class ofRobotState : public ofRobotPosition {
	public:
		ofRobotState(float wristAngle = FLT_MAX, float wristRotate = FLT_MAX, float gripper = FLT_MAX) :ofRobotPosition(wristAngle, wristRotate, gripper) {  }

		float getWristAngle() { return getPtr()[0]; }
		float getWristRotation()  { return getPtr()[1]; }
		float getGripper()  { return getPtr()[2]; }

		void echo();

	};

	enum RobotCommand { None, Reset, Push, Pop, Move, LowLevelTest, HighLevelTest, UserDefinded, Translate, Sleep, Circle };// command and basic commands.  Derive object or create functions to create more commands

	class RobotCommandData {
	public:
		typedef std::tuple<ofRobotPosition, ofRobotState, uint8_t> robotArmCommandData;

		RobotCommandData(const robotArmCommandData& data) { this->data = data; }
		RobotCommandData(float float1 = 0.0f) {  this->float1 = float1; }
		RobotCommandData(int int1 = 0) { this->int1 = int1; }
	
		RobotCommandData(ofRobotPosition pos, ofRobotState state, uint8_t delta) {
			data = robotArmCommandData(pos, state, delta);
		}

		ofRobotPosition& getPoint() { return std::get<0>(data); }
		ofRobotState& getState() { return std::get<1>(data); }
		uint8_t& getDelta() { return std::get<2>(data); }

		robotArmCommandData data;
		
		float float1;//bugbug abstract out type
		int   int1;

	};																												//pos, state, delta time 

	class ofRobotCommand {
	public:
		// commands and only be 0.0 to +/- 1.0 (0 to +/- 100%)
		ofRobotCommand(float xPercent, float yPercent = NoRobotValue, float zPercent = NoRobotValue, float wristAnglePercent = NoRobotValue, float wristRotatePercent = NoRobotValue, float gripperPercent = NoRobotValue, uint8_t delta = RobotBaseClass::maxDelta()) {
			add(xPercent, yPercent, zPercent, wristAngle, wristAnglePercent, gripperPercent, delta);
		}
		// object based
		ofRobotCommand(const RobotCommandData&cmd) {
			add(cmd);
		}
		ofRobotCommand(const RobotCommand&cmd) {
			this->cmd = cmd;
		}
		ofRobotCommand(const RobotCommand&cmd, int i) {
			this->cmd = cmd;
			add(RobotCommandData(i));
		}
		void add(const RobotCommandData& data) { vectorOfCommandData.push_back(data); }

		void addSay(const string& say) { voice.add(say); }

		void SetDeleteWhenDone(bool b = true) { deleteWhenDone = b; }
		bool OKToDelete() { return deleteWhenDone; }

		void echo();
	    void add(float xPercent, float yPercent = NoRobotValue, float zPercent = NoRobotValue, float wristAnglePercent = NoRobotValue, float wristRotatePercent = NoRobotValue, float gripperPercent = NoRobotValue, uint8_t sleep = RobotBaseClass::minDelta()) {
			add(RobotCommandData(ofRobotPosition(xPercent, yPercent, zPercent), ofRobotState(wristAnglePercent, wristRotatePercent, gripperPercent), sleep));
		}
		void add(const ofRobotPosition& position, const ofRobotState& state= ofRobotState(), uint8_t delta = RobotState::minDelta()) {
			add(RobotCommandData(position, state, delta));
		}

		// one command can have mulitiple data or 0 data
		vector<RobotCommandData> vectorOfCommandData;		
		RobotCommand cmd= UserDefinded;
	private:
		bool deleteWhenDone = true; // false to repeat command per every draw occurance
		ofRobotVoice voice;
		
	};

	//bugbug work on naming, "of" vs. "Trossen" or maybe tr and of?
	class ofRobotJoints : public RobotJoints {
	public:
		ofRobotJoints(uint8_t *data) : RobotJoints(data) {}
		ofRobotJoints() : RobotJoints(nullptr) {}
		ofRobotJoints(const robotType& typeOfRobot) : RobotJoints(nullptr, typeOfRobot) { };
		ofRobotJoints(uint8_t* data, const robotType& typeOfRobot) : RobotJoints(data, typeOfRobot) {  }

		virtual void echo() {};
		void sendToRobot(ofRobotSerial* serial);
	};

	class ofRobot;

	class ofRobotCommands : protected ofRobotJoints {
	public:

	    // a robot is required for life of this object
		ofRobotCommands(ofRobot *robot);

		void echo(); 

		void sendData(RobotCommandData&data);
		void sendResults(vector<RobotCommandData>& results);
		void sendResults() {
			sendResults(results);
		}
		// put command data in a known state
		void reset();

		// move or draw based on the value in moveOrDraw
		virtual void draw();
		void update();
		void setFillMode(int mode) { fillmode = mode; }
		void add(const ofRobotCommand& cmd);
		void add(RobotCommandData&data) { results.push_back(data); }
		bool moveOrDraw = true; // false means draw
		int fillmode = 0;
		void sleep(int millisSleep) const { if (millisSleep > -1) ofSleepMillis(millisSleep); }
		void sanityTestHighLevel();
		void drawCircle(float r);
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
		
		ofRobot *robot = nullptr; // owner

	private:
		vector<RobotCommandData> results; // results of executing commands
		vector<ofRobotCommand> vectorOfRobotCommands; // one more more points
		void testdata();
		ofRobotPosition currentPosition; // default to 0,0,0
		stack<ofRobotPosition> stack; // bugbug once working likely to include colors, brush size etc
									  // built in commands
		void sanityTestLowLevel();

		void move(const ofRobotPosition& pos);
		void set(RobotCommandData& request);
	};

	// the robot itself
	class ofRobot {
	public:
		
		friend class ofRobotCommands;

		ofRobot(const string& name, robotType type) { 
			this->name = name;  
			this->type = type; 
		}

		void setup(int deviceID);
		void update();
		void draw();
		void echo(); // echos positions
		void setPause(bool pause = true) { this->pause = pause; }//bugbug go to threads

		shared_ptr<ofRobotCommands> commands=nullptr;

		void setName(const string&name) { this->name = name; }
		string&getName() { return name; }

		RobotTypeID getTypeID() { return type.second; }

		shared_ptr<RobotValueRanges> userDefinedRanges=nullptr; // none set by default

	private:
		ofRobotSerial serial; // talking to the robots
		uint8_t data[RobotState::count];// one data instance per robot
		robotType type;
		bool pause = false;
		string name;
	};

	// all robots being managed
	class ofRobotFamly {
	public:

		void setup();
		void update();
		void draw();

		//InterbotiXPhantomXPincherArm for example
		void set(RobotTypeID id= AllRobotTypes) {
			ofLogNotice() << "set id " << id;
			idToUse = id;
		}

		shared_ptr<ofRobot> getRobot(int index, RobotTypeID id = AllRobotTypes);

	private:
		RobotTypeID idToUse= AllRobotTypes;
		vector<shared_ptr<ofRobot>> robots;
	};

}