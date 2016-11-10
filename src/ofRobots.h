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

	class ofRobotVoice {
	public:
		void draw() {}//bugbug enumerate and say, bring in SAPI 11 or such
		void add(const string& say) { thingsToSay.push_back(say); }
		vector<string> thingsToSay;
	};

	#define NoRobotValue FLT_MAX
	inline bool valueIsSet(float v) { return v != NoRobotValue; }

	// positions are defined as % change of all range of joint, from the current position
	// RobotPositions can only be 0.0 to +/- 1.0 (0 to +/- 100%)
	class ofRobotPosition : public ofPoint {
	public:
		//=FLT_MAX means not set
		ofRobotPosition(float xPercent = NoRobotValue, float yPercent = NoRobotValue, float zPercent = NoRobotValue) { setPercents(xPercent, yPercent, zPercent); }
		
		void setPercents(float xPercent = NoRobotValue, float yPercent = NoRobotValue, float zPercent = NoRobotValue);
		virtual void echo();

		float getX() { return x; }
		float getY() { return y; }
		float getZ() { return z; } // want to make sure we do not access data directly so we can range check

		ofRobotPosition&operator=(const ofRobotPosition&);

	protected:
		bool validRange(float f);

	};

	class ofRobotState : public ofRobotPosition {
	public:
		ofRobotState(float wristAngle = NoRobotValue, float wristRotate = NoRobotValue, float gripper = NoRobotValue) :ofRobotPosition(wristAngle, wristRotate, gripper) {  }

		float getWristAngle() { return getPtr()[0]; }
		float getWristRotation()  { return getPtr()[1]; }
		float getGripper()  { return getPtr()[2]; }

		void echo();

	};

	enum RobotCommand { None, Reset, Push, Pop, Move, LowLevelTest, HighLevelTest, UserDefinded, Translate, Sleep, RobotCircle, RobotLineTo, RobotMoveTo};// command and basic commands.  Derive object or create functions to create more commands

	// high level interface to robot data data
	class RobotCommandData {
	public:

		RobotCommandData(const ofRobotPosition&position, const ofRobotState&state, uint8_t delta=maxDelta()) {
			this->position = position;
			this->delta = delta;
			this->state = state;
		}
		RobotCommandData(float float1 = 0.0f) {  this->float1 = float1; }
		RobotCommandData(int int1) { this->int1 = int1; }
		RobotCommandData(const ofRobotPosition&position) { this->position = position; }

		ofRobotPosition& getPoint() { return position; }
		ofRobotState& getState() { return state; }
		uint8_t& getDelta() { return delta; }
		
		float float1=0.0f;
		int   int1=0;
	
		uint8_t delta;
		ofRobotState state;
		ofRobotPosition position;
	};																												

	class ofRobotCommand {
	public:
		// commands and only be 0.0 to +/- 1.0 (0 to +/- 100%)
		ofRobotCommand(float xPercent, float yPercent = NoRobotValue, float zPercent = NoRobotValue, float wristAnglePercent = NoRobotValue, float wristRotatePercent = NoRobotValue, float gripperPercent = NoRobotValue, uint8_t delta = maxDelta()) {
			add(xPercent, yPercent, zPercent, wristAngle, wristAnglePercent, gripperPercent, delta);
		}
		// object based
		ofRobotCommand(const RobotCommandData&cmd) {
			add(cmd);
		}
		ofRobotCommand(const RobotCommand&cmd) {
			set(cmd);
		}
		ofRobotCommand(const RobotCommand&cmd, int i) {
			set(cmd);
			add(RobotCommandData(i));
		}
		ofRobotCommand(const RobotCommand&cmd, const RobotCommandData& data) {
			reset(cmd, data);
		}
		void add(const RobotCommandData& data) { vectorOfCommandData.push_back(data); }

		void reset(const RobotCommand&cmd, const RobotCommandData& data) { clear(vectorOfCommandData); set(cmd); add(data); }
		void reset() { clear(vectorOfCommandData); set(UserDefinded);  }

		void addSay(const string& say) { voice.add(say); }

		void SetDeleteWhenDone(bool b = true) { deleteWhenDone = b; }
		bool OKToDelete() { return deleteWhenDone; }

		void echo();
	    void add(float xPercent, float yPercent = NoRobotValue, float zPercent = NoRobotValue, float wristAnglePercent = NoRobotValue, float wristRotatePercent = NoRobotValue, float gripperPercent = NoRobotValue, uint8_t sleep = maxDelta()) {
			add(RobotCommandData(ofRobotPosition(xPercent, yPercent, zPercent), ofRobotState(wristAnglePercent, wristRotatePercent, gripperPercent), sleep));
		}
		void add(const ofRobotPosition& position, const ofRobotState& state= ofRobotState(), uint8_t delta = maxDelta()) {
			add(RobotCommandData(position, state, delta));
		}
		void addX(float x= NoRobotValue, uint8_t delta = maxDelta()) {
			add(RobotCommandData(ofRobotPosition(x), ofRobotState(), delta));
		}
		void addY(float y = NoRobotValue, uint8_t delta = maxDelta()) {
			add(RobotCommandData(ofRobotPosition(NoRobotValue, y), ofRobotState(), delta));
		}
		void addZ(float z = NoRobotValue, uint8_t delta = maxDelta()) {
			add(RobotCommandData(ofRobotPosition(NoRobotValue, NoRobotValue, z), ofRobotState(), delta));
		}
		void addGripper(float gripper = NoRobotValue, uint8_t delta = maxDelta()) {
			add(RobotCommandData(ofRobotPosition(), ofRobotState(NoRobotValue, NoRobotValue, gripper), delta));
		}
		void addWristRotate(float value = NoRobotValue, uint8_t delta = maxDelta()) {
			add(RobotCommandData(ofRobotPosition(), ofRobotState(NoRobotValue, value, NoRobotValue), delta));
		}
		void addWristAngle(float value = NoRobotValue, uint8_t delta = maxDelta()) {
			add(RobotCommandData(ofRobotPosition(), ofRobotState(value, NoRobotValue, NoRobotValue), delta));
		}
		static ofRobotCommand getSleep(int duration) { return ofRobotCommand(Sleep, duration); }

		void set(const RobotCommand& cmd) { this->cmd = cmd; }
		const RobotCommand&getCommand() { return cmd; }
		vector<RobotCommandData>&getVector() {	return vectorOfCommandData;	}

	private:
		// one command can have mulitiple data or 0 data
		vector<RobotCommandData> vectorOfCommandData;
		RobotCommand cmd = UserDefinded;
		bool deleteWhenDone = true; // false to repeat command per every draw occurance
		ofRobotVoice voice;
	};

	class ofRobot;

	class ofRobotCommands : protected ofRobotJoints {
	public:
		void echo(); 

		// put command data in a known state
		// a robot is required for life of this object
		void setup(ofRobot*, robotArmMode);

		// move or draw based on the value in moveOrDraw
		virtual void draw();
		void update();

		void add(const ofRobotCommand& cmd) {
			vectorOfRobotCommands.push_back(cmd);
		}

	protected:

		void sanityTestHighLevel(vector<ofRobotCommand>&commands);
		void circle(vector<ofRobotCommand>&commands, float r);
		void line(vector<ofRobotCommand>&commands, const ofRobotPosition& to);
		void move(vector<ofRobotCommand>&commands, const ofRobotPosition& to);
		void penUp(vector<ofRobotCommand>&commands);
		void penDown(vector<ofRobotCommand>&commands);
		//void translate(float x, float y) { getPose().position.setPercents(x, y);}
		void pushMatrix() { stack.push(pose); }
		void popMatrix() { 
			if (stack.size() == 0) {
				ofRobotTrace(ErrorLog) << "popMatrix on empty stack" << std::endl;
			}
			else {
				setPose(stack.top());
				stack.pop();
			}
		}
		void setPoint(ofRobotPosition pt);
		void setState(ofRobotState pt);
		
		ofRobot *robot = nullptr; // owner

	private:
		vector<ofRobotCommand> expandedResults;
		vector<ofRobotCommand> vectorOfRobotCommands; // one more more points
		void testdata();
		stack<Pose> stack; // bugbug once working likely to include colors, brush size etc
									  // built in commands
		void sanityTestLowLevel();
		void sendData(vector<RobotCommandData>&data);
		void sendExpandedResults(vector<ofRobotCommand>& results);
		void move(const ofRobotPosition& pos);
		void set(RobotCommandData& request);
	};

	// the robot itself
	class ofRobot {
	public:
		
		friend class ofRobotCommands;
		ofRobot() {}
		ofRobot(const string& name, robotType type) { 
			this->name = name;  
			this->info = type;
		}

		void setup();
		void update();
		void draw();
		void echo(); // echos positions
		void setPause(bool pause = true) { this->pause = pause; }//bugbug go to threads

		ofRobotCommands commands;

		void setName(const string&name) { this->name = name; }
		string& getName() { return name; }

		shared_ptr<RobotValueRanges> userDefinedRanges=nullptr; // none set by default

		void validate();
		ofTrosseRobotSerial serial; // talking to the robots
		ArmInfo info;

	private:
		
		int servoCount;
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