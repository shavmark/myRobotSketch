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
		float getX() const { return x; }
		float getY() const { return y; }
		float getZ() const { return z; } // want to make sure we do not access data directly so we can range check

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
		//pos, state, sleep 
		typedef std::tuple<ofRobotPosition, ofRobotState, RobotArmDelta> robotCommandRequest;

		enum RobotCommand { None, UserDefinded, Translate, Sleep, Circle };// command and basic commands.  Derive object or create functions to create more commands

		// commands and only be 0.0 to +/- 1.0 (0 to +/- 100%)
		ofRobotCommand(float xPercent, float yPercent = NoRobotValue, float zPercent = NoRobotValue, float wristAnglePercent = NoRobotValue, float wristRotatePercent = NoRobotValue, float gripperPercent = NoRobotValue, RobotArmDelta delta = 255) {
			add(setCommand(xPercent, yPercent, zPercent, wristAngle, wristAnglePercent, gripperPercent, delta));
			type = UserDefinded;
		}
		// object based
		ofRobotCommand(const robotCommandRequest&state) {
			add(state);
			type = UserDefinded;
		}
		ofRobotCommand(RobotCommand command) { type = command; }
		ofRobotCommand(RobotCommand command, float f) { type = command; floatdata = f; }

		void add(const robotCommandRequest& data) { vectorData.push_back(data); }

		void addSay(const string& say) { voice.add(say); }

		void SetDeleteWhenDone(bool b = true) { deleteWhenDone = b; }
		bool OKToDelete() { return deleteWhenDone; }

		void echo() const;
		static robotCommandRequest setCommand(float xPercent, float yPercent = NoRobotValue, float zPercent = NoRobotValue, float wristAnglePercent = NoRobotValue, float wristRotatePercent = NoRobotValue, float gripperPercent = NoRobotValue, RobotArmDelta sleep = RobotJointsState::slowestDelta) {
			return robotCommandRequest(ofRobotPosition(xPercent, yPercent, zPercent), ofRobotState(wristAnglePercent, wristRotatePercent, gripperPercent), sleep);
		}
		static robotCommandRequest setCommand(const ofRobotPosition& position, const ofRobotState& state= ofRobotState(), RobotArmDelta sleep = RobotJointsState::slowestDelta) {
			return robotCommandRequest(position, state, sleep);
		}
		vector<robotCommandRequest> vectorData;
		RobotCommand commandType() { return type; }
		void drawCircle();
		float getFloatData() { return floatdata; }
	private:
		bool deleteWhenDone = true; // false to repeat command per every draw occurance
		ofRobotVoice voice;
		RobotCommand type= UserDefinded;
		float floatdata=0.0f;
	};

	class ofRobot;

	class ofRobotCommands : protected RobotJoints {
	public:
		enum BuiltInCommandNames { Reset, LowLevelTest, HighLevelTest, Push, Pop, Move };// command and basic commands.  Derive object or create functions to create more commands

	    // a robot is required for life of this object
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
		void sleep(int millisSleep) const { if (millisSleep > -1) ofSleepMillis(millisSleep); }

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
		void testdata();
		ofRobotPosition currentPosition; // default to 0,0,0
		stack<ofRobotPosition> stack; // bugbug once working likely to include colors, brush size etc
									  // built in commands
		void sanityTestLowLevel();
		void sanityTestHighLevel();
		void move(const ofRobotPosition& pos);
		void setTuple(ofRobotCommand::robotCommandRequest request);
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

		shared_ptr<ofRobotCommands> commands=nullptr;

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