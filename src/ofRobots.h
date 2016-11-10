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
		virtual void trace();

		float getX() { return x; }
		float getY() { return y; }
		float getZ() { return z; } // want to make sure we do not access data directly so we can range check

		ofRobotPosition&operator=(const ofRobotPosition&);

	protected:
		bool validRange(float f);

	};
	// helper that masks position (x,y etc)
	class ofRobotArmState : public ofRobotPosition {
	public:
		ofRobotArmState(float wristAngle = NoRobotValue, float wristRotate = NoRobotValue, float gripper = NoRobotValue) :ofRobotPosition(wristAngle, wristRotate, gripper) {  }

		float getWristAngle() { return getPtr()[0]; }
		float getWristRotation()  { return getPtr()[1]; }
		float getGripper()  { return getPtr()[2]; }

		void trace();
	};

	enum RobotCommand { None, Reset, Push, Pop, Move, LowLevelTest, RegressionTest, UserDefinded, Translate, Sleep, RobotCircle, RobotLineTo, RobotMoveTo};// command and basic commands.  Derive object or create functions to create more commands

	// high level interface to robot data data
	class RobotArmCommandData {
	public:

		RobotArmCommandData(const ofRobotPosition&position, const ofRobotArmState&state, uint8_t delta=maxDelta()) {
			this->position = position;
			this->delta = delta;
			this->state = state;
		}
		RobotArmCommandData(float float1 = 0.0f) {  this->float1 = float1; }
		RobotArmCommandData(int int1) { this->int1 = int1; }
		RobotArmCommandData(const ofRobotPosition&position) { this->position = position; }

		ofRobotPosition& getPoint() { return position; }
		ofRobotArmState& getState() { return state; }
		uint8_t& getDelta() { return delta; }
		
		float float1=0.0f;
		int   int1=0;
	
		uint8_t delta;
		ofRobotArmState state;
		ofRobotPosition position;
	};																												

	class ofRobotArmCommand {
	public:
		// commands and only be 0.0 to +/- 1.0 (0 to +/- 100%)
		ofRobotArmCommand(float xPercent, float yPercent = NoRobotValue, float zPercent = NoRobotValue, float wristAnglePercent = NoRobotValue, float wristRotatePercent = NoRobotValue, float gripperPercent = NoRobotValue, uint8_t delta = maxDelta()) {
			add(xPercent, yPercent, zPercent, wristAngle, wristAnglePercent, gripperPercent, delta);
		}
		// object based
		ofRobotArmCommand(const RobotArmCommandData&cmd) {	add(cmd);	}
		ofRobotArmCommand(const RobotCommand&cmd) {	set(cmd);	}
		ofRobotArmCommand(const RobotCommand&cmd, int i) {	set(cmd);	add(RobotArmCommandData(i));	}
		ofRobotArmCommand(const RobotCommand&cmd, const RobotArmCommandData& data) {	setup(cmd, data);		}

		void add(const RobotArmCommandData& data) { vectorOfCommandData.push_back(data); }
		void setup(const RobotCommand&cmd, const RobotArmCommandData& data) { clear(vectorOfCommandData); set(cmd); add(data); }
		void SetDeleteWhenDone(bool b = true) { deleteWhenDone = b; }
		bool OKToDelete() { return deleteWhenDone; }

		void trace();
	    void add(float xPercent, float yPercent = NoRobotValue, float zPercent = NoRobotValue, float wristAnglePercent = NoRobotValue, float wristRotatePercent = NoRobotValue, float gripperPercent = NoRobotValue, uint8_t sleep = maxDelta()) {
			add(RobotArmCommandData(ofRobotPosition(xPercent, yPercent, zPercent), ofRobotArmState(wristAnglePercent, wristRotatePercent, gripperPercent), sleep));
		}
		void add(const ofRobotPosition& position, const ofRobotArmState& state= ofRobotArmState(), uint8_t delta = maxDelta()) {
			add(RobotArmCommandData(position, state, delta));
		}
		void addX(float x= NoRobotValue, uint8_t delta = maxDelta()) {
			add(RobotArmCommandData(ofRobotPosition(x), ofRobotArmState(), delta));
		}
		void addY(float y = NoRobotValue, uint8_t delta = maxDelta()) {
			add(RobotArmCommandData(ofRobotPosition(NoRobotValue, y), ofRobotArmState(), delta));
		}
		void addZ(float z = NoRobotValue, uint8_t delta = maxDelta()) {
			add(RobotArmCommandData(ofRobotPosition(NoRobotValue, NoRobotValue, z), ofRobotArmState(), delta));
		}
		void addGripper(float gripper = NoRobotValue, uint8_t delta = maxDelta()) {
			add(RobotArmCommandData(ofRobotPosition(), ofRobotArmState(NoRobotValue, NoRobotValue, gripper), delta));
		}
		void addWristRotate(float value = NoRobotValue, uint8_t delta = maxDelta()) {
			add(RobotArmCommandData(ofRobotPosition(), ofRobotArmState(NoRobotValue, value, NoRobotValue), delta));
		}
		void addWristAngle(float value = NoRobotValue, uint8_t delta = maxDelta()) {
			add(RobotArmCommandData(ofRobotPosition(), ofRobotArmState(value, NoRobotValue, NoRobotValue), delta));
		}
		static ofRobotArmCommand getSleep(int duration) { return ofRobotArmCommand(Sleep, duration); }

		void set(const RobotCommand& cmd) { this->cmd = cmd; }
		const RobotCommand&getCommand() { return cmd; }
		vector<RobotArmCommandData>&getVectorOfParameters() {	return vectorOfCommandData;	}

	private:
		// one command can have mulitiple data or 0 data
		vector<RobotArmCommandData> vectorOfCommandData;
		RobotCommand cmd = UserDefinded;
		bool deleteWhenDone = true; // false to repeat command per every draw occurance
	};

	class ofTrRobotArm : public ofTrRobotArmInternals {
	public:

		ofTrRobotArm(const string& name, const robotType& type) :ofTrRobotArmInternals(type) { setName(name); }
		ofTrRobotArm() :ofTrRobotArmInternals() {}
		void trace(); 

		// put command data in a known state
		// a robot is required for life of this object
		void setup(robotArmMode);

		// move or draw based on the value in moveOrDraw
		void draw();
		void update();

		void add(const ofRobotArmCommand& cmd) {	vectorOfCommands.push_back(cmd);	}

		void setName(const string&name) { this->name = name; }
		string& getName() { return name; }

		ofTrosseRobotSerial serial; // talking to the robots

	protected:

		void sanityTestHighLevel(vector<ofRobotArmCommand>&commands);
		void circleMacro(vector<ofRobotArmCommand>&commands, float r);
		void lineMacro(vector<ofRobotArmCommand>&commands, const ofRobotPosition& to);
		void moveMacro(vector<ofRobotArmCommand>&commands, const ofRobotPosition& to);
		void penUpMacro(vector<ofRobotArmCommand>&commands);
		void penDownMacro(vector<ofRobotArmCommand>&commands);
		//void translate(float x, float y) { getPose().position.setPercents(x, y);}
		void pushMatrix() { stack.push(pose); }
		void popMatrix();

		void setPoint(ofRobotPosition pt);
		void setState(ofRobotArmState pt);

		int servoCount=0;

	private:
		shared_ptr<RobotValueRanges> userDefinedRanges = nullptr; // none set by default
		string name;
		void validate();
		
		vector<ofRobotArmCommand> vectorOfCommands; // one more more points
		void testdata();
		stack<Pose> stack; // bugbug once working likely to include colors, brush size etc
									  // built in commands
		void sanityTestLowLevel();
		void sendData(vector<RobotArmCommandData>&data);
		void sendExpandedResults(vector<ofRobotArmCommand>& results);
		void set(RobotArmCommandData& request);
	};
	
	// the robot itself
	class ofRobot {
	public:
		
		ofRobot() {}
		ofRobot(const string& name) { 	this->name = name;  }

		void setup();
		void update();
		void draw();
		void trace(); // echos positions

		void setName(const string&name) { this->name = name; }
		string& getName() { return name; }

		vector<shared_ptr<ofTrRobotArm>> arms; // bugbug at some point maybe add wrappers etc when more is known

	private:
		string name;
		vector<ofRobotVoice> voices;//bugbug can be anywhere and also add ofEyes (Kinetc, CV etc)
	};

}