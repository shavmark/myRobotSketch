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

	enum RobotCommand { None, Reset, TrossenMove, LowLevelTest, RegressionTest, UserDefinded, Translate, Sleep, RobotCircle, RobotLineTo, RobotMoveTo, PenPose};// command and basic commands.  Derive object or create functions to create more commands

	// high level interface to robot data data
	class RobotArmCommandData {
	public:

		RobotArmCommandData(const ofRobotPosition&position, const ofRobotArmState&state, uint8_t delta=maxDelta()) {
			this->position = position;
			this->delta = delta;
			this->state = state;
		}
		RobotArmCommandData(float float1 = 0.0f, uint8_t delta = maxDelta()) {  this->float1 = float1; this->delta = delta;		}
		RobotArmCommandData(int int1, uint8_t delta = maxDelta()) { this->int1 = int1; this->delta = delta;		}
		RobotArmCommandData(const ofRobotPosition&position, uint8_t delta = maxDelta()) { this->position = position; this->delta = delta;		}
		RobotArmCommandData(uint8_t delta) {  this->delta = delta; }

		ofRobotPosition& getPoint() { return position; }
		ofRobotArmState& getState() { return state; }
		uint8_t& getDelta() { return delta; }
		
		float float1=0.0f;
		int   int1=0;
	
		uint8_t delta;
		ofRobotArmState state;
		ofRobotPosition position;
	};				

	// one command with data assoicated with that command
	class ofRobotArmCommand {
	public:
		// commands and only be 0.0 to +/- 1.0 (0 to +/- 100%)
		ofRobotArmCommand(float xPercent, float yPercent = NoRobotValue, float zPercent = NoRobotValue, float wristAnglePercent = NoRobotValue, float wristRotatePercent = NoRobotValue, float gripperPercent = NoRobotValue, uint8_t delta = maxDelta()) {
			addParameter(xPercent, yPercent, zPercent, wristAngle, wristAnglePercent, gripperPercent, delta);
		}
		// object based
		ofRobotArmCommand(const RobotArmCommandData&parameter) {	addParameter(parameter);	}
		ofRobotArmCommand(const RobotCommand&cmd, uint8_t delta = maxDelta()) { set(cmd);	addDelta(delta); }
		ofRobotArmCommand(const RobotCommand&cmd, int i, uint8_t delta = maxDelta()) {	set(cmd);	addParameter(RobotArmCommandData(i, delta));	}
		ofRobotArmCommand(const RobotCommand&cmd, const RobotArmCommandData& parameter) {	setup(cmd, parameter);	}
		ofRobotArmCommand(robotArmJointType type, float value, uint8_t delta = maxDelta());

		void setup(const RobotCommand&cmd, const RobotArmCommandData& data);

		void SetDeleteWhenDone(bool b = true) { deleteWhenDone = b; }
		bool OKToDelete() { return deleteWhenDone; }

		void trace();
		
		void addDelta(uint8_t delta) { addParameter(RobotArmCommandData(delta)); }
		void addParameter(robotArmJointType type, float value, uint8_t delta = maxDelta());
		void addParameter(const RobotArmCommandData& data) { vectorOfCommandData.push_back(data); }
		void addParameter(float xPercent, float yPercent = NoRobotValue, float zPercent = NoRobotValue, float wristAnglePercent = NoRobotValue, float wristRotatePercent = NoRobotValue, float gripperPercent = NoRobotValue, uint8_t sleep = maxDelta()) {
			addParameter(RobotArmCommandData(ofRobotPosition(xPercent, yPercent, zPercent), ofRobotArmState(wristAnglePercent, wristRotatePercent, gripperPercent), sleep));
		}
		void addParameter(const ofRobotPosition& position, const ofRobotArmState& state= ofRobotArmState(), uint8_t delta = maxDelta()) {
			addParameter(RobotArmCommandData(position, state, delta));
		}

		void set(const RobotCommand& cmd) { this->cmd = cmd; }

		const RobotCommand&getCommand() { return cmd; }
		vector<RobotArmCommandData>&getVectorOfParameters() {	return vectorOfCommandData;	}
		vector<RobotArmCommandData> vectorOfCommandData;

	private:
		// one command can have mulitiple data or 0 data
		RobotCommand cmd = UserDefinded;
		bool deleteWhenDone = true; // false to repeat command per every draw occurance
	};


	class ofTrRobotArm : public ofTrRobotArmInternals {
	public:
		typedef void(ofTrRobotArm::*pArmfunction)(vector<ofRobotArmCommand>&commands, vector<RobotArmCommandData>&data);

		ofTrRobotArm(const string& name, const robotType& type) :ofTrRobotArmInternals(type) {	setName(name); 		}
		ofTrRobotArm() :ofTrRobotArmInternals() {	}

		void trace(); 

		// put command data in a known state
		// a robot is required for life of this object
		void setup(robotMode);

		// move or draw based on the value in moveOrDraw
		void draw();
		void update();

		void add(const ofRobotArmCommand& cmd) {	vectorOfCommands.push_back(cmd);	}

	protected:
		//bugbug need to get the delta in here too
		void penPose(vector<ofRobotArmCommand>&commands, vector<RobotArmCommandData>&data);
		void regressionTest(vector<ofRobotArmCommand>&commands, vector<RobotArmCommandData>&data);
		void sanityTestLowLevel(vector<ofRobotArmCommand>&commands, vector<RobotArmCommandData>&data);
		void circleMacro(vector<ofRobotArmCommand>&commands, vector<RobotArmCommandData>&data);
		void lineMacro(vector<ofRobotArmCommand>&commands, vector<RobotArmCommandData>&data);
		void moveMacro(vector<ofRobotArmCommand>&commands, vector<RobotArmCommandData>&data);
		void justSendItNowMacro(vector<ofRobotArmCommand>&commands, vector<RobotArmCommandData>&data);

		void setPoint(ofRobotPosition pt);
		void setState(ofRobotArmState pt);

		int servoCount=0;

	private:
		map<RobotCommand, pArmfunction> funcMap;
		shared_ptr<RobotValueRanges> userDefinedRanges = nullptr; // none set by default
		void validate();

		vector<ofRobotArmCommand> vectorOfCommands; // one more more points
		void testdata();
		void sendData(vector<RobotArmCommandData>&data);
		void sendExpandedResults(vector<ofRobotArmCommand>& results);
		void set(RobotArmCommandData& request);

	};
	
	class xyRobot : public iRobot {
	public:
		xyRobot() : iRobot() {  }
		xyRobot(shared_ptr<ofRobotSerial>  driver) :iRobot(driver) {  }

		void setup();
		void update(xyDataToSend&); // direct access, not put in vector
		void draw();

		void add(XYCommands cmd, const xyMotion& point);
		void add(XYCommands cmd, const vector<xyMotion>& points);
		void setColor(const ofColor& color) { currentColor = color; } //bugbug this is big, needs to be designed

		void setFill(bool fill = true) {} // bugbug big, needs to be figured out
		void setFillType() {} //bugbug big, needs to be figured, out things like Heavy, light, wait to dry etc
		void spritz() {} // keep things moist
		void getStrokeColor() {}
		void setStrokeWidth() {}

		void rectangleMacro(const xyMotion& point2, const xyMotion& point3, const xyMotion& point4, float angle = 0);
		void rotate(const xyMotion& center, float angle, xyMotion& point);

		void add(const xyDataToSend& cmd) { vectorOfCommands.push_back(cmd); }
		size_t getCount() { return vectorOfCommands.size(); }

	private:
		bool readResults(int8_t cmd);
		vector<xyDataToSend> vectorOfCommands;
		void sendit(xyDataToSend&data);
		ofColor currentColor;//bugbug build color soon...
		int sentAtAtime = 3; // packets to send at one time

	};


	// the robot itself
	class ofMasterRobot : public BasicBot {
	public:
		ofMasterRobot() : BasicBot() {}
		ofMasterRobot(const string& name) : BasicBot(name) {  }

		void setup(const vector<string>& arguments);
		bool setup(RobotBrand id, int port, int baudrate);

		void update();
		void draw();
		void trace() const;

		shared_ptr<ofTrRobotArm>getTrossen(int index);
		shared_ptr<xyRobot>getXyRobot(int index);

	private:
		// a robot contains other robots
		vector<shared_ptr<iRobot>> robotHelpers; // bugbug at some point maybe add wrappers etc when more is known
		vector<ofRobotVoice> voices;//bugbug can be anywhere and also add ofEyes (Kinetc, CV etc)
	};

}