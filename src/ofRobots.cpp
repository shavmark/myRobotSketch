#include "ofApp.h"
#include <algorithm> 
#define _USE_MATH_DEFINES
#include <math.h>

namespace RobotArtists {

	// echo, ignoring null bytes
	void ofRobotSerial::echoRawBytes(uint8_t *bytes, int count) {
		std::stringstream buffer;
		for (int i = 0; i < count; ++i) {
			buffer << " bytes[" << i << "] = " << (int)bytes[i]; // echo in one line
		}
		ofRobotTrace() << buffer.str() << std::endl;
	}

	// get pose data from serial port bugbug decode this
	void ofRobotSerial::readPose() {
		if (available() == 0) {
			return;
		}

		ofRobotTrace() << "RobotSerial::readPose" << std::endl;

		uint8_t *bytes = new uint8_t[1000];//bugbug clean this up
		int i = 0;
		for (; i < 1000; ++i) {
			if (readBytesInOneShot(&bytes[i], 1) == 1) {
				if (bytes[i] == '\r') {
					i++;
					readBytesInOneShot(&bytes[i], 1);// get other eol marker
					i++;
					break;
				}
			}
			else {
				break; // data is messed up, try to move on
			}
		}
		bytes[i] = 0;
		if (i == 5) { // input data is fixed size format etc, just trace stuff
			ArmIDResponsePacket(bytes);
		}
		else if (i == 10) {
			echoRawBytes(bytes, 10);
			ArmIDResponsePacket(bytes); // 2 signs ons come back some times, likely a timing issue?
			ArmIDResponsePacket(bytes); // first 2 bytes are sign on type
		}
		else {
			ofRobotTrace() << bytes << std::endl;
		}
		delete bytes;

	}
	int ofRobotSerial::readBytesInOneShot(uint8_t *bytes, int bytesMax) {
		int result = 0;
		if (available() > 0) {
			if ((result = readBytes(bytes, bytesMax)) == OF_SERIAL_ERROR) {
				ofRobotTrace(ErrorLog) << "serial failed" << std::endl;
				return 0;
			}
			while (result == OF_SERIAL_NO_DATA) {
				result = readBytes(bytes, bytesMax);
				if (result == OF_SERIAL_ERROR) {
					ofRobotTrace(ErrorLog) << "serial failed" << std::endl;
					return 0;
				}
				if (result != OF_SERIAL_NO_DATA) {
					return result;
				}
			}
		}
		return result;
	}
	int ofRobotSerial::readAllBytes(uint8_t *bytes, int bytesRequired) {
		int readIn = 0;
		if (bytes) {
			*bytes = 0;// null out to show data read
			int bytesRemaining = bytesRequired;
			// loop until we've read everything
			while (bytesRemaining > 0) {
				// check for data
				if (available() > 0) {
					// try to read - note offset into the bytes[] array, this is so
					// that we don't overwrite the bytes we already have
					int bytesArrayOffset = bytesRequired - bytesRemaining;
					int result = readBytes(&bytes[bytesArrayOffset], bytesRemaining);

					// check for error code
					if (result == OF_SERIAL_ERROR) {
						// something bad happened
						ofRobotTrace(ErrorLog) << "unrecoverable error reading from serial" << std::endl;
						// bail out
						break;
					}
					else if (result == OF_SERIAL_NO_DATA) {
						// nothing was read, try again
					}
					else {
						// we read some data!
						readIn += result;
						bytesRemaining -= result;
					}
				}
			}
		}
		return readIn;
	}

	// bool readOnly -- just read serial do not send request
	robotType ofRobotSerial::ArmIDResponsePacket(uint8_t *bytes) {
		if (bytes != nullptr) {
			robotArmMode armMode = (robotArmMode)bytes[2];
			switch (armMode) {
			case IKM_IK3D_CARTESIAN:
				ofRobotTrace() << "arm mode IKM_IK3D_CARTESIAN" << std::endl;
				break;
			case IKM_IK3D_CARTESIAN_90:
				ofRobotTrace() << "arm mode IKM_IK3D_CARTESIAN_90" << std::endl;
				break;
			case IKM_CYLINDRICAL:
				ofRobotTrace() << "arm mode IKM_CYLINDRICAL" << std::endl;
				break;
			case IKM_CYLINDRICAL_90:
				ofRobotTrace() << "arm mode IKM_CYLINDRICAL_90" << std::endl;
				break;
			default:
				ofRobotTrace() << "arm mode IKM_BACKHOE mode?" << std::endl;
				break;
			}
			RobotTypeID id = unknownRobotType;
			switch (bytes[1]) {
			case 2:
				id = InterbotiXPhantomXReactorArm;
				ofRobotTrace() << "InterbotiXPhantomXReactorArm" << std::endl;
				break;
			}
			return robotType(armMode, id);
		}
		return robotType(IKM_NOT_DEFINED, unknownRobotType);
	}


	robotType ofRobotSerial::waitForRobot() {
		ofRobotTrace() << "wait for mr robot..." << std::endl;

		waitForSerial();

		robotType type = createUndefinedRobotType();
		uint8_t bytes[31];

		int readin = readBytesInOneShot(bytes, 5);
		if (readin == 5) {
			type = ArmIDResponsePacket(bytes);
			if (type.first == IKM_NOT_DEFINED) {
				ofRobotTrace(ErrorLog) << "invalid robot type" << std::endl;
				return type;
			}
		}
		else {
			ofRobotTrace(ErrorLog) << "invalid robot sign on" << std::endl;
			return type;
		}

		// get sign on echo from device
		readin = readBytesInOneShot(bytes, 30);
		bytes[readin] = 0;
		ofRobotTrace() << bytes << std::endl;
		return type;
	}

	void ofRobotSerial::write(uint8_t* data, int count) {
		// from http://learn.trossenrobotics.com/arbotix/arbotix-communication-controllers/31-arm-link-reference.html

		//If you are sending packets at an interval, do not send them faster than 30hz(one packet every 33ms).
		// no need to hurry packets so just want the minimum amount no matter what
		ofSleepMillis(100); // 100 ms seems ok, to much less and we start to overrun bugbug is this true?  
		int sent = writeBytes(data, count);

		ofRobotTrace() << "write sent = " << sent << std::endl;

		readPose(); // pose is sent all the time
		readPose(); // how often are two sent?

	}
	void ofRobotState::echo() const {
		ofRobotTrace() << "WristAngle=" << (set[0] ? ofToString(getWristAngle()) : "<not set>") << std::endl;
		ofRobotTrace() << "WristRotatation=" << (set[1] ? ofToString(getWristRotation()) : "<not set>") << std::endl;
		ofRobotTrace() << "Gripper=" << (set[2] ? ofToString(getGripper()) : "<not set>") << std::endl;
	}
	void ofRobotPosition::echo() const {
		ofRobotTrace() << "x=" << (set[0] ? ofToString(x) : "<not set>") << std::endl;
		ofRobotTrace() << "y=" << (set[1] ? ofToString(y) : "<not set>") << std::endl;
		ofRobotTrace() << "z=" << (set[2] ? ofToString(z) : "<not set>") << std::endl;
	}
	// can be +//
	bool ofRobotPosition::validRange(float f) {
		if (abs(f) >= 0.0f && abs(f) <= 1.0f) {
			return true;
		}
		return false;
	}

	void ofRobotPosition::setPercents(float xPercent, float yPercent, float zPercent) {
		if (xPercent != NoRobotValue && validRange(xPercent)) {
			x = xPercent;
			set[0] = true;
		}
		else {
			set[0] = false;
		}
		if (yPercent != NoRobotValue && validRange(yPercent)) {
			y = yPercent;
			set[1] = true;
		}
		else {
			set[1] = false;
		}
		if (zPercent != NoRobotValue && validRange(zPercent)) {
			z = zPercent;
			set[2] = true;
		}
		else {
			set[2] = false;
		}
	}

	void ofRobotCommands::echo() const {
		for (const auto& cmd : cmdVector) {
			cmd.echo();
		}
	}

	// x - wrist angle, y is wrist rotate, z is gripper (using ofVec3f so its features can be leverage)
	void ofRobotCommands::setState(ofRobotState statePercent) {
		if (isCylindrical()) {
			if (statePercent.set[0]) {
				setWristAngle(getMin(wristAngle) + (statePercent.getWristAngle() * (getMax(wristAngle) - getMin(wristAngle))));
			}
			if (statePercent.set[1]) {
				setWristRotate(getMin(wristRotate) + (statePercent.getWristRotation() * (getMax(wristRotate) - getMin(wristRotate))));
			}
			if (statePercent.set[2]) {
				setGripper(getMin(Gripper) + (statePercent.getGripper() * (getMax(Gripper) - getMin(Gripper))));
			}
		}
	}

	//+/- .001 to 1.000, 0 means ignore 
	void ofRobotCommands::setPoint(ofRobotPosition ptPercent) {
		// only Cylindrical supported by this function, mainly the setx one
		if (isCylindrical()) {
			//ofMap
			if (ptPercent.set[0]) {
				float gx = ptPercent.getX();
				float mx = getMax(X);
				float mn = getMin(X);
				setX(getMin(X) + (ptPercent.getX() * (getMax(X) - getMin(X))));
			}
			if (ptPercent.set[1]) {
				setY(getMin(Y) + (ptPercent.getY() * (getMax(Y) - getMin(Y))));
			}
			if (ptPercent.set[2]) {
				setZ(getMin(Z) + (ptPercent.getZ() * (getMax(Z) - getMin(Z))));
			}
		}
		else {
			ofRobotTrace(ErrorLog) << "setPoint not supported" << std::endl;
		}
	}

	// add ranges checking
	void ofRobotCommands::add(const ofRobotCommand& cmd) {
		cmdVector.push_back(cmd);
	}

	void ofRobotCommands::sanityTestHighLevel() {
		TraceBaseClass() << "high level sanityTest" << std::endl;
		reset();

		// add one command with main points/states
		ofRobotCommand cmd(0.3f, 0.6f, NoRobotValue, 1.0f, -1.0f, 0.5f);
		cmd.add(ofRobotCommand::robotCommandState(ofRobotPosition(0.3f, 0.6f), ofRobotState(1.0f, -1.0f, 0.5f)));// need to be percents!!
		cmd.add(ofRobotCommand::robotCommandState(ofRobotPosition(NoRobotValue, NoRobotValue, 0.3f), ofRobotState()));// need to be percents!!
		cmd.add(ofRobotCommand::robotCommandState(ofRobotPosition(NoRobotValue, NoRobotValue, 1.0f), ofRobotState()));// need to be percents!!

		// add a new command, either way works
		add(ofRobotCommand(1000)); // sleep

		setLowLevelCommand(NoArmCommand);
		setDelta(255);
		setButton();
	}
	void ofRobotCommands::send(ofRobotSerial* serial) {

		echo();
		if (serial) {
			serial->write(getData(), count);
		}
	}

	// set basic data that moves a little bit after starting up. does low level writes only. Does not call reset() or any high level function
	void ofRobotCommands::sanityTestLowLevel() {
		ofRobotTrace() << "low level sanityTest" << std::endl;
		setX(300); // absolution position vs. percentages
		setY(150);
		setZ(150);
		setWristAngle(30);
		setWristRotate(120);
		setGripper(0);
		setLowLevelCommand(NoArmCommand);
		setDelta(255);
		setButton();
	}


	ofRobotCommands::ofRobotCommands(ofRobot *robot) :RobotJoints(robot->data, robot->type) {
		this->robot = robot;
		if (robot) {
			//typedef pair<robotType, robotArmJointType> SpecificJoint
			setUserDefinedRanges(SpecificJoint(robot->getType(), X), robot->userDefinedRanges);
		}
	}
	void ofRobotCommands::reset() { // setup can be ignored for a reset is not required
		setStartState();
		send(&robot->serial); // send the mode, also resets the robot
		setDefaultState();
		clear(cmdVector);
	}

	//ofPushMatrix();
	//ofTranslate(400, 300);
	//ofPopMatrix();

	// draw circle at current positon
	void ofRobotCommands::DrawCircle(double radius)
	{
		// do a move like OF does so drawing always starts at current
		float slice = 2 * M_PI / 10;
		for (int i = 0; i < 10; i++) {
			float angle = slice * i;
			int newX = (int)(0 + 10 * cos(angle));
			int newY = (int)(0 + 10 * sin(angle));
			ofRobotTrace() << newX << " " << newY << std::endl;
			//add(ofRobotCommand(newX, newY)); // need to be percents!! bugbug make this a json player, then the creators of json are the engine
		}
		int i = 0;
		/*
		double slice = 2 * M_PI / points;
		for (int i = 0; i < points; i++)	{
			double angle = slice * i;
			float newX = (float)(center.x + radius * cos(angle));
			float newY = (float)(center.y + radius * sin(angle));
			add(ofRobotCommand(newX, newY)); // need to be percents!! bugbug make this a json player, then the creators of json are the engine
		}
		*/
	}

	void ofRobotCommands::userDefined() {
		TraceBaseClass() << "userDefined" << std::endl;
		reset();
		DrawCircle(0.50f);
		add(ofRobotCommand(1000)); // sleep
	}
	void ofRobotCommands::draw() {

		if (robot) {
			switch (name) {
			case ofRobotCommands::HighLevelTest:
				sanityTestHighLevel(); // populates cmdVector
				break;
			case ofRobotCommands::LowLevelTest:
				sanityTestLowLevel();
				break;
			case ofRobotCommands::UserDefined:
				userDefined();
				break;
			case ofRobotCommands::Push:
				pushMatrix();
				break;
			case ofRobotCommands::Pop:
				popMatrix();
				break;
			case ofRobotCommands::Translate:
				//translate();
				break;
			}

			vector< ofRobotCommand >::iterator it = cmdVector.begin();
			while (it != cmdVector.end()) {
				// draw out all the points & states, sleeping as needed
				for (const auto& a : it->vectorData) {
					setPoint(a.first);
					setState(a.second);
					it->sleep(); // sleep if requested
					send(&robot->serial);
				}
				if (it->OKToDelete()) {
					it = cmdVector.erase(it);
				}
				else {
					++it;
				}
			}
		}
	}

	void ofRobotCommand::echo() const {
		for (const auto& a : vectorData) {
			a.first.echo();
			a.second.echo();
		}
	}

	void ofRobot::setup() {

		serial.listDevices(); // let viewer see thats out there

		serial.setup(1, 38400);//bugbug get from xml

							   // start with default mode
		robotType defaultType;
		if (!robotTypeIsError(defaultType = serial.waitForRobot())) {
			ofRobotTrace() << "robot setup complete" << std::endl;
		}

		ofRobotTrace() << "use IKM_CYLINDRICAL" << std::endl;
		type = robotType(IKM_CYLINDRICAL, defaultType.second);

		// do one time setup
		RobotJoints jv(data, type);
		jv.oneTimeSetup(); // do one time setup of static data

	}
	void ofRobot::update() {
	}

	void ofRobot::echo() {
		commands.echo();
	}

	void ofRobot::draw() {
		if (pause) {
			return;
		}
		commands.draw();
	}

}
