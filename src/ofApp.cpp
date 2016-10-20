#include "ofApp.h"
#include <algorithm> 
// https://msdn.microsoft.com/en-us/library/windows/desktop/aa387285(v=vs.85).aspx
//https://github.com/sparkle-project/Sparkle
// shared across all robots and joints
RobotValueRanges RobotJoints::hardwareRanges;
template<typename T>void clear(vector< T > vect) {
	vector< T >::iterator it = vect.begin();
	while (it != vect.end()) {
		it = vect.erase(it);
	}
}

// tracing helper
string echoJointType(SpecificJoint joint) {
	std::stringstream buffer;
	buffer << "SpecificJoint<robotType, robotArmJointType> = " << joint.first.first << ", " << joint.first.second << ", " << joint.second;
	return buffer.str();
}

void RobotJoints::set(SpecificJoint type, int minVal, int maxVal, int defaultVal) {
	ofLogNotice() << "RobotJoints::set" << echoJointType(type) << " - min = " << minVal << " max = " << maxVal << " default = " << defaultVal;

	hardwareRanges.minValue[type] = minVal;
	hardwareRanges.maxValue[type] = maxVal;
	hardwareRanges.defaultValue[type] = defaultVal;
}

void RobotJoints::setX(int x) {
	ofLogNotice() << "try to set x=" << x;
	if (inRange(X, x)) {
		setLowLevelX(x, addMagicNumber());
	}
}
void RobotJoints::setY(int y) {
	ofLogNotice() << "try to set y=" << y;
	if (inRange(Y, y)) {
		setLowLevelY(y);
	}
}
void RobotJoints::setZ(int z) {
	ofLogNotice() << "try to set z=" << z;
	if (inRange(Z, z)) {
		setLowLevelZ(z);
	}
}
void RobotJoints::setWristAngle(int a) {
	ofLogNotice() << "try to set setWristAngle=" << a;
	if (inRange(wristAngle, a)) {
		setLowLevelWristAngle(a);
	}
}
void RobotJoints::setWristRotate(int a) {
	ofLogNotice() << "try to set setWristRotate=" << a;
	if (inRange(wristRotate, a)) {
		setLowLevelWristRotate(a);
	}
}
void RobotJoints::setGripper(int distance) {
	ofLogNotice() << "try to set setGripper=" << distance;
	if (inRange(Gripper, distance)) {
		setLowLevelGripper(distance);
	}
}
int RobotJointsState::get(uint16_t high, uint16_t low) {
	int number = -1;
	if (data) {
		int number = (data[low] << 8) + data[high];
	}
	return number;
}

void RobotJointsState::set(uint16_t high, uint16_t low, int val) {
	ofLogNotice() << "set " << val;
	set(high, highByte(val));
	set(low, lowByte(val));
}

void ofRobotCommands::send(ofRobotSerial* serial) {
	
	echo();
	if (serial) {
		serial->write(getData(), count);
	}
}
// return core data making sure its set properly
uint8_t *RobotJointsState::getData() { 
	set(headerByteOffset, 255); 
	getChkSum(); 
	return data; 
}

robotLowLevelCommand RobotJointsState::getStartCommand(robotType type) {
	if (type.first == IKM_IK3D_CARTESIAN) {
		return setArm3DCartesianStraightWristAndGoHome; // bugbug support all types once the basics are working
	}
	if (type.first == IKM_IK3D_CARTESIAN_90) {
		return setArm3DCartesian90DegreeWristAndGoHome; // bugbug support all types once the basics are working
	}
	if (type.first == IKM_CYLINDRICAL_90) {
		return setArm3DCylindrical90DegreeWristAndGoHome; // bugbug support all types once the basics are working
	}
	if (type.first == IKM_CYLINDRICAL) {
		return setArm3DCylindricalStraightWristAndGoHome; // bugbug support all types once the basics are working
	}
	return unKnownCommand;//bugbug support all types once the basics are working
}

int RobotJoints::getDefaultValue(robotArmJointType type) { 
	if (userDefinedRanges && userDefinedRanges->defaultValue.find(SpecificJoint(typeOfRobot, type)) != userDefinedRanges->defaultValue.end()) {
		return userDefinedRanges->defaultValue[SpecificJoint(typeOfRobot, type)];
	}
	return  hardwareRanges.defaultValue[SpecificJoint(typeOfRobot, type)];
}
void RobotValueRanges::setDefault(SpecificJoint joint, int value, int max, int min) { 
	if (value < max && value > min) {
		defaultValue[joint] = value;
	}
	else {
		ofLogError() << "default value out of range " << value << " range is " << min << ", " << max;
	}
}
void RobotValueRanges::setMin(SpecificJoint joint, int value, int min, int max) {
	if (value < max && value > min) {
		minValue[joint] = value;
	}
	else {
		ofLogError() << "minValue out of range " << value << " range is " << min << ", " << max;
	}
}
void RobotValueRanges::setMax(SpecificJoint joint, int value, int min, int max) {
	if (value < max && value > min) {
		maxValue[joint] = value;
	}
	else {
		ofLogError() << "maxValue out of range " << value << " range is " << min << ", " << max;
	}
}

int RobotJoints::getMin(robotArmJointType type) { 
	if (userDefinedRanges && userDefinedRanges->minValue.find(SpecificJoint(typeOfRobot, type)) != userDefinedRanges->minValue.end()) {
		return userDefinedRanges->minValue[SpecificJoint(typeOfRobot, type)];
	}
	return hardwareRanges.minValue[SpecificJoint(typeOfRobot, type)]; 
}
int RobotJoints::getMid(robotArmJointType type) { 
	return (getMax(type) - getMin(type)) / 2; //bugbug works for robot types? 
}

int RobotJoints::getMax(robotArmJointType type) { 
	if (userDefinedRanges && userDefinedRanges->maxValue.find(SpecificJoint(typeOfRobot, type)) != userDefinedRanges->maxValue.end()) {
		return userDefinedRanges->maxValue[SpecificJoint(typeOfRobot, type)];
	}
	return hardwareRanges.maxValue[SpecificJoint(typeOfRobot, type)];
}

bool RobotJoints::inRange(robotArmJointType type, int value) {
	if (value > getMax(type) || value < getMin(type)) {
		ofLogError() << "out of range " << value << " (" << getMin(type) << ", " << getMax(type) << ")";
		return false;
	}
	return true;
}
// needs to only be called one time -- uses static data to save time/space. backhoe not supported
void RobotJoints::oneTimeSetup() {
	
	set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, InterbotiXPhantomXReactorArm), X), -300, 300, 0);
	set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, InterbotiXPhantomXReactorArm), X), -300, 300, 0);
	set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, InterbotiXPhantomXReactorArm), Y), 50, 350, 235);
	set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, InterbotiXPhantomXReactorArm), Y), 20, 150, 140);
	set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, InterbotiXPhantomXReactorArm), Z), 20, 250, 210);
	set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, InterbotiXPhantomXReactorArm), Z), 10, 150, 30);
	set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, InterbotiXPhantomXReactorArm), wristAngle), -30, 30, 0);
	set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, InterbotiXPhantomXReactorArm), wristAngle), -90, -45, -90);
	set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, InterbotiXPhantomXReactorArm), wristRotate), 0, 1023, 512);
	set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, InterbotiXPhantomXReactorArm), wristRotate), 0, 1023, 512);
	set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN, InterbotiXPhantomXReactorArm), Gripper), 0, 512, 512);
	set(SpecificJoint(createRobotType(IKM_IK3D_CARTESIAN_90, InterbotiXPhantomXReactorArm), Gripper), 0, 512, 512);


	set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, InterbotiXPhantomXReactorArm), X), 0, 1023, 512);
	set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, InterbotiXPhantomXReactorArm), X), 0, 1023, 512);
	set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, InterbotiXPhantomXReactorArm), Y), 50, 350, 235);
	set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, InterbotiXPhantomXReactorArm), Y), 20, 150, 140);
	set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, InterbotiXPhantomXReactorArm), Z), 20, 250, 210);
	set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, InterbotiXPhantomXReactorArm), Z), 10, 150, 30);
	set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, InterbotiXPhantomXReactorArm), wristAngle), -30, 30, 0);
	set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, InterbotiXPhantomXReactorArm), wristAngle), -90, -45, -90);
	set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, InterbotiXPhantomXReactorArm), wristRotate), 0, 1023, 512);
	set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, InterbotiXPhantomXReactorArm), wristRotate), 0, 1023, 512);
	set(SpecificJoint(createRobotType(IKM_CYLINDRICAL, InterbotiXPhantomXReactorArm), Gripper), 0, 512, 512);
	set(SpecificJoint(createRobotType(IKM_CYLINDRICAL_90, InterbotiXPhantomXReactorArm), Gripper), 0, 512, 512);

	// mark end of list for debugging
	set(SpecificJoint(createUndefinedRobotType(), JointNotDefined), 0, 0, 0); // should not be set to this while running, only during object init

	return;

}

// echo, ignoring null bytes
void echoRawBytes(uint8_t *bytes, int count) {
	std::stringstream buffer;
	for (int i = 0; i < count; ++i) {
		buffer << " bytes[" << i << "] = " << (int)bytes[i]; // echo in one line
	}
	ofLogNotice() << buffer.str();
}
// get pose data from serial port bugbug decode this
void ofRobotSerial::readPose() {
	if (available() == 0) {
		return;
	}

	ofLogNotice() << "RobotSerial::readPose";

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
	else if (i == 10){
		echoRawBytes(bytes, 10);
		ArmIDResponsePacket(bytes); // 2 signs ons come back some times, likely a timing issue?
		ArmIDResponsePacket(bytes); // first 2 bytes are sign on type
	}
	else {
		ofLogNotice() << bytes;
	}
	delete bytes;

}
int ofRobotSerial::readBytesInOneShot(uint8_t *bytes, int bytesMax) {
	int result = 0;
	if (available() > 0) {
		if ((result = readBytes(bytes, bytesMax)) == OF_SERIAL_ERROR) {
			ofLogError() << "serial failed";
			return 0;
		}
		while (result == OF_SERIAL_NO_DATA) {
			result = readBytes(bytes, bytesMax);
			if (result == OF_SERIAL_ERROR) {
				ofLogError() << "serial failed";
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
				int result = readBytes(&bytes[bytesArrayOffset],	bytesRemaining);

				// check for error code
				if (result == OF_SERIAL_ERROR) {
					// something bad happened
					ofLog(OF_LOG_ERROR, "unrecoverable error reading from serial");
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
			ofLogNotice() << "arm mode IKM_IK3D_CARTESIAN";
			break;
		case IKM_IK3D_CARTESIAN_90:
			ofLogNotice() << "arm mode IKM_IK3D_CARTESIAN_90";
			break;
		case IKM_CYLINDRICAL:
			ofLogNotice() << "arm mode IKM_CYLINDRICAL";
			break;
		case IKM_CYLINDRICAL_90:
			ofLogNotice() << "arm mode IKM_CYLINDRICAL_90";
			break;
		default:
			ofLogNotice() << "arm mode IKM_BACKHOE mode?";
			break;
		}
		RobotTypeID id= unknownRobotType;
		switch (bytes[1]) {
		case 2:
			id = InterbotiXPhantomXReactorArm;
			ofLogNotice() << "InterbotiXPhantomXReactorArm";
			break;
		}
		return robotType(armMode, id);
	}
	return robotType(IKM_NOT_DEFINED, unknownRobotType);
}


robotType ofRobotSerial::waitForRobot() {
	ofLogNotice() << "wait for mr robot...";
	
	waitForSerial();

	robotType type = createUndefinedRobotType();
	uint8_t bytes[31];

	int readin = readBytesInOneShot(bytes, 5);
	if (readin == 5) {
		type = ArmIDResponsePacket(bytes);
		if (type.first == IKM_NOT_DEFINED) {
			ofLogError() << "invalid robot type";
			return type;
		}
	}
	else {
		ofLogError() << "invalid robot sign on";
		return type;
	}

	// get sign on echo from device
	readin = readBytesInOneShot(bytes, 30);
	bytes[readin] = 0;
	ofLogNotice() << bytes;
	return type;
}
void ofRobot::setup() {

	clear(cmds);

	serial.listDevices(); // let viewer see thats out there

	serial.setup(1, 38400);//bugbug get from xml

	// start with default mode
	robotType defaultType;
	if (!robotTypeIsError(defaultType = serial.waitForRobot())) {
		ofLogNotice() << "robot setup complete";
	}

	ofLogNotice() << "use IKM_CYLINDRICAL";
	type = robotType(IKM_CYLINDRICAL, defaultType.second);

	// do one time setup
	RobotJoints jv(data, type);
	jv.oneTimeSetup(); // do one time setup of static data
					   
}
void ofRobot::update() {
}

void ofRobotCommands::echo() const {
	for (const auto& cmd :cmdVector) {
		cmd.echo();
	}
}
void RobotCommand::echo() const {
	pointPercent.echo();
	settingsPercent.echo();
}
void ofRobot::echo() {
	for (auto& cmd : cmds) {
		cmd->echo();
	}
}

void ofRobot::draw() {
	if (pause) {
		return;
	}
	for (auto& cmd : cmds) {
		cmd->draw();
	}
}
// user defined can never be greater than hardware min/max
void RobotJoints::setUserDefinedRanges(SpecificJoint joint, RobotValueRanges *userDefinedRanges) {
	
	if (userDefinedRanges->minValue[joint] < hardwareRanges.minValue[joint] || userDefinedRanges->maxValue[joint] > hardwareRanges.maxValue[joint]) {
		ofLogError("RobotJoints::setUserDefinedRanges") << "value out of range ignored ";
		return;
	}
	this->userDefinedRanges = userDefinedRanges; 
}

void ofDrawingRobot::setup() {

	ofRobot::setup(); // set base class first

	//bugbug swing arm x and y, space means break and you get size that way. use a menu and xml for this
	// set ranges so percents work against these. leave Y as is bugbug figure this all out
	RobotJoints jv(getType());

	// only set what we care about, let the rest stay at default. userDefinedRanges is a sparse array of sorts
	userDefinedRanges.setMin(createJoint(X, getType().first, getType().second), jv.getMid(X) - 300, jv.getMin(X), jv.getMax(X));
	userDefinedRanges.setMax(createJoint(X, getType().first, getType().second), jv.getMid(X) + 300, jv.getMin(X), jv.getMax(X));

}
void RobotJointsState::set(uint16_t offset, uint8_t b) { 
	ofLogNotice() << "set data[" << offset << "] = " << (uint16_t)b;
	if (data){
		data[offset] = b;
	}
}


void RobotJointsState::echo() {
	if (!data) {
		ofLogNotice("RobotJointsState::echo") << "no data to echo";
		return;
	}
#define ECHO(a)ofLogNotice() << "echo[" << a << "] = "  << std::hex << (unsigned int)data[a] << "h "  <<  std::dec <<(unsigned int)data[a] << "d "<< #a;
	ECHO(headerByteOffset)
	ECHO(xLowByteOffset)
	ECHO(yHighByteOffset)
	ECHO(yLowByteOffset)
	ECHO(zHighByteOffset)
	ECHO(zLowByteOffset)
	ECHO(wristAngleHighByteOffset)
	ECHO(wristAngleLowByteOffset)
	ECHO(wristRotateHighByteOffset)
	ECHO(wristRotateLowByteOffset)
	ECHO(gripperHighByteOffset)
	ECHO(gripperLowByteOffset)
	ECHO(deltaValBytesOffset)
	ECHO(buttonByteOffset)
	ECHO(extValBytesOffset)
	ECHO(checksum)
}

uint8_t RobotJointsState::getChkSum() {
	if (data) {
		uint16_t sum = 0;
		for (int i = xHighByteOffset; i <= extValBytesOffset; ++i) {
			sum += data[i];
		}
		uint16_t invertedChecksum = sum % 256;//isolate the lowest byte 8 or 16?

		data[checksum] = 255 - invertedChecksum; //invert value to get file checksum
		return data[checksum];

	}
	return 0;
}

void ofRobotSerial::write(uint8_t* data, int count) {
	// from http://learn.trossenrobotics.com/arbotix/arbotix-communication-controllers/31-arm-link-reference.html
		
	//If you are sending packets at an interval, do not send them faster than 30hz(one packet every 33ms).
	// no need to hurry packets so just want the minimum amount no matter what
	ofSleepMillis(100); // 100 ms seems ok, to much less and we start to overrun bugbug is this true?  
	int sent = writeBytes(data, count);

	ofLogNotice() << "write sent = " << sent;

	readPose(); // pose is sent all the time
	readPose(); // how often are two sent?

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

void ofRobotPosition::echo() const {
	ofLogNotice() << "x=" << (set[0] ? ofToString(x) : "<not set>");
	ofLogNotice() << "y=" << (set[1] ? ofToString(y) : "<not set>");
	ofLogNotice() << "z=" << (set[2] ? ofToString(z) : "<not set>");
}
// can be +//
bool ofRobotPosition::validRange(float f) {
	if (abs(f) >= 0.0f && abs(f) <= 1.0f) {
		return true;
	}
	ofLogError() << "float out of range (0.0 to +/- 1.0) or 0 to 100% of range " << abs(f);
	return false;
}

void ofRobotState::echo() const {
	ofLogNotice() << "WristAngle=" << (set[0] ? ofToString(getWristAngle()) : "<not set>");
	ofLogNotice() << "WristRotatation=" << (set[1] ? ofToString(getWristRotation()) : "<not set>");
	ofLogNotice() << "Gripper=" << (set[2] ? ofToString(getGripper()) : "<not set>");
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

//+/- .001 to 1.000, 0 means ignore 
void ofRobotCommands::setPoint(ofRobotPosition ptPercent) {
	// only Cylindrical supported by this function, mainly the setx one
	if (isCylindrical()) {
		//ofMap
		if (ptPercent.set[0]) {
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
		ofLogError("Command::setPoint") << "setPoint not supported";
	}
}
// RobotPositions can only be 0.0 to +/- 1.0 (0 to +/- 100%)
void RobotCommand::init(const ofRobotPosition& pointPercent, const ofRobotState& settingsPercent, int millisSleep, bool deleteWhenDone) {
	this->pointPercent = pointPercent;
	this->settingsPercent = settingsPercent;
	this->deleteWhenDone = deleteWhenDone;
	this->millisSleep = millisSleep;
}

// add ranges checking
void ofRobotCommands::add(const RobotCommand& cmd, BuiltInCommandNames name) { 
	cmdVector.push_back(cmd); 
	this->name = name; 
}

void ofRobotCommands::sizing() {
	ofLogNotice() << "sizing";
	reset();

	add(RobotCommand(0.0f)); // need to be percents!!
	add(RobotCommand(5000));// sleep

	add(RobotCommand(1.0f)); // need to be percents!!
	add(RobotCommand(5000));

	add(RobotCommand(0.5f, 0.0f)); // need to be percents!!
	add(RobotCommand(5000));

	add(RobotCommand(NoRobotValue, 1.0f)); // need to be percents!!
	add(RobotCommand(5000));

	//z not sized

	setLowLevelCommand(NoArmCommand);
	setDelta(255);
}
void ofRobotCommands::sanityTestHighLevel() {
	ofLogNotice() << "high level sanityTest";
	reset();
	add(RobotCommand(0.3f, 0.6f, NoRobotValue, 1.0f, -1.0f, 0.5f)); // need to be percents!!
	add(RobotCommand(NoRobotValue, NoRobotValue, 0.3f)); // too close could hit the bottom
	add(RobotCommand(NoRobotValue, NoRobotValue, 1.0f));
	add(RobotCommand(1000)); // sleep
	setLowLevelCommand(NoArmCommand);
	setDelta(255);
	setButton();
}

// set basic data that moves a little bit after starting up. does low level writes only. Does not call reset() or any high level function
void ofRobotCommands::sanityTestLowLevel() {
	ofLogNotice() << "low level sanityTest";
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

// "home" and set data matching state
void RobotJoints::setDefaultState() {
	ofLogNotice() << "setDefaults";
	setX(getDefaultValue(X));
	setY(getDefaultValue(Y));
	setZ(getDefaultValue(Z));
	setWristAngle(getDefaultValue(wristAngle));
	setWristRotate(getDefaultValue(wristRotate));
	setGripper(getDefaultValue(Gripper));
	setDelta(getDeltaDefault());
	setLowLevelCommand(NoArmCommand);
	setButton();
}
// will block until arm is ready
robotType RobotJoints::setStartState() {
	ofLogNotice() << "setStartState " << typeOfRobot.first << " " << typeOfRobot.second;
	setLowLevelCommand(getStartCommand(typeOfRobot));
	return typeOfRobot;
}

shared_ptr<ofRobotCommands> ofRobot::add(ofRobotCommands::BuiltInCommandNames name) {
	shared_ptr<ofRobotCommands> p = make_shared<ofRobotCommands>(this, name);
	if (p) {
		cmds.push_back(p);
	}
	return p;
}

ofRobotCommands::ofRobotCommands(ofRobot *robot, BuiltInCommandNames name) :RobotJoints(robot->data, robot->type) {
	this->robot = robot;  
	if (robot) {
		//typedef pair<robotType, robotArmJointType> SpecificJoint
		setUserDefinedRanges(SpecificJoint(robot->getType(), X), &robot->userDefinedRanges);
	}
	this->name = name;
}
void ofRobotCommands::reset() { // setup can be ignored for a reset is not required
	setStartState();
	send(&robot->serial); // send the mode, also resets the robot
	setDefaultState();
	clear(cmdVector);
}
void ofRobotCommands::draw() {

	if (robot) {
		switch (name) {
		case ofRobotCommands::HighLevelTest:
			sanityTestHighLevel(); // will populate cmdVector
			break;
		case ofRobotCommands::LowLevelTest:
			sanityTestLowLevel(); 
			break;
		case ofRobotCommands::Sizing:
			sizing();
			break;
		}

		vector< RobotCommand >::iterator it = cmdVector.begin();
		while (it != cmdVector.end()) {
			//{UserDefined, LowLevelTest, HighLevelTest, Sizing

			setPoint(it->pointPercent);
			setState(it->settingsPercent);
			it->sleep(); // sleep if requested
			send(&robot->serial);// move
			if (it->OKToDelete()) {
				it = cmdVector.erase(it);
			}
			else {
				++it;
			}
		}
	}
}

//--------------------------------------------------------------
void ofApp::setup(){
	robot.setup();
}

//--------------------------------------------------------------
void ofApp::update(){
	
	robot.update();

	shared_ptr<ofRobotCommands> cmd = robot.add(ofRobotCommands::HighLevelTest);

	/*
	shared_ptr<RobotCommands> cmd = robot.createAndAdd<RobotCommands>(RobotCommand());
	cmd->reset(); // home the device
	cmd->setup(0.0001, 0, 0.0, 0.0, 0.0); // far left

	shared_ptr<rectangleCommand> cmd2 = robot.createCommand<rectangleCommand>();
	cmd2->reset(); // home the device
	cmd2->millisSleep = 10000;// wait before moving
	cmd2->setup(1.0, 0, 0.0, 0.0, 0.0); // far right
	robot.add(cmd2);

	shared_ptr<rectangleCommand> cmd3 = robot.createCommand<rectangleCommand>();
	cmd3->reset(); // home the device
	cmd3->millisSleep = 10000;// wait before moving
	cmd3->setup(0.0, 1.0, 0.0, 0.0, 0.0); 
	robot.add(cmd3);

	shared_ptr<rectangleCommand> cmd4 = robot.createCommand<rectangleCommand>();
	cmd4->reset(); // home the device
	cmd4->millisSleep = 10000;// wait before moving
	cmd4->setup(0.0, 0.0001, 0.0, 0.0, 0.0);
	robot.add(cmd4);

	//motion->setup(); // start a new motion bugbug outside of testing  like now this is only done one time
	//path.push(motion);
	//shared_ptr<RobotMotion> joints = make_shared<RobotMotion>(serial, data);
	//joints->setup(SignOnDance);
	//path.push(joints);
	/*
	return;
	data.setCommand(EnableArmMovement);
	data.setY(50);
	path.push(data);
	data.reset();
	data.setY(350);
	path.push(data);
	return;
	data.reset();
	data.setX(10); // 300 max 60 units covers about 3", 20 units is .75" so given the arch its not just inches
	data.setY(350);
	path.push(data);
	data.reset();
	data.setZ(250);
	data.setX(-10); // -300 max
	path.push(data);
	data.reset();
	data.setCommand(DelayArm, 1000);
	path.push(data);

	loc.reset();
	loc.moveYout(260);
	loc.moveZup(250);
	path.push(loc);
	loc.reset();
	loc.moveXleft(-212);
	path.push(loc);
	loc.reset();
	loc.moveXleft(200);
	loc.moveYout(200);
	path.push(loc);
	*/

}

//--------------------------------------------------------------
void ofApp::draw(){
	robot.draw();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if (key == ' ') {
		robot.setPause();//bugbug need a thread/semaphore to do this
 		robot.echo();
		robot.setPause(false);
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
