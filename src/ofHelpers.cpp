#include "ofApp.h"

// echo, ignoring null bytes
void ofRobotSerial::echoRawBytes(uint8_t *bytes, int count) {
	std::stringstream buffer;
	for (int i = 0; i < count; ++i) {
		buffer << " bytes[" << i << "] = " << (int)bytes[i]; // echo in one line
	}
	RobotTrace() << buffer.str() << std::endl;
}

// get pose data from serial port bugbug decode this
void ofRobotSerial::readPose() {
	if (available() == 0) {
		return;
	}

	RobotTrace() << "RobotSerial::readPose" << std::endl;

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
		ofLogNotice() << bytes;
	}
	delete bytes;

}
int ofRobotSerial::readBytesInOneShot(uint8_t *bytes, int bytesMax) {
	int result = 0;
	if (available() > 0) {
		if ((result = readBytes(bytes, bytesMax)) == OF_SERIAL_ERROR) {
			RobotTrace(true) << "serial failed" << std::endl;
			return 0;
		}
		while (result == OF_SERIAL_NO_DATA) {
			result = readBytes(bytes, bytesMax);
			if (result == OF_SERIAL_ERROR) {
				RobotTrace(true) << "serial failed" << std::endl;
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
					RobotTrace(true) << "unrecoverable error reading from serial" << std::endl;
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
			RobotTrace() << "arm mode IKM_IK3D_CARTESIAN" << std::endl;
			break;
		case IKM_IK3D_CARTESIAN_90:
			RobotTrace() << "arm mode IKM_IK3D_CARTESIAN_90" << std::endl;
			break;
		case IKM_CYLINDRICAL:
			RobotTrace() << "arm mode IKM_CYLINDRICAL" << std::endl;
			break;
		case IKM_CYLINDRICAL_90:
			RobotTrace() << "arm mode IKM_CYLINDRICAL_90" << std::endl;
			break;
		default:
			RobotTrace() << "arm mode IKM_BACKHOE mode?" << std::endl;
			break;
		}
		RobotTypeID id = unknownRobotType;
		switch (bytes[1]) {
		case 2:
			id = InterbotiXPhantomXReactorArm;
			RobotTrace() << "InterbotiXPhantomXReactorArm" << std::endl;
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
			RobotTrace(true) << "invalid robot type" << std::endl;
			return type;
		}
	}
	else {
		RobotTrace(true) << "invalid robot sign on" << std::endl;
		return type;
	}

	// get sign on echo from device
	readin = readBytesInOneShot(bytes, 30);
	bytes[readin] = 0;
	RobotTrace() << bytes << std::endl;
	return type;
}

void ofRobotSerial::write(uint8_t* data, int count) {
	// from http://learn.trossenrobotics.com/arbotix/arbotix-communication-controllers/31-arm-link-reference.html

	//If you are sending packets at an interval, do not send them faster than 30hz(one packet every 33ms).
	// no need to hurry packets so just want the minimum amount no matter what
	ofSleepMillis(100); // 100 ms seems ok, to much less and we start to overrun bugbug is this true?  
	int sent = writeBytes(data, count);

	RobotTrace() << "write sent = " << sent << std::endl;

	readPose(); // pose is sent all the time
	readPose(); // how often are two sent?

}
