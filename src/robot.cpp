#include "robot.h"

RobotTrace& RobotTrace::operator<<(manip1 fp) {
	std::ostringstream check;
	message << fp;
	check << fp;
	if (check.str()[0] == '\n') {
		sendline();
	}
	return *this;
}	
RobotTrace& RobotTrace::operator<<(manip2 fp) {
	message << fp;
	return *this;
}
RobotTrace& RobotTrace::operator<<(manip3 fp) {
	std::ostringstream check;
	message << fp;
	return *this;
}

