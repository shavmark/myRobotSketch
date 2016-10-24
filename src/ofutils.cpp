#include "ofApp.h"
#include "ofUtils.h"

namespace RobotArtists {

	TraceBaseClass& TraceBaseClass::operator<<(manip1 fp) {
		std::ostringstream check;
		check << fp;
		if (check.str()[0] == '\n') {
			if (type == ErrorLog) {
				sendErrorline();
			}
			else {
				sendLogline();
			}
		}
		else {
			message << fp;
		}
		return *this;
	}
	TraceBaseClass& TraceBaseClass::operator<<(manip2 fp) {
		message << fp;
		return *this;
	}
	TraceBaseClass& TraceBaseClass::operator<<(manip3 fp) {
		std::ostringstream check;
		message << fp;
		return *this;
	}

}