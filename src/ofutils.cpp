#include "ofApp.h"
#include "ofUtils.h"

namespace RobotArtists {

	TraceBaseClass& TraceBaseClass::operator<<(manip1 fp) {
		std::ostringstream check;
		message << fp;
		check << fp;
		if (check.str()[0] == '\n') {
			if (type == ErrorLog) {
				sendErrorline();
			}
			else {
				sendLogline();
			}
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