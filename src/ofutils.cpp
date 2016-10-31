
/*
ofUtils.cpp - openframeworks based classes for managing robots
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

#include "ofApp.h"
#include "ofUtils.h"

namespace RobotArtists {

	TraceBaseClass& TraceBaseClass::operator<<(manip1 fp) {
		std::ostringstream check;
		check << fp;
		if (check.str()[0] == '\n') {
			sendline();
		}
		message << fp;
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