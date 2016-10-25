/*
ofUtils.h - openframeworks based classes for managing robots
Copyright (c) 2016 Mark J Shavlik.  All right reserved.This file is part of myRobot.

myRobot is free software : you can redistribute it and / or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

myRobot is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with myRobot.If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once
#include <cstdint>
#include <ios>
#include <iostream>
#include <vector>
#include <sstream>

// basic OF related classes etc, not specific to any solutions such as Robots. 

namespace RobotArtists {

	enum TraceType { DebugLog, TraceLog, ErrorLog, FatalErrorLog };

	inline uint16_t bytes_to_u16(uint8_t high, uint8_t low) {
		// robot data seems to be big endian, most os seem to be little
		return (((uint16_t)high) & 255) << 8 | (low & 255);
	}

	template<typename T>void clear(std::vector< T > vect) {
		std::vector< T >::iterator it = vect.begin();
		while (it != vect.end()) {
			it = vect.erase(it);
		}
	}

	class TraceBaseClass {
	public:

		TraceBaseClass(TraceType type = TraceLog) { this->type = type; }

		// overload for manipulators
		typedef std::ostream& (*manip1)(std::ostream&);
		typedef std::basic_ios< std::ostream::char_type, std::ostream::traits_type > ios_type;
		typedef ios_type& (*manip2)(ios_type&);
		typedef std::ios_base& (*manip3)(std::ios_base&);
		TraceBaseClass& operator<<(manip1 fp);
		TraceBaseClass& operator<<(manip2 fp);
		TraceBaseClass& operator<<(manip3 fp);
		template <class T>TraceBaseClass& operator<<(const T& value) {
			message << value;
			send();
			return *this;
		}

	protected:
		std::ostringstream message;
		// capture messages as they come in
		virtual void send() {
			//std::cout << formatMessage(); // default usage
		}
		// just watch lines bugbug only error and log supported right now
		virtual void sendErrorline() {
		}
		virtual void sendLogline() {
		}
		TraceType type;

		//bugbug json format
		std::string formatMessage() { return message.str(); }

	private:

	};
	class ofRobotTrace : public TraceBaseClass {
	public:
		ofRobotTrace(TraceType type = TraceLog) : TraceBaseClass(type) {}
		virtual void sendErrorline() {
			ofLogError() << formatMessage(); //bugbug support all log types from OF
		}
		virtual void sendLogline() {
			ofLogNotice() << formatMessage();
		}
	};
	class ofRobotTraceNetwork : public TraceBaseClass {
	public:
		ofRobotTraceNetwork(TraceType type = TraceLog) : TraceBaseClass(type) {}
		virtual void sendErrorline() {
			ofLogError() << formatMessage(); //bugbug support all log types from OF
		}
		virtual void sendLogline() {
			ofLogNotice() << formatMessage();
		}
	};

	class ofRobotSay : public TraceBaseClass {
	public:
		//bugbug send to say code via network
		virtual void sendErrorline() {
			ofLogError() << formatMessage(); //bugbug support all log types from OF
		}
		virtual void sendLogline() {
			ofLogNotice() << formatMessage();
		}
	};

}