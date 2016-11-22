/*
ofUtils.h - openframeworks based classes for managing robots
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
#include <cstdint>
#include <ios>
#include <iostream>
#include <vector>
#include <sstream>

// basic OF related classes etc, not specific to any solutions such as Robots. 

namespace RobotArtists {

	enum TraceType { DebugLog, TraceLog, WarningLog, ErrorLog, FatalErrorLog };

	template<typename T>void ClearVector(std::vector< T > vect) {
		std::vector< T >::iterator it = vect.begin();
		while (it != vect.end()) {
			it = vect.erase(it);
		}
	}

	class TraceBaseClass {
	public:

		TraceBaseClass(const string & module) { this->type = TraceLog; this->module = module; }
		TraceBaseClass(TraceType type = TraceLog, const string & module = "") { this->type = type; this->module = module; }

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
			//std::cout << formatMessage(); // default usage, pick send or sendline, send sends data as it comes in
		}
		// just watch lines 
		virtual void sendline() {
			std::cout <<"C++ message " << formatMessage();
		}
		TraceType type;
		std::string module;
		//bugbug json format
		std::string formatMessage() { return message.str(); }

	private:

	};

	// supports openframeworks so TraceBaseClass can be used w/o down the road, enabling all objects to use the same logging object (w or w/o of)
	class ofRobotTrace : public TraceBaseClass {
	public:
		ofRobotTrace(const string &msg) :TraceBaseClass(msg) {};
		ofRobotTrace(TraceType type = TraceLog) : TraceBaseClass(type) {}
		virtual void sendline() {
			switch (type) {
			case DebugLog:
				ofLogVerbose(module) << formatMessage();
				break;
			case TraceLog:
				ofLogNotice(module) << formatMessage();
				break;
			case WarningLog:
				break;
			case ErrorLog:
				ofLogError(module) << formatMessage();
				break;
			case FatalErrorLog:
				ofLogFatalError(module) << formatMessage();
				break;
			}
		}
	};
	class ofRobotTraceNetwork : public TraceBaseClass {
	public:
		ofRobotTraceNetwork(TraceType type = TraceLog) : TraceBaseClass(type) {}
		virtual void sendErrorline() {
			//bugbug fill in
		}
		virtual void sendLogline() {
			//bugbug fill in
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