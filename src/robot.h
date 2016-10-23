#pragma once
#include <iostream>
#include <string>
#include <sstream>     

inline uint16_t bytes_to_u16(uint8_t high, uint8_t low) {
	// robot data seems to be big endian, most os seem to be little
	return (((uint16_t)high) & 255) << 8 | (low & 255);
}

class RobotTrace {
public:

	RobotTrace(bool error = false) { isError = error; }

	// overload for manipulators
	typedef std::ostream& (*manip1)(std::ostream&);

	typedef std::basic_ios< std::ostream::char_type, std::ostream::traits_type > ios_type;
	typedef ios_type& (*manip2)(ios_type&);

	typedef std::ios_base& (*manip3)(std::ios_base&);
	RobotTrace& operator<<(manip1 fp) {
		std::ostringstream check;
		message << fp;
		check << fp;
		if (check.str()[0] == '\n') {
			sendline();
		}
		return *this;
	}
	RobotTrace& operator<<(manip2 fp) {
		message << fp;
		return *this;
	}
	RobotTrace& operator<<(manip3 fp) {
		std::ostringstream check;
		message << fp;
		return *this;
	}

	template <class T>RobotTrace& operator<<(const T& value) {
		message << value;
		send();
		return *this;
	}
protected:
	std::ostringstream message;
	// capture messages as they come in
	virtual void send() {
		std::cout << message.str();
	}
	// just watch lines
	virtual void sendline() {
	}
	bool isError = false;
private:
};

class RobotBaseClass {
public:
	RobotBaseClass(RobotTrace *tracer) { this->tracer = tracer; }
	~RobotBaseClass() {	if (tracer) delete tracer;	}
	RobotTrace& getTracer(bool Error = false) {
		if (tracer) {
			return *tracer;
		}
		return defaultTracer;
	}
	RobotTrace& getErrorTracer() { return getTracer(true); }
private:
	RobotTrace *tracer;
	RobotTrace defaultTracer;
};

