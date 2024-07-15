#include "Debug.h"
#include <sstream>

std::string createFilename(std::string prefix, int num, int secondNum) {
	std::stringstream ss;
	ss << prefix << num;
	if (secondNum != 0) {
		ss << secondNum;
	}
	ss << ".png";
	return ss.str();
}

