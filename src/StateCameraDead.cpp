#include "StateCameraDead.h"
#include <iostream>
#include <unistd.h>

StateCameraDead::StateCameraDead() : IVisionState() {
	
}

StateCameraDead::StateCameraDead(SmartCamera* ca, SDKCommunicator* sc, StateEnum* st) : IVisionState(ca, NULL, sc, st, NULL) {
	
}

StateCameraDead::~StateCameraDead() {

}

bool StateCameraDead::enterState() {
	std::cout << "Entering CameraDead State." << std::endl;
	return true;
}

bool StateCameraDead::stateAction() {
	//basically just attempt to restart camera (has a sleep in between because if you don't wait for a bit, the camera tends not to start correctly)
	std::cout << "Performing CameraDead State action." << std::endl;
	if (!cam->close()) {
		std::cout << "Couldn't close camera." << std::endl;
		return false;
	}
	sleep(2);
	if (!cam->setup()) {
		std::cout << "Couldn't start camera." << std::endl;
		return false;
	}
	(*nextState) = Listen;
	return true;
}

bool StateCameraDead::leaveState() {
	std::cout << "Leaving CameraDead State." << std::endl;
	return true;
}
