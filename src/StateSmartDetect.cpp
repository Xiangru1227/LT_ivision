#include "StateSmartDetect.h"
#include <iostream>

StateSmartDetect::StateSmartDetect() : IVisionState() {

}

StateSmartDetect::StateSmartDetect(SmartCamera* ca, SDKCommunicator* sc, StateEnum* st, MovementCalculator* mo) : IVisionState(ca, NULL, sc, st, mo){

}

StateSmartDetect::~StateSmartDetect() {

}

bool StateSmartDetect::enterState() {
	std::cout << "Entering Smart Detect State." << std::endl;

	//TODO: turn off flash

	//make sure camera has started video capture
	usingCamera = true;
	cam->startVideo();

	return true;
}

bool StateSmartDetect::stateAction() {
	std::cout << "Performing Smart Detect State action." << std::endl;
	cam->updateDetectionImage();
	return true;
}

bool StateSmartDetect::leaveState() {
	std::cout << "Leaving Smart Detect State." << std::endl;
	return true;
}
