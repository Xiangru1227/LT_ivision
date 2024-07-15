#include "StateListen.h"
#include <iostream>
#include <unistd.h>

StateListen::StateListen() : IVisionState() {

}

StateListen::StateListen(SDKCommunicator* sc, StateEnum* st) : IVisionState(NULL, NULL, sc, st, NULL) {

}

StateListen::~StateListen() {

}

bool StateListen::enterState() {
	std::cout << "Entering Listen(idle) State." << std::endl;
	usingCamera = true;
	firmware->setFlashBrightness(0.0f);
	return true;
}

bool StateListen::stateAction() {
	//just continuously sleeps for a second at a time
	//std::cout << "Performing Listen State action." << std::endl;
	usleep(200000);
	return true;
}

bool StateListen::leaveState() {
	std::cout << "Leaving Listen(idle) State." << std::endl;
	return true;
}
