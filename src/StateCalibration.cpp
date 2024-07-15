#include "StateCalibration.h"
#include <iostream>
#include <unistd.h>
#include "MovementCalculator.h"

StateCalibration::StateCalibration() : IVisionState() {
	finished = false;
}

StateCalibration::StateCalibration(SmartCamera* ca, iVisionClient* fi, SDKCommunicator* sc, StateEnum* st, MovementCalculator* mo) : IVisionState(ca, fi, sc, st, mo) {
	finished = false;
}

StateCalibration::~StateCalibration() {

}

bool StateCalibration::enterState() {
	std::cout << "Entering Calibration State." << std::endl;

	cam->resetParallaxCalibration();
	//TODO: turn off flash
	usingCamera = true;
	cam->startVideo();

	//firmware->setFlashDuration(8.0f);
	//firmware->setFlashOffset(0.0f);
	firmware->setPSDLockFlag(true);
	firmware->setFlashInTK(false);
	//firmware->setFlashBrightness(1.0f);
	firmware->setFlashDuration(40.0f);
	firmware->setFlashOffset(1.0f);
	finished = false;
	sleep(2);
	// firmware->setFlashBrightness(0.0f);
	// firmware->setFlashInTK(false);
	
	std::cout << "Ready for calibration." << std::endl;

	return true;
}

bool StateCalibration::stateAction() {
	//std::cout << "Performing Calibration State action." << std::endl;
	if (!finished) {
		if (firmware->hasSMRLock()/* && firmware->SMRStableDuration() > 0*/) {
		if (cam->parallaxCalibrationStep(firmware->currentDistance() / 1000)) {
			std::cout << "Finished parallax calibration." << std::endl;
			//(*nextState) = Video;
			finished = true;
			//mov->filterCalibData();
			firmware->setFlashInTK(false);
			firmware->setFlashBrightness(0.5f);
		}
		}
	}
	else {
		usleep(100000);
	}

	return true;
}

bool StateCalibration::leaveState() {
	bool smoothening_flag = cam->Filter_calibration();
	std::cout << "Leaving Calibration State." << std::endl;
	return smoothening_flag;
}
