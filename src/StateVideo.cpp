#include "StateVideo.h"
#include <iostream>


StateVideo::StateVideo() : IVisionState() {
	
}

StateVideo::StateVideo(SmartCamera* ca, iVisionClient* fi, SDKCommunicator* sc, StateEnum* st,  MovementCalculator* mo) : IVisionState(ca, fi, sc, st, mo){
	
}

StateVideo::~StateVideo() {

}

bool StateVideo::enterState() {
	std::cout << "Entering Video State." << std::endl;

	//make sure camera has started video capture
	// cam->setRegCamProp();
	// cam->stopVideo();
	usingCamera = true;
	cam->stopVideo();
	//load smart camera calibration data to prepare for SMR tracking
	if (!cam->loadCalibration())
		return false;
	cam->startVideo();
	firmware->setPSDLockFlag(true);
	// firmware->setFlashBrightness(0.95);
	cam->clearSMRTracking();
	cam->clearSMRTargets();
	firmware->setFlashInTK(false);
	firmware->setLedAlwaysOn(false);

	return true;
}

bool StateVideo::stateAction() {
	//cam->updateDetectionImage();
	TrackerInfo tkr = firmware->getTrackerInfo();
	if (firmware->hasSMRLock()){
		firmware->setFlashInTK(false);
		cam->reportTrackerLock(true, tkr.az, tkr.el, true, tkr.distance / 1000, firmware->SMRStableDuration());
		if (cam->getIprobeMode()) {
			if (!cam->getIprobeLocked()) {
				cam->setIprobeLocked(true);
			}
		}
	} else {
		firmware->setFlashInTK(false);
		if (cam->getIprobeLocked()) {
			cam->setIprobeLocked(false);
		}
	}
	usleep(100000);
	// updateSMRTracking();
	return true;
}

bool StateVideo::leaveState() {
	std::cout << "Leaving Video State." << std::endl;
	return true;
}
