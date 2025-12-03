#include "StateVideo.h"
#include <iostream>


StateVideo::StateVideo() : IVisionState() {
	
}

StateVideo::StateVideo(SmartCamera* ca, iVisionClient* fi, SDKCommunicator* sc, StateEnum* st,  MovementCalculator* mo) : IVisionState(ca, fi, sc, st, mo){
	
}

StateVideo::~StateVideo() {

}

bool StateVideo::enterState() {
	firmware->setFlashInTK(false);
	std::cout << "Entering Video State." << std::endl;

	//reset tracking, targeter defaults to single SMR tracking, so just need to clear targets
	cam->clearSMRTargets();
	if(firmware->is_Spiral()){
		firmware->StopSearch();
	}
	usingCamera = true;
	cam->stopVideo();
	//load smart camera calibration data to prepare for SMR tracking
	if (!cam->loadCalibration())
		return false;
	// If iProbe mode is ON and already locked, keep iProbe exposure/gain on state switch
	if (cam->getIprobeMode() && cam->getIprobeLocked()) {
		cam->setIprobeCameraProp();
		cam->video_stream_active = false;
	} else {
		cam->setVideoStreamCameraProp();
		cam->iprobe_stream_active = false;
	}
	cam->startVideo();
	firmware->setPSDLockFlag(true);
	
	// firmware->setFlashDuration(40);
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
	if (cam->getIprobeMode() && cam->getIprobeLocked()) {
		if(!cam->iprobe_stream_active){
			cam->stopVideo();
			cam->setIprobeCameraProp();
			cam->startVideo();
		}
	}
	else {
		if(cam->iprobe_stream_active){
			cam->stopVideo();
			cam->setVideoStreamCameraProp();
			cam->startVideo();
			cam->iprobe_stream_active = false;
		}
	}
	usleep(100000);
	// updateSMRTracking();
	return true;
}

bool StateVideo::leaveState() {
	firmware->setFlashInTK(false);
	std::cout << "Leaving Video State." << std::endl;
	return true;
}
