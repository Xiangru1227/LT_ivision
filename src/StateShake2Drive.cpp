#include "StateShake2Drive.h"
#include <iostream>
#include <unistd.h>


StateShake2Drive::StateShake2Drive() : IVisionState() {

	currentShake2DriveState = ShakeWatch;
}

StateShake2Drive::StateShake2Drive(SmartCamera* ca, iVisionClient* fi, SDKCommunicator* sc, StateEnum* st, MovementCalculator* mo) : IVisionState(ca, fi, sc, st, mo) {

	currentShake2DriveState = ShakeWatch;
}

StateShake2Drive::~StateShake2Drive() {

}

bool StateShake2Drive::enterState() {
	std::cout << "Entering Shake to Drive State." << std::endl;

	//reset tracking, targeter defaults to single SMR tracking, so just need to clear targets
	cam->clearSMRTargets();

	//make sure camera has started video capture
	usingCamera = true;
	cam->startVideo();

	//load smart camera calibration data to prepare for SMR tracking
	if (!cam->loadCalibration())
		return false;

	//make sure PSD is set to lock
	firmware->setPSDLockFlag(true);

	currentShake2DriveState = ShakeWatch;

	return true;
}

bool StateShake2Drive::stateAction() {
	//perform different action depending on which state it's in
	switch(currentShake2DriveState) {
		case ShakeWatch:
			return watchState();
		case ShakeTrack:
			return trackState();
		case ShakeLocked:
			return lockState();
		default:
			return lockState();
	}
}

bool StateShake2Drive::leaveState() {
	std::cout << "Leaving Shake to Drive State." << std::endl;
	return true;
}


bool StateShake2Drive::watchState() {

	updateSMRTracking();

	if (cam->watchForShakingSMR()) {
		currentShake2DriveState = ShakeTrack;
	}
	return true;
}

bool StateShake2Drive::trackState() {
	
	//if locked onto SMR, make sure everything is set up for future tracking, but don't actually do anything
	TrackerInfo tkr = firmware->getTrackerInfo();
	if (tkr.locked) {
		if (firmware->SMRStableDuration() > 5)
			currentShake2DriveState = ShakeLocked;
		//cam->reportTrackerLock(true, false, tkr.distance / 1000, firmware->SMRStableDuration());
		return true;
	}
	cam->reportTrackerLock(false, tkr.az, tkr.el);

	updateSMRTracking();

	//get desired movement towards SMR
	Movement mov = cam->findTrackingMovement();
	if (mov.type == MoveBy)
		return firmware->sendMoveBy(mov.az, mov.el);
	else if (mov.type == MoveTo)
		return firmware->sendMoveTo(mov.az, mov.el);

	return true;
}

bool StateShake2Drive::lockState() {

	TrackerInfo tkr = firmware->getTrackerInfo();
	if (tkr.locked) {
		cam->reportTrackerLock(true, tkr.az, tkr.el, false, tkr.distance / 1000, firmware->SMRStableDuration());
		return true;
	}
	cam->reportTrackerLock(false, tkr.az, tkr.el);
	cam->clearSMRTracking();

	//could just continue to wait, but for now it'll wait for a bit and then restart
	usleep(100000);
	currentShake2DriveState = ShakeWatch;
	cam->clearSMRTargets();
	return true;

}
