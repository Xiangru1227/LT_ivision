#include "StateTeach2Drive.h"
#include <iostream>
#include <vector>
#include <unistd.h>

StateTeach2Drive::StateTeach2Drive() : IVisionState() {

	currentTeachState = TeachBegin;
	lockCount = 0;
}

StateTeach2Drive::StateTeach2Drive(SmartCamera* ca, iVisionClient* fi, SDKCommunicator* sc, StateEnum* st, MovementCalculator* mo) : IVisionState(ca, fi, sc, st, mo) {

	currentTeachState = TeachBegin;
	lockCount = 0;
}

StateTeach2Drive::~StateTeach2Drive() {

}

bool StateTeach2Drive::enterState() {
	std::cout << "Entering Teach 2 Drive State." << std::endl;

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

	//start by waiting for teaching instructions
	currentTeachState = TeachTeaching;

	usleep(500000);

	return true;
}

bool StateTeach2Drive::stateAction() {
	//perform different action depending on which state it's in
	switch(currentTeachState) {
		case TeachBegin:
			return beginState();
		case TeachTeaching:
			return teachingState();
		case TeachTrack:
			return trackState();
		case TeachLocked:
			return lockState();
		case TeachEnd:
			return endState();
		default:
			return endState();
	}
}

bool StateTeach2Drive::leaveState() {
	std::cout << "Leaving Teach 2 Drive State." << std::endl;
	return true;
}

bool StateTeach2Drive::beginState() {
	//probably don't need this, or just move teaching state to begin state
	usleep(200000);
	return true;
}

bool StateTeach2Drive::teachingState() {

	//TODO: actually handle SDK communication
	//TODO: files will actually probably be stored on the SDK side, so will need to handle that differently, but leaving it for now since I don't know exactly what the SDK will be doing
	//if receive teach finish, store target list and begin tracking
	updateSMRTracking();
	if (false) {
		cam->finalizeSMRTargets();
		currentTeachState = TeachTrack;
	}

	//if receive save to file, save target list to file
	if (false) {
		cam->saveTeachFile();
	}

	//if receive load from file, load target list from file
	if (false) {
		cam->loadTeachFile();
	}

	//if receive clear list, clear the target list
	if (false) {
		cam->clearSMRTargets();
	}

	//if receive coordinates, add closest SMR to target list
	if (false) {
		cam->addTeachTarget(0.0f, 0.0f);
	}

	return true;
}

bool StateTeach2Drive::trackState() {

	//if locked onto SMR, make sure everything is set up for future tracking, but don't actually do anything
	TrackerInfo tkr = firmware->getTrackerInfo();
	if (tkr.locked) {
		if (firmware->SMRStableDuration() > 5)
			currentTeachState = TeachLocked;
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

bool StateTeach2Drive::lockState() {

	TrackerInfo tkr = firmware->getTrackerInfo();
	if (tkr.locked) {
		cam->reportTrackerLock(true, tkr.az, tkr.el, false, tkr.distance / 1000, firmware->SMRStableDuration());
		lockCount++;
		if (lockCount < 500) {
			usleep(100);
			return true;
		}
		firmware->setPSDLockFlag(false);
		firmware->sendMoveBy(0,-.1f);
		usleep(40000);
		firmware->setPSDLockFlag(true);
		return true;
	}
	else {
		cam->reportTrackerLock(false, tkr.az, tkr.el);
		cam->clearSMRTracking();
		lockCount = 0;

		if (cam->finishedAllSMRTargets()) {
			currentTeachState = TeachTeaching;
		}
		else {
			currentTeachState = TeachTrack;
		}
	}
	return true;
}


bool StateTeach2Drive::endState() {
	//just do nothing while waiting for new command or something, might have to do some special checking for commands, but not sure
	usleep(200000);
	return true;
}
