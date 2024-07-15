#include "StateMultiSMR.h"
#include <iostream>
#include <vector>
#include <string>
#include <unistd.h>
#include <chrono>
using namespace std::chrono; 

StateMultiSMR::StateMultiSMR() : IVisionState() {

	currentMultiSMRState = MultiBegin;
	initialCount = 0;
	lockCount = 0;
	automatic_mode = true;
	next_smr_cmd = false;
	incorrectLock = 0;
	last_smr_has_been_targeted = false;
}

StateMultiSMR::StateMultiSMR(SmartCamera* ca, iVisionClient* fi, SDKCommunicator* sc, StateEnum* st, MovementCalculator* mo) : IVisionState(ca, fi, sc, st, mo) {

	currentMultiSMRState = MultiBegin;
	initialCount = 0;
	lockCount = 0;
	automatic_mode = false;
	next_smr_cmd = false;
	incorrectLock = 0;
	last_smr_has_been_targeted = false;
}

StateMultiSMR::~StateMultiSMR() {

}

bool StateMultiSMR::enterState() {
	std::cout << "Entering Multi SMR State." << std::endl;


	//reset tracking, targeter defaults to single SMR tracking, so just need to clear targets
	cam->clearSMRTargets();

	//make sure camera has started video capture
	usingCamera = true;
	
	//cam->stopVideo(); // stop the video to set the cam properties in image detection settings
	//load smart camera calibration data to prepare for SMR tracking
	if (!cam->loadCalibration())
		return false;
	cam->startVideo();
	//make sure PSD is set to lock
	firmware->setPSDLockFlag(true);
	firmware->setFlashInTK(true);


	//start by searching for multiple SMR targets
	currentMultiSMRState = MultiBegin;

	last_smr_has_been_targeted = false;

	usleep(500000);

	return true;
}

bool StateMultiSMR::stateAction() {
	//since this state tracks multiple SMRs, it should always be tracking SMRs
	updateSMRTracking();

	//perform different action depending on which state it's in
	switch(currentMultiSMRState) {
		case MultiBegin:
			return beginState();
		case MultiTrack:
			return trackState();
		case MultiLocked:
			return lockState();
		case MultiEnd:
			return endState();
		default:
			return endState();
	}
}

bool StateMultiSMR::leaveState() {
	std::cout << "Leaving Multi SMR State." << std::endl;
	firmware->StopSearch();
	cam->Spiral_counter = 0;
	return true;
}

bool StateMultiSMR::beginState() {
	if (initialCount <= 5)
		initialCount++;

	if (initialCount > 5) {
		next_smr_cmd = false;
		int smrTargets = cam->addAllObservedSMRTargets(false /* firmware->hasSMRLock() don't use locked SMR, this generates an extraneous target */,
				firmware->currentAzimuth(), firmware->currentElevation(), firmware->currentDistance());
		if (smrTargets > 0) {
			cam->finalizeSMRTargets();
			if (firmware->hasSMRLock()) {
				currentMultiSMRState = MultiLocked;
			}
			else {
				currentMultiSMRState = MultiTrack;
			}
			//may be better to use this
			//startAz = firmware->currentAzimuth();
			//startEl = firmware->currentElevation();
			//more advanced technique to find a "base point", but kind of depends on distance
			cam->findBasePoint(startAz, startEl);
		} else {
			last_smr_has_been_targeted = true;
		}
		initialCount = 0;
		std::cout << "Multiple SMRs: " << smrTargets << std::endl;
	}

	return true;
}

bool StateMultiSMR::trackState() {

	//if(!firmware->move_flag)
	updateSMRTracking();
	if(firmware->is_Spiral())
		cam->Spiral_counter++;
	//if locked onto SMR, check if it's the correct SMR - if so, move to lock state, if not, break lock and keep searching
	//TrackerInfo tkr = firmware->getTrackerInfo();
	if (firmware->hasSMRLock()) {
		std::cout << "Has SMR lock." << std::endl;
		
		cam->Spiral_counter = 0;
		cam->no_mv_cnt_ang = 0;
		cam->no_mv_cnt_img = 0;
		//firmware->StopSearch();
		if (firmware->SMRStableDuration() > 0.2f) {
			if (cam->lockedOntoCurrentTarget(firmware->currentAzimuth(), firmware->currentElevation(), firmware->currentDistance() / 1000)) {
				currentMultiSMRState = MultiLocked;
				std::cout << "On correct target." << std::endl;
				incorrectLock = 0;
			}
			else {
				last_lock_az = firmware->currentAzimuth();
				last_lock_el = firmware->currentElevation();
				incorrectLock++;
				if (incorrectLock > 20) {
					currentMultiSMRState = MultiBegin;
					cam->clearSMRTargets();
				}
				else {
					firmware->setPSDLockFlag(false);
				}
				std::cout << "Not on correct target." << std::endl;
			}
		}
		cam->reportTrackerLock(true, firmware->currentAzimuth(), firmware->currentElevation(), cam->cam_auto_calib, firmware->currentDistance() / 1000, firmware->SMRStableDuration());
		return true;
	}
	cam->reportTrackerLock(false, firmware->currentAzimuth(), firmware->currentElevation());
	//mov->buf = 0;
	if (!firmware->getPSDLockFlag()) {
		float diff_az = firmware->currentAzimuth() - last_lock_az;
		float diff_el = firmware->currentElevation() - last_lock_el;
		float dist_from_last_lock = std::sqrt(diff_az * diff_az + diff_el * diff_el);
		if (dist_from_last_lock > 1.0f) {
			firmware->setPSDLockFlag(true);
		}
	}

	//get desired movement towards SMR
	Movement mov = cam->findTrackingMovement();
	if (mov.type == MoveBy) {
		//std::cout << "Moving by " << mov.az << ' ' << mov.el << '\n';
		return firmware->sendMoveBy(mov.az, mov.el);
	} else if (mov.type == MoveTo) {
		//std::cout << "Moving to " << mov.az << ' ' << mov.el << " radius: " << mov.radius << '\n';
		return firmware->sendMoveTo(mov.az, mov.el, mov.radius);
	}
	else if (mov.type == Spiral) {
		std::cout << "Doing Spiral Search" << '\n';
		firmware->SetSpiralSearch(mov.az, mov.el);
		return firmware->StartSearch(cam->spiral_dist*1000, cam->spiral_freq);
	}
	return true;
}

bool StateMultiSMR::lockState() {

	TrackerInfo tkr = firmware->getTrackerInfo();

	// float diffaz = std::abs(tkr.az - startAz);
	// float diffel = std::abs(tkr.el - startEl);

	// float moveaz = std::abs(tkr.az - lastAz);
	// float moveel = std::abs(tkr.el - lastEl);

	// lastAz = tkr.az;
	// lastEl = tkr.el;

	//std::cout << "Current angles: " << tkr.az << ", " << tkr.el << std::endl;
	cam->reportTrackerLock(tkr.locked, tkr.az, tkr.el, true, tkr.distance / 1000, firmware->SMRStableDuration());
	if (lockCount < 20) {
		lockCount++;
	}
	else {
		lockCount = 0;
	}
	if (!last_smr_has_been_targeted) {
		if (cam->finishedAllSMRTargets()) {
			std::cout << "Finished SMR targets." << '\n';
			last_smr_has_been_targeted = true;
		}
	}

	if (next_smr_cmd || (automatic_mode && lockCount >= 20)) {
		next_smr_cmd = false;
		cam->incrementSMRTarget();
		last_lock_az = firmware->currentAzimuth();
		last_lock_el = firmware->currentElevation();
		if (cam->finishedAllSMRTargets()) {
			currentMultiSMRState = MultiBegin;
			std::cout << "Finished all targets." << std::endl;
			//cam->clearSMRTracking();
			//cam->clearSMRTargets();
			cam->resetTargetLoop();
		}
		else {
			firmware->setPSDLockFlag(false);
			currentMultiSMRState = MultiTrack;
			std::cout << "Finished target " << cam->getCurrentTarget() << std::endl;
		}
	}


	// if (tkr.locked || diffaz > 0.5f || diffel > 0.5f || moveaz > 0.05f || moveel > 0.05f) {
	// 	cam->reportTrackerLock(firmware->hasSMRLock(), tkr.az, tkr.el, false, tkr.distance / 1000, firmware->SMRStableDuration());
	// 	//std::cout << "Tracker locked with distance: " << tkr.distance/1000 << " and angles " << tkr.az << ", " << tkr.el << std::endl;
	// 	lockCount++;
	// 	if (lockCount < 500) {
	// 		//lockCount++;
	// 		usleep(1000);
	// 	}
	// 	else if (next_smr_cmd || (automatic_mode && lockCount == 500)) {
	// 		//std::cout << "Trying to get to target " << cam->getCurrentTarget() << ", is locked onto target " << cam->lockedOnTargetIndex(tkr.az, tkr.el, tkr.distance / 1000) << std::endl;
	// 		if (cam->lockedOntoCurrentTarget(tkr.az, tkr.el, tkr.distance / 1000)) {
	// 			cam->incrementSMRTarget();
	// 		}
	// 		firmware->setPSDLockFlag(false);
	// 		usleep(100000);
	// 	}
	// 	else {
	// 		firmware->sendMoveTo(startAz, startEl);
	// 		usleep(50000);
	// 	}
	// }
	// else {
	// 	firmware->setPSDLockFlag(true);
	// 	cam->reportTrackerLock(false, firmware->currentAzimuth(), firmware->currentElevation());
	// 	cam->clearSMRTracking();
	// 	lockCount = 0;

	// 	if (cam->finishedAllSMRTargets()) {
	// 		currentMultiSMRState = MultiBegin;
	// 		std::cout << "Finished all targets." << std::endl;
	// 		cam->clearSMRTargets();
	// 	}
	// 	else {
	// 		currentMultiSMRState = MultiTrack;
	// 		std::cout << "Finished target " << cam->getCurrentTarget() << std::endl;
	// 	}
	// }
	return true;



	// if (firmware->hasSMRLock()) {
	// 	cam->reportTrackerLock(true, false, tkr.distance / 1000, firmware->SMRStableDuration());
	// 	lockCount++;
	// 	if (lockCount < 500) {
	// 		usleep(1000);
	// 		return true;
	// 	}
	// 	firmware->setPSDLockFlag(false);
	// 	usleep(100000);
	// 	//float jog_distance = std::max(-3000.0f / tkr.distance, -4.0f);
	// 	//firmware->sendMoveBy(0, jog_distance);
	// 	firmware->sendMoveTo(startAz, startEl);
	// 	usleep(500000);
	// 	firmware->setPSDLockFlag(true);
	// 	usleep(500000);
	// 	return true;
	// }
	// else {
	// 	cam->reportTrackerLock(false);
	// 	cam->clearSMRTracking();
	// 	lockCount = 0;

	// 	if (cam->finishedAllSMRTargets()) {
	// 		currentMultiSMRState = MultiBegin;
	// 		std::cout << "Finished all targets." << std::endl;
	// 	}
	// 	else {
	// 		currentMultiSMRState = MultiTrack;
	// 		std::cout << "Finished target " << cam->getCurrentTarget() << std::endl;
	// 	}
	// }
	// return true;
}


bool StateMultiSMR::endState() {
	//just do nothing while waiting for new command or something, might have to do some special checking for commands, but not sure
	std::cout << "In end state." << std::endl;
	usleep(200000);
	return true;
}

bool StateMultiSMR::handleSDKControls(ControlState& state) {
	//std::cout << "Handling SDK controls in MultiSMR state." << std::endl;
	if (state.flags.go_to_next_smr) {
		if (!last_smr_has_been_targeted) {
			next_smr_cmd = true;
		}

		std::cout << "Got next SMR command." << std::endl;
		bool sdk_ret = !last_smr_has_been_targeted;
		std::cout << "Returning " << sdk_ret << '\n';

		//TODO: probably change this so that it actually checks for some kind of success and indicates the progress through all SMRs
		sdk->sendAck(CameraCommunication::NextSMR, sdk_ret);
	}
	return true;
}
