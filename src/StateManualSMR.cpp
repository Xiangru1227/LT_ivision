#include "StateManualSMR.h"
#include <iostream>
#include <unistd.h>
#include <chrono>
using namespace std::chrono; 

StateManualSMR::StateManualSMR() : IVisionState() {
	update_target = false;
	target_x = 0;
	target_y = 0;
}

StateManualSMR::StateManualSMR(SmartCamera* ca, iVisionClient* fi, SDKCommunicator* sc, StateEnum* st, MovementCalculator* mo) : IVisionState(ca, fi, sc, st, mo) {
	update_target = false;
	target_x = 0;
	target_y = 0;
}

StateManualSMR::~StateManualSMR() {

}

bool StateManualSMR::enterState() {
	std::cout << "Entering Manual SMR State." << std::endl;

	//reset tracking, targeter defaults to single SMR tracking, so just need to clear targets
	cam->clearSMRTargets();
	
	if(firmware->is_Spiral()){
		firmware->StopSearch();
	}

	//make sure camera has started video capture
	usingCamera = true;
	
	cam->stopVideo(); // stop the cam to set the image detection properties to the camera
	//load smart camera calibration data to prepare for SMR tracking
	if (!cam->loadCalibration())
		return false;
	cam->startVideo();
	//make sure PSD is set to lock
	firmware->setPSDLockFlag(true);
	firmware->setFlashInTK(true);

	return true;
}

//TODO: keep tracking SMRs while locked and allow leaving the SMR when new input is given
bool StateManualSMR::stateAction() {
	//std::cout << "Performing Manual SMR State action." << std::endl;

	// //if(!firmware->move_flag)
	// milliseconds mil(1000); 
    // mil = mil*20;
	// std::cout << "Spiral Count: " << cam->Spiral_counter << std::endl;
	// while(firmware->SpiralInProgress && cam->Spiral_counter > 0) {
	// 	std::cout << "TimeOut Initiated: " << std::endl;
	// 	auto begin = std::chrono::steady_clock::now();
	// 	while (std::chrono::steady_clock::now() - begin < std::chrono::milliseconds(45000)) {
	// 		if (firmware->hasSMRLock() || state.flags.user_click)
	// 			break;
	// 	}
	// 	firmware->StopSearch();
	// }
	updateSMRTracking();
	TrackerInfo tkr = firmware->getTrackerInfo();

	if(firmware->is_Spiral())
		cam->Spiral_counter++;

	// check for image intensity, exercise auto exposure if needed
	if (cam->checkNeedsAutoExposure()){
		cam->stopVideo();
			if (!cam->loadCalibration())
				return false;
		cam->startVideo();
	}


	if (firmware->hasSMRLock()) {
		// update SMR targeter (or perhaps tracker) with SMR distance, to improve future tracking, make sure targeter is ready to begin tracking again
		cam->reportTrackerLock(true, tkr.az, tkr.el, cam->cam_auto_calib, tkr.distance / 1000, firmware->SMRStableDuration());
		
		cam->Spiral_counter = 0;
		cam->no_mv_cnt_ang = 0;
		cam->no_mv_cnt_img = 0;
		//firmware->StopSearch();
		// cam->EL_search_timeout_ct = 0;
		// cam->Spiral_counter = 0;
		//cam->clearSMRTracking();
		last_lock_az = firmware->currentAzimuth();
		last_lock_el = firmware->currentElevation();
		if (update_target) {
			firmware->setPSDLockFlag(false);
		}
		std::cout << "Tracker locked with distance: " << tkr.distance/1000 << std::endl;
		
		//auto exposure running during the lock
		cam->auto_exp_counter++;
		//std::cout <<"auto exp cntr:" << cam->auto_exp_counter << std::endl;
		if(cam->auto_exp_counter >= cam->auto_exp_reset_interval){
			cam->auto_exp_counter = 0;
			cam->stopVideo();
				if (!cam->loadCalibration())
					return false;
			cam->startVideo();
		}
		return true;
	}
	cam->reportTrackerLock(false, tkr.az, tkr.el);
	//mov->buf = 0;
	if (!firmware->getPSDLockFlag()) {
		float diff_az = firmware->currentAzimuth() - last_lock_az;
		float diff_el = firmware->currentElevation() - last_lock_el;
		float dist_from_last_lock = std::sqrt(diff_az * diff_az + diff_el * diff_el);
		if (dist_from_last_lock > 0.2f) {
			//std::cout << "Far from last lock, setting PSD lock true." << std::endl;
			firmware->setPSDLockFlag(true);
		}
	}
	else {
		//std::cout << "PSD enabled." << std::endl;
	}

	
	//get desired movement towards SMR
	Movement mov = cam->findManualMovement(update_target, target_x, target_y);
	if (update_target) {
		update_target = false;
	}
	if (mov.type == MoveBy && firmware->trackerStill()) {
		return firmware->sendMoveBy(mov.az, mov.el);
	}
	else if (mov.type == MoveTo && firmware->trackerStill()) {
		return firmware->sendMoveTo(mov.az, mov.el, mov.radius);
	}
	else if (mov.type == Spiral) {
		std::cout << "Doing Spiral Search" << '\n';
		firmware->SetSpiralSearch(mov.az, mov.el);
		return firmware->StartSearch(cam->spiral_dist*1000, cam->spiral_freq);
	}

	return true;
}

bool StateManualSMR::leaveState() {
	firmware->StopSearch();
	// cam->EL_search_timeout_ct = 0;
	cam->Spiral_counter = 0;
	std::cout << "Leaving Manual SMR State." << std::endl;
	return true;
}

bool StateManualSMR::handleSDKControls(ControlState& state) {
	//std::cout << "Handling SDK controls in ManualSMR state." << std::endl;
	if (state.flags.user_click) {

		update_target = true;
		target_x = state.click_point.x;
		target_y = state.click_point.y;

		std::cout << "Got click with target " << target_x << ", " << target_y << std::endl;
		//firmware->StopSearch();
		//TODO: probably change this so that it actually checks for some kind of success
		sdk->sendAck(CameraCommunication::UserClick, true);
	}
	return true;
}