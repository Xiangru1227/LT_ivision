#include "StateSingleSMR.h"
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <unistd.h>
#include <chrono>
using namespace std::chrono; 

StateSingleSMR::StateSingleSMR() : IVisionState() {

}

StateSingleSMR::StateSingleSMR(SmartCamera* ca, iVisionClient* fi, SDKCommunicator* sc, StateEnum* st, MovementCalculator* mo) : IVisionState(ca, fi, sc, st, mo) {

}

StateSingleSMR::~StateSingleSMR() {

}

bool StateSingleSMR::enterState() {
	std::cout << "Entering Single SMR State." << std::endl;

	//reset tracking, targeter defaults to single SMR tracking, so just need to clear targets
	cam->clearSMRTargets();
	
	if(firmware->is_Spiral()){
		firmware->StopSearch();
	}
	//make sure camera has started video capture
	usingCamera = true;
	//load smart camera calibration data to prepare for SMR tracking
	cam->stopVideo();
	if (!cam->loadCalibration())
		return false;
	cam->startVideo();
	//make sure PSD is set to lock
	firmware->setPSDLockFlag(true);
	firmware->setFlashInTK(true);
	return true;
}

bool StateSingleSMR::stateAction() {
	//std::cout << "Performing Single SMR state action." << std::endl;

	//if locked onto SMR, make sure everything is set up for future tracking, but don't actually do anything
	TrackerInfo tkr = firmware->getTrackerInfo();

	if(firmware->is_Spiral()){
		cam->Spiral_counter++;
		std::cout<< "Spiral in Progress: " << cam->Spiral_counter << std::endl;
	}
	
	if (cam->checkNeedsAutoExposure()){
		cam->stopVideo();
			if (!cam->loadCalibration())
				return false;
		cam->startVideo();
	}
		
	if (firmware->hasSMRLock()) {
		firmware->setFlashInTK(false);
		// not the best way to see the mode, need to revisit.
		cam->Spiral_counter = 0;
		cam->no_mv_cnt_ang = 0;
		cam->no_mv_cnt_img = 0;
		// update SMR targeter (or perhaps tracker) with SMR distance, to improve future tracking, make sure targeter is ready to begin tracking again
		cam->reportTrackerLock(true, tkr.az, tkr.el, cam->cam_auto_calib, tkr.distance / 1000, firmware->SMRStableDuration());
		cam->clearSMRTracking();
		
		cam->auto_exp_counter++;
		//std::cout <<"auto exp cntr:" << cam->auto_exp_counter << std::endl;
		if(cam->auto_exp_counter >= cam->auto_exp_reset_interval){
			cam->auto_exp_counter = 0;
			cam->stopVideo();
				if (!cam->loadCalibration())
					return false;
			cam->startVideo();
		}
		
		printCounter++;
		if (printCounter == printInterval) {
			std::cout << "Tracker locked with (" << tkr.az << ", " << tkr.el << ", " << tkr.distance / 1000 << ") " << std::endl;
			printCounter = 0;  // Reset counter after printing
		}
		
		return true;
	} else {
		cam->setIprobeMode(false);
		firmware->setFlashInTK(true);
	}
	cam->reportTrackerLock(false, tkr.az, tkr.el);
	
	updateSMRTracking();
	//get desired movement towards SMR
	Movement mov = cam->findTrackingMovement();
	//std::cout << "Back to single state machine" << std::endl;
	if (mov.type == MoveBy && firmware->trackerStill() ){
		//std::cout << "Moving by " << mov.az << ' ' << mov.el << '\n';
		return firmware->sendMoveBy(mov.az, mov.el);
	}
	else if (mov.type == MoveTo && firmware->trackerStill()) {
		//std::cout << "Moving to " << mov.az << ' ' << mov.el << " radius: " << mov.radius << '\n';
		return firmware->sendMoveTo(mov.az, mov.el, mov.radius);
	}
	else if (mov.type == Spiral) {
		if (cam->spiral_dist > 0.02f) {
			firmware->SetSpiralSearch(mov.az, mov.el);
			std::cout << "Doing Spiral Search, Dist: " << cam->spiral_dist*1000 <<'\n';
			return firmware->StartSearch(cam->spiral_dist*1000, cam->spiral_freq);
		}
		else 
			return true;
	}
	
	return true;
}

bool StateSingleSMR::leaveState() {
	firmware->StopSearch();
	cam->Spiral_counter = 0;
	std::cout << "Leaving Single SMR State." << std::endl;
	return true;
}
