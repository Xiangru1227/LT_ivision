#include "IVisionState.h"
#include <iostream>
#include <unistd.h>
#include <tuple>

IVisionState::IVisionState() {
	cam = NULL;
	firmware = NULL;
	sdk = NULL;

	nextState = NULL;
	mov - NULL;
	usingCamera = true;
}

IVisionState::IVisionState(SmartCamera* ca, iVisionClient* fi, SDKCommunicator* sc, StateEnum* st, MovementCalculator* mo) {
	cam = ca;
	firmware = fi;
	sdk = sc;
	mov = mo;
	nextState = st;

	usingCamera = true;
}

IVisionState::~IVisionState() {

}



bool IVisionState::handleSDKControls(ControlState& state) {
	return true;
}


//get two new images, process them and update observed SMR data
void IVisionState::updateSMRTracking() {
	cam->updateSMRImages();
	//reset the auto calib buff when the lock is lost
	//mov->reset_buf();
	ImgAngles ang1 = firmware->getPrevImageAngles();
	ImgAngles ang2 = firmware->getImageAngles();
	cam->updateSMRTracking(ang1.az_top, ang1.el_top, ang1.az_bot, ang1.el_bot, ang2.az_top, ang2.el_top, ang2.az_bot, ang2.el_bot);
}


