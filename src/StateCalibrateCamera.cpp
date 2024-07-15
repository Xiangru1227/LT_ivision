#include "StateCalibrateCamera.h"
#include <iostream>
#include <unistd.h>

StateCalibrateCamera::StateCalibrateCamera() : IVisionState() {

}

StateCalibrateCamera::StateCalibrateCamera(SmartCamera* ca, iVisionClient* fi, SDKCommunicator* sc, StateEnum* st, MovementCalculator* mo) : IVisionState(ca, fi, sc, st, mo){

}

StateCalibrateCamera::~StateCalibrateCamera() {

}

//sets up camera for calibration, then starts video capture
bool StateCalibrateCamera::enterState() {
	std::cout << "Entering Calibrate Camera State." << std::endl;
	cam->setupCameraCalibration();
	//TODO: will probably revisit how turning flash on or off is handled
	firmware->setFlashBrightness(0.0f);
	usingCamera = true;
	cam->startVideo();
	return true;
}

//performs camera calibration step
//(search for calibration pattern in image, do calibration once enough images processed)
bool StateCalibrateCamera::stateAction() {
	std::cout << "Performing Calibrate Camera State action." << std::endl;
	usleep(500000);
	if(cam->cameraCalibrationStep())
		(*nextState) = Video;
	return true;
}

//stop video capture
bool StateCalibrateCamera::leaveState() {
	std::cout << "Leaving Calibrate Size State." << std::endl;
	return true;
}

//right now, doesn't handle any special SDK commands
//bool StateCalibrateCamera::handleSDKControls(ControlState state) {
//	return true;
//}

