#include "IVisionStateMachine.h"

#include "StateSingleSMR.h"
#include "StateMultiSMR.h"
#include "StateCalibration.h"
#include "StateCalibrateSize.h"
#include "StateCalibrateCamera.h"
#include "StateListen.h"
#include "StateCameraDead.h"
#include "StateSmartDetect.h"
#include "StateVideo.h"
#include "StateManualSMR.h"
#include "StateShake2Drive.h"
#include "StateTeach2Drive.h"

#include <iostream>
#include <stdlib.h>
#include <fstream>

IVisionStateMachine::IVisionStateMachine() {

	runVideo = false;
	runDetect = false;

	//initial current state should be begin, but the next state is whatever you want the first "active" state to be, so should probably expose this outside the class somehow
	currentState = Begin;
	nextState = Video;

	fw = new iVisionClient();
	cam = new SmartCamera(fw);
	

	//TODO: probably completely remove CameraDead state and just handle it with restartCamera, also probably remove SmartDetect and Video states, and just replace them with like a general Camera state, also probably rename Listen state to Idle

	//add state objects to map, so they can perform operations
	stateMap.insert(std::pair<StateEnum, std::unique_ptr<IVisionState>>(SingleSMR, std::unique_ptr<IVisionState>(new StateSingleSMR(cam, fw, &sdk, &nextState, mo))));
	stateMap.insert(std::pair<StateEnum, std::unique_ptr<IVisionState>>(MultiSMR, std::unique_ptr<IVisionState>(new StateMultiSMR(cam, fw, &sdk, &nextState, mo))));
	stateMap.insert(std::pair<StateEnum, std::unique_ptr<IVisionState>>(Calibration, std::unique_ptr<IVisionState>(new StateCalibration(cam, fw, &sdk, &nextState, mo))));
	stateMap.insert(std::pair<StateEnum, std::unique_ptr<IVisionState>>(Calibration2, std::unique_ptr<IVisionState>(new StateCalibrateSize(cam, fw, &sdk, &nextState, mo))));
	stateMap.insert(std::pair<StateEnum, std::unique_ptr<IVisionState>>(Calibration3, std::unique_ptr<IVisionState>(new StateCalibrateCamera(cam, fw, &sdk, &nextState, mo))));
	stateMap.insert(std::pair<StateEnum, std::unique_ptr<IVisionState>>(Listen, std::unique_ptr<IVisionState>(new StateListen(&sdk, &nextState))));
	stateMap.insert(std::pair<StateEnum, std::unique_ptr<IVisionState>>(CameraDead, std::unique_ptr<IVisionState>(new StateCameraDead(cam, &sdk, &nextState))));
	stateMap.insert(std::pair<StateEnum, std::unique_ptr<IVisionState>>(SmartDetect, std::unique_ptr<IVisionState>(new StateSmartDetect(cam, &sdk, &nextState, mo))));
	stateMap.insert(std::pair<StateEnum, std::unique_ptr<IVisionState>>(Video, std::unique_ptr<IVisionState>(new StateVideo(cam, fw, &sdk, &nextState, mo))));
	stateMap.insert(std::pair<StateEnum, std::unique_ptr<IVisionState>>(ManualSMR, std::unique_ptr<IVisionState>(new StateManualSMR(cam, fw, &sdk, &nextState, mo))));
	stateMap.insert(std::pair<StateEnum, std::unique_ptr<IVisionState>>(Shake2Drive, std::unique_ptr<IVisionState>(new StateShake2Drive(cam, fw, &sdk, &nextState, mo))));
	stateMap.insert(std::pair<StateEnum, std::unique_ptr<IVisionState>>(Teach2Drive, std::unique_ptr<IVisionState>(new StateTeach2Drive(cam, fw, &sdk, &nextState, mo))));

	counter = 0;

}

IVisionStateMachine::~IVisionStateMachine() {

	//clean up state objects
	//for (std::map<StateEnum, IVisionState*>::iterator it=stateMap.begin(); it!=stateMap.end(); ++it) {
	//	delete it->second;
	//}
	delete cam;
	delete fw;

}


//set up modules that will be used by states
bool IVisionStateMachine::initialize(StateEnum startState, bool startDetectPlus, bool useFirmware, std::string fw_ip) {

#ifdef LT_IVISION_USE_FAKE_IMG_TAKEN_FLAG
	cam->AttachTimestampObserver(&fake_img_taken_flag_);
	fw->SetFakeImgTakenFlag(&fake_img_taken_flag_);
#endif
	nextState = startState;
	pwrManager.setHighPowerMode();
	cam->setObjectDetectorActive(startDetectPlus);
	if (!cam->setup()) {
		std::cout << "Couldn't start camera!" << std::endl;
		return false;
	}
	//set up communication
	if (!sdk.setup()) {
		std::cout << "Couldn't set up connection to SDK!" << std::endl;
		return false;
	}
	if (useFirmware) {
		if (!fw->setupConnection(fw_ip)) {
			std::cout << "Couldn't set up connection to firmware!" << std::endl;
			return false;
		}
	}
	return true;
}

//run state machine until next state is set to End
int IVisionStateMachine::mainLoop() {
	while (nextState != End) {
		enterNextState();
		stateAction();
		updateState();
		counter++;
	}
	return finish();
}

//detect change in state and call new state's Enter State function
bool IVisionStateMachine::enterNextState() {
	if (nextState != currentState || sdk.receivedStateChange) {
		sdk.receivedStateChange = false;
		stateMap[nextState]->enterState();
		currentState = nextState;
		updatePowerMode();
	}
	return true;
}

//perform current state's State Action
bool IVisionStateMachine::stateAction() {
	return stateMap[currentState]->stateAction();
}

//check for external commands to change state and call current state's Leave State function if changing state
bool IVisionStateMachine::updateState() {

	//check for input from SDK
	if (!processSDKUpdate())
		return false;

	//if camera isn't needed, turn it off
	if (!stateMap[currentState]->isUsingCamera() && !runVideo && !runDetect)
		cam->stopVideo();

	//check temperature
	//float currentTemperature = pwrManager.getTemperature();
	//std::cout << "Update State Func" << currentTemperature << std::endl;

	//if changing state, leave current state
	if (nextState != currentState) {
		//stop all tracker movement when changing state (mostly to stop circular search)
		fw->makeStill();
		//set camera mode on (this should probably go somewhere else but maybe ok for now)
		if (nextState == Video) {
			fw->setCamMode(false);
		}
		else {
			fw->setCamMode(true);
		}
		if (!stateMap[currentState]->leaveState()) {
			return false;
		}
	}
	return true;
}

//clean up modules
int IVisionStateMachine::finish() {
	
#ifdef LT_IVISION_USE_FAKE_IMG_TAKEN_FLAG
	fw->SetFakeImgTakenFlag(nullptr);
	cam->DetachTimestampObserver(&fake_img_taken_flag_);
#endif
	if (!fw->closeConnection()) {
		std::cout << "Couldn't close connection to firmware!" << std::endl;
		return -1;
	}
	if (!sdk.close()) {
		std::cout << "Couldn't close connection to host!" << std::endl;
		return -1;
	}
	if (!cam->close()) {
		std::cout << "Couldn't stop camera!" << std::endl;
		return -1;
	}
	return 0;
}

//if in listen (idle) state, set to low power (5W) mode, otherwise use high power (10W) mode
void IVisionStateMachine::updatePowerMode() {
	if (nextState == Listen)
		pwrManager.setLowPowerMode();
	else
		pwrManager.setHighPowerMode();
}









//start sending jpeg images to SDK
void IVisionStateMachine::startVideoThread() {
	if (!runVideo) {
		runVideo = true;
		videoThread = std::thread(SendImages, this);
	}
}


//stop sending jpeg images to SDK
void IVisionStateMachine::endVideoThread() {
	if (runVideo) {
		runVideo = false;
		videoThread.join();
	}
}

//grab full res jpeg image and send to SDK
void IVisionStateMachine::sendFullResImages() {
	char* buffer;
	unsigned long size;
	// std::cout << "size: " << size << std::endl;
	if (!cam->grabNextProcessJpeg(&buffer, size)) {
		std::cout << "[ERROR] failed to grab image" << std::endl;
		return;
	}
	if (size != 0) {
		std::vector<float> params = std::vector<float>();
		std::vector<std::string> msg = {"image_data"};
		// std::cout << "Sending full res image acknowledgement" << std::endl;
		if (!sdk.sendAck(CameraCommunication::GetFullResolutionImage, true, params, msg, buffer, size)){
			std::cout << "[ERROR] Send full res jpeg image failed." << std::endl;
		}
	}
}

//send video images in separate thread
void IVisionStateMachine::SendImages(IVisionStateMachine* iv) {
	iv->sendImages();
}


//grab jpeg image and send to SDK
void IVisionStateMachine::sendImages() {

	while (runVideo) {
		char* buffer;
		unsigned long size;
		if (!cam->grabNextJpeg(&buffer, size)) {
			continue;
		}
		
		if (!cam->iProbeDetection()) {
			// std::cout << "Failed to update iProbe centroids." << std::endl;
		}
		
		std::vector<ImagePoint> smrs;
		//TODO: get SMR list from smart camera
		std::vector<SMRData> observedSMRs = cam->getTrackedSMRs();
		
		auto detectedCentroids = cam->getCentroids();
		// std::cout << "iProbe centroid size: " << detectedCentroids.size() << std::endl;

		uint64_t Timestamp = cam->getTimeStamp();

		// std::ofstream os("TimeStamp.txt", std::ios::app);
		// os << "Timestamp in St Ma  " << Timestamp <<" nano seconds\n";
		//std::cout << "observedSMRs sent to SDK " << observedSMRs.size() << std::endl;
		for (int i = 0; i < observedSMRs.size(); i++) {
			ImagePoint ip;
			ip.x = observedSMRs[i].getImgX() / 3264.0; //hardcoed for now
			ip.y = observedSMRs[i].getImgY() / 2464.0; //hardcoed for now
			//std::cout << "SDK SMR coordinates: "<< ip.x << ", " << ip.y << std::endl;
			smrs.push_back(ip);
		}
		if (size != 0) {
			// std::cout << "Timestamp: " << Timestamp << std::endl;
			
			if (!sdk.sendImageBuffer(buffer, size, smrs, detectedCentroids, Timestamp)) {
			//if (!sdk.sendImageBuffer(buffer, size, smrs)) {
				std::cout << "Send video image failed." << std::endl;
				// std::cout << buffer << std::endl;
			}
			cam->setCentroids(std::vector<cv::Point2f>());
		}
	}
}




//start using a DNN to do object detection (runs on GPU)
void IVisionStateMachine::startDetectThread() {
	if (!runDetect) {
		runDetect = true;
		detectThread = std::thread(PerformDetect, this);
	}
}


//stop using a DNN to do object detection
void IVisionStateMachine::endDetectThread() {
	if (runDetect) {
		runDetect = false;
		detectThread.join();
	}
}


//run object detection in separate thread
void IVisionStateMachine::PerformDetect(IVisionStateMachine* iv) {
	iv->performObjectDetection();
}


//initialize neural network, then continually detect objects in images
void IVisionStateMachine::performObjectDetection() {
	if (cam->setupObjectDetector(detectNet::FACENET)) {
		while (runDetect) {
			cam->detectObjects();
		}
		cam->closeObjectDetector();
	}
}


//update state from SDK, perform required actions, pass 
bool IVisionStateMachine::processSDKUpdate() {
	ControlState controls = sdk.getControlState();

	if (!stateMap[currentState]->handleSDKControls(controls))
		return false;

	return handleSDKControls(controls);
}


bool IVisionStateMachine::handleSDKControls(ControlState state) {

	ControlFlags flags = state.flags;

	if (flags.change_iv_state) {
		nextState = state.iv_state;
		//sdk.sendAck(CameraCommunication::SetOpMode, true);
	}

	//send calibration data to SDK
	if (flags.get_calibration_data) {
		//sdk.sendAck(CameraCommunication::CalibrationParameters, true, std::vector<float>());
	}

	//get current camera properties
	CameraProperties prop = cam->getProperties();

	//send desired properties to SDK
	if (flags.get_exposure)
		sdk.sendAck(CameraCommunication::GetExposure, true, CreateParamList((float)prop.exposure));
	if (flags.get_analog_gain)
		sdk.sendAck(CameraCommunication::GetAnalogGain, true, CreateParamList((float)prop.analog_gain));
	if (flags.get_digital_gain)
		sdk.sendAck(CameraCommunication::GetDigitalGain, true, CreateParamList((float)prop.digital_gain));
	if (flags.get_fps)
		sdk.sendAck(CameraCommunication::GetFPS, true, CreateParamList((float)prop.fps));
	if (flags.get_sensor_resolution)
		sdk.sendAck(CameraCommunication::GetSensorResolution, true, CreateParamList((float)prop.res_x, (float)prop.res_y));
	if (flags.get_display_resolution)
		sdk.sendAck(CameraCommunication::GetDisplayResolution, true, CreateParamList((float)prop.video_res_x, (float)prop.video_res_y));
	if (flags.get_video_roi) {
		//sdk.sendAck(CameraCommunication::GetROIVideoStream, true, CreateParamList(prop.video_roi_x, prop.video_roi_y, prop.video_roi_width, prop.video_roi_height));
	}
	if (flags.get_FC_in_FB) {
		std::cout << "sending extrinsic matrix" << std::endl;
		auto fc_in_fb = cam->getCameraExtrinsicMat();
		sdk.sendAck(CameraCommunication::GetFCinFB, true, 
			CreateParamList((float)fc_in_fb.at<double>(0, 0), 
							(float)fc_in_fb.at<double>(0, 1), 
							(float)fc_in_fb.at<double>(0, 2),
							(float)fc_in_fb.at<double>(1, 0), 
							(float)fc_in_fb.at<double>(1, 1), 
							(float)fc_in_fb.at<double>(1, 2),
							(float)fc_in_fb.at<double>(2, 0), 
							(float)fc_in_fb.at<double>(2, 1), 
							(float)fc_in_fb.at<double>(2, 2)));
	}
	if (flags.get_int_mat) {
		std::cout << "sending intrinsic matrix" << std::endl;
		auto cam_mat = cam->getCameraIntrinsicMat();
		sdk.sendAck(CameraCommunication::GetCameraIntrinsicMatrix, true, 
			CreateParamList((float)cam_mat.at<double>(0, 0), 
							(float)cam_mat.at<double>(1, 1), 
							(float)cam_mat.at<double>(0, 2),
							(float)cam_mat.at<double>(1, 2)));
	}

	prop.exposure = (flags.set_exposure) ? (double)state.exposure : prop.exposure;
	prop.digital_gain = (flags.set_digital_gain) ? state.digital_gain : prop.digital_gain;
	prop.analog_gain = (flags.set_analog_gain) ? state.analog_gain : prop.analog_gain;
	prop.fps = (flags.set_fps) ? (int)state.fps : prop.fps;
	prop.res_x = (flags.set_sensor_resolution) ? (unsigned int)state.sensor_resolution.width : prop.res_x;
	prop.res_y = (flags.set_sensor_resolution) ? (unsigned int)state.sensor_resolution.height : prop.res_y;
	prop.video_res_x = (flags.set_display_resolution) ? (unsigned int)state.display_resolution.width : prop.video_res_x;
	prop.video_res_y = (flags.set_display_resolution) ? (unsigned int)state.display_resolution.height : prop.video_res_y;
	prop.video_roi_x = (flags.set_video_roi) ? state.roi.x : prop.video_roi_x;
	prop.video_roi_y = (flags.set_video_roi) ? state.roi.y : prop.video_roi_y;
	prop.video_roi_width = (flags.set_video_roi) ? state.roi.width : prop.video_roi_width;
	prop.video_roi_height = (flags.set_video_roi) ? state.roi.height : prop.video_roi_height;

	if (flags.send_full_res_img) {
		// std::cout << "Handling send full res image request." << std::endl;
		if (cam->running()){
			prop.exposure = 13.0;
			prop.digital_gain = 1.0;
			prop.analog_gain = 1.0;
			fw->setLedAlwaysOn(true);
			sendFullResImages();
		}
	}

	// enter/exit iProbe mode
	if (flags.enter_iprobe) {
		if (!cam->getIprobeMode()) {
			cam->setIprobeMode(true);
		}
	}
	if (flags.exit_iprobe) {																																																															
		if (cam->getIprobeMode()) {
			cam->setIprobeMode(false);
		}
	}

	//send changes to smart camera
	bool success = cam->setProperties(prop);

	//determine if camera has changed properties, if so, restart camera to apply update
	bool changed = flags.set_exposure || flags.set_analog_gain || flags.set_digital_gain || flags.set_fps || flags.set_sensor_resolution || flags.set_display_resolution || flags.set_video_roi || flags.enter_iprobe || flags.exit_iprobe || flags.send_full_res_img;
	if (success && changed) {
		success = resetVideo();
	}

	//send acknowledgements for set camera property requests
	if (flags.set_exposure)
		sdk.sendAck(CameraCommunication::SetExposure, success, CreateParamList((float)prop.exposure));
	if (flags.set_analog_gain)
		sdk.sendAck(CameraCommunication::SetAnalogGain, success, CreateParamList(prop.analog_gain));
	if (flags.set_digital_gain)
		sdk.sendAck(CameraCommunication::SetDigitalGain, success, CreateParamList(prop.digital_gain));
	if (flags.set_fps)
		sdk.sendAck(CameraCommunication::SetFPS, success, CreateParamList((float)prop.fps));
	if (flags.set_sensor_resolution)
		sdk.sendAck(CameraCommunication::SetSensorResolution, success, CreateParamList((float)prop.res_x, (float)prop.res_y));
	if (flags.set_display_resolution)
		sdk.sendAck(CameraCommunication::SetDisplayResolution, success, CreateParamList((float)prop.video_res_x, (float)prop.video_res_y));
	if (flags.set_video_roi)
		sdk.sendAck(CameraCommunication::SetROIVideoStream, success, CreateParamList(prop.video_roi_x, prop.video_roi_y, prop.video_roi_width, prop.video_roi_height));

	//start/stop video capture and streaming
	if (flags.start_video) {
		std::cout << "Handling start video request." << std::endl;
		bool success = false;
		if (cam->running())
			success = true;
		else
			success = cam->startVideo();
		if (success) {
			startVideoThread();
		}
		//sdk.sendAck(CameraCommunication::StartVideo, success);
	}
	else if (flags.stop_video) {
		endVideoThread();
		//bool success = cam.stopVideo();
		bool success = true;
		//sdk.sendAck(CameraCommunication::StopVideo, success);
	}

	if (flags.outdoor_exp){
		cam->setOutdoorExp();
		std::cout << "Setting exposure mode to outdoor iVisionComm" << std::endl;
	}
	else if (flags.indoor_exp){
		cam->setIndoorExp();
		std::cout << "Setting exposure mode to indoor iVisionComm" << std::endl;
	}

	// if(flags.get_version_number){
	// 	std::string msg;
	// 	msg.append(std::to_string(MAJOR_REVISION));
	// 	msg.append(".");
	// 	msg.append(std::to_string(MINOR_REVISION));
	// 	msg.append(" : ");
	// 	msg.append(__TIME__);
	// 	msg.append(" / ");
	// 	msg.append(__DATE__);

	// 	sdk.sendAck(CameraCommunication::GetVersionNumber, true, std::vector<float>(), CreateMsgList(msg));
	// }
	return true;
}

// ErrorType TaskManager::GetFWVersion(void) {
//   auto return_status = ErrorType::Success;
//   proto_msg msg;

//   msg.str_msg.append(std::to_string(MAJOR_REVISION));
//   msg.str_msg.append(".");
//   msg.str_msg.append(std::to_string(MINOR_REVISION));
//   msg.str_msg.append(" : ");
//   msg.str_msg.append(__TIME__);
//   msg.str_msg.append(" / ");
//   msg.str_msg.append(__DATE__);

//   pServer->sendAck(LaserTrackerCommunication::GetFWVersion, return_status, msg);

//   return return_status;
// }


//add parameters to list to send in acknowledgement to SDK
// std::vector<float> IVisionStateMachine::CreateParamList(float param1, float param2, float param3, float param4) {
// 	std::vector<float> list;
// 	list.push_back(param1);
// 	if (param2 >= 0)
// 		list.push_back(param2);
// 	if (param3 >= 0)
// 		list.push_back(param3);
// 	if (param4 >= 0)
// 		list.push_back(param4);
// 	return list;
// }
std::vector<float> IVisionStateMachine::CreateParamList(float param1, float param2, float param3, float param4, float param5, float param6, float param7, float param8, float param9) {
    std::vector<float> list;
    if (!std::isnan(param1)) list.push_back(param1);
    if (!std::isnan(param2)) list.push_back(param2);
    if (!std::isnan(param3)) list.push_back(param3);
    if (!std::isnan(param4)) list.push_back(param4);
    if (!std::isnan(param5)) list.push_back(param5);
    if (!std::isnan(param6)) list.push_back(param6);
    if (!std::isnan(param7)) list.push_back(param7);
    if (!std::isnan(param8)) list.push_back(param8);
    if (!std::isnan(param9)) list.push_back(param9);
    return list;
}


//string param to send string to sdk
std::vector<std::string> IVisionStateMachine::CreateMsgList(std::string message1) {
	std::vector<std::string> list;

	list.push_back(message1);
	// if (!message2.empty())
	// 	list.push_back(message1);
	return list;
}

bool IVisionStateMachine::resetVideo() {
	bool wasStreaming = runVideo;
	bool wasDetecting = runDetect;
	endVideoThread();
	endDetectThread();
	if (cam->stopVideo()) {
		// std::this_thread::sleep_for(std::chrono::milliseconds(300));
		if (cam->startVideo()) {
			if (wasStreaming)
				startVideoThread();
			if (wasDetecting)
				startDetectThread();
			return true;
		}
	}
	return false;
}

bool IVisionStateMachine::restartCamera() {
	bool wasStreaming = runVideo;
	bool wasDetecting = runDetect;
	endVideoThread();
	endDetectThread();
	if (!cam->stopVideo()) {
		
	}
	if (!cam->close()) {
		std::cout << "Couldn't close camera." << std::endl;
		return false;
	}
	sleep(2);
	if (!cam->setup()) {
		std::cout << "Couldn't start camera." << std::endl;
		return false;
	}
	
	if (stateMap[currentState]->isUsingCamera() || wasStreaming || wasDetecting) {
		if (!cam->startVideo())
			return false;
	}

	if (wasStreaming)
		startVideoThread();
	if (wasDetecting)
		startDetectThread();

	return true;
}
