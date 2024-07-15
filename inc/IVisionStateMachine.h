#pragma once

#include <map>
#include "IVisionState.h"
#include "SmartCamera.h"
#include "FirmwareCommunicator.h"
#include "iVisionClient.h"
#include "SDKCommunicator.h"
#include "PowerManager.h"
#include "CameraInterface.h"

class IVisionStateMachine {
public:
	//constructor/destructor
	IVisionStateMachine();
	~IVisionStateMachine();

	//set up any necessary modules for state machine operation
	bool initialize(StateEnum startState, bool startDetectPlus, bool useFirmware, std::string fw_ip);

	//bool IsStateChanged() {return next_state == current_state;}
	
	//run the state machine, continues until the next state is set to End
	int mainLoop();
	//reset video so new camera properties can take effect
	bool resetVideo();
private:
	//current state and desired next state
	StateEnum currentState;
	StateEnum nextState;

	//modules used to perform operations
	SmartCamera *cam;
	//FirmwareCommunicator firmware;
	iVisionClient *fw;
	//Camera Interface access
	CameraInterface *camInf;

	//using movement calculator module
	MovementCalculator *mo;
	SDKCommunicator sdk;
	PowerManager pwrManager;

	//mapping from current state to the object representing that state
	std::map<StateEnum, std::unique_ptr<IVisionState>> stateMap;

	//state machine operation functions
	bool enterNextState();
	bool stateAction();
	bool updateState();
	
	//clean up the modules used in the state machine
	int finish();

	//switch to low or high power mode depending on next state
	void updatePowerMode();

	//counter just for testing/debugging
	unsigned int counter;

#ifdef LT_IVISION_USE_FAKE_IMG_TAKEN_FLAG
	FakeImgTakenFlag fake_img_taken_flag_;
#endif




	//handle updates from SDK
	bool processSDKUpdate();
	bool handleSDKControls(ControlState state);

	//video stream thread (call start and end to start and stop video streaming)
	static void SendImages(IVisionStateMachine* iv);
	void sendImages();
	std::thread videoThread;
	bool runVideo;
	void startVideoThread();
	void endVideoThread();

	//object detection thread (call start and end to start and stop DNN object detection)
	static void PerformDetect(IVisionStateMachine* iv);
	void performObjectDetection();
	std::thread detectThread;
	bool runDetect;
	void startDetectThread();
	void endDetectThread();

	//restart camera in case it is somehow not working
	bool restartCamera();

	//add parameters to SDK acknowledgements
	static std::vector<float> CreateParamList(float param1, float param2 = -1, float param3 = -1, float param4 = -1);
	static std::vector<std::string> CreateMsgList(std::string message1);
};
