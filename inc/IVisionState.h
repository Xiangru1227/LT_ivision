#pragma once

#include "SmartCamera.h"
//#include "FirmwareCommunicator.h"
#include "iVisionClient.h"
#include "SDKCommunicator.h"
#include "MovementCalculator.h"

class IVisionState {
public:
	//constructors/destructor
	IVisionState();
	IVisionState(SmartCamera* ca, iVisionClient* fi, SDKCommunicator* sc, StateEnum* st, MovementCalculator* mo);
	~IVisionState();

	//state functions
	virtual bool enterState() = 0;
	virtual bool stateAction() = 0;
	virtual bool leaveState() = 0;

	//interpret SDK controls
	virtual bool handleSDKControls(ControlState& state);

	bool isUsingCamera() { return usingCamera; }

protected:
	//pointers to modules used by this state (modules are part of the state machine, and these pointers give this state access to the ones it needs)
	SmartCamera* cam;
	iVisionClient* firmware;
	SDKCommunicator* sdk;
	//pointer to the state machine's next state, so can change this if you want to change to a different state
	StateEnum* nextState;

	MovementCalculator* mov;

	//find SMRs in new images from camera
	void updateSMRTracking();

	bool usingCamera;

};
