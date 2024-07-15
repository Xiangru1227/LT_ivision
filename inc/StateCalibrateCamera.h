#pragma once

#include "IVisionState.h"

class StateCalibrateCamera : public IVisionState {
public:
	//constructors/destructor
	StateCalibrateCamera();
	StateCalibrateCamera(SmartCamera* ca, iVisionClient* fi, SDKCommunicator* sc, StateEnum* st, MovementCalculator* mo);
	~StateCalibrateCamera();

	//state functions
	bool enterState();
	bool stateAction();
	bool leaveState();

private:
	//perform actions requested by SDK
	//bool handleSDKControls(ControlState state);
};
