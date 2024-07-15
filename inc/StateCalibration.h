#pragma once

#include "IVisionState.h"

class StateCalibration : public IVisionState {
public:
	//constructors/destructor
	StateCalibration();
	StateCalibration(SmartCamera* ca, iVisionClient* fi, SDKCommunicator* sc, StateEnum* st, MovementCalculator* mo);
	~StateCalibration();

	//state functions
	bool enterState();
	bool stateAction();
	bool leaveState();

private:
	bool finished;
};
