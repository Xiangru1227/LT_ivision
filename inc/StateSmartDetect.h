#pragma once

#include "IVisionState.h"

class StateSmartDetect : public IVisionState {
public:
	//constructors/destructor
	StateSmartDetect();
	StateSmartDetect(SmartCamera* ca, SDKCommunicator* sc, StateEnum* st, MovementCalculator* mo);
	~StateSmartDetect();

	//state functions
	bool enterState();
	bool stateAction();
	bool leaveState();

private:
	
};
