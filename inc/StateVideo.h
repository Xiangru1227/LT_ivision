#pragma once

#include "IVisionState.h"

class StateVideo : public IVisionState {
public:
	//constructors/destructor
	StateVideo();
	StateVideo(SmartCamera* ca, iVisionClient* fi, SDKCommunicator* sc, StateEnum* st, MovementCalculator* mo);
	~StateVideo();

	//state functions
	bool enterState();
	bool stateAction();
	bool leaveState();

private:
	
};
