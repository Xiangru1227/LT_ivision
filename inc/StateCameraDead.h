#pragma once

#include "IVisionState.h"

class StateCameraDead : public IVisionState {
public:
	//constructors/destructor
	StateCameraDead();
	StateCameraDead(SmartCamera* ca, SDKCommunicator* sc, StateEnum* st);
	~StateCameraDead();

	//state functions
	bool enterState();
	bool stateAction();
	bool leaveState();

private:
	
};
