#pragma once

#include "IVisionState.h"

class StateListen : public IVisionState {
public:
	//constructors/destructor
	StateListen();
	StateListen(SDKCommunicator* sc, StateEnum* st);
	~StateListen();

	//state functions
	bool enterState();
	bool stateAction();
	bool leaveState();

private:

};
