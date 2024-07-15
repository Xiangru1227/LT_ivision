#pragma once

#include "IVisionState.h"

class StateSingleSMR : public IVisionState {
public:
	//constructors/destructor
	StateSingleSMR();
	StateSingleSMR(SmartCamera* ca, iVisionClient* fi, SDKCommunicator* sc, StateEnum* st, MovementCalculator* mo);
	~StateSingleSMR();

	TrackerInfo tkr;
	//state functions
	bool enterState();
	bool stateAction();
	bool leaveState();

private:
	
};
