#pragma once

#include "IVisionState.h"

class StateManualSMR : public IVisionState {
public:
	//constructors/destructor
	StateManualSMR();
	StateManualSMR(SmartCamera* ca, iVisionClient* fi, SDKCommunicator* sc, StateEnum* st, MovementCalculator* mo);
	~StateManualSMR();

	//state functions
	bool enterState();
	bool stateAction();
	bool leaveState();
	bool handleSDKControls(ControlState& state);

private:
	bool update_target;
	float target_x, target_y;
	float last_lock_az, last_lock_el;
};
