#pragma once

#include "IVisionState.h"

enum MultiSMRState { MultiBegin, MultiTrack, MultiLocked, MultiEnd };

class StateMultiSMR : public IVisionState {
public:
	//constructors/destructor
	StateMultiSMR();
	StateMultiSMR(SmartCamera* ca, iVisionClient* fi, SDKCommunicator* sc, StateEnum* st, MovementCalculator* mo);
	~StateMultiSMR();

	//state functions
	bool enterState();
	bool stateAction();
	bool leaveState();
	bool handleSDKControls(ControlState& state);

private:

	//current operating mode of this state...or current operating state of this mode...I should really straighten out my terms
	MultiSMRState currentMultiSMRState;

	int initialCount;

	int lockCount;

	float startAz, startEl;
	float lastAz, lastEl;

	bool automatic_mode;
	bool next_smr_cmd;
	float last_lock_az, last_lock_el;

	int incorrectLock;

	bool last_smr_has_been_targeted;

	//helper functions describing behavior in each state
	bool beginState();
	bool trackState();
	bool lockState();
	bool endState();
};
