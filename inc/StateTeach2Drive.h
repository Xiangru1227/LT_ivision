#pragma once

#include "IVisionState.h"

enum TeachState { TeachTeaching, TeachBegin, TeachTrack, TeachLocked, TeachEnd };

class StateTeach2Drive : public IVisionState {
public:
	//constructors/destructor
	StateTeach2Drive();
	StateTeach2Drive(SmartCamera* ca, iVisionClient* fi, SDKCommunicator* sc, StateEnum* st, MovementCalculator* mo);
	~StateTeach2Drive();

	//state functions
	bool enterState();
	bool stateAction();
	bool leaveState();

private:

	//current operating mode of this state...or current operating state of this mode...I should really straighten out my terms
	TeachState currentTeachState;

	int lockCount;

	//helper functions describing behavior in each state
	bool beginState();
	bool teachingState();
	bool trackState();
	bool lockState();
	bool endState();

};
