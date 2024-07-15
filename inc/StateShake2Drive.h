#pragma once

#include "IVisionState.h"


enum Shake2DriveState { ShakeWatch, ShakeTrack, ShakeLocked };

class StateShake2Drive : public IVisionState {
public:
	//constructors/destructor
	StateShake2Drive();
	StateShake2Drive(SmartCamera* ca, iVisionClient* fi, SDKCommunicator* sc, StateEnum* st, MovementCalculator* mo);
	~StateShake2Drive();

	//state functions
	bool enterState();
	bool stateAction();
	bool leaveState();

private:

	//current operating mode of this state
	Shake2DriveState currentShake2DriveState;

	//helper functions describing behavior in each state
	bool watchState();
	bool trackState();
	bool lockState();
};
