#pragma once

#include "IVisionState.h"

class StateCalibrateSize : public IVisionState {
public:
	//constructors/destructor
	StateCalibrateSize();
	StateCalibrateSize(SmartCamera* ca, iVisionClient* fi, SDKCommunicator* sc, StateEnum* st, MovementCalculator* mo);
	~StateCalibrateSize();

	//state functions
	bool enterState();
	bool stateAction();
	bool leaveState();

private:

	//std::vector<CalibrationPoint> calibPoints;
	float lastDistance;

	std::vector<float> currentSizes;

	//store calibration points to file
	bool writeCalibrationToFile();

};
