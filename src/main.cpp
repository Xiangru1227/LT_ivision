#include "IVisionStateMachine.h"

int main(int argc, char* argv[]) {

	printf("iVision ver: %d.%d @ %s %s\n",MAJOR_REVISION,MINOR_REVISION, __TIME__, __DATE__);
	IVisionStateMachine stateMachine;

	bool startingDetectPlus = false;
	bool useFirmware = true;
	StateEnum startingState = Video;
	std::string fw_ip;

	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "detectplus") == 0)
			startingDetectPlus = true;
		else if (strcmp(argv[i], "nofirmware") == 0)
			useFirmware = false;
		else if (strcmp(argv[i], "SingleSMR") == 0)
			startingState = SingleSMR;
		else if (strcmp(argv[i], "Video") == 0)
			startingState = Video;
		else if (strcmp(argv[i], "MultiSMR") == 0)
			startingState = MultiSMR;
		else if (strcmp(argv[i], "ManualSMR") == 0)
			startingState = ManualSMR;
		else if (strcmp(argv[i], "Shake2Drive") == 0)
			startingState = Shake2Drive;
		else if (strcmp(argv[i], "Teach2Drive") == 0)
			startingState = Teach2Drive;
		else if (strcmp(argv[i], "SmartDetect") == 0)
			startingState = SmartDetect;
		else if (strcmp(argv[i], "Calibration") == 0)
			startingState = Calibration;
		else if (strcmp(argv[i], "Calibration2") == 0)
			startingState = Calibration2;
		else if (strcmp(argv[i], "Calibration3") == 0)
			startingState = Calibration3;
		else if (strcmp(argv[i], "Listen") == 0)
			startingState = Listen;
		else if (strcmp(argv[i], "CameraDead") == 0)
			startingState = CameraDead;
		else
			fw_ip = argv[i];
	}

	if (!stateMachine.initialize(startingState, startingDetectPlus, useFirmware, fw_ip)) {
		return -1;
	}
	return stateMachine.mainLoop();
}
