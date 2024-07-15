#pragma once

#include <string>
#include <thread>
#include <mutex>
#include <vector>
#include <fstream>
#include "SimpleTCPSocket.h"
#include "CameraCommunication.pb.h"

//correct ports for SDK communication
#define UDP_PORT 4576
#define COMM_PORT 4577
#define VIDEO_PORT 4578

//iVision Firmware Version Numbers
#define MAJOR_REVISION 1
#define MINOR_REVISION 03
#define SUB_REVISION NULL

//incorrect ports for local testing
//#define UDP_PORT 5061
//#define COMM_PORT 5060
//#define VIDEO_PORT 5059

//iVision operating states (should be set by the SDK, ultimately)
enum StateEnum { Begin, SingleSMR, MultiSMR, Calibration, Calibration2, Calibration3, Listen, CameraDead, Video, ManualSMR, Shake2Drive, Teach2Drive, SmartDetect, End };

//rectangular box defined by edges
struct Box {
	float top;
	float bottom;
	float left;
	float right;
};

//rectangular box defined by point and size
struct RegionOfInterest {
	float x;
	float y;
	float width;
	float height;
};

//image resolution in pixels
struct Resolution {
	float width;
	float height;
};

//point within image in pixels
struct ImagePoint {
	float x;
	float y;
};

//flags for notifying iVision when a command is received from SDK
struct ControlFlags {
	bool start_video;
	bool stop_video;
	bool get_calibration_data;
	bool get_exposure;
	bool get_analog_gain;
	bool get_digital_gain;
	bool get_fps;
	bool get_sensor_resolution;
	bool get_display_resolution;
	bool get_video_roi;
	bool set_exposure;
	bool set_analog_gain;
	bool set_digital_gain;
	bool set_fps;
	bool set_sensor_resolution;
	bool set_display_resolution;
	bool set_video_roi;
	bool go_to_next_smr;
	bool user_click;
	bool change_iv_state;
	bool get_version_number;
};

//flags and data representing state desired by SDK
struct ControlState {
	ControlFlags flags;		//flags to control actions
	bool video_on;		//video currently on
	float exposure;		//camera exposure (in ms)
	float analog_gain;		//camera analog gain
	float digital_gain;		//camera digital gain
	float fps;		//camera framerate (frames per second)
	Resolution sensor_resolution;		//camera full sensor resolution
	Resolution display_resolution;		//video stream resolution
	RegionOfInterest roi;		//video stream region of interest
	ImagePoint click_point;		//point in image where user clicked (normalized 0-1)
	StateEnum iv_state;		//desired iVision state
};

class SDKCommunicator {
public:
	//constructor/destructor
	SDKCommunicator();
	~SDKCommunicator();

	//start receive threads
	bool setup();
	//end receive threads
	bool close();

	uint64_t TimeStamp = 0;

	//send jpeg encoded image to SDK
	bool sendImageBuffer(char* buf, unsigned long data_size, std::vector<ImagePoint> smrs, uint64_t Timestamp);
	//send command acknowledgement to SDK
	bool sendAck(CameraCommunication::MethodId method, bool success, std::vector<float> params = std::vector<float>(), std::vector<std::string> msg = std::vector<std::string>());

	bool receivedStateChange = false;
	//get control state and reset control flags (so requested action will be performed only once)
	ControlState getControlState();

private:
	//TCP servers (comm - controls/acks, video - video streaming)
	SimpleTCPServerSocket comm, video;
	//UDP server (responds to SDK broadcast)
	SimpleUDPSocket broadcast;

	//send data to communication or video channel
	bool sendToComm(char* buf, unsigned long size);
	bool sendToVideo(char* buf, unsigned long size);

	//reset TCP servers so they can receive a new connection
	void resetServers();

	//thread for running communication (control/ack) server
	std::thread commThread;
	bool commRunning;
	static void RunCommServer(SDKCommunicator *sc);
	void runComm();

	//thread for running video (streaming) server
	std::thread videoThread;
	bool videoRunning;
	static void RunVideoServer(SDKCommunicator *sc);
	void runVideo();

	//thread for running broadcast server
	std::thread broadcastThread;
	bool commConnected, videoConnected;
	bool broadcastRunning;
	static void RunBroadcastServer(SDKCommunicator *sc);
	void runBroadcast();

	//control state received from SDK
	ControlState state;
	std::mutex controlAccess;

	//update control state
	bool updateControls(CameraCommunication::Commands *cmd);
	void updateGetCalibration();
	void updateVideoOn(bool on);
	void updateGetExposure();
	void updateSetExposure(float exposure);
	void updateGetDigitalGain();
	void updateSetDigitalGain(float gain);
	void updateSetVideoROI(float x, float y, float w, float h);
	void updateGetAnalogGain();
	void updateSetAnalogGain(float gain);
	void updateGetFPS();
	void updateSetFPS(float fps);
	void updateGetSensorRes();
	void updateSetSensorRes(float w, float h);
	void updateGetDisplayRes();
	void updateSetDisplayRes(float w, float h);
	void updateNextSMR();
	void updateUserClick(float x, float y);
	void updateState(StateEnum st);
	void updateGetFWVersion();

	//accept connection to TCP server
	static bool SetupServer(SimpleTCPServerSocket& server, const unsigned int portnum);

	//generate acknowledgement with given parameters
	static std::string CreateAck(CameraCommunication::MethodId method, bool success, std::vector<float> params = std::vector<float>(), std::vector<std::string> msg = std::vector<std::string>());
	//generate video data message
	static std::string CreateVideoData(char* buf, unsigned long data_size, std::vector<ImagePoint> smrs, uint64_t Timestamp);
	//read length of command
	static unsigned int ReadCommandHeader(char * buffer);
	//read full command
	static CameraCommunication::Commands ReadCommandBody(char* buf, unsigned int size);
};
