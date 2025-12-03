#pragma once

#include <string>
#include <thread>
#include <mutex>
#include <vector>
#include <fstream>
#include <opencv2/core.hpp>
#include "SimpleTCPSocket.h"
#include "CameraInterface.h"
#include "SmartCamera.h"
#include "CameraCommunication.pb.h"

//correct ports for SDK communication
#define UDP_PORT 4576
#define COMM_PORT 4577
#define VIDEO_PORT 4578

//iVision Firmware Version Numbers
#define MAJOR_REVISION 1
#define MINOR_REVISION 05 
#define SUB_REVISION NULL

//incorrect ports for local testing
//#define UDP_PORT 5061
//#define COMM_PORT 5060
//#define VIDEO_PORT 5059

//iVision operating states (should be set by the SDK, ultimately)
enum StateEnum { Begin, SingleSMR, MultiSMR, Calibration, Calibration2, Calibration3, Listen, CameraDead, Video, ManualSMR, Shake2Drive, Teach2Drive, SmartDetect, End };

enum ExposureMode { Indoor, Outdoor };
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
	bool enter_iprobe;
	bool exit_iprobe;
	bool get_FC_in_FB;
	bool get_int_mat;
	bool outdoor_exp;
	bool indoor_exp;
	bool send_full_res_img;
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
	ExposureMode exp_mode;		//desired exposure mode
	cv::Mat FC_in_FB;		//transform matrix
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
	bool sendImageBuffer(char* buf, unsigned long data_size, std::vector<ImagePoint> smrs, std::vector<cv::Point2f> detectedCentroids, uint64_t Timestamp);
	//bool sendImageBuffer(char* buf, unsigned long data_size, std::vector<ImagePoint> smrs);
	//send command acknowledgement to SDK
	bool sendAck(CameraCommunication::MethodId method, bool success, std::vector<float> params = std::vector<float>(), std::vector<std::string> msg = std::vector<std::string>(), const char* imageBuffer = nullptr, unsigned long imageSize = 0);

	bool receivedStateChange = false;
	//get control state and reset control flags (so requested action will be performed only once)
	ControlState getControlState();

private:
	//TCP servers (comm - controls/acks, video - video streaming)
	SimpleTCPServerSocket comm, video;
	//UDP server (responds to SDK broadcast)
	SimpleUDPSocket broadcast;

	
	CameraInterface *cam1;

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
	void updateEnterIprobeMode();
	void updateExitIprobeMode();
	void updateGetFCinFB();
	void updateGetIntMat();
	void updateExposureMode(ExposureMode exp);
	void updateGetFullResImg();

	//accept connection to TCP server
	static bool SetupServer(SimpleTCPServerSocket& server, const unsigned int portnum);

	//generate acknowledgement with given parameters
	static std::string CreateAck(CameraCommunication::MethodId method, bool success, std::vector<float> params = std::vector<float>(), std::vector<std::string> msg = std::vector<std::string>(), const char* imageBuffer = nullptr, unsigned long imageSize = 0);
	//generate video data message
	static std::string CreateVideoData(char* buf, unsigned long data_size, std::vector<ImagePoint> smrs, std::vector<cv::Point2f> detectedCentroids, uint64_t Timestamp);
	//static std::string CreateVideoData(char* buf, unsigned long data_size, std::vector<ImagePoint> smrs);
	//create ParamList to send ack to sdk
	std::vector<float> CreateParamList(float param1, float param2 = -1, float param3 = -1, float param4 = -1);
	
	static std::vector<std::string> CreateMsgList(std::string message1);
	//read length of command
	static unsigned int ReadCommandHeader(char * buffer);
	//read full command
	static CameraCommunication::Commands ReadCommandBody(char* buf, unsigned int size);

};
