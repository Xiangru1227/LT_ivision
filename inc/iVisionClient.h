#pragma once

#include <string>
#include <atomic>
#include <vector>
#include <thread>
#include <mutex>
#include <deque>
#include "SimpleTCPSocket.h"
#include "iVisionCommunication.pb.h"

#define IV_UDP_PORT 4010
#define IV_COMM_PORT 4011
#define IV_RT_PORT 4012

#ifdef LT_IVISION_USE_FAKE_IMG_TAKEN_FLAG
class FakeImgTakenFlag;
#endif

enum trk_OP_Mode { Idle = 0, Servo = 1, Track = 2,TrackIdle = 3, IndexSearch = 4, SpiralSearch = 5, CameraSearch = 6};

struct TrackerInfo
{	
	int timestamp;
    float el;
	float az;
	float distance;
	bool locked;
    bool imgTaken;
	iVisionCommunication::OperationMode trk_op_mode;
	bool refMode;
};

//since the camera has a rolling shutter, the tracker can be at different angles when the top and bottom of the image are captured
struct ImgAngles
{
	float az_top;
	float el_top;
	float az_bot;
	float el_bot;
};

class iVisionClient {
public:
	//constructor/destructor
	iVisionClient();
	~iVisionClient();

	//do anything necessary to set up connection to firmware
	bool setupConnection(std::string fw_ip);

	//clean up anything necessary when done with connection
	bool closeConnection();

	//tell the firmware to jog the camera by some amount
	bool sendMoveBy(float x, float y);
	bool sendMoveTo(float x, float y, float radius = 0.0f);
	bool SetSpiralSearch(float x, float y);
	bool StartSearch(float dist, float freq);
	bool StopSearch();
	bool makeStill();
	std::atomic<bool> move_flag{false};
    //set if tracker locks to SMR
	bool setPSDLockFlag(bool lock);
	bool getPSDLockFlag() { return psdLocked; }
	bool setFlashInTK(bool flash);
	bool setCamMode(bool on);

    //set flash configuration
	bool setFlashOffset(float offset);
	bool setFlashDuration(float duration);
	bool setFlashBrightness(float brightness);

    //check if tracker has been still for a while
	bool trackerStill();
    //not totally sure what this is, I don't know if it's based on SMR movement or just how long it's been locked onto it
	float SMRStableDuration();
	//just if the firmware is locked on to an SMR
	bool hasSMRLock();

	//get current angles and distance from tracker
	float currentAzimuth();
	float currentElevation();
	float currentDistance();

    //get all current tracker data
	TrackerInfo getTrackerInfo();
    //get tracker data from last image
    TrackerInfo getImageTrackerInfo();
    //get tracker data from 2 images ago
    TrackerInfo getPrevImageTrackerInfo();
	//get tracker angles for previous images
	ImgAngles getImageAngles();
	ImgAngles getPrevImageAngles();

	int stable_Tidle_duration;
	bool is_Spiral();
	bool is_camera_search();
	bool is_Tidle();
	bool SetSpiral(bool spiral) {spiral = SpiralInProgress;}
	bool GetSpiral(){return SpiralInProgress;}

#ifdef LT_IVISION_USE_FAKE_IMG_TAKEN_FLAG
	void SetFakeImgTakenFlag(FakeImgTakenFlag* new_fake_img_taken_flag) {
		std::lock_guard<std::mutex> lock(fake_img_taken_flag_mutex_);
		fake_img_taken_flag_ = new_fake_img_taken_flag;
	}
#endif

private:

	SimpleTCPClientSocket comm, rt;
	//UDP server (responds to SDK broadcast)
	SimpleBroadcastSocket broadcast;

	std::string fw_ip_addr;

	int smrStableCount;
	bool SpiralInProgress = false;

    std::mutex dataLock;
    TrackerInfo lastData, imgData, imgData2;
	float lastValidDistance;
	int print_cnt = 0;
	float imgAzSlope, imgElSlope;

	float flashOffset, flashDuration, flashBrightness;
	bool psdLocked, flashInTK;

	bool sendState();

	//thread for running communication (control/ack) server
	std::thread commThread;
	bool commRunning;
    bool commConnected;
	static void *RunCommClient(void *cl) {
		((iVisionClient*)cl)->runComm();
	}
	void runComm();

	//thread for running video (streaming) server
	std::thread rtThread;
	bool rtRunning;
    bool rtConnected;
	static void *RunRTClient(void *cl) {
		((iVisionClient*)cl)->runRT();
	}
	void runRT();

	//thread for running broadcast server
	std::thread broadcastThread;
	bool broadcastRunning;
    bool gotBroadcastResponse;
	static void *RunBroadcastClient(void *cl) {
		((iVisionClient*)cl)->runBroadcast();
	}
	void runBroadcast();

#ifdef LT_IVISION_USE_FAKE_IMG_TAKEN_FLAG
	std::mutex fake_img_taken_flag_mutex_;
	FakeImgTakenFlag* fake_img_taken_flag_;
#endif

    bool sendCommand(iVisionCommunication::MethodId method, std::vector<float> params = std::vector<float>());
	bool sendOpCommand(iVisionCommunication::OperationMode OpMode, std::vector<float> params = std::vector<float>());
    bool sendToComm(char* buf, unsigned long size);
    bool sendBroadcast();

    TrackerInfo extractTrackerInfo(iVisionCommunication::TrackerData tkdata);

	//generate command with given parameters
	static std::string CreateCommand(iVisionCommunication::MethodId method, std::vector<float> params = std::vector<float>());
	static std::string CreateOpCommand(iVisionCommunication::OperationMode OpMode, std::vector<float> params = std::vector<float>());
	//read length of received message
	static unsigned int ReadHeader(char * buffer);
	//read full message
	static iVisionCommunication::TrackerData ReadTrackerDataBody(char* buf, unsigned int size);
    static iVisionCommunication::Acknowledgement ReadAckBody(char* buf, unsigned int size);
};