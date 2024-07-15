#pragma once

//#include <tuple>
#include <thread>
#include <mutex>
#include <deque>

struct MoveCommand
{
	unsigned int header;
	float az;
	float el;
	unsigned short mode;
};

union UartWriteBuf
{
	MoveCommand m;	
	char c[14];
};

struct TrackerData
{	
	float el;
	float az;
	float distance;
	char locked1;
	char locked2;
};

union UartReadBuf
{
	TrackerData t;
	char c[14];
};

struct TrackerTimestamp
{
	TrackerData data;
	long time;
};

struct TrkAngles
{
	float az;
	float el;
};

class FirmwareCommunicator {
public:
	//constructor/destructor
	FirmwareCommunicator();
	~FirmwareCommunicator();

	//do anything necessary to set up connection to firmware
	bool setupConnection();

	//clean up anything necessary when done with connection
	bool closeConnection();

	//current detected SMR distance (if firmware is locked on SMR)
	float currentDistance();

	//not totally sure what this is, I don't know if it's based on SMR movement or just how long it's been locked onto it
	float SMRStableDuration();

	//just if the firmware is locked on to an SMR
	bool hasSMRLock();

	//tell the firmware to jog the camera by some amount
	bool sendMoveBy(float x, float y);

	bool sendMoveTo(float x, float y);

	bool setPSDLockFlag(bool lock);

	bool setFlashOffset(unsigned short offset);

	bool setFlashDuration(unsigned char duration);

	bool setFlashBrightness(unsigned char brightness);

	bool trackerStill();

	//get current azimuth and elevation of the Radian
	float currentAzimuth();
	float currentElevation();
	TrkAngles currentAzEl();

	TrackerData getTrackerData();

	TrkAngles getAnglesFromTime(long time);

private:
	//not legit, remove once legit communication happens
	int calibStep;

	int uartFd;
	UartWriteBuf writeBuf;
	UartReadBuf readBuf;

	bool sendRequest();
	void receiveResponse();
	static void receiveHelper(FirmwareCommunicator* fc);

	int smrStableCount;

	TrackerTimestamp timestamps[20];
	int lastTimestamp;
	bool oneCycleComplete;

	std::thread receiveThread;
	std::mutex timestampsAccess;

	bool running;
	bool trackerLocks;

	std::mutex readBufAccess;
	void updateReadBuf(char* buf);
	static TrkAngles bundleAngles(float az, float el);

	bool queueRequest(unsigned int header, float az, float el, unsigned short mode);
	bool queueRequest(MoveCommand mc);

	std::thread sendThread;
	std::mutex sendQueueAccess;
	static void sendHelper(FirmwareCommunicator* fc);
	void sendCommands();
	std::deque<MoveCommand> commandsToSend;
};
