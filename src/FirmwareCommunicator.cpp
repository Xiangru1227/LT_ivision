#include "FirmwareCommunicator.h"

#include <fstream>
#include <iostream>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <chrono>
#include <algorithm>
#include <cmath>

FirmwareCommunicator::FirmwareCommunicator() {
	calibStep = 0;
	lastTimestamp = 0;
	oneCycleComplete = false;
	for (int i = 0; i < 14; i++) {
		writeBuf.c[i] = 0;
		readBuf.c[i] = 0;
	}
	smrStableCount = 0;
	trackerLocks = true;
}

FirmwareCommunicator::~FirmwareCommunicator() {

}

//doesn't do anything yet
bool FirmwareCommunicator::setupConnection() {

	system("echo nvidia | sudo -S chmod 666 /dev/ttyTHS1");
	system("echo nvidia | sudo -S systemctl stop nvgetty");

	struct termios options;
	uartFd = open("/dev/ttyTHS1", O_RDWR | O_NOCTTY | O_NDELAY);

	if (uartFd < 0)
		return false;

	fcntl(uartFd, F_SETFL, 0);
	tcgetattr(uartFd, &options);
	cfsetispeed(&options, B115200);
	cfsetospeed(&options, B115200);
	cfmakeraw(&options);
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	options.c_oflag &= ~OPOST;
	tcsetattr(uartFd, TCSANOW, &options);

	running = true;

	sendThread = std::thread(sendHelper, this);
	receiveThread = std::thread(receiveHelper, this);

	return true;
}

//doesn't do anything yet
bool FirmwareCommunicator::closeConnection() {

	running = false;

	receiveThread.join();
	sendThread.join();

	close(uartFd);
	return true;
}

//should get distance to SMR from firmware, however that will be done (in mm, I believe)
float FirmwareCommunicator::currentDistance() {

	std::lock_guard<std::mutex> guard(readBufAccess);
	return readBuf.t.distance;

}

//should get how long the SMR has been stable (not sure what it means, and presumably this is in ms)
float FirmwareCommunicator::SMRStableDuration() {
	return smrStableCount;
}

//check the firmware to see if it is locked onto an SMR, maybe by checking the current operation mode, or maybe there's some other value that indicates this
bool FirmwareCommunicator::hasSMRLock() {
	std::lock_guard<std::mutex> guard(readBufAccess);
	return readBuf.t.locked1 == 2;
}


//tell the firmware to perform a jog by a certain amount (x corresponds to azimuth, y corresponds to elevation)
bool FirmwareCommunicator::sendMoveBy(float x, float y) {

	unsigned short mode = (trackerLocks) ? 0x0000 : 0x0002;
	return queueRequest(0x80808080, x, y, mode);
}

bool FirmwareCommunicator::sendMoveTo(float x, float y) {

	unsigned short mode = (trackerLocks) ? 0x0001 : 0x0003;
	return queueRequest(0x80808080, x, y, mode);
}

bool FirmwareCommunicator::setPSDLockFlag(bool lock) {
	
	trackerLocks = lock;
	unsigned short mode = (trackerLocks) ? 0x8000 : 0x8002;
	return queueRequest(0x80808080, 0, 0, mode);
}

//well this thing will need to change, but can work it out later
bool FirmwareCommunicator::setFlashOffset(unsigned short offset) {
	
	return queueRequest(0x80808080, 0, 0, offset | 0x0004);
}

bool FirmwareCommunicator::setFlashDuration(unsigned char duration) {

	return queueRequest(0x80808080, 0, 0, duration | 0x0005);
}

bool FirmwareCommunicator::setFlashBrightness(unsigned char brightness) {

	return queueRequest(0x80808080, 0, 0, brightness | 0x0006);
}

float FirmwareCommunicator::currentAzimuth() {
	std::lock_guard<std::mutex> guard(readBufAccess);
	return readBuf.t.az;
}

float FirmwareCommunicator::currentElevation() {
	std::lock_guard<std::mutex> guard(readBufAccess);
	return readBuf.t.el;
}

TrkAngles FirmwareCommunicator::currentAzEl() {
	std::lock_guard<std::mutex> guard(readBufAccess);
	return bundleAngles(readBuf.t.az, readBuf.t.el);
}

TrackerData FirmwareCommunicator::getTrackerData() {
	
	std::lock_guard<std::mutex> guard(readBufAccess);
	return readBuf.t;
}

TrkAngles FirmwareCommunicator::getAnglesFromTime(long time) {
	float az = readBuf.t.az;
	float el = readBuf.t.el;

	//std::cout << "Looking for time: " << time << std::endl;

	//mutex
	std::lock_guard<std::mutex> guard(timestampsAccess);

	int endIndex = (oneCycleComplete) ? 20 : lastTimestamp + 1;
	for (int i = 0; i < endIndex; i++) {
		int prevIndex = (i == 0) ? endIndex - 1 : i - 1;
		//std::cout << "Timestamp " << i << ": " << timestamps[i].time << " - " << timestamps[i].data.az << ", " << timestamps[i].data.el << std::endl;
		if (time >= timestamps[prevIndex].time && time <= timestamps[i].time) {
			int diff1 = (int)(time - timestamps[prevIndex].time);
			int diff2 = (int)(timestamps[i].time - time);
			int total = (int)(timestamps[i].time - timestamps[prevIndex].time);
			az = (timestamps[prevIndex].data.az * diff2 + timestamps[i].data.az * diff1) / total;
			el = (timestamps[prevIndex].data.el * diff2 + timestamps[i].data.el * diff1) / total;
		}
		else if (time >= timestamps[prevIndex].time && i == lastTimestamp) {
			int evenMorePreviousIndex = (prevIndex == 0) ? endIndex - 1 : prevIndex - 1;
			int diff = (int)(time - timestamps[prevIndex].time);
			int total = (int)(timestamps[prevIndex].time - timestamps[evenMorePreviousIndex].time);
			float deltaAz = timestamps[prevIndex].data.az - timestamps[evenMorePreviousIndex].data.az;
			float deltaEl = timestamps[prevIndex].data.el - timestamps[evenMorePreviousIndex].data.el;
			
			az = timestamps[prevIndex].data.az + (deltaAz * diff) / total;
			el = timestamps[prevIndex].data.el + (deltaEl * diff) / total;

			//az = timestamps[prevIndex].data.az;
			//el = timestamps[prevIndex].data.el;
		}
	}

	return bundleAngles(az, el);
}

bool FirmwareCommunicator::trackerStill() {
	std::lock_guard<std::mutex> guard(timestampsAccess);
	int endIndex = (oneCycleComplete) ? 20 : lastTimestamp + 1;
	for (int i = 0; i < endIndex; i++) {
		
		int prevIndex = (i == 0) ? endIndex - 1 : i - 1;
		float diffAz = timestamps[i].data.az - timestamps[prevIndex].data.az;
		float diffEl = timestamps[i].data.el - timestamps[prevIndex].data.el;
		float diff = std::sqrt(diffAz*diffAz + diffEl*diffEl);
		if (diff > .01f)
			return false;
		
	}
	return true;
}

bool FirmwareCommunicator::sendRequest() {
	char sum = 0;
	for (int i = 4; i < 14; i++) {
		sum += writeBuf.c[i];
	}
	writeBuf.c[14] = -sum;
	int n = write(uartFd, writeBuf.c, 15);
	if (n != 15)
		return false;

	return true;
}

bool FirmwareCommunicator::queueRequest(unsigned int header, float az, float el, unsigned short mode) {
	MoveCommand mc;

	//format write buffer
	mc.header = header;
	mc.az = az;
	mc.el = el;
	mc.mode = mode;

	return queueRequest(mc);
}

bool FirmwareCommunicator::queueRequest(MoveCommand mc) {
	std::lock_guard<std::mutex> guard(sendQueueAccess);
	commandsToSend.push_back(mc);
	return true;
}

void FirmwareCommunicator::sendHelper(FirmwareCommunicator* fc) {
	fc->sendCommands();
}

void FirmwareCommunicator::sendCommands() {
	while (running) {
		if (!commandsToSend.empty()) {
			std::lock_guard<std::mutex> guard(sendQueueAccess);
			writeBuf.m = commandsToSend.front();
			commandsToSend.pop_front();
			sendRequest();
		}
		usleep(2000);
	}
}

void FirmwareCommunicator::receiveHelper(FirmwareCommunicator* fc) {
	fc->receiveResponse();
}

void FirmwareCommunicator::receiveResponse() {

	while (running) {
		char buf[18];
		int n = read(uartFd, buf, 18);

		//get time
		auto nowTime = std::chrono::system_clock::now();
		long currentTime = nowTime.time_since_epoch().count();
		//std::cout << "Current time: " << currentTime << std::endl;

		if (buf[0] == 0x80 && buf[1] == 0x80 && buf[2] == 0x80 && buf[3] == 0x80) {

			float previousDistance = readBuf.t.distance;

			//memcpy(fc->readBuf.c, buf + 4, sizeof(readBuf));
			updateReadBuf(buf);

			//std::cout << "Received distance: " << fc->readBuf.t.distance << std::endl;
			//std::cout << "Received locked: " << (int)fc->readBuf.t.locked1 << std::endl;
			//std::cout << "Received locked 2: " << (int)fc->readBuf.t.locked2 << std::endl;

			float difference = readBuf.t.distance - previousDistance;
			if (difference < 100.0f && difference > -100.0f)
				smrStableCount++;
			else
				smrStableCount = 0;

			//mutex lock
			std::lock_guard<std::mutex> guard(timestampsAccess);
			
			//copy to ts
			timestamps[lastTimestamp].data = readBuf.t;
			timestamps[lastTimestamp].time = currentTime - 1250000;

			lastTimestamp++;
			if (lastTimestamp >= 20) {
				lastTimestamp = 0;
				oneCycleComplete = true;
			}
		}

	}



	//std::cout << "Received " << n << " bytes." << std::endl;

}

void FirmwareCommunicator::updateReadBuf(char* buf) {
	std::lock_guard<std::mutex> guard(readBufAccess);
	memcpy(readBuf.c, buf + 4, sizeof(readBuf));
}

TrkAngles FirmwareCommunicator::bundleAngles(float az, float el) {
	TrkAngles ta;
	ta.az = az;
	ta.el = el;
	return ta;
}
