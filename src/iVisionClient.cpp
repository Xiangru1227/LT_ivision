#include "iVisionClient.h"
#include "unistd.h"
#include <sys/socket.h>
#include <iostream>
#include <unistd.h>
#include "CameraInterface.h"
#include <sstream>
#include <google/protobuf/io/zero_copy_stream_impl_lite.h>
#include <google/protobuf/io/coded_stream.h>
#include <chrono>
#include <vector>

#include <string.h>
//constructor/destructor
iVisionClient::iVisionClient() {
	commConnected = false;
	rtConnected = false;
	commRunning = false;
	rtRunning = false;
	broadcastRunning = false;
    gotBroadcastResponse = false;

    smrStableCount = 0;
	imgAzSlope = 0;
	imgElSlope = 0;

	lastValidDistance = 0;

	flashOffset = 0.0f;
	flashDuration = 0.0f;
	flashBrightness = 0.0f;
	psdLocked = false;
	flashInTK = false;
#ifdef LT_IVISION_USE_FAKE_IMG_TAKEN_FLAG
	fake_img_taken_flag_ = nullptr;
#endif
}

iVisionClient::~iVisionClient() {

}

//do anything necessary to set up connection to firmware
bool iVisionClient::setupConnection(std::string fw_ip) {

	fw_ip_addr = fw_ip;

	//for ILT
	fw_ip_addr = "127.0.0.1";
	commRunning = true;
	commThread = std::thread(RunCommClient, this);
	rtRunning = true;
	rtThread = std::thread(RunRTClient, this);
	broadcastRunning = true;
	broadcastThread = std::thread(RunBroadcastClient, this);

    return true;
}

//clean up anything necessary when done with connection
bool iVisionClient::closeConnection() {
	std::cout << "Closing connection to controller." << std::endl;
	broadcastRunning = false;
	commRunning = false;
	rtRunning = false;
	
	broadcastThread.join();
	rtThread.join();
	commThread.join();

    return true;
}


//tell the firmware to jog the camera by some amount
bool iVisionClient::sendMoveBy(float x, float y) {
    std::vector<float> prm;
    prm.push_back(x);
    prm.push_back(y);
	//std::cout << "send move by: " << x << "," << y <<  std::endl;
	if(lastData.trk_op_mode == trk_OP_Mode::Track || lastData.trk_op_mode == trk_OP_Mode::TrackIdle)
    	return sendCommand(iVisionCommunication::MoveBy, prm);
	return true;
}

bool iVisionClient::sendMoveTo(float x, float y, float radius) {
	
	float cur_az = currentAzimuth();
	float cur_el = currentElevation();
    std::vector<float> prm;
    prm.push_back(x);
    prm.push_back(y);
	prm.push_back(radius);
	smrStableCount = 0;
	//std::cout << "send move to fw: " << x << "," << y <<  std::endl;
	if(lastData.trk_op_mode == trk_OP_Mode::Track || lastData.trk_op_mode == trk_OP_Mode::TrackIdle)
		return sendCommand(iVisionCommunication::MoveTo, prm);
	return true;

}

bool iVisionClient::SetSpiralSearch(float x, float y){

	std::vector<float> prm;
	prm.push_back(x);
    prm.push_back(y);
	if(lastData.trk_op_mode == trk_OP_Mode::Track || lastData.trk_op_mode == trk_OP_Mode::TrackIdle)
		return sendCommand(iVisionCommunication::SetSpiral, prm);
	return true;
}

bool iVisionClient::StartSearch(float dist, float freq){
	//std::cout << "cmd to Spiral, Dist: " << dist << std::endl;
	SpiralInProgress = true;
	std::vector<float> prm;
	SetSpiral(true);
	prm.push_back(dist);
    prm.push_back(freq);
	smrStableCount = 0;
	if(lastData.trk_op_mode == trk_OP_Mode::Track || lastData.trk_op_mode == trk_OP_Mode::TrackIdle)
		return sendCommand(iVisionCommunication::StartSpiral, prm);
	return true;
}

bool iVisionClient::StopSearch(){
	std::cout << "stopping spiral search" << std::endl;
	SpiralInProgress = false;
	SetSpiral(false);
	return sendCommand(iVisionCommunication::StopSpiral);	
}


bool iVisionClient::makeStill() {
	TrackerInfo current = getTrackerInfo();
	return sendMoveTo(current.az, current.el, 0);
}

bool iVisionClient::setPSDLockFlag(bool lock) {
	psdLocked = lock;
    if (lock)
        return sendCommand(iVisionCommunication::SetPSDLocked);
    else
        return sendCommand(iVisionCommunication::SetPSDUnlocked);
}

bool iVisionClient::setFlashInTK(bool flash) {
	flashInTK = flash;
	if (flash)
		return sendCommand(iVisionCommunication::SetFlashInTKOn);
	else
		return sendCommand(iVisionCommunication::SetFlashInTKOff);
}


//TODO: all of these
bool iVisionClient::setFlashOffset(float offset) {
	flashOffset = offset;
    std::vector<float> prm;
    prm.push_back(offset);
    return sendCommand(iVisionCommunication::SetFlashOffset, prm);
}

bool iVisionClient::setFlashDuration(float duration) {
	flashDuration = duration;
    std::vector<float> prm;
    prm.push_back(duration);
    return sendCommand(iVisionCommunication::SetFlashDuration, prm);
}

bool iVisionClient::setFlashBrightness(float brightness) {
	flashBrightness = brightness;
    std::vector<float> prm;
    prm.push_back(brightness);
    return sendCommand(iVisionCommunication::SetFlashBrightness, prm);
}

bool iVisionClient::setCamMode(bool on) {
	if (on) {
		return sendCommand(iVisionCommunication::SetCamModeOn);
	}
	else {
		return sendCommand(iVisionCommunication::SetCamModeOff);
	}
}

bool iVisionClient::setLedAlwaysOn(bool on) {
	if (on) {
		return sendCommand(iVisionCommunication::SetLEDState, {1.0f});
	}
	else {
		return sendCommand(iVisionCommunication::SetLEDState, {0.0f});
	}
}

bool iVisionClient::sendState() {
	return setPSDLockFlag(psdLocked) && setFlashInTK(flashInTK) && setFlashOffset(flashOffset) && setFlashDuration(flashDuration) && setFlashBrightness(flashBrightness);
}


bool iVisionClient::trackerStill() {
    std::lock_guard<std::mutex> guard(dataLock);
	//std::cout << "SMR Stable Count:" << smrStableCount << std::endl;
    return (smrStableCount > 400);
}

//not totally sure what this is, I don't know if it's based on SMR movement or just how long it's been locked onto it
float iVisionClient::SMRStableDuration() {
    std::lock_guard<std::mutex> guard(dataLock);
    return smrStableCount * 0.04762f;
}

//just if the firmware is locked on to an SMR
bool iVisionClient::hasSMRLock() {
	//std::cout << "Checking smr lock" << std::endl;
    return getTrackerInfo().locked && getImageTrackerInfo().locked && getPrevImageTrackerInfo().locked;
}

//get current azimuth and elevation of the Radian
float iVisionClient::currentAzimuth() {
    return getTrackerInfo().az;
}

float iVisionClient::currentElevation() {
    return getTrackerInfo().el;
}

//current detected SMR distance (if firmware is locked on SMR)
float iVisionClient::currentDistance() {
	std::lock_guard<std::mutex> guard(dataLock);
    return lastValidDistance;
}


TrackerInfo iVisionClient::getTrackerInfo() {
    std::lock_guard<std::mutex> guard(dataLock);
	//std::cout << "Get Tkr Info: " << lastData.locked << std::endl;
    return lastData;
}

bool iVisionClient::is_Spiral() {
	//std::cout << "Is Spiral? " << lastData.trk_op_mode << std::endl;
	if(lastData.trk_op_mode == trk_OP_Mode::SpiralSearch)
		return true;
	return false;
}

bool iVisionClient::is_camera_search() {
	std::cout << "is CamSearch? " << lastData.trk_op_mode << std::endl;
	if(lastData.trk_op_mode == trk_OP_Mode::CameraSearch)
		return true;
	return false;
}

bool iVisionClient::is_Tidle(){
	std::cout << "is CamSearch? " << lastData.trk_op_mode << std::endl;
	if(lastData.trk_op_mode == trk_OP_Mode::TrackIdle)
		return true;
	return false;
}
TrackerInfo iVisionClient::getImageTrackerInfo() {
    std::lock_guard<std::mutex> guard(dataLock);
	//std::cout << "1st data: " << imgData.locked <<std::endl;
    return imgData;
}

TrackerInfo iVisionClient::getPrevImageTrackerInfo() {
    std::lock_guard<std::mutex> guard(dataLock);
	//std::cout << "2nd data: " << imgData2.locked <<std::endl;
    return imgData2;
}

//use the difference in angle between the last two images to estimate the angles for top and bottom of last image
ImgAngles iVisionClient::getImageAngles() {
	ImgAngles ang;
	std::lock_guard<std::mutex> guard(dataLock);
	ang.az_top = imgData.az;
	ang.el_top = imgData.el;
	ang.az_bot = imgData.az + imgAzSlope;
	ang.el_bot = imgData.el + imgElSlope;
	return ang;
}

//use the difference in angle between the last two images to estimate the angles for top and bottom of previous image
ImgAngles iVisionClient::getPrevImageAngles() {
	ImgAngles ang;
	std::lock_guard<std::mutex> guard(dataLock);
	ang.az_top = imgData2.az;
	ang.el_top = imgData2.el;
	ang.az_bot = imgData2.az + imgAzSlope;
	ang.el_bot = imgData2.el + imgElSlope;
	return ang;
}

//TODO: all of the client threads
void iVisionClient::runComm() {
	while (commRunning) {
        if (gotBroadcastResponse && !commConnected) {
            char* ip_buf;
            broadcast.GetBroadcastIpAddr(&ip_buf);
			std::cout << "ip_buf: " << ip_buf << std::endl;
			commConnected = comm.connect(ip_buf, IV_COMM_PORT);
			if (commConnected) {
				sendState();
			}
        }
        else if (commConnected) {
            char buf[8];
            int currentReceived = comm.recv(buf, 4, MSG_PEEK);
			//if connection has been lost, reset the servers
			if (currentReceived == 0) {
				std::cout << "Coomm server lost, resetting the servers" << std::endl;
                comm.close();
				commConnected = false;
			}
			//if data received, read length and get full command
			else if (currentReceived > 0) {
				unsigned int packetSize = ReadHeader(buf);
				//std::cout << "Received packet size = " << packetSize << std::endl;
				char *buf2 = new char[packetSize+4];
				currentReceived = comm.recv(buf2, packetSize + 4, MSG_WAITALL);
				//std::cout << "Current received: " << currentReceived << std::endl;
				if (currentReceived > 0) {
                    iVisionCommunication::Acknowledgement ack = ReadAckBody(buf2, packetSize);
                    //maybe check ack, but generally not necessary here
				}
				delete[] buf2;
			}
        }
        else {
            usleep(100000);
        }
	}
    if (commConnected) {
        comm.close();
        commConnected = false;
    }
}

void iVisionClient::runRT() {
	while (rtRunning) {

         if (gotBroadcastResponse && !rtConnected) {
            char* ip_buf;
            broadcast.GetBroadcastIpAddr(&ip_buf);
            rtConnected = rt.connect(ip_buf, IV_RT_PORT);
			//rtConnected = rt.connect("192.168.0.168", IV_RT_PORT);
        }
        else if (rtConnected) {
            char buf[8];
            int currentReceived = rt.recv(buf, 4, MSG_PEEK);
			//if connection has been lost, reset the servers
			if (currentReceived == 0) {
				std::cout << "RT server lost, resetting the servers" << std::endl;
                rt.close();
				rtConnected = false;
			}
			//if data received, read length and get full command
			else if (currentReceived > 0) {
				unsigned int packetSize = ReadHeader(buf);
				//std::cout << "Received packet size = " << packetSize << std::endl;
				char *buf2 = new char[packetSize+4];
				currentReceived = rt.recv(buf2, packetSize + 4, MSG_WAITALL);
				//std::cout << "Current received: " << currentReceived << std::endl;
				if (currentReceived > 0) {
                    iVisionCommunication::TrackerData tkdata = ReadTrackerDataBody(buf2, packetSize);
					//TODO: update realtime data
                    TrackerInfo tkinfo = extractTrackerInfo(tkdata);
					//std::cout << "Received tk data with timestamp " << tkinfo.timestamp << "locked: " << tkinfo.locked << std::endl;
                    std::lock_guard<std::mutex> guard(dataLock);
					//if current distance not valid (because using reference or not locked on SMR), replace distance with last valid distance
					//ref mode currently not 	, also might not matter usually				
					
					// Ensure valid data
					if (/*tkinfo.distance > 0 && tkinfo.distance != 45000.0f &&*/  !tkinfo.refMode) {
				
						// Store last valid distance if locked
						if (tkinfo.locked) {
							lastValidDistance = tkinfo.distance;
							smrStableCount = 0;
						} 	
						// Detect change in distance_command - for Dusty application
						//std::cout << "Distance Command: " << tkinfo.distance_command << std::endl;
						else if (tkinfo.jog_heartBeat != last_jog_hb) {
							last_jog_hb = tkinfo.jog_heartBeat;
							lastValidDistance = tkinfo.distance_command; // Store last valid distance
							//std::cout << "distance_command changed, updating distance to: " << lastValidDistance  << std::endl;
						}
				
						// Get current timestamp
						auto now = std::chrono::steady_clock::now();
				
						// Store current AZ & EL with timestamp
						azElBuffer.push_back({tkinfo.az, tkinfo.el});
						timeBuffer.push_back(now);
				
						// Maintain buffer size (Remove old data)
						if (azElBuffer.size() > REQUIRED_SAMPLES) {
							azElBuffer.erase(azElBuffer.begin());
							timeBuffer.erase(timeBuffer.begin());
						}
				
						// Compute max deviation over the last 500ms
						float maxDiffAZ = 0.0f, maxDiffEL = 0.0f;
						for (const auto& data : azElBuffer) {
							maxDiffAZ = std::max(maxDiffAZ, std::abs(data.first - tkinfo.az));
							maxDiffEL = std::max(maxDiffEL, std::abs(data.second - tkinfo.el));
						}
				
						// Check stability over 500ms
						if (maxDiffAZ <= AZ_THRESHOLD && maxDiffEL <= EL_THRESHOLD) {
							smrStableCount++;  // Position stable, increment count
						} else {
							//std::cout << "Gimble not still, reset stable count" << std::endl;
							smrStableCount = 0;  // Motion detected, reset count
						}
				
						//Debugging Information
						// std::cout << "Max Diff AZ: " << maxDiffAZ 
						// << " | Max Diff EL: " << maxDiffEL
						// << " | Stable Count: " << smrStableCount
						// << " | Locked: " << tkinfo.locked << std::endl;
				
					} else {
						smrStableCount = 0;  // Reset stability count for invalid data
						azElBuffer.clear();
						timeBuffer.clear();
					}
                    
                    if (tkinfo.imgTaken) {
						//std::cout << "Received new az, el: " << lastData.az << ' ' << lastData.el << '\n';
                        imgData2 = imgData;
                        imgData = lastData;
						imgAzSlope = imgData.az - imgData2.az;
						imgElSlope = imgData.el - imgData2.el;
                    }
					lastData = tkinfo;
					//std::cout << "Received angles: " << lastData.az << ", " << lastData.el << std::endl;
				}
				delete[] buf2;
			}
        }
        else {
            usleep(100000);
        }
    }
    if (rtConnected) {
        rt.close();
        rtConnected = false;
    }
}

void iVisionClient::runBroadcast() {
    broadcast.init_broadcast(IV_UDP_PORT, fw_ip_addr);

	while (broadcastRunning) {

		char recv[100] = {0};
		int bytesReceived = 0;
		gotBroadcastResponse = false;
		
		int response_sent_cnt = 0;

		//until TCP servers have connection, keep checking for broadcast
		while (!commConnected && !rtConnected) {
		
			std::cout << "Not connected loop begin." << std::endl;

			//listen for and receive broadcast
			if (!gotBroadcastResponse) {
                if (sendBroadcast()) {
                    bytesReceived = broadcast.RecvBroadcast(recv, 100, 0);
                    if (bytesReceived > 0) {
						std::cout << "Received Broadcast" << std::endl;
                        unsigned int packetSize = ReadHeader(recv);
                        iVisionCommunication::Acknowledgement ack = ReadAckBody(recv, packetSize);
                        if (ack.method() == iVisionCommunication::Broadcast && ack.success())
                            gotBroadcastResponse = true;
                    }
                }
			}
			else {
				usleep(100000);		
			}
		}
		//as long as TCP servers have connection, just wait and do nothing
		while (commConnected || rtConnected) {
			usleep(100000);
		}
	}
}

bool iVisionClient::sendCommand(iVisionCommunication::MethodId method, std::vector<float> params) {
    std::string msg = CreateCommand(method, params);
	return sendToComm((char*)msg.data(), msg.size());
}

bool iVisionClient::sendToComm(char* buf, unsigned long size) {
	if (commConnected)
		return comm.send(buf, size) > 0;
	return false;
}

bool iVisionClient::sendBroadcast() {
    std::string msg = CreateCommand(iVisionCommunication::Broadcast);
	int send_ret = broadcast.SendBroadcast((char*)msg.data(), msg.size());
    return send_ret > 0;
}


TrackerInfo iVisionClient::extractTrackerInfo(iVisionCommunication::TrackerData tkdata) {
    TrackerInfo tkinfo;
    tkinfo.timestamp = tkdata.heart_beat();
    tkinfo.az = tkdata.az_angle();
    tkinfo.el = tkdata.el_angle();
    tkinfo.distance = tkdata.distance();
    tkinfo.locked = tkdata.locked();
	tkinfo.trk_op_mode = tkdata.op_mode();
	tkinfo.distance_command = tkdata.distance_command();
	tkinfo.jog_heartBeat = tkdata.jog_hb();
#ifdef LT_IVISION_USE_FAKE_IMG_TAKEN_FLAG
	{
		std::lock_guard<std::mutex> lock(fake_img_taken_flag_mutex_);
		if (fake_img_taken_flag_) {
			tkinfo.imgTaken = fake_img_taken_flag_->ImgTakenSinceLastCall();
		} else {
			tkinfo.imgTaken = false;
		}
	}
#else
    tkinfo.imgTaken = tkdata.img_flag();
#endif
	tkinfo.refMode = tkdata.ref_mode();
    return tkinfo;
}


std::string iVisionClient::CreateCommand(iVisionCommunication::MethodId method, std::vector<float> params) {
	//create acknowledgement (protobuf object)
	iVisionCommunication::BasicCommands cmd;

		// std::ofstream os("commandLog.txt", std::ios::app);
		// os << "Sending command: " << method << " number of args: " << params.size() << '\n';
	cmd.set_method(method);
	for (int i = 0; i < params.size(); i++)
		cmd.add_payload_parameters(params[i]);

	//serialize acknowledgement and return string
	int size = (int)cmd.ByteSize() + 4;
	char* buf = new char[size];
	google::protobuf::io::ArrayOutputStream aos(buf, size);
	google::protobuf::io::CodedOutputStream coded_output(&aos);
	coded_output.WriteLittleEndian32((int)cmd.ByteSize());
	std::string msg;
	if (cmd.SerializePartialToCodedStream(&coded_output)) {
		msg = std::string(buf, size);
	}
	delete[] buf;
	return msg;
}

unsigned int iVisionClient::ReadHeader(char * buffer) {
	unsigned int size;
	google::protobuf::io::ArrayInputStream ais(buffer, 4);
	google::protobuf::io::CodedInputStream coded_input(&ais);
	bool ret = coded_input.ReadLittleEndian32(&size);
	if (ret)
		return size;
	return 0;
}

iVisionCommunication::TrackerData iVisionClient::ReadTrackerDataBody(char* buf, unsigned int size) {
	iVisionCommunication::TrackerData tkd;
	google::protobuf::io::ArrayInputStream ais(buf, size + 4);
	google::protobuf::io::CodedInputStream coded_input(&ais);
	coded_input.ReadLittleEndian32(&size);
	int msgLimit = coded_input.PushLimit(size);
	bool ret = tkd.ParseFromCodedStream(&coded_input);
	coded_input.PopLimit(msgLimit);
	//std::cout << "Received tracker data " << tkd.heart_beat() << std::endl;
	return tkd;
}

iVisionCommunication::Acknowledgement iVisionClient::ReadAckBody(char* buf, unsigned int size) {
	iVisionCommunication::Acknowledgement ack;
	google::protobuf::io::ArrayInputStream ais(buf, size + 4);
	google::protobuf::io::CodedInputStream coded_input(&ais);
	coded_input.ReadLittleEndian32(&size);
	int msgLimit = coded_input.PushLimit(size);
	bool ret = ack.ParseFromCodedStream(&coded_input);
	coded_input.PopLimit(msgLimit);
	//std::cout << "Received ack " << ack.method() << std::endl;
	return ack;
}