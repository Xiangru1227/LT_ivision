#include "SDKCommunicator.h"
#include <sys/socket.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <google/protobuf/io/zero_copy_stream_impl_lite.h>
#include <google/protobuf/io/coded_stream.h>

//constructor
SDKCommunicator::SDKCommunicator() {

	commConnected = false;
	videoConnected = false;
	commRunning = false;
	videoRunning = false;
	broadcastRunning = false;

	memset(&(state.flags), 0, sizeof(ControlFlags));
}

//destructor
SDKCommunicator::~SDKCommunicator() {
	
}




//start threads to run TCP and UDP servers
bool SDKCommunicator::setup() {

	broadcast.setup(UDP_PORT);

	commRunning = true;
	commThread = std::thread(RunCommServer, this);
	videoRunning = true;
	videoThread = std::thread(RunVideoServer, this);
	broadcastRunning = true;
	broadcastThread = std::thread(RunBroadcastServer, this);

	return true;
}


//stop servers and join threads
bool SDKCommunicator::close() {

	broadcastRunning = false;
	commRunning = false;
	videoRunning = false;
	resetServers();
	broadcastThread.join();
	videoThread.join();
	commThread.join();

	return true;
}




//send a jpeg encoded image from buffer to video streaming channel
//bool SDKCommunicator::sendImageBuffer(char* buf, unsigned long data_size, std::vector<ImagePoint> smrs, uint64_t Timestamp) {
bool SDKCommunicator::sendImageBuffer(char* buf, unsigned long data_size, std::vector<ImagePoint> smrs) {
	//std::string response = CreateVideoData(buf, data_size, smrs, Timestamp);
	std::string response = CreateVideoData(buf, data_size, smrs);
	return sendToVideo((char*)response.data(), response.size());
}


//generate acknowledgement and send it to communication channel
bool SDKCommunicator::sendAck(CameraCommunication::MethodId method, bool success, std::vector<float> params, std::vector<std::string> msg) {
	std::string response = CreateAck(method, success, params, msg);
	//std::ofstream os("AckSentLog.txt", std::ios::app);
	//os << "Method " << method << " success" << success << "\n";
	return sendToComm((char*)response.data(), response.size());
}


//return a copy of the current control state, then reset the control flags
ControlState SDKCommunicator::getControlState() {
	std::lock_guard<std::mutex> guard(controlAccess);
	ControlState copy = state;
	memset(&(state.flags), 0, sizeof(ControlFlags));
	return copy;
}




//send buffer to communication channel
bool SDKCommunicator::sendToComm(char* buf, unsigned long size) {
	if (commConnected)
		return comm.send(buf, size) > 0;
	return false;
}


//send buffer to video channel
bool SDKCommunicator::sendToVideo(char* buf, unsigned long size) {
	if (videoConnected)
		return video.send(buf, size) > 0;
	return false;
}




//end any server connections, stop video streaming if it's running
void SDKCommunicator::resetServers() {
	videoConnected = false;
	commConnected = false;
	std::lock_guard<std::mutex> guard(controlAccess);
	if (state.video_on) {
		state.flags.stop_video = true;
		state.video_on = false;
	}
}


//run communication channel server in separate thread
void SDKCommunicator::RunCommServer(SDKCommunicator *sc) {
	sc->runComm();
}


//wait for connection to comm server, then listen for commands until connection is closed
void SDKCommunicator::runComm() {

	//accepts new connections as long as program is running
	while (commRunning) {

		//accept connection
		if (SetupServer(comm, COMM_PORT)) {
			commConnected = true;
			std::cout << "Set up comm server." << std::endl;
		}

		//listen for commands as long as server has connection
		char buf[1000];
		while (commConnected) {
			int currentReceived = comm.recv(buf, 4, MSG_PEEK);
			//if connection has been lost, reset the servers
			if (currentReceived == 0) {
				resetServers();
			}
			//if data received, read length and get full command
			else if (currentReceived > 0) {
				unsigned int packetSize = ReadCommandHeader(buf);
				currentReceived = comm.recv(buf, packetSize + 4, MSG_WAITALL);
				if (currentReceived > 0) {
					CameraCommunication::Commands cmd = ReadCommandBody(buf, packetSize);
					//std::cout << "Received command " << (int)cmd.method() << std::endl;
					//update control state based on command
					updateControls(&cmd);
				}
			}
			
		}
		//once connection is ended, reset the server
		comm.closeserver();
	}
}


//run video channel server in separate thread
void SDKCommunicator::RunVideoServer(SDKCommunicator *sc) {
	sc->runVideo();
}


//wait for connection to video server, then wait until connection is closed
void SDKCommunicator::runVideo() {

	//accepts new connections as long as program is running
	while (videoRunning) {

		//accept connection
		if (SetupServer(video, VIDEO_PORT)) {
			std::cout << "Set up video server." << std::endl;
			videoConnected = true;
		}

		//do nothing as long as server has connection
		while (videoConnected) {
			usleep(100000);
		}

		//once connection is ended, reset the server
		video.closeserver();
	}
}


//run broadcast UDP server in separate thread
void SDKCommunicator::RunBroadcastServer(SDKCommunicator *sc) {
	sc->runBroadcast();
}


//wait for broadcast message, then send response
void SDKCommunicator::runBroadcast() {

	std::cout << "Starting broadcast server." << std::endl;

	//as long as program is running, keep waiting for broadcast whenever there is no TCP connection
	while (broadcastRunning) {

		char recv[100] = {0};
		int bytesReceived = 0;
		bool gotBroadcast = false;

		//until TCP servers have connection, keep checking for broadcast
		while (!commConnected && !videoConnected) {

			//listen for and receive broadcast
			if (!gotBroadcast) {
				bytesReceived = broadcast.recv(recv, 100);
				if (bytesReceived > 0) {
					std::cout << "Received broadcast." << std::endl;
					unsigned int packetSize = ReadCommandHeader(recv);
					CameraCommunication::Commands cmd = ReadCommandBody(recv, packetSize);
					if (cmd.method() == CameraCommunication::BroadcastCamera)
						gotBroadcast = true;
				}
			}
			//send response to broadcast
			else {
				std::string response = CreateAck(CameraCommunication::BroadcastCamera, true);
				broadcast.send((char*)response.data(), response.size());
				usleep(200000);
			}
		}
		//as long as TCP servers have connection, just wait and do nothing
		while (commConnected || videoConnected) {
			usleep(100000);
		}
	}
}


//attempt to accept a new connection to TCP server (will time out and return false, but shouldn't cause a problem)
bool SDKCommunicator::SetupServer(SimpleTCPServerSocket& server, const unsigned int portnum) {
	if (!server.bind(portnum)) {
		std::cout << "Failed to bind server." << std::endl;
		return false;
	}
	if (!server.listen())
		return false;
	return server.accept();
}



//set control state based on received commands
bool SDKCommunicator::updateControls(CameraCommunication::Commands *cmd) {
	std::lock_guard<std::mutex> guard(controlAccess);
	StateEnum desiredState;
	switch(cmd->method()) {
		//get calibration parameters
		case CameraCommunication::CalibrationParameters:
			updateGetCalibration();
		break;
		//start video streaming
		case CameraCommunication::StartVideo:
			updateVideoOn(true);
		break;
		//stop video streaming
		case CameraCommunication::StopVideo:
			updateVideoOn(false);
		break;
		//get exposure
		case CameraCommunication::GetExposure:
			updateGetExposure();
		break;
		//set serialNumber
		case CameraCommunication::GetVersionNumber:
			updateGetFWVersion();
		break;
		//set exposure
		case CameraCommunication::SetExposure:
			if (cmd->parameters_size() > 0)
				updateSetExposure(cmd->parameters(0));
		break;
		//get digital gain
		case CameraCommunication::GetDigitalGain:
			updateGetDigitalGain();
		break;
		//set digital gain
		case CameraCommunication::SetDigitalGain:
			if (cmd->parameters_size() > 0)
				updateSetDigitalGain(cmd->parameters(0));
		break;
		//set video ROI
		case CameraCommunication::SetROIVideoStream:
			if (cmd->parameters_size() >= 4)
				updateSetVideoROI(cmd->parameters(0), cmd->parameters(1), cmd->parameters(2), cmd->parameters(3));
		break;
		//broadcast
		case CameraCommunication::BroadcastCamera:

		break;
		//get analog gain
		case CameraCommunication::GetAnalogGain:
			updateGetAnalogGain();
		break;
		//set analog gain
		case CameraCommunication::SetAnalogGain:
			if (cmd->parameters_size() > 0)
				updateSetAnalogGain(cmd->parameters(0));
		break;
		//get frames per second
		case CameraCommunication::GetFPS:
			updateGetFPS();
		break;
		//set frames per second
		case CameraCommunication::SetFPS:
			if (cmd->parameters_size() > 0)
				updateSetFPS(cmd->parameters(0));
		break;
		//get sensor resolution
		case CameraCommunication::GetSensorResolution:
			updateGetSensorRes();
		break;
		//set sensor resolution
		case CameraCommunication::SetSensorResolution:
			if (cmd->parameters_size() >= 2)
				updateSetSensorRes(cmd->parameters(0), cmd->parameters(1));
		break;
		//get display resolution
		case CameraCommunication::GetDisplayResolution:
			updateGetDisplayRes();
		break;
		//set display resolution
		case CameraCommunication::SetDisplayResolution:
			if (cmd->parameters_size() >= 2)
				updateSetDisplayRes(cmd->parameters(0), cmd->parameters(1));
		break;
		//command to move to next SMR
		case CameraCommunication::NextSMR:
			updateNextSMR();
		break;
		//handle user click
		case CameraCommunication::UserClick:
			if (cmd->parameters_size() >= 2)
				updateUserClick(cmd->parameters(0), cmd->parameters(1));
		break;
		//set iVision op mode
		case CameraCommunication::SetOpMode:
			if (cmd->opmode() == CameraCommunication::Idle)
				updateState(Video);
			else if (cmd->opmode() == CameraCommunication::SingleSMR)
				updateState(SingleSMR);
			else if (cmd->opmode() == CameraCommunication::MultiSMR)
				updateState(MultiSMR);
			else if (cmd->opmode() == CameraCommunication::ManualSMR)
				updateState(ManualSMR);
			else if (cmd->opmode() == CameraCommunication::Teach2Drive)
				updateState(Teach2Drive);
			else if (cmd->opmode() == CameraCommunication::Shake2Drive)
				updateState(Shake2Drive);
			else if (cmd->opmode() == CameraCommunication::CamCalibration)
				updateState(Calibration3);
			else if (cmd->opmode() == CameraCommunication::TrackCalibration)
				updateState(Calibration);
		break;
		default:
			
		break;
	}
	return true;
}


//set control flag to get calibration data
void SDKCommunicator::updateGetCalibration() {
	state.flags.get_calibration_data = true;
}


//set control flag to either start or stop video, update video_on
void SDKCommunicator::updateVideoOn(bool on) {
	if (state.video_on != on) {
		state.flags.start_video = on;
		state.flags.stop_video = !on;
		state.video_on = on;
	}
}


//set control flag to get exposure
void SDKCommunicator::updateGetExposure() {
	state.flags.get_exposure = true;
}


//set control flag to set exposure, update desired exposure
void SDKCommunicator::updateSetExposure(float exposure) {
	state.flags.set_exposure = true;
	state.exposure = exposure;
}


//set control flag to get digital gain
void SDKCommunicator::updateGetDigitalGain() {
	state.flags.get_digital_gain = true;
}


//set control flag to set digital gain, update desired gain
void SDKCommunicator::updateSetDigitalGain(float gain) {
	state.flags.set_digital_gain = true;
	state.digital_gain = gain;
}


//set control flag to set video ROI, update desired ROI
void SDKCommunicator::updateSetVideoROI(float x, float y, float w, float h) {
	state.flags.set_video_roi = true;
	state.roi.x = x;
	state.roi.y = y;
	state.roi.width = w;
	state.roi.height = h;
}


//set control flag to get analog gain
void SDKCommunicator::updateGetAnalogGain() {
	state.flags.get_analog_gain = true;
}


//set control flag to set analog gain, update desired gain
void SDKCommunicator::updateSetAnalogGain(float gain) {
	state.flags.set_analog_gain = true;
	state.analog_gain = gain;
}


//set control flag to get framerate
void SDKCommunicator::updateGetFPS() {
	state.flags.get_fps = true;
}


//set control flag to set framerate, update desired framerate
void SDKCommunicator::updateSetFPS(float fps) {
	state.flags.set_fps = true;
	state.fps = fps;
}


//set control flag to get sensor resolution
void SDKCommunicator::updateGetSensorRes() {
	state.flags.get_sensor_resolution = true;
}


//set control flag to set sensor resolution, update desired resolution
void SDKCommunicator::updateSetSensorRes(float w, float h) {
	state.flags.set_sensor_resolution = true;
	state.sensor_resolution.width = w;
	state.sensor_resolution.height = h;
}


//set control flag to get display resolution
void SDKCommunicator::updateGetDisplayRes() {
	state.flags.get_display_resolution = true;
}


//set control flag to set display resolution, update desired resolution
void SDKCommunicator::updateSetDisplayRes(float w, float h) {
	state.flags.set_display_resolution = true;
	state.display_resolution.width = w;
	state.display_resolution.height = h;
}

void SDKCommunicator::updateNextSMR() {
	//std::cout << "Updating Next SMR." << std::endl;
	state.flags.go_to_next_smr = true;
}

void SDKCommunicator::updateUserClick(float x, float y) {
	state.flags.user_click = true;
	state.click_point.x = x;
	state.click_point.y = y;
}

void SDKCommunicator::updateState(StateEnum st) {
	state.flags.change_iv_state = true;
	state.iv_state = st;
	receivedStateChange = true;
}

void SDKCommunicator::updateGetFWVersion(){
	state.flags.get_version_number = true;
}



//given message components, serialize the data into a buffer (wrapped in a string) so that it's ready to send
std::string SDKCommunicator::CreateAck(CameraCommunication::MethodId method, bool success, std::vector<float> params, std::vector<std::string> msg) {

	//create acknowledgement (protobuf object)
	CameraCommunication::Acknowledgement ack;
	ack.set_method(method);
	ack.set_success(success);
	for (int i = 0; i < params.size(); i++)
		ack.add_parameters(params[i]);
	
	for (int i = 0; i < msg.size(); i++)
		ack.set_parameter_string(msg[i]);

	//serialize acknowledgement and return string
	int size = (int)ack.ByteSize() + 4;
	char* buf = new char[size];
	google::protobuf::io::ArrayOutputStream aos(buf, size);
	google::protobuf::io::CodedOutputStream coded_output(&aos);
	coded_output.WriteLittleEndian32((int)ack.ByteSize());
	std::string response;
	if (ack.SerializePartialToCodedStream(&coded_output)) {
		response = std::string(buf, size);
	}
	delete[] buf;
	return response;
}

//given image data and list of observed SMRs, serialize the data into a buffer to send to SDK
std::string SDKCommunicator::CreateVideoData(char* buf, unsigned long data_size, std::vector<ImagePoint> smrs, uint64_t Timestamp) {

	std::string img_data(buf, data_size);
	CameraCommunication::VideoData vid;
	vid.set_image(img_data);
	for (int i = 0; i < smrs.size(); i++) {
		vid.mutable_smr()->add_x(smrs[i].x);
		vid.mutable_smr()->add_y(smrs[i].y);
	}
	vid.set_opmode(CameraCommunication::Idle);
	vid.set_timestamp(Timestamp);

	//serialize data and return string
	int size = (int)vid.ByteSize() + 4;
	char* buf2 = new char[size];
	google::protobuf::io::ArrayOutputStream aos(buf2, size);
	google::protobuf::io::CodedOutputStream coded_output(&aos);
	coded_output.WriteLittleEndian32((int)vid.ByteSize());
	std::string response;
	if (vid.SerializePartialToCodedStream(&coded_output)) {
		response = std::string(buf2, size);
	}
	delete[] buf2;
	return response;
}


//read size of command packet
unsigned int SDKCommunicator::ReadCommandHeader(char * buffer) {
	unsigned int size;
	google::protobuf::io::ArrayInputStream ais(buffer, 4);
	google::protobuf::io::CodedInputStream coded_input(&ais);
	bool ret = coded_input.ReadLittleEndian32(&size);
	if (ret)
		return size;
	return 0;
}


//given the size found from reading the header, read the full command packet
CameraCommunication::Commands SDKCommunicator::ReadCommandBody(char* buf, unsigned int size) {
	CameraCommunication::Commands cmd;
	google::protobuf::io::ArrayInputStream ais(buf, size + 4);
	google::protobuf::io::CodedInputStream coded_input(&ais);
	coded_input.ReadLittleEndian32(&size);
	auto msgLimit = coded_input.PushLimit(size);
	bool ret = cmd.ParseFromCodedStream(&coded_input);
	coded_input.PopLimit(msgLimit);
	return cmd;
}
