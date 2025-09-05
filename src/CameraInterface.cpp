#include "CameraInterface.h"
#include <string.h>
#include <iostream>
#include <algorithm>
#include <opencv2/core.hpp>
#include <chrono>
#include <time.h>
#include <queue>


using namespace Argus;
using namespace EGLStream;

//set default values
CameraInterface::CameraInterface() {

	default_properties.res_x = 3264;
	default_properties.res_y = 2464;
	default_properties.video_res_x = 960;
	default_properties.video_res_y = 720;
	default_properties.video_roi_x = 0;
	default_properties.video_roi_y = 0;
	default_properties.video_roi_width = 1;
	default_properties.video_roi_height = 1;
	default_properties.fps = 21;
	default_properties.exposure = 1.0;
	default_properties.stream_exposure = 2.0;
	default_properties.stream_analog_gain = 1.0;
	default_properties.stream_digital_gain = 1.5;
	default_properties.analog_gain = 1.5f;
	default_properties.digital_gain = 1.0f;
	default_properties.flash_on = true;
	default_properties.FC_in_FB = cv::Mat::eye(3, 3, CV_64F);

	properties = default_properties;

	bufferSize = (nextPowerOfTwo(properties.res_x) * properties.res_y * 3) / 2;
	//processBuffer = NULL;
	jpegBuffer = NULL;
	m_JpegEncoder = NvJPEGEncoder::createJPEGEncoder("jpenenc");
	m_dmabuf_proc = -1;
	m_dmabuf_proc2 = -1;
	m_dmabuf_video = -1;

	running = false;
}

//destructor (nothing to do now, although there is some dynamic allocation, it's cleaned up elsewhere because putting it here caused problems with the camera)
CameraInterface::~CameraInterface() {
	if (m_JpegEncoder)
        	delete m_JpegEncoder;
}

//start the camera - called at beginning of program
bool CameraInterface::initCamera() {	

	// Create the CameraProvider object and get the core interface.
	cameraProvider = UniqueObj<CameraProvider>(CameraProvider::create());
	ICameraProvider *iCameraProvider = interface_cast<ICameraProvider>(cameraProvider);
	if (!iCameraProvider)
		ORIGINATE_ERROR("Failed to create CameraProvider");

	// Get the camera devices.
	iCameraProvider->getCameraDevices(&cameraDevices);
	if (cameraDevices.size() == 0)
		ORIGINATE_ERROR("No cameras available");

	Argus::Status st;

	// Create the capture session using the first device and get the core interface.
	captureSession = UniqueObj<CaptureSession>(
	iCameraProvider->createCaptureSession(cameraDevices[0], &st));

	if (st == STATUS_UNAVAILABLE) {
		std::cout << "Camera provider is already in use." << std::endl;
	}

	return true;
}

//exit the camera (stops video capture if it's running) - called at end of program
bool CameraInterface::closeCamera() {

	Argus::ICaptureSession *iCaptureSession = interface_cast<ICaptureSession>(captureSession);
	if (!iCaptureSession)
		ORIGINATE_ERROR("Failed to get ICaptureSession interface");

	Argus::Status st = iCaptureSession->waitForIdle(1000000000);
	if (st == STATUS_TIMEOUT)
		exit(1);
	// Destroy the output stream to end the consumer thread.
	captureStream.reset();
	videoStream.reset();
	iCaptureSession->cancelRequests();
	cameraProvider.reset();
	cameraDevices.clear();
	captureSession.reset();
	request.reset();
	m_consumer.reset();
	m_consumer2.reset();

	std::cout << "Camera closed." << std::endl;

	return true;
}

//apply settings and start video capture
bool CameraInterface::startCamera() {
	if (running)
		return false;
	applyCameraProperties();
	createConsumers();
	startVideoCapture();
	return true;
}

//stop video capture
bool CameraInterface::stopCamera() {
	if (!running)
		return false;
	std::lock_guard<std::mutex> guard(stopGuard);
	if (!stopVideoCapture())
		return false;
	clearConsumers();
	return true;
}

//set the interface camera/image properties, sets AOI to full image (this should be called before the camera starts, or at least before the camera properties are applied)
void CameraInterface::setCameraProperties(CameraProperties prop) {
	properties = prop;
}

void CameraInterface::restoreDefaultProperties() {
	properties = default_properties;
}

//turns the flash output on
bool CameraInterface::enableFlash() {

	return true;
}

//turns the flash output off
bool CameraInterface::disableFlash() {

	return true;
}

//get the most recent high resolution image, specifying which components you want (Y, U, V, or floating point RGBA)
//noCopyYUV true just stores image in YUV buffers, false copies images to separate openCV Mat
//yuvAOI specifies area of interest for YUV images
bool CameraInterface::nextProcessImage(cv::Mat& yImage, bool getY, cv::Mat& uImage, bool getU, cv::Mat& vImage, bool getV, cv::Mat& rgbaImage, bool getRGBA, bool noCopyYUV, unsigned long &Timestamp, cv::Rect yuvAOI) {

	std::lock_guard<std::mutex> guard(stopGuard);

	unsigned long imgTime;
	int result = updateBuffer(m_dmabuf_proc, captureStream, m_consumer, imgTime, 0);
	Timestamp = imgTime;
	// std::cout << "timestamp in nestprocessimage: " << Timestamp << std::endl;
	if (result < 0) {
		return false;
	}
	else if (result == 0) {
		return true;
	}

	if (NvBufferMemMap(m_dmabuf_proc, 0, NvBufferMem_Read_Write, &y_buffer) < 0) {
		ORIGINATE_ERROR("Can't map buffer.");
		return false;
	}
	if (NvBufferMemMap(m_dmabuf_proc, 1, NvBufferMem_Read_Write, &u_buffer) < 0) {
		ORIGINATE_ERROR("Can't map buffer.");
		return false;
	}
	if (NvBufferMemMap(m_dmabuf_proc, 2, NvBufferMem_Read_Write, &v_buffer) < 0) {
		ORIGINATE_ERROR("Can't map buffer.");
		return false;
	}


	uint32_t width, height, pitch, smallPitch;
	getBufferParams(m_dmabuf_proc, width, height, pitch, smallPitch);
	//std::cout << "w=" << width << ", h=" << height << ", p=" << pitch << ", sp=" << smallPitch << std::endl;
	std::thread thread1;
	std::thread thread2;
	std::thread thread3;
	std::thread thread4;

	cv::Mat tempRGBA;
	cv::Rect detectAOI;

	if (getRGBA) {
		tempRGBA = cv::Mat(height, width, CV_32FC4);
		detectAOI = cv::Rect(0, 0, width/2, height/2);

		thread1 = std::thread(ConvertImage, detectAOI.x, detectAOI.y, detectAOI.x + detectAOI.width, detectAOI.y + detectAOI.height/4, (char*)y_buffer, (char*)u_buffer, (char*)v_buffer, pitch, smallPitch, std::ref(tempRGBA));
		thread2 = std::thread(ConvertImage, detectAOI.x, detectAOI.y + detectAOI.height/4, detectAOI.x + detectAOI.width, detectAOI.y + detectAOI.height/2, (char*)y_buffer, (char*)u_buffer, (char*)v_buffer, pitch, smallPitch, std::ref(tempRGBA));
		thread3 = std::thread(ConvertImage, detectAOI.x, detectAOI.y + detectAOI.height/2, detectAOI.x + detectAOI.width, detectAOI.y + 3*detectAOI.height/4, (char*)y_buffer, (char*)u_buffer, (char*)v_buffer, pitch, smallPitch, std::ref(tempRGBA));
		thread4 = std::thread(ConvertImage, detectAOI.x, detectAOI.y + 3*detectAOI.height/4, detectAOI.x + detectAOI.width, detectAOI.y + detectAOI.height, (char*)y_buffer, (char*)u_buffer, (char*)v_buffer, pitch, smallPitch, std::ref(tempRGBA));
	}

	if (yuvAOI.width == 0 && yuvAOI.height == 0) {
		yuvAOI.x = 0;
		yuvAOI.y = 0;
		yuvAOI.width = width;
		yuvAOI.height = height;
	}

	cv::Rect halfAOI = cv::Rect(yuvAOI.x / 2, yuvAOI.y / 2, yuvAOI.width / 2, yuvAOI.height / 2);

	if (noCopyYUV) {	
		cv::Mat temp1 = cv::Mat(height, pitch, CV_8UC1, y_buffer);
		yImage = temp1(yuvAOI);
		cv::Mat temp2 = cv::Mat(height / 2, smallPitch, CV_8UC1, u_buffer);
		uImage = temp2(halfAOI);
		cv::Mat temp3 = cv::Mat(height / 2, smallPitch, CV_8UC1, v_buffer);
		vImage = temp3(halfAOI);
	}
	else {
		if (getY) {
			cv::Mat temp = cv::Mat(height, pitch, CV_8UC1, y_buffer);
			cv::Mat tempY = temp(yuvAOI);
			tempY.copyTo(yImage);
		}
		if (getU) {
			cv::Mat temp = cv::Mat(height / 2, smallPitch, CV_8UC1, u_buffer);
			cv::Mat tempU = temp(halfAOI);
			tempU.copyTo(uImage);
		}
		if (getV) {
			cv::Mat temp = cv::Mat(height / 2, smallPitch, CV_8UC1, v_buffer);
			cv::Mat tempV = temp(halfAOI);
			tempV.copyTo(vImage);
		}

		//releaseBuffers();
	}

	if (getRGBA) {
		thread1.join();
		thread2.join();
		thread3.join();
		thread4.join();
		rgbaImage = tempRGBA;
	}
	if (!noCopyYUV) {
		releaseBuffers();
	}

	return true;
}

//update YUV buffers with most recent images
bool CameraInterface::updateProcessImages(unsigned long& firstTime, unsigned long& secondTime) {

	std::lock_guard<std::mutex> guard(stopGuard);

	unsigned long imgTime1, imgTime2;
	int result1 = updateBuffer(m_dmabuf_proc, captureStream, m_consumer, imgTime1);
	int result2 = updateBuffer(m_dmabuf_proc2, captureStream, m_consumer, imgTime2);

	//update and store first and second process image times

	//return false if image capture failed
	if (result1 < 0 || result2 < 0) {
		return false;
	}
	//return true without updating YUV buffers if above desired FPS or if images not consecutive 
	else if (result1 == 0 || result2 == 0 || result1 != result2 - 1) {
		return true;
	}

	firstProcessImageTime = imgTime1;
	secondProcessImageTime = imgTime2;
	firstTime = firstProcessImageTime;
	secondTime = secondProcessImageTime;

	if (NvBufferMemMap(m_dmabuf_proc, 0, NvBufferMem_Read_Write, &y_buffer) < 0) {
		ORIGINATE_ERROR("Can't map buffer.");
		return false;
	}
	if (NvBufferMemMap(m_dmabuf_proc, 1, NvBufferMem_Read_Write, &u_buffer) < 0) {
		ORIGINATE_ERROR("Can't map buffer.");
		return false;
	}
	if (NvBufferMemMap(m_dmabuf_proc, 2, NvBufferMem_Read_Write, &v_buffer) < 0) {
		ORIGINATE_ERROR("Can't map buffer.");
		return false;
	}

	if (NvBufferMemMap(m_dmabuf_proc2, 0, NvBufferMem_Read_Write, &y_buffer2) < 0) {
		ORIGINATE_ERROR("Can't map buffer.");
		return false;
	}
	if (NvBufferMemMap(m_dmabuf_proc2, 1, NvBufferMem_Read_Write, &u_buffer2) < 0) {
		ORIGINATE_ERROR("Can't map buffer.");
		return false;
	}
	if (NvBufferMemMap(m_dmabuf_proc2, 2, NvBufferMem_Read_Write, &v_buffer2) < 0) {
		ORIGINATE_ERROR("Can't map buffer.");
		return false;
	}

	return true;
}

//provide access (through OpenCV Mats) to 2 most recent images in YUV buffers, also access timestamps for those images
bool CameraInterface::lastTwoProcessImages(cv::Mat& yFirst, cv::Mat& uFirst, cv::Mat& vFirst, cv::Mat& ySecond, cv::Mat& uSecond, cv::Mat& vSecond, unsigned long& firstTime, unsigned long& secondTime) {

	uint32_t width, height, pitch, smallPitch;
	getBufferParams(m_dmabuf_proc, width, height, pitch, smallPitch);

	//TODO: add get RGBA for smart detection here?


	cv::Rect yuvAOI = cv::Rect(0, 0, width, height);
	cv::Rect halfAOI = cv::Rect(yuvAOI.x / 2, yuvAOI.y / 2, yuvAOI.width / 2, yuvAOI.height / 2);


	cv::Mat temp1 = cv::Mat(height, pitch, CV_8UC1, y_buffer);
	yFirst = temp1(yuvAOI);
	cv::Mat temp2 = cv::Mat(height / 2, smallPitch, CV_8UC1, u_buffer);
	uFirst = temp2(halfAOI);
	cv::Mat temp3 = cv::Mat(height / 2, smallPitch, CV_8UC1, v_buffer);
	vFirst = temp3(halfAOI);

	cv::Mat temp4 = cv::Mat(height, pitch, CV_8UC1, y_buffer2);
	ySecond = temp4(yuvAOI);
	cv::Mat temp5 = cv::Mat(height / 2, smallPitch, CV_8UC1, u_buffer2);
	uSecond = temp5(halfAOI);
	cv::Mat temp6 = cv::Mat(height / 2, smallPitch, CV_8UC1, v_buffer2);
	vSecond = temp6(halfAOI);

	firstTime = firstProcessImageTime;
	secondTime = secondProcessImageTime;

	// std::ofstream os("commandLog.txt", std::ios::app);
	// os << "first Img TimeStamp " << firstProcessImageTime << " Second Image Timestamp: " << secondProcessImageTime << '\n';

	return true;
}

//get new video image encoded as JPEG
bool CameraInterface::nextVideoImage(char** buf, unsigned long& data_size, unsigned long &timeStamp) {
	return nextVideoImage(buf, data_size, cv::Point2f(properties.video_roi_x, properties.video_roi_y), cv::Point2f(properties.video_roi_width, properties.video_roi_height), timeStamp);
}

//get new video image encoded as JPEG and limited to the specified ROI
bool CameraInterface::nextVideoImage(char** buf, unsigned long& data_size, cv::Point2f roiCorner, cv::Point2f roiSize, unsigned long &timeStamp) {

	std::lock_guard<std::mutex> guard(stopGuard);

	unsigned long imgTime;
	int result = updateBuffer(m_dmabuf_video, videoStream, m_consumer2, imgTime, 30);
	if (result <= 0) {
		*buf = (char*)jpegBuffer;
		data_size = 0;
		return result == 0;
	}
	timeStamp = imgTime;

	SmartROI region = getROI(properties.video_res_x, properties.video_res_y, roiCorner, roiSize);
	return jpegToBuffer(m_dmabuf_video, buf, data_size, region.roi, region.full_img);
}

bool CameraInterface::nextProcessImageJpeg(char** buf, unsigned long& data_size, unsigned long &timeStamp) {
	return nextProcessImageJpeg(buf, data_size, cv::Point2f(properties.video_roi_x, properties.video_roi_y), cv::Point2f(properties.video_roi_width, properties.video_roi_height), timeStamp);
}

//get new high resolution image encoded as JPEG with specified ROI
bool CameraInterface::nextProcessImageJpeg(char** buf, unsigned long& data_size, cv::Point2f roiCorner, cv::Point2f roiSize, unsigned long &timeStamp) {

	std::lock_guard<std::mutex> guard(stopGuard);

	unsigned long imgTime;
	if (updateBuffer(m_dmabuf_proc, captureStream, m_consumer, imgTime) <= 0) {
		return false;
	}
	timeStamp = imgTime;

	SmartROI region = getROI(properties.res_x, properties.res_y, roiCorner, roiSize);
	return jpegToBuffer(m_dmabuf_proc, buf, data_size, region.roi, region.full_img);
}

//helper function to wait for new image and place it in NvBuffer
int CameraInterface::updateBuffer(int& fd, Argus::UniqueObj<Argus::OutputStream>& ostream, Argus::UniqueObj<EGLStream::FrameConsumer>& fconsumer, unsigned long& imgTime, int fps_limit) {
	if (!running)
		return -1;

	Argus::OutputStream* m_stream = ostream.get();
	EGLStream::IFrameConsumer *iFrameConsumer = interface_cast<IFrameConsumer>(fconsumer);
	Argus::IEGLOutputStream *iStream = interface_cast<IEGLOutputStream>(m_stream);

	// Acquire a frame.
	Argus::Status stat;
	UniqueObj<Frame> frame(iFrameConsumer->acquireFrame(1000000000, &stat));
	if (stat == STATUS_UNAVAILABLE) {
		std::lock_guard<std::mutex> guard(closeGuard);
		stopCamera();
		closeCamera();
		sleep(2);
		initCamera();
		startCamera();
	}

	UniqueObj<EGLStream::MetadataContainer> container(EGLStream::MetadataContainer::create(iStream->getEGLDisplay(), iStream->getEGLStream()));
	IArgusCaptureMetadata *iArgusMeta = interface_cast<IArgusCaptureMetadata>(container.get());
	if (!iArgusMeta)
	ORIGINATE_ERROR("Failed to create Metadata Container");
	CaptureMetadata *metadata = iArgusMeta->getMetadata();
	ICaptureMetadata *iCaptureMeta = interface_cast<ICaptureMetadata>(metadata);
	if (!iCaptureMeta)
	ORIGINATE_ERROR("Unable to get Capture Metadata from frame");
     
	//getting timestamp and storing the diff as a pair
	imgTime = iCaptureMeta->getSensorTimestamp();
	// std::cout << "timestamp in updateBuffer: " << imgTime << std::endl;
	
	// std::ofstream os("TimeStamp.txt", std::ios::app);
	// os << "Present Time: " << present_time <<" nano seconds\n";

	auto currentTime = std::chrono::system_clock::now();

	IFrame *iFrame = interface_cast<IFrame>(frame);
	if (!iFrame)
		return -1;


	struct timespec ts;
	long fullTime = 0;
	long diffTime = 0;
	if (clock_gettime(CLOCK_MONOTONIC, &ts) == 0) {
		auto compareTime = std::chrono::system_clock::now();
		fullTime = ts.tv_sec * 1000000000 + ts.tv_nsec;
		diffTime = compareTime.time_since_epoch().count() - fullTime;
	}

	//std::cout << "Diff time: " << diffTime << std::endl;

	long frameTime = iFrame->getTime();// * 1000;
	//if (diffTime > 0)
	//	imgTime = frameTime + diffTime - 3380000;
	//else
	//	imgTime = currentTime.time_since_epoch().count() - 5000000;

	//imgTime = frameTime + diffTime - 30000000;



	int divider = std::max(properties.fps / fps_limit, 1);

	if (iFrame->getNumber() % divider != 0 && fps_limit > 0) {
		return 0;
	}



	// Get the IImageNativeBuffer extension interface.
	NV::IImageNativeBuffer *iNativeBuffer =
	interface_cast<NV::IImageNativeBuffer>(iFrame->getImage());
	if (!iNativeBuffer)
		ORIGINATE_ERROR("IImageNativeBuffer not supported by Image.");

	// If we don't already have a buffer, create one from this image.
	// Otherwise, just blit to our buffer.
	if (fd == -1)
	{
		std::cout << "Create NV Buffer." << std::endl;		
		//need to use pitch linear format (block linear just does weird stuff)          
		fd = iNativeBuffer->createNvBuffer(iStream->getResolution(),
                                                     NvBufferColorFormat_YUV420,
                                                     NvBufferLayout_Pitch);
		if (fd == -1)
			std::cout << "\tFailed to create NvBuffer" << std::endl;
	}
	else if (iNativeBuffer->copyToNvBuffer(fd) != STATUS_OK)
	{
		ORIGINATE_ERROR("Failed to copy frame to NvBuffer.");
	}
#ifdef LT_IVISION_USE_FAKE_IMG_TAKEN_FLAG
	//update the last frame acquired timestamp
	this->NotifyTimestampObservers(std::chrono::system_clock::now());
#endif
	return iFrame->getNumber();
}

//encode image from NvBuffer and place in JPEG buffer for external use
bool CameraInterface::jpegToBuffer(int fd, char** buf, unsigned long& data_size, cv::Rect roi, bool full_image) {
	if (full_image) {
		unsigned long size = properties.res_x * properties.res_y * 3 / 2;
		m_JpegEncoder->encodeFromFd(fd, JCS_YCbCr, &jpegBuffer, size);
		*buf = (char*)jpegBuffer;
		data_size = size;
	}
	else {
		int tempFd;
		NvBufferCreateParams createParams;
		createParams.width = roi.width;
		createParams.height = roi.height;
		createParams.payloadType = NvBufferPayload_SurfArray;
		createParams.layout = NvBufferLayout_Pitch;
		createParams.colorFormat = NvBufferColorFormat_YUV420;
		createParams.nvbuf_tag = NvBufferTag_NONE;
		if (NvBufferCreateEx(&tempFd, &createParams) != 0)
			std::cout << "Error creating NvBuffer." << std::endl;

		NvBufferRect sRect;
		sRect.top = roi.x;
		sRect.left = roi.y;
		sRect.width = roi.width;
		sRect.height = roi.height;

		NvBufferRect dRect;
		dRect.top = 0;
		dRect.left = 0;
		dRect.width = roi.width;
		dRect.height = roi.height;

		NvBufferTransformParams transformParams;
		transformParams.transform_flag = NVBUFFER_TRANSFORM_CROP_SRC;
		transformParams.transform_flip = NvBufferTransform_None;
		transformParams.transform_filter = NvBufferTransform_Filter_Nearest;
		transformParams.src_rect = sRect;
		transformParams.dst_rect = dRect;
		transformParams.session = NULL;

		if (NvBufferTransform (fd, tempFd, &transformParams) != 0)
			std::cout << "Error transforming NvBuffer." << std::endl;

		unsigned long size = properties.res_x * properties.res_y * 3 / 2;
		m_JpegEncoder->encodeFromFd(tempFd, JCS_YCbCr, &jpegBuffer, size);
		*buf = (char*)jpegBuffer;
		data_size = size;

		if (NvBufferDestroy(tempFd) != 0)
			std::cout << "Error destroying NvBuffer." << std::endl;
	}

	return true;
}

//unmap first set of YUV buffers
bool CameraInterface::releaseBuffers() {
	return releaseSomeBuffers(m_dmabuf_proc, y_buffer, u_buffer, v_buffer);
}

//unmap second set of YUV buffers
bool CameraInterface::releaseBuffers2() {
	return releaseSomeBuffers(m_dmabuf_proc2, y_buffer2, u_buffer2, v_buffer2);
}

//unmap both sets of YUV buffers
bool CameraInterface::releaseAllBuffers() {
	return releaseBuffers() && releaseBuffers2();
}

//helper function for unmapping YUV buffers
bool CameraInterface::releaseSomeBuffers(int fd, void *y, void *u, void *v) {
	if (NvBufferMemUnMap(fd, 0, &y) < 0) {
			ORIGINATE_ERROR("Can't unmap buffer.");
			return false;
	}
	if (NvBufferMemUnMap(fd, 1, &u) < 0) {
			ORIGINATE_ERROR("Can't unmap buffer.");
			return false;
	}
	if (NvBufferMemUnMap(fd, 2, &v) < 0) {
			ORIGINATE_ERROR("Can't unmap buffer.");
			return false;
	}

	return true;
}

//find image buffer properties
void CameraInterface::getBufferParams(int fd, uint32_t& width, uint32_t& height, uint32_t& pitch, uint32_t& smallPitch) {

	NvBufferParams params;
	if (NvBufferGetParams(fd, &params) < 0) {
		width = properties.res_x;
		height = properties.res_y;
		pitch = nextPowerOfTwo(properties.res_x);
		smallPitch = nextPowerOfTwo(properties.res_x/2);
	}
	else {
		width = params.width[0];
		height = params.height[0];
		pitch = params.pitch[0];
		smallPitch = params.pitch[1];
	}
}


//helper function to determine if ROI is full image resolution
SmartROI CameraInterface::getROI(int img_w, int img_h, cv::Point2f roiCorner, cv::Point2f roiSize) {
	int left = (int)(img_w * roiCorner.x);
	int top = (int)(img_h * roiCorner.y);
	int right = (int)(left + roiSize.x * img_w);
	int bottom = (int)(top + roiSize.y * img_h);
	left = std::min(std::max(left, 0), img_w);
	right = std::min(std::max(right, 0), img_w);
	top = std::min(std::max(top, 0), img_w);
	bottom = std::min(std::max(bottom, 0), img_w);

	SmartROI sr;
	if (top >= bottom || left >= right) {
		sr.roi = cv::Rect(0, 0, img_w, img_h);
		sr.full_img = true;
	}
	else {
		sr.roi = cv::Rect(left, top, right - left, bottom - top);
		sr.full_img = (left == 0 && top == 0 && right == img_w && bottom == img_h);
	}
	return sr;
}

//helper function for estimating buffer sizes
unsigned int CameraInterface::nextPowerOfTwo(uint32_t width) {
	unsigned int temp = (unsigned int) width;
	int shift = 0;
	while (temp != 0) {
		temp = temp >> 1;
		shift++;
	}
	temp = 1;
	temp = temp << shift;
	return temp;
}

//convert YUV images to floating point RGBA (format necessary for DNN object detection)
void CameraInterface::ConvertImage(unsigned int startX, unsigned int startY, unsigned int stopX, unsigned int stopY, char* ybuf, char* ubuf, char* vbuf, uint32_t pitch, uint32_t smallPitch, cv::Mat& output) {
	float *p1, *p2;
	for (unsigned int y = startY; y < stopY; y++) {
		unsigned int twiceY = y<<1;
		p1 = output.ptr<float>(twiceY);
		p2 = output.ptr<float>(twiceY+1);
		unsigned int bigY1 = pitch * twiceY;
		unsigned int bigY2 = bigY1 + pitch;
		unsigned int smallY = smallPitch * y;
		for (unsigned int x = startX; x < stopX; x++) {
			unsigned int twiceX = x<<1;
			float yval1 = (float)ybuf[bigY1 + twiceX];
			float yval2 = (float)ybuf[bigY1 + twiceX + 1];
			float yval3 = (float)ybuf[bigY2 + twiceX];
			float yval4 = (float)ybuf[bigY2 + twiceX + 1];
			float uval = (float)(ubuf[smallY + x]) - 128;
			float vval = (float)(vbuf[smallY + x]) - 128;
			float blueAdd = 1.773f * uval;
			float greenAdd = -.714f * vval - .344f * uval;
			float redAdd = 1.403f * vval;
			unsigned int bigX = (x << 3);
			p1[bigX] = yval1 + blueAdd;		//blue
			p1[bigX + 1] = yval1 + greenAdd;		//green
			p1[bigX + 2] = yval1 + redAdd;		//red
			p1[bigX + 3] = 255.0f;		//alpha
			p1[bigX + 4] = yval2 + blueAdd;		//blue
			p1[bigX + 5] = yval2 + greenAdd;		//green
			p1[bigX + 6] = yval2 + redAdd;		//red
			p1[bigX + 7] = 255.0f;		//alpha
			p2[bigX] = yval3 + blueAdd;		//blue
			p2[bigX + 1] = yval3 + greenAdd;		//green
			p2[bigX + 2] = yval3 + redAdd;		//red
			p2[bigX + 3] = 255.0f;		//alpha
			p2[bigX + 4] = yval4 + blueAdd;		//blue
			p2[bigX + 5] = yval4 + greenAdd;		//green
			p2[bigX + 6] = yval4 + redAdd;		//red
			p2[bigX + 7] = 255.0f;		//alpha
		}
	}
}



//once stored camera properties have been set, actually apply them to the camera
bool CameraInterface::applyCameraProperties() {

	Argus::ICaptureSession *iCaptureSession = interface_cast<ICaptureSession>(captureSession);
	if (!iCaptureSession)
		ORIGINATE_ERROR("Failed to get ICaptureSession interface");

	ICameraProperties *iCameraProperties = interface_cast<ICameraProperties>(cameraDevices[0]);
	if (!iCameraProperties)
		ORIGINATE_ERROR("Failed to get ICameraProperties interface");

	// Create the OutputStream.
	std::cout << "Creating output stream" << std::endl;
	UniqueObj<OutputStreamSettings> streamSettings(iCaptureSession->createOutputStreamSettings(STREAM_TYPE_EGL));
	IEGLOutputStreamSettings *iStreamSettings = interface_cast<IEGLOutputStreamSettings>(streamSettings);
	if (!iStreamSettings)
		ORIGINATE_ERROR("Failed to get IEGLOutputStreamSettings interface");

	iStreamSettings->setMetadataEnable(true);
	iStreamSettings->setPixelFormat(PIXEL_FMT_YCbCr_420_888);

	iStreamSettings->setResolution(Size2D<uint32_t>(properties.res_x, properties.res_y));
	captureStream = (UniqueObj<OutputStream>)iCaptureSession->createOutputStream(streamSettings.get());

	iStreamSettings->setResolution(Size2D<uint32_t>(properties.video_res_x, properties.video_res_y));
	videoStream = (UniqueObj<OutputStream>)iCaptureSession->createOutputStream(streamSettings.get());

	

	ISensorMode *iSensorMode;
	std::vector<SensorMode*> sensorModes;
	iCameraProperties->getBasicSensorModes(&sensorModes);
	if (sensorModes.size() == 0)
		ORIGINATE_ERROR("Failed to get sensor modes");

	// Create capture request and enable output stream.
	request = UniqueObj<Request>(iCaptureSession->createRequest());
	IRequest *iRequest = interface_cast<IRequest>(request);
	if (!iRequest)
		ORIGINATE_ERROR("Failed to create Request");
	iRequest->enableOutputStream(captureStream.get());
	iRequest->enableOutputStream(videoStream.get());

	//std::cout << "Available Sensor modes :" << std::endl;
	uint32_t sensor_mode = 0;
	for (uint32_t i = 0; i < sensorModes.size(); i++) {
		iSensorMode = interface_cast<ISensorMode>(sensorModes[i]);
		Size2D<uint32_t> resolution = iSensorMode->getResolution();
		//std::cout << "[" << i << "] W=" << resolution.width() << " H=" << resolution.height() << std::endl;
		if (properties.res_x == resolution.width() && properties.res_y == resolution.height()) {
			sensor_mode = i;
		}
	}
	//std::cout << "Chosen sensor mode: " << sensor_mode << std::endl;

	ISourceSettings *iSourceSettings = interface_cast<ISourceSettings>(iRequest->getSourceSettings());
	if (!iSourceSettings)
		ORIGINATE_ERROR("Failed to get ISourceSettings interface");

	IAutoControlSettings *requestAutoControlSettings = interface_cast<IAutoControlSettings>(iRequest->getAutoControlSettings());

	Range<float> digitalGainRange(requestAutoControlSettings->getIspDigitalGainRange());
	//std::cout << "Digital gain range is " << digitalGainRange.min() << " to " << digitalGainRange.max() << "." << std::endl;

	requestAutoControlSettings->setIspDigitalGainRange(Range<float>(1.0f, properties.digital_gain));
	//requestAutoControlSettings->setAwbLock(true);
	requestAutoControlSettings->setAeAntibandingMode(AE_ANTIBANDING_MODE_OFF); // to control the flickering with artificial lighting
	requestAutoControlSettings->setAeLock(false); // AE off
	// requestAutoControlSettings->setAwbMode(AWB_MODE_DAYLIGHT); // Color temp auto
	requestAutoControlSettings->setColorSaturationBias(1.0f);
	// Check sensor mode index
	if (sensor_mode >= sensorModes.size())
		ORIGINATE_ERROR("Sensor mode index is out of range");
	SensorMode *sensorMode = sensorModes[sensor_mode];
	iSensorMode = interface_cast<ISensorMode>(sensorMode);
	iSourceSettings->setSensorMode(sensorMode);

	// Check fps
	Range<uint64_t> sensorDuration(iSensorMode->getFrameDurationRange());
	//std::cout << "Sensor duration range is " << sensorDuration.min() << " to " << sensorDuration.max() << "." << std::endl;
	Range<uint64_t> exposureTime = iSourceSettings->getExposureTimeRange();
	//std::cout << "Exposure time range is " << exposureTime.min() << " to " << exposureTime.max() << "." << std::endl;
	iSourceSettings->setExposureTimeRange(Range<uint64_t>((uint64_t)(properties.exposure * 1000000),(uint64_t)(properties.exposure * 1000000)));
	//iSourceSettings->setExposureTimeRange(Range<uint64_t>(5000000,5000000));
	Range<uint64_t> desireDuration(1e9/properties.fps+0.9);
	if (desireDuration.min() < sensorDuration.min() || desireDuration.max() > sensorDuration.max()) {
		std::cout << "Requested FPS out of range. Fall back to 30" << std::endl;
		properties.fps = 30;
	}
	iSourceSettings->setFrameDurationRange(Range<uint64_t>(1e9/properties.fps));
	//iSourceSettings->setFrameDurationRange(Range<uint64_t>(1e9/120));
	//std::cout << "Frame duration range: " << iSourceSettings->getFrameDurationRange().max() << std::endl;

	Range<float> gainRange(iSourceSettings->getGainRange());
	//std::cout << "Gain range is " << gainRange.min() << " to " << gainRange.max() << "." << std::endl;

	iSourceSettings->setGainRange(Range<float>(1.0f, properties.analog_gain));
	//iSourceSettings->setGainRange(Range<float>(1.0f, 1.0f));

	std::cout << "Cam Prop Applied" << std::endl;
	return true;
}

//create output streams that actually capture images from camera
bool CameraInterface::createConsumers() {
	//do stuff in thread initialization functions here
	Argus::OutputStream* m_stream = captureStream.get();
	Argus::OutputStream* m_stream2 = videoStream.get();

	// Create the FrameConsumer.
	m_consumer = UniqueObj<FrameConsumer>(FrameConsumer::create(m_stream));
	if (!m_consumer)
		ORIGINATE_ERROR("Failed to create FrameConsumer");
	m_consumer2 = UniqueObj<FrameConsumer>(FrameConsumer::create(m_stream2));
	if (!m_consumer2)
		ORIGINATE_ERROR("Failed to create FrameConsumer");

	bufferSize = (nextPowerOfTwo(properties.res_x) * properties.res_y * 3) / 2;
	//processBuffer = new unsigned char[bufferSize];
	//f (!processBuffer)
	//	return false;

	jpegBuffer = new unsigned char[bufferSize];
	if (!jpegBuffer)
		return false;

	
	std::cout << "Consumers Created" << std::endl;
	return true;
}

//remove output streams that capture images from camera
bool CameraInterface::clearConsumers() {
	if (jpegBuffer)
		delete [] jpegBuffer;
	//if (processBuffer)
	//	delete [] processBuffer;
	if (m_dmabuf_proc != -1) {
		NvBufferDestroy(m_dmabuf_proc);
		m_dmabuf_proc = -1;
	}
	if (m_dmabuf_proc2 != -1) {
		NvBufferDestroy(m_dmabuf_proc2);
		m_dmabuf_proc2 = -1;
	}
	if (m_dmabuf_video != -1) {
		NvBufferDestroy(m_dmabuf_video);
		m_dmabuf_video = -1;
	}
		

	return true;
}

//start repeatedly capturing images
bool CameraInterface::startVideoCapture() {

	Argus::OutputStream* m_stream = captureStream.get();
	Argus::OutputStream* m_stream2 = videoStream.get();

	Argus::ICaptureSession *iCaptureSession = interface_cast<ICaptureSession>(captureSession);
	if (!iCaptureSession)
		ORIGINATE_ERROR("Failed to get ICaptureSession interface");

	// Submit capture requests.
	std::cout << "Starting repeat capture requests." << std::endl;
	if (iCaptureSession->repeat(request.get()) != STATUS_OK)
		ORIGINATE_ERROR("Failed to start repeat capture request");

	Argus::IEGLOutputStream *iStream = interface_cast<IEGLOutputStream>(m_stream);

	if (!iStream)
		ORIGINATE_ERROR("Failed to get ICaptureSession interface");
		
	// Wait until the producer has connected to the stream.
	std::cout << "Waiting until producer is connected..." << std::endl;
	if (iStream->waitUntilConnected() != STATUS_OK)
		ORIGINATE_ERROR("Stream failed to connect.");

	Argus::IEGLOutputStream *iStream2 = interface_cast<IEGLOutputStream>(m_stream2);

	// Wait until the producer has connected to the stream.
	std::cout << "Waiting until producer is connected..." << std::endl;
	if (iStream2->waitUntilConnected() != STATUS_OK)
		ORIGINATE_ERROR("Stream failed to connect.");

	//usleep(1000000);

	running = true;

	std::cout << "Video started" << std::endl;
	return true;
}

//stop repeatedly capturing images
bool CameraInterface::stopVideoCapture() {

	running = false;

	Argus::ICaptureSession *iCaptureSession = interface_cast<ICaptureSession>(captureSession);
	if (!iCaptureSession) {
		ORIGINATE_ERROR("Failed to get ICaptureSession interface");
		return false;
	}

	// Stop the repeating request and wait for idle.
	//iCaptureSession->stopRepeat();
	iCaptureSession->cancelRequests();

	Argus::Status st = iCaptureSession->waitForIdle(3000000000);
    if (st != Argus::STATUS_OK) {
        ORIGINATE_ERROR("Camera failed to enter idle state.");
        return false;
    }

	return true;
}

#ifdef LT_IVISION_USE_FAKE_IMG_TAKEN_FLAG
void CameraInterface::AttachTimestampObserver(api::common::Observer<ImageCaptureTimestamp>* obs) {
	if (obs)
		this->timestamp_publisher_.RegisterObserver(obs);
}
void CameraInterface::DetachTimestampObserver(api::common::Observer<ImageCaptureTimestamp>* obs) {
	if (obs)
		this->timestamp_publisher_.RegisterObserver(obs);
}

void CameraInterface::NotifyTimestampObservers(const ImageCaptureTimestamp& timestamp) {
	this->timestamp_publisher_.NotifyObservers(timestamp);
}
#endif