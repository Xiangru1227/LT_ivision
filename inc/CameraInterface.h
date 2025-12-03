#pragma once

#include <opencv2/core.hpp>
#include <mutex>
#include <thread>
#include <vector>	

#include "Error.h"
#include <Argus/Argus.h>
#include <EGLStream/EGLStream.h>
#include <Argus/Ext/SensorTimestampTsc.h>
#include <EGLStream/NV/ImageNativeBuffer.h>
#include <NvJpegEncoder.h>
#include <Argus/Ext/BlockingSessionCameraProvider.h>
#include <EGLStream/MetadataContainer.h>

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fstream>
#include <sys/mman.h>

#include "global_types.h"
#include "PublisherObserver.h"

// #define USER_AUTO_EXPOSURE_PRINT(...) \
//         (printf("USER AUTO EXPOSURE SAMPLE: " __VA_ARGS__),fflush(stdout))

// #define EXIT_IF_TRUE(val,msg)   \
//         {if ((val)) {USER_AUTO_EXPOSURE_PRINT("%s\n",msg); return false;}}
// #define EXIT_IF_NULL(val,msg)   \
//         {if (!val) {USER_AUTO_EXPOSURE_PRINT("%s\n",msg); return false;}}
// #define EXIT_IF_NOT_OK(val,msg) \
//         {if (val!=Argus::STATUS_OK) {USER_AUTO_EXPOSURE_PRINT("%s\n",msg); return false;}}
//camera properties (shocking, I know, but what else can I say to sum it up?)
//note: the (0-1) for video ROI means it represents proportions of the image, not pixels
struct CameraProperties {
	unsigned int res_x;	//sensor resolution width
	unsigned int res_y;	//sensor resolution height
	unsigned int video_res_x;	//display (video) resolution width
	unsigned int video_res_y;	//display (video) resolution height
	float video_roi_x;	//video region of interest corner x (0-1)
	float video_roi_y;	//video region of interest corner y (0-1)
	float video_roi_width;	//video region of interest width (0-1)
	float video_roi_height;	//video region of interest height (0-1)
	int fps;	//camera framerate (frames per second)
	double exposure;	//camera exposure (in milliseconds)
	double stream_exposure; // exposure for just streaming	
	float stream_analog_gain; // analog gain for video streaming
	float stream_digital_gain; // digital gain for video streaming
	float analog_gain;	//camera analog gain
	float digital_gain;	//camera digital gain
	bool flash_on;	//flash on
};

//stores ROI in pixel units along with bool representing if ROI is the entire image
struct SmartROI {
	cv::Rect roi;
	bool full_img;
};

class CameraInterface {

public:
	//constructor/destructor
	CameraInterface();
	~CameraInterface();


	//open and close camera
	bool initCamera();
	bool closeCamera();


	//start video capture and apply current camera properties
	bool startCamera();
	//stop video capture
	bool stopCamera();


	//is video capture currently running?
	bool isRunning() {return running;}


	//set/get camera/image properties
	void setCameraProperties(CameraProperties prop);
	void restoreDefaultProperties();
	CameraProperties getCameraProperties() { return properties; }
	cv::Size getResolution() {return cv::Size(properties.res_x, properties.res_y);}

	uint64_t present_time;

	//enable/disable flash - currently doesn't do anything
	bool enableFlash();
	bool disableFlash();


	//retrieve OpenCV images
	//get any of image YUV components, and/or full image in RGBA format, can also specify YUV ROI and whether YUV data should be copied to separate buffer
	bool nextProcessImage(cv::Mat& yImage, bool getY, cv::Mat& uImage, bool getU, cv::Mat& vImage, bool getV, cv::Mat& rgbaImage, bool getRGBA, bool noCopyYUV,  unsigned long &imgTime, cv::Rect yuvAOI = cv::Rect(0,0,0,0));
	//wait for two new images and then put them in the YUV buffers
	bool updateProcessImages(unsigned long& firstTime, unsigned long& secondTime);
	//retrieve two most recent images from buffers
	bool lastTwoProcessImages(cv::Mat& yFirst, cv::Mat& uFirst, cv::Mat& vFirst, cv::Mat& ySecond, cv::Mat& uSecond, cv::Mat& vSecond, unsigned long& firstTime, unsigned long& secondTime);


	//retrieve JPEG images
	bool nextVideoImage(char** buf, unsigned long& data_size, unsigned long &timeStamp);	//get next jpeg encoded video image
	bool nextVideoImage(char** buf, unsigned long& data_size, cv::Point2f roiCorner, cv::Point2f roiSize, unsigned long &timeStamp);	//get next jpeg encoded video image with ROI (0-1)
	bool nextProcessImageJpeg(char** buf, unsigned long& data_size, unsigned long &timeStamp);
	bool nextProcessImageJpeg(char** buf, unsigned long& data_size, cv::Point2f roiCorner, cv::Point2f roiSize, unsigned long &timeStamp);	//get jpeg encoded full resolution image with ROI (0-1)


	//release buffers that are mapped when getting the process image with noCopyYUV
	//(to process images with no extra copying, call nextProcessImage with noCopyYUV = true, do processing, then call releaseBuffers)
	bool releaseBuffers();
	bool releaseBuffers2();
	bool releaseAllBuffers();

#ifdef LT_IVISION_USE_FAKE_IMG_TAKEN_FLAG
	void AttachTimestampObserver(api::common::Observer<ImageCaptureTimestamp>* obs);
	void DetachTimestampObserver(api::common::Observer<ImageCaptureTimestamp>* obs);
#endif
private:

	//Argus objects
	Argus::UniqueObj<Argus::CameraProvider> cameraProvider;	
	std::vector<Argus::CameraDevice*> cameraDevices;
	Argus::UniqueObj<Argus::CaptureSession> captureSession;
	Argus::UniqueObj<Argus::OutputStream> captureStream;
	Argus::UniqueObj<Argus::OutputStream> videoStream;
	Argus::UniqueObj<Argus::Request> request;
	Argus::UniqueObj<EGLStream::FrameConsumer> m_consumer;
	Argus::UniqueObj<EGLStream::FrameConsumer> m_consumer2;
	int m_dmabuf_proc, m_dmabuf_proc2, m_dmabuf_video;

#ifdef LT_IVISION_USE_FAKE_IMG_TAKEN_FLAG
	api::common::Publisher<ImageCaptureTimestamp> timestamp_publisher_;
#endif

	//hardware (I think) jpeg encoder
	NvJPEGEncoder *m_JpegEncoder;


	//buffers
	void *y_buffer, *y_buffer2, *u_buffer, *u_buffer2, *v_buffer, *v_buffer2;
	unsigned int bufferSize;
	unsigned char *jpegBuffer, *processBuffer;


	//current image properties
	CameraProperties properties;
	CameraProperties stream_properties;
	//default camera properties (will use these unless they are modified by SDK)
	CameraProperties default_properties;


	//timestamps for two most recent images, make SMR tracking more accurate when tracker moving
	unsigned long firstProcessImageTime, secondProcessImageTime;


	//mutex to make sure program exits from only one thread
	std::mutex closeGuard;
	std::mutex stopGuard;


	//video capture is currently running
	bool running;


	//camera control
	bool applyCameraProperties();	//send stored camera properties to camera driver
	bool createConsumers();	//set up consumer threads to give access to images
	bool clearConsumers();	//stop/reset consumer threads
	bool startVideoCapture();	//start repeated image capture requests
	bool stopVideoCapture();	//stop repeated image capture requests


	//bring new image from output stream into buffer
	int updateBuffer(int& fd, Argus::UniqueObj<Argus::OutputStream>& ostream, Argus::UniqueObj<EGLStream::FrameConsumer>& fconsumer, unsigned long& imgTime, int fps_limit = -1);


	//encode image to jpeg and put it in buffer
	bool jpegToBuffer(int fd, char** buf, unsigned long& data_size, cv::Rect roi, bool full_image);


	//determine image size in buffer so it can be correctly processed
	void getBufferParams(int fd, uint32_t& width, uint32_t& height, uint32_t& pitch, uint32_t& smallPitch);


	//find next highest power of two (rough estimate of image pitch, given image width)
	static unsigned int nextPowerOfTwo(uint32_t width);


	//scale ROI based on image size and determine if ROI fills entire image
	static SmartROI getROI(int img_w, int img_h, cv::Point2f roiCorner, cv::Point2f roiSize);


	//convert image data in buffer to RGBA OpenCV Mat
	static void ConvertImage(unsigned int startX, unsigned int startY, unsigned int stopX, unsigned int stopY, char* ybuf, char* ubuf, char* vbuf, uint32_t pitch, uint32_t smallPitch, cv::Mat& output);


	//helper functions for unmapping YUV buffers
	bool releaseSomeBuffers(int fd, void *y, void *u, void *v);

#ifdef LT_IVISION_USE_FAKE_IMG_TAKEN_FLAG
	void NotifyTimestampObservers(const ImageCaptureTimestamp& timestamp);
#endif
};

#ifdef LT_IVISION_USE_FAKE_IMG_TAKEN_FLAG
class FakeImgTakenFlag : public api::common::Observer<ImageCaptureTimestamp> {
	public:
		FakeImgTakenFlag() : lastCapturedImgTime_(), lastFlagCheckedImgTime_(), alternate_(false) {}
		virtual ~FakeImgTakenFlag() { }
		bool ImgTakenSinceLastCall() {
			std::lock_guard<std::mutex> lock(lastTimeMutex_);
			if (lastCapturedImgTime_ != lastFlagCheckedImgTime_) {
				auto num_microseconds_since_last_frame = 
					std::chrono::duration_cast<std::chrono::microseconds>(lastCapturedImgTime_ - lastFlagCheckedImgTime_).count();
				//std::cout << "Got frame at " << num_microseconds_since_last_frame << " us after the previous one\n";
				lastFlagCheckedImgTime_ = lastCapturedImgTime_;
				bool alternate_old = alternate_;
				alternate_ = !alternate_;
				if (alternate_old) {
					return false;
				} else {
					return true;
				}
			} else {
				return false;
			}
		}
		virtual void Update(const ImageCaptureTimestamp& timestamp) override {
			std::lock_guard<std::mutex> lock(lastTimeMutex_);
			lastCapturedImgTime_ = timestamp;
		}
	private:
		std::mutex lastTimeMutex_;
		ImageCaptureTimestamp lastCapturedImgTime_;
		ImageCaptureTimestamp lastFlagCheckedImgTime_;
		bool alternate_;
};
#endif