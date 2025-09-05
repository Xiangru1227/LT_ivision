#pragma once

#include <vector>
#include <tuple>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaarithm.hpp>
#include <cudaColorspace.h>
#include <cudaMappedMemory.h>
#include <cudaUtility.h>
#include <cuda_runtime.h>
#include "CameraInterface.h"
#include "ImageProcessor.h"
#include "ObjectDetector.h"
#include "SMRTracker.h"
#include "SMRTargeter.h"
#include "SMRSearchControl.h"
#include "MovementCalculator.h"
#include "CameraCalibrationManager.h"
#include "CamCalibReader.h"
#include "iVisionClient.h"
#include "IprobeCalibReader.h"

struct ContourFilterParams {
    double min_circularity = 0.6;
    double max_circularity = 1.0;
    double min_convexity = 0.6;
    double max_convexity = 1.0;
    double min_inertia_ratio = 0.3;
};

class SmartCamera {
public:

	//constructor/destructor
	explicit SmartCamera(iVisionClient *firmaware);
	~SmartCamera();

	
	void reset_tracking_state();

	//camera control

	bool setup();		//initialize camera (should be called once upon starting program, unless intentionally trying to restart camera)
	bool startVideo();	//starts video (should be called to start video capture anytime it's not running)
	bool stopVideo();	//stops video (should be called to stop video capture anytime it is running)
	bool close();		//close camera (should be called once before exiting the program, unless intentionally trying to restart camera)
	bool setProperties(CameraProperties prop);	//set camera properties (should be called before startVideo)
	CameraProperties getProperties();	//current camera properties
	void setAutoLockCameraProp(); // used to change camera properties for single,Multi, manual SMR state
	void setRegCamProp(); // used to change the cam propeties  to default after autolock state
	bool enableCameraFlash();	//turn on camera flash (should be called while video capture is running)
	bool disableCameraFlash();	//turn off camera flash (should be called while video capture is running)
	bool running();		//is camera currently capturing images (i.e. startVideo has been called)?
	void Set_back_cam(bool set_cam);
	void Set_Spiral_Thresh(float spiral_thresh);
	void Set_Spiral_timeout(float set_spiral_timeout);
	void Set_Spiral_freq(float set_spiral_freq);
	void Set_auto_calib(bool set_auto_calib);
	//image capture
	bool grabNextProcessJpeg(char** buf, unsigned long& data_size);
	bool grabNextJpeg(char** buf, unsigned long& data_size);				//get most recent image, used in Video mode
	unsigned long imgTime;
	uint64_t getTimeStamp();

	//SMR data
	std::vector<SMRData> getTrackedSMRs();

	//SMR tracking

	//update SMR tracking
	bool updateSMRImages();

	//Commenting this below declaration to test differential processing - Abhay
	bool updateSMRTracking(float img2AzTop, float img2ElTop, float img2AzBot, float img2ElBot);
	const int DistEstSMRTypeAvg = 2;
	const int DistEstSMRTypeSolid = 1;
	float EstDist;
	float spiral_dist;
	//bool updateSMRTracking(float img1AzTop, float img1ElTop, float img1AzBot, float img1ElBot);
	void clearSMRTracking();

	//iProbe keypoints detection functions
	std::vector<cv::Point2f> getCentroids();
	void setCentroids(const std::vector<cv::Point2f>& newCentroids);
	bool getIprobeLocked();
	void setIprobeLocked(bool value);
	bool getIprobeMode();
	void setIprobeMode(bool value);
	cv::Mat getCameraIntrinsicMat();
	cv::Mat getCameraExtrinsicMat();
	
	bool iProbeDetection();
	bool getSmrCoord(double distance);
	bool getIprobeParams(double distance);
	bool prepareIprobeCUDAInput(const cv::Mat& y, const cv::Mat& u, const cv::Mat& v);
    bool runIprobeCUDAStages();
    void cleanupCudaMemory();
    void initCudaMemory(int width, int height);
	void initPinnedHostMemory(int width, int height, size_t y_step, size_t uv_step);
	bool updateTriggerROI(const cv::Size& imageSize, float roiRange);
	bool detectTriggerLED(const cv::cuda::GpuMat& d_bgr);
	bool updateIprobeROI(const cv::Size& imageSize, float roiRange);
	bool preprocessDetectionImage(const cv::cuda::GpuMat& d_bgr, cv::cuda::GpuMat& d_out);
	bool updateIprobeCentroids();
	bool filterContours(const std::vector<std::vector<cv::Point>>& inputContours, std::vector<std::vector<cv::Point>>& outputContours, const ContourFilterParams& params);
	void drawCentroids(const cv::Mat& image, cv::Point2f smrCoord);

	//AutoExposure
	int auto_exp_reset_interval;
	int auto_exp_counter;
	float targetIntensity;
	double avg_intensity;
	double target_intensity_for_thresh_high;
	double target_intensity_for_thresh_low;
	void setTargetInt_Thresh(float targetIntHigh, float targetIntLow){target_intensity_for_thresh_high = targetIntHigh; target_intensity_for_thresh_low = targetIntLow;}
	void setTargetIntensity_forAutoExp(float targetInt){targetIntensity = targetInt;}
	void setAutoExpResetInterval(int interval){auto_exp_reset_interval = interval;}
	double adjustFlashBrightness(double avgIntensity, float flash_targetIntensity);
	bool checkNeedsAutoExposure();
	double getAvgIntensity(cv::Mat img);

	//Teach2Drive
	bool addTeachTarget(float imgX, float imgY);
	bool saveTeachFile();
	bool loadTeachFile();

	cv::Point2f prev_trkAngles;
	//Shake2Drive
	bool watchForShakingSMR();

	//ManualSMR
	Movement findManualMovement(bool setTarget, float imgX = 0, float imgY = 0);

	//single SMR tracking control
	Movement findTrackingMovement();
	bool  SpiralInAction = false;
	int EL_search_timeout_ct = 0;
	int Spiral_counter = 0;
	bool SpiralUnderProgress = false;
	int auto_calib_buf = 0;
	bool back_cam = 0;
	bool cam_auto_calib = false;
	float spiral_thresh = 1.0;
	int spiral_timeout = 50;
	float spiral_freq = 0.004;
	Movement diff_ang;
	int no_mv_cnt_ang; //used to check if the tracker is stuck because destination angle not reached	
	int no_mv_cnt_img; // used to check if the laser is at the edge of the smr
	int target_not_acq_cntr;
	void reportTrackerLock(bool locked, float az, float el, bool updateCalibration = false, float smrDistance = -1, int stableDuration = 0);

	//SMR target control
	void clearSMRTargets();
	void resetTargetLoop();
	void addSMRTarget(float az, float el, float dist = -1);
	int addAllObservedSMRTargets(bool curr_locked = false, float curr_az = 0, float curr_el = 0, float curr_dist = -1);
	void finalizeSMRTargets();
	bool finishedAllSMRTargets();
	bool finishedAllSMRTargetsExceptOne();
	int getCurrentTarget();
	bool findBasePoint(float &az, float &el);
	int getNumTargets();
	int lockedOnTargetIndex(float az, float el, float distance);
	void incrementSMRTarget();
	bool lockedOntoCurrentTarget(float az, float el, float distance);
	
#ifdef LT_IVISION_USE_FAKE_IMG_TAKEN_FLAG
	void AttachTimestampObserver(api::common::Observer<ImageCaptureTimestamp>* obs);
	void DetachTimestampObserver(api::common::Observer<ImageCaptureTimestamp>* obs);
#endif

	//calibration

	bool loadCalibration();		//load camera (and eventually parallax) calibration from file
	
	bool Filter_calibration();

	//parallax calibration functions
	void resetParallaxCalibration();
	bool parallaxCalibrationStep(float smrDistance);

	//camera calibration
	void setupCameraCalibration();	//ready data structures for new calibration run
	bool cameraCalibrationStep();	//analyze one image for calibration
	void setOutdoorExp();
	void setIndoorExp();

	//object detection

	bool updateDetectionImage();		//give newest full resolution image to object detector
	bool setupObjectDetector(detectNet::NetworkType type);	//prepare network and buffers for object detection
	bool closeObjectDetector();	//clean up object detector
	void setObjectDetectorActive(bool a);	//set whether object detection is performed or not
	bool objectDetectorIsActive();	//tell whether object detection is being performed or not
	bool detectObjects();		//perform object detection
	std::vector<DetectedObject> getDetectedObjects();	//return list of detected objects

private:

	//modules that actually do the work

	CameraInterface cam;
	ImageProcessor imProc;
	ObjectDetector detector;
	SMRTracker tracker;
	SMRTargeter targeter;
	SMRSearchControl searcher;
	MovementCalculator moveCalc;
	CameraCalibrationManager camCalib;
	CalibHandle cHandle;
	IprobeCalibHandle iprobeCalib;

	// iProbe variables
	bool iProbeLocked = false;
	bool enterIprobeMode = false;
	bool camParamLoaded = false;
	bool camLedOn = false;
	std::vector<cv::Point2f> centroids;
	IprobeCalibData iprobeParams;
	const cv::Size imageSize = cv::Size(3264, 2464);

	int bgr_width = -1;
    int bgr_height = -1;
	float iprobeDist;
	float prevMinArea = -1.0f;
	float prevMaxArea = -1.0f;
	size_t pitch_y = 0, pitch_u = 0, pitch_v = 0;
	size_t pitch_green = 0;
    uchar *d_y = nullptr, *d_u = nullptr, *d_v = nullptr;
	uchar* d_green = nullptr;
	uchar* pinnedY = nullptr;
	uchar* pinnedU = nullptr;
	uchar* pinnedV = nullptr;
	cv::Point2f smrCoord;
	cv::Point2f triggerLed;
	cv::Rect triggerROI;
	cv::Rect detectionROI;
	cv::Mat trigger_binary;
    cv::Mat processed;
	cv::Mat kernel_morph;
	cv::cuda::GpuMat d_eroded;
	cv::cuda::GpuMat d_green_gpu;
	cv::cuda::GpuMat d_green_trigger;
	cv::cuda::GpuMat d_binary_trigger;
	cv::cuda::GpuMat d_processed_roi;
    cudaStream_t stream = nullptr;
	std::vector<cv::KeyPoint> keypoints_trigger;
	cv::SimpleBlobDetector::Params blobParams;
	cv::Ptr<cv::SimpleBlobDetector> blobDetector;
	cv::Ptr<cv::cuda::Filter> gaussFilter;
	cv::Ptr<cv::cuda::CannyEdgeDetector> cannyDetector;
	cv::Ptr<cv::cuda::Filter> dilateFilter;
	cv::Ptr<cv::cuda::Filter> openFilter;
	
	//additional object to use firmware stuff in Smart Camera
	iVisionClient* firmware_;

	//exp setting
	bool outdoor_mode = false;
	bool indoor_mode = true;

	//states required for the tracker moving
	enum tracking_movement_State {
        INIT, SEARCH_IN_PROGRESS, FIRSTJOG, SPIRAL_MOVEMENT, REGULAR_JOG, TARGET_NOT_FOUND, STUCK_HANDLING
    };
	//movement tracking state initialization
	tracking_movement_State currentState = INIT;
	
	//AutoExpo
	double getAutoExposureValue();
	//helper functions
	bool updateCameraCalibration(cv::Mat cameraMatrix, cv::Mat distortionCoefficients, cv::Size imageSize);	//store new camera calibration data

	void setSingleSMRTarget(float az, float el, float dist = -1);
	cv::Point3f getLaserCoordinates();			//used to find location of SMR (which is reflecting the red laser) in Calibration mode
	float getRedSum(cv::Point2f lastCoordinates, float approximateDistance);
	cv::Mat grabNextImageRGBA();
	cv::Mat grabNextImageBW();		//get full resolution black and white image as OpenCV Mat - probably shouldn't be used externally, but not sure
	void smrAnglesFromLock(float &az, float &el, float distance);
};
