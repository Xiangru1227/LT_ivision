#include "SmartCamera.h"
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <algorithm>
#include "ImageProcessor.h"
#include "Debug.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/calib3d.hpp>
#include "kernel.cuh"
#include <chrono>

//constructor
SmartCamera::SmartCamera(iVisionClient *firmware): firmware_(firmware) {
	camCalib.setCalibHandle(&cHandle);
	moveCalc.setCalibHandle(&cHandle);
	tracker.setCalibrationManager(&camCalib);
	tracker.setCameraInterface(&cam);

	blobParams.filterByColor = true;
	blobParams.blobColor = 255;
	blobParams.filterByArea = true;
	blobParams.filterByCircularity = true;
	blobParams.minCircularity = 0.2f;
	blobParams.filterByConvexity = true;
	blobParams.minConvexity = 0.2f;
	blobParams.filterByInertia = true;
	blobParams.minInertiaRatio = 0.2f;

	gaussFilter = cv::cuda::createGaussianFilter(CV_8UC1, CV_8UC1, cv::Size(5, 5), 0);
    cannyDetector = cv::cuda::createCannyEdgeDetector(70.0, 150.0);
    kernel_morph = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
	dilateFilter = cv::cuda::createMorphologyFilter(cv::MORPH_DILATE, CV_8UC1, kernel_morph);
    kernel_morph_open = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
	openFilter = cv::cuda::createMorphologyFilter(cv::MORPH_OPEN, CV_8UC1, kernel_morph_open);
}

//destructor (don't need to do anything now)
SmartCamera::~SmartCamera() {
	cleanupCudaMemory();
}






//initializes the camera
bool SmartCamera::setup() {
	return cam.initCamera();
}

//starts running the video (and currently starts the detector pipeline in full mode, not sure where that should actually go)
bool SmartCamera::startVideo() {
	//std::cout << "Start Cam Status: " << cam.startCamera() << std::endl;
	auto_exp_counter = 0;
	return cam.startCamera();
}

//stops capturing video
bool SmartCamera::stopVideo() {
	return cam.stopCamera();
}

//close the camera
bool SmartCamera::close() {
	return cam.closeCamera();
}

//set camera properties
bool SmartCamera::setProperties(CameraProperties prop) {
	cam.setCameraProperties(prop);
	return true;
}

void SmartCamera::setVideoStreamCameraProp(){
	CameraProperties prop;
	prop.res_x = 3264;
	prop.res_y = 2464;
	prop.video_res_x = 960;
	prop.video_res_y = 720;
	prop.video_roi_x = 0;
	prop.video_roi_y = 0;
	prop.video_roi_width = 1;
	prop.video_roi_height = 1;
	prop.fps = 21;
	prop.exposure = 25;;
	prop.analog_gain = 2.5f;
	prop.digital_gain = 2.5f;
	video_stream_active = true;
	setProperties(prop);
}

void SmartCamera::setAutoLockCameraProp(){
	CameraProperties prop;
	prop.res_x = 3264;
	prop.res_y = 2464;
	prop.video_res_x = 960;
	prop.video_res_y = 720;
	prop.video_roi_x = 0;
	prop.video_roi_y = 0;
	prop.video_roi_width = 1;
	prop.video_roi_height = 1;
	prop.fps = 21;
	prop.exposure = getAutoExposureValue();
	prop.analog_gain = 2.0f;
	prop.digital_gain = 2.0f;
	setProperties(prop);
}

void SmartCamera::setIprobeCameraProp(){
	CameraProperties prop;
	prop.res_x = 3264;
	prop.res_y = 2464;
	prop.video_res_x = 960;
	prop.video_res_y = 720;
	prop.video_roi_x = 0;
	prop.video_roi_y = 0;
	prop.video_roi_width = 1;
	prop.video_roi_height = 1;
	prop.fps = 21;
	prop.exposure = 10.0;
	prop.analog_gain = 1.0f;
	prop.digital_gain = 1.0f;
	iprobe_stream_active = true;
	setProperties(prop);
}

CameraProperties SmartCamera::getProperties() {
	return cam.getCameraProperties();
}

//turn the camera flash on while it's running
bool SmartCamera::enableCameraFlash() {
	return cam.enableFlash();
}

//turn the camera flash off while it's running
bool SmartCamera::disableCameraFlash() {
	return cam.disableFlash();
}

//images are currently being captured
bool SmartCamera::running() {
	return cam.isRunning();
}

bool SmartCamera::checkNeedsAutoExposure(){
	cv::Mat img = grabNextImageBW();
	avg_intensity = getAvgIntensity(img);
	//std::cout << "Getting Avg Intensity: " <<  avg_intensity << std::endl;
	if(avg_intensity <= target_intensity_for_thresh_low ||  avg_intensity >= target_intensity_for_thresh_high){
		//std::cout << "exp adjustment reqd: " <<  avg_intensity << std::endl;
		return true;
	}
	return false;
}


double SmartCamera::getAvgIntensity(cv::Mat img ){
	return  cv::mean(img)[0];
}

double SmartCamera::getAutoExposureValue() {
    float exposureAdjustment = (targetIntensity / avg_intensity);  // Compute adjustment factor
	//std::cout << "Exposure Calc:" << exposureAdjustment << " with avg Int:" << avg_intensity << " Target Int: "  << targetIntensity <<std::endl;
	return exposureAdjustment;
}

bool SmartCamera::grabNextProcessJpeg(char** buf, unsigned long& data_size) {
	unsigned long imageTime;
	return cam.nextProcessImageJpeg(buf, data_size, imageTime);
}

//return the most recent image from the camera (block until new image is received)
bool SmartCamera::grabNextJpeg(char** buf, unsigned long& data_size) {
	// if (!getIprobeMode() || !getIprobeLocked()) {
		return cam.nextVideoImage(buf, data_size, this->imgTime);
	// }
	// else {
	// 	unsigned long imageTime;
	// 	return cam.nextVideoImage(buf, data_size, imageTime);
	// }
}

uint64_t SmartCamera::getTimeStamp() {
	return this->imgTime;
}

std::vector<SMRData> SmartCamera::getTrackedSMRs() {
	return tracker.getCurrentTrackedSMRs();
}

std::vector<cv::Point2f> SmartCamera::getCentroids() {
	return this->centroids;
}

void SmartCamera::setCentroids(const std::vector<cv::Point2f>& newCentroids) {
	this->centroids = newCentroids;
}

bool SmartCamera::getIprobeLocked() {
	return this->iProbeLocked;
}

void SmartCamera::setIprobeLocked(bool value) {
    this->iProbeLocked = value;
}

bool SmartCamera::getIprobeMode() {
	return this->enterIprobeMode;
}

void SmartCamera::setIprobeMode(bool value) {
    this->enterIprobeMode = value;
}

//wait for new images for SMR tracking, update buffers, and get image timestamps
bool SmartCamera::updateSMRImages() {
	unsigned long firstTime, secondTime;
	if (!cam.updateProcessImages(firstTime, secondTime))
		return false;

	return true;
}

//process last two images to find SMRs, update the SMR tracker
bool SmartCamera::updateSMRTracking(float img2AzTop, float img2ElTop, float img2AzBot, float img2ElBot) {
	//clearing the auto calibration buffer in Movement Calculator when the beam is broken
	// std::cout << "In updateSMRTracking." << std::endl;

	moveCalc.reset_buf();
	
	

	cv::Mat src1Y, src1U, src1V, src2Y, src2U, src2V;
	unsigned long firstImgTime, secondImgTime;


	//get images
	if (!cam.lastTwoProcessImages(src1Y, src1U, src1V, src2Y, src2U, src2V, firstImgTime, secondImgTime)) {
		return false;
	}

	//imwrite(createFilename("absimg",1, 2), src3Y);
	cv::Size imgRes = cam.getResolution();


	//get SMR coordinates in image
	//TODO: need to make sure the sign is right on these angles (must test)

	cv::Point2f imgXY1Top = camCalib.singleXYFromAzEl(img2AzTop, img2ElTop, cam.getResolution());
	cv::Point2f imgXY1Bot = camCalib.singleXYFromAzEl(img2AzBot, img2ElBot, cam.getResolution());

	ImgDelta id;

	std::vector<cv::Point2f> redSmrCamAngles;
	ImageSMRs imgsmrs = imProc.getCoordinates(src1Y, src1U, src1V, src2Y, src2U, src2V, false , id, redSmrCamAngles);

	if (!cam.releaseAllBuffers())
		std::cout << "Releasing buffers failed." << std::endl;

	// for (int i = 0; i < redSmrCamAngles.size(); i++) {
	// 	std::cout << "Red SMR: " << redSmrCamAngles[i].x << ", " << redSmrCamAngles[i].y << std::endl;
	// }

	//use information from image processing to determine which image/time/angles the observed SMRs correspond to
	std::vector<SMRData> observedSMRs;
	//unsigned long lastRelevantTime = 0;
	float camAz = 0;
	float camEl = 0;
	
	if (!imgsmrs.observed.empty() && firmware_->trackerStill()) {
	//if (false) {

		float azTop, elTop, azBot, elBot;

		if (imgsmrs.inFirstImage) {
			//lastRelevantTime = firstImgTime;
			camAz = (img2AzTop + img2AzBot) / 2;
			camEl = (img2ElTop + img2ElBot) / 2;
		}
	

		std::vector<cv::Point2f> ptList;
		//std::cout << "Observed SMRs:" << std::endl;
		for (int i = 0; i < imgsmrs.observed.size(); i++) {
			//std::cout << "SMR at: " << imgsmrs.observed[i].imgX << ", " << imgsmrs.observed[i].imgY << std::endl;
			ptList.push_back(cv::Point2f(imgsmrs.observed[i].imgX, imgsmrs.observed[i].imgY));
		}

		std::vector<cv::Point2f> smrCamAngles = camCalib.azElFromXY(ptList, cam.getResolution());

		for (int i = 0; i < imgsmrs.observed.size(); i++) {
			//std::cout << "Observed SMR area: " << imgsmrs.observed[i].imgA << std::endl;
			SMRData data (smrCamAngles[i].x, smrCamAngles[i].y, imgsmrs.observed[i].imgA, imgsmrs.observed[i].imgW, imgsmrs.observed[i].imgH);
		
			// std::cout << "SMR cam angles: " << smrCamAngles[i].x << ", " << smrCamAngles[i].y << std::endl;
			// std::cout << "Camera angles: " << camAz << ", " << camEl << std::endl;
			data.setAz(data.getImgAz());
			data.setEl(data.getImgEl());
			//std::cout << "AZ: " << " = " << data.getAz() <<  ", EL: " << " = " << data.getEl() << std::endl;
			data.setImgCoordinates(imgsmrs.observed[i].imgX, imgsmrs.observed[i].imgY, imgRes);
			observedSMRs.push_back(data);
		}
	}
	else {
		
		//lastRelevantTime = secondImgTime;
		camAz = (img2AzTop + img2AzBot) / 2;
		camEl = (img2ElTop + img2ElBot) / 2;

	}

	//std::cout << "Cam angles: " << camAz << ", " << camEl << std::endl;

	//TODO: convert red SMR coordinates to angles, remove any that aren't possibly the laser
	redSmrCamAngles = camCalib.azElFromXY(redSmrCamAngles, cam.getResolution());
	std::vector<cv::Point2f> laserSMRs;
	for (int i = 0; i < redSmrCamAngles.size(); i++) {
		float laser_distance = moveCalc.getLaserDistance(redSmrCamAngles[i]);
		// std::cout << "smt cam Red SMR: " << redSmrCamAngles[i].x << ", " << redSmrCamAngles[i].y << std::endl;
		// std::cout << "Laser Dist: " << laser_distance << std::endl;
		if (laser_distance >= 0) {	
			
			laserSMRs.push_back(redSmrCamAngles[i]);
			searcher.setLaserObservedDistance(laser_distance);
		}
	}

	//std::cout << "Observed SMRs: " << observedSMRs.size() << std::endl;
	tracker.updateTracking(observedSMRs, camAz, camEl, camCalib.pixelAngle(imgRes), laserSMRs, back_cam);
	if (!laserSMRs.empty()) {
		searcher.laserObserved();
	}
	if (!tracker.getCurrentTrackedSMRs().empty()) {
		searcher.smrObserved();
	}
	

	//see if this works without screwing things up
	//targeter.updateTargets(tracker.getCurrentTrackedSMRs());

	return true;
}

//clear all currently tracked SMRs
void SmartCamera::clearSMRTracking() {
	tracker.clearTracking();
}

cv::Mat SmartCamera::getCameraIntrinsicMat() {
	if(!camCalib.loadCalibrationFromFile()) {
		std::cerr << "Failed to load camera calibration file." << std::endl;
		return cv::Mat();
	}
	camCalib.initializeUndistortionMaps(cam.getResolution());

	return camCalib.getScaledCameraMatrix(this->imageSize).clone();
}

cv::Mat SmartCamera::getCameraExtrinsicMat() {
	if(!camCalib.loadCalibrationFromFile()) {
		std::cerr << "Failed to load camera calibration file." << std::endl;
		return cv::Mat();
	}
	camCalib.initializeUndistortionMaps(cam.getResolution());

	return camCalib.getFCinFB().clone();
}

bool SmartCamera::iProbeDetection() {
	using namespace std::chrono;
	auto t0 = high_resolution_clock::now();

    if (!getIprobeMode()) {
        if (camLedOn) {
			camLedOn = false;
			firmware_->setLedAlwaysOn(false);
		}
        setCentroids(std::vector<cv::Point2f>());
        // std::cout << "Not in iProbe mode." << std::endl;
        return false;
    }
	// std::cout << "In iProbe mode." << std::endl;

    if (!getIprobeLocked()) {
        if (camLedOn) {
			camLedOn = false;
			firmware_->setLedAlwaysOn(false);
		}
        setCentroids(std::vector<cv::Point2f>());
        // std::cout << "Not locked on iProbe." << std::endl;
        return false;
    }
    
    if (!this->camParamLoaded) {
        if(!camCalib.loadCalibrationFromFile()) {
            setCentroids(std::vector<cv::Point2f>());
            std::cerr << "Failed to load camera calibration file." << std::endl;
            return false;
        }
        camCalib.initializeUndistortionMaps(cam.getResolution());

        if (!moveCalc.loadParallaxCalibrationFile()) {
            setCentroids(std::vector<cv::Point2f>());
            std::cerr << "Failed to load parallax calibration file." << std::endl;
            return false;
        }

        if (iprobeCalib.readIprobeCalibrationFile() != 0) {
            setCentroids(std::vector<cv::Point2f>());
            std::cerr << "Failed to load iProbe calibration file." << std::endl;
            return false;
        }

        this->camParamLoaded = true;
    }
	
	this->iprobeDist = firmware_->currentDistance();

	if (!getSmrCoord(this->iprobeDist / 1000)) {
		setCentroids(std::vector<cv::Point2f>());
		return false;
	}

    if (!getIprobeParams(this->iprobeDist)) {
		setCentroids(std::vector<cv::Point2f>());
		return false;
	}

    cv::Mat y, u, v, rgba;
	unsigned long imgTime;
    if (!cam.nextProcessImage(y, true, u, true, v, true, rgba, false, true, imgTime)) {
		std::cerr << "Grab image failed." << std::endl;
        return false;
	}
	
	if (y.empty() || u.empty() || v.empty()) {
        std::cerr << "Invalid image grabbed" << std::endl;
        return false;
    }

	// auto t1 = high_resolution_clock::now();
	// std::cout << "[TIMER] Check state, Load files, Get SMR and Grab images: " << duration_cast<milliseconds>(t1 - t0).count() << " ms\n";

	if (!prepareIprobeCUDAInput(y, u, v)) {
        if (camLedOn) {
			camLedOn = false;
			firmware_->setLedAlwaysOn(false);
		}
        setCentroids(std::vector<cv::Point2f>());
		return false;
    }

    if (!runIprobeCUDAStages()) {
        setCentroids(std::vector<cv::Point2f>());
		return false;
    }

	// auto t2 = high_resolution_clock::now();

    if (!updateIprobeCentroids()) {
		// drawCentroids(this->processed, smrCoord);
        setCentroids(std::vector<cv::Point2f>());
		return false;
    }

	auto t3 = high_resolution_clock::now();
	// std::cout << "[TIMER] Detect iProbe: " << duration_cast<milliseconds>(t3 - t2).count() << " ms\n";

    std::cout << "Detected centroid size: " << centroids.size() << std::endl;

	std::cout << "[TIMER] Total iProbe Processing time: " << duration_cast<milliseconds>(t3 - t0).count() << " ms\n\n";

	// Optional: estimate pose from detected centroids using PnP with 17 iProbe object points
	// if (centroids.size() >= 17) {
	// 	std::vector<cv::Point3f> objectPoints;
	// 	objectPoints.emplace_back(-0.2551f, 54.145f, 87.206f);
	// 	objectPoints.emplace_back(0.402f, 54.146f, -87.0352f);
	// 	objectPoints.emplace_back(-87.21f, 54.3134f, 0.0942200149f);
	// 	objectPoints.emplace_back(87.833f, 54.205f, -0.5047f);
	// 	objectPoints.emplace_back(0.3109f, 42.979f, 62.505f);
	// 	objectPoints.emplace_back(-0.3822f, 26.8662f, 43.228f);
	// 	objectPoints.emplace_back(-0.404940651f, 11.206f, 24.441f);
	// 	objectPoints.emplace_back(0.74f, 42.5786f, -62.505f);
	// 	objectPoints.emplace_back(-0.3046f, 26.943f, -43.563f);
	// 	objectPoints.emplace_back(0.2606f, 10.989f, -24.306f);
	// 	objectPoints.emplace_back(-62.615f, 43.2588f, -0.6616f);
	// 	objectPoints.emplace_back(-43.194f, 26.989f, -0.367f);
	// 	objectPoints.emplace_back(-24.489f, 11.437f, -0.4907f);
	// 	objectPoints.emplace_back(62.867f, 42.978f, -0.114292058f);
	// 	objectPoints.emplace_back(43.645f, 26.9465f, 0.4722f);
	// 	objectPoints.emplace_back(24.317f, 10.84f, 0.254f);
	// 	objectPoints.emplace_back(-0.3887f, -0.7839f, 0.3647382232f);

	// 	std::vector<cv::Point2f> imagePoints;
	// 	for (int i = 0; i < 17; ++i) imagePoints.push_back(centroids[i]);

	// 	cv::Mat K = getCameraIntrinsicMat();
	// 	if (!K.empty()) {
	// 		cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
	// 		cv::Mat rvec, tvec;
	// 		bool pnp_ok = cv::solvePnP(objectPoints, imagePoints, K, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
	// 		if (pnp_ok) {
	// 			std::cout << "[PnP] rvec = [" << rvec.at<double>(0,0) << ", "
	// 					  << rvec.at<double>(1,0) << ", "
	// 					  << rvec.at<double>(2,0) << "], "
	// 					  << "tvec = [" << tvec.at<double>(0,0) << ", "
	// 					  << tvec.at<double>(1,0) << ", "
	// 					  << tvec.at<double>(2,0) << "]" << std::endl;
	// 		} else {
	// 			std::cout << "[PnP] solvePnP failed." << std::endl;
	// 		}
	// 	} else {
	// 		std::cout << "[PnP] Camera intrinsic matrix unavailable." << std::endl;
	// 	}
	// }

    return true;
}

template <typename T>
T lerp(T a, T b, T t) {
    return a + t * (b - a);
}

bool SmartCamera::getSmrCoord(double distance) {
    auto& parallaxCalibPoints = moveCalc.getParallaxCalibPoints();
	
    if (parallaxCalibPoints.empty()) {
		std::cerr << "Failed to get parallax calibration points." << std::endl;
		this->smrCoord = cv::Point2f(-1.0, -1.0);
		return false;
    }

    std::vector<double> distances;
    std::vector<double> azimuths;
    std::vector<double> elevations;

    for (const auto& point : parallaxCalibPoints) {
        distances.push_back(point.distance);
        azimuths.push_back(point.az);
        elevations.push_back(point.el);
    }

    cv::Size imgSize(3264, 2464);
    cv::Point2f imageCenter(imgSize.width / 2.0f, imgSize.height / 2.0f);

	double az = 0.0;
    double el = 0.0;
    bool found = false;
	
    for (size_t i = 1; i < distances.size(); ++i) {
        if (distance >= distances[i - 1] && distance < distances[i]) {
            double t = (distance - distances[i - 1]) / (distances[i] - distances[i - 1]);
            az = lerp(azimuths[i - 1], azimuths[i], t);
            el = lerp(elevations[i - 1], elevations[i], t);
            found = true;
            break;
        }
    }

	if (!found) {
        if (distance < distances.front()) {
            az = azimuths.front();
            el = elevations.front();
        } else if (distance >= distances.back()) {
            az = azimuths.back();
            el = elevations.back();
        }
    }

    this->smrCoord = cv::Point2f(static_cast<float>(az + imageCenter.x), static_cast<float>(el + imageCenter.y));

    return true;
}

bool SmartCamera::getIprobeParams(double distance) {
    const auto& calibData = iprobeCalib.getIprobeCalibData();

    if (calibData.empty()) {
		std::cerr << "Failed to get iProbe calibration parameters." << std::endl;
        iprobeParams.distance = 0.0;
		iprobeParams.ROI_range = 0.0;
		iprobeParams.LED_range = 0.0;
		iprobeParams.Centroid_area = 0.0;
		return false;
    }
	
    if (distance < calibData.front().distance) {
        double dist1 = calibData[0].distance;
        double dist2 = calibData[1].distance;
        double ratio = (distance - dist1) / (dist2 - dist1);

        IprobeCalibData extrapolatedData;
        extrapolatedData.distance = distance;
        extrapolatedData.ROI_range = calibData[0].ROI_range + ratio * (calibData[1].ROI_range - calibData[0].ROI_range);
        extrapolatedData.LED_range = calibData[0].LED_range + ratio * (calibData[1].LED_range - calibData[0].LED_range);
        extrapolatedData.Centroid_area = calibData[0].Centroid_area + ratio * (calibData[1].Centroid_area - calibData[0].Centroid_area);
		
		iprobeParams = extrapolatedData;
        return true;
    }
	
    for (size_t i = 1; i < calibData.size(); ++i) {
        if (distance <= calibData[i].distance) {
            double dist1 = calibData[i - 1].distance;
            double dist2 = calibData[i].distance;
            double ratio = (distance - dist1) / (dist2 - dist1);

            IprobeCalibData interpolatedData;
            interpolatedData.distance = distance;
            interpolatedData.ROI_range = calibData[i - 1].ROI_range + ratio * (calibData[i].ROI_range - calibData[i - 1].ROI_range);
            interpolatedData.LED_range = calibData[i - 1].LED_range + ratio * (calibData[i].LED_range - calibData[i - 1].LED_range);
            interpolatedData.Centroid_area = calibData[i - 1].Centroid_area + ratio * (calibData[i].Centroid_area - calibData[i - 1].Centroid_area);

			iprobeParams = interpolatedData;
            return true;
        }
    }
	
    double dist1 = calibData[calibData.size() - 2].distance;
    double dist2 = calibData.back().distance;
    double ratio = (distance - dist2) / (dist2 - dist1);

    IprobeCalibData extrapolatedData;
    extrapolatedData.distance = distance;
    extrapolatedData.ROI_range = calibData.back().ROI_range + ratio * (calibData.back().ROI_range - calibData[calibData.size() - 2].ROI_range);
    extrapolatedData.LED_range = calibData.back().LED_range + ratio * (calibData.back().LED_range - calibData[calibData.size() - 2].LED_range);
    extrapolatedData.Centroid_area = calibData.back().Centroid_area + ratio * (calibData.back().Centroid_area - calibData[calibData.size() - 2].Centroid_area);

	iprobeParams = extrapolatedData;
    return true;
}

bool SmartCamera::prepareIprobeCUDAInput(const cv::Mat& y, const cv::Mat& u, const cv::Mat& v) {
	// using namespace std::chrono;
	// auto t0 = high_resolution_clock::now();

    int width = y.cols, height = y.rows;

    if (d_y == nullptr || bgr_width != width || bgr_height != height) {
        initCudaMemory(width, height);
        initPinnedHostMemory(width, height, y.step, u.step);
        bgr_width = width;
        bgr_height = height;
    }

    memcpy(pinnedY, y.data, y.rows * y.step);
    memcpy(pinnedU, u.data, u.rows * u.step);
    memcpy(pinnedV, v.data, v.rows * v.step);

    cudaMemcpy2DAsync(d_y, pitch_y, pinnedY, y.step, y.cols, y.rows, cudaMemcpyHostToDevice, stream);
    cudaMemcpy2DAsync(d_u, pitch_u, pinnedU, u.step, u.cols, u.rows, cudaMemcpyHostToDevice, stream);
    cudaMemcpy2DAsync(d_v, pitch_v, pinnedV, v.step, v.cols, v.rows, cudaMemcpyHostToDevice, stream);

    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        std::cerr << "[CUDA ERROR] Memcpy async failed: " << cudaGetErrorString(err) << std::endl;
        return false;
    }

	// auto t1 = high_resolution_clock::now();
	// std::cout << "[TIMER] Memory copy: " << duration_cast<milliseconds>(t1 - t0).count() << " ms\n";

    launchYUV2Green_CUDA(d_y, d_u, d_v, d_green, width, height, pitch_u, pitch_y, pitch_green, stream);
    cudaStreamSynchronize(stream);

	// auto t2 = high_resolution_clock::now();
	// std::cout << "[TIMER] YUV to green channel: " << duration_cast<milliseconds>(t2 - t1).count() << " ms\n";

    return true;
}

bool SmartCamera::runIprobeCUDAStages() {
	// using namespace std::chrono;
	// auto t0 = high_resolution_clock::now();
	
    if (!updateTriggerROI(d_green_gpu.size(), iprobeParams.LED_range)) {
		if (camLedOn) {
			camLedOn = false;
			firmware_->setLedAlwaysOn(false);
		}
        std::cerr << "Failed to update trigger LED ROI.\n";
        return false;
    }

	// // download full bgr image with roi for debug
	// {
    //     cv::Mat debug_full_bgr;
    //     d_green_gpu.download(debug_full_bgr);
    //     cv::rectangle(debug_full_bgr, triggerROI, cv::Scalar(0, 0, 255), 2);
    //     cv::Point trigger_center(triggerROI.x + triggerROI.width / 2,
    //                              triggerROI.y + triggerROI.height / 2);
    //     cv::drawMarker(debug_full_bgr, trigger_center, cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 20, 2);
    //     cv::Point smr_pt(static_cast<int>(smrCoord.x), static_cast<int>(smrCoord.y));
    //     cv::drawMarker(debug_full_bgr, smr_pt, cv::Scalar(255, 0, 0), cv::MARKER_STAR, 20, 2);
    //     cv::imwrite("debug_bgr_with_roi_and_smr.png", debug_full_bgr);
    // }

	// // download roi for debug
	// cv::Mat debug_bgr_trigger;
	// {
	// 	cv::cuda::GpuMat d_trigger_roi = d_green_gpu(triggerROI);
	// 	d_trigger_roi.download(debug_bgr_trigger);
	// 	cv::imwrite("debug_trigger_roi.png", debug_bgr_trigger);
	// 	std::cout << "[DEBUG] triggerROI = " << triggerROI << ", "
	// 			<< "cols = " << debug_bgr_trigger.cols
	// 			<< ", rows = " << debug_bgr_trigger.rows 
	// 			<< "smr coordinate: " << smrCoord
	// 			<< "iprobe distance: " << this->iprobeDist << std::endl;
	// }
	// return false;

    if (!detectTriggerLED(d_green_gpu)) {
		if (camLedOn) {
			camLedOn = false;
			firmware_->setLedAlwaysOn(false);
		}
        // std::cout << "Failed to detect Trigger LED." << std::endl;
        return false;
    }
	if (!camLedOn) {
		camLedOn = true;
		firmware_->setLedAlwaysOn(true);
	}

	// auto t1 = high_resolution_clock::now();
	// std::cout << "[TIMER] Trigger LED ROI and detection: " << duration_cast<milliseconds>(t1 - t0).count() << " ms\n";

    if (!updateIprobeROI(d_green_gpu.size(), iprobeParams.ROI_range)) {
        std::cerr << "Failed to update detection ROI.\n";
        return false;
    }

    cv::cuda::GpuMat d_bgr_roi = d_green_gpu(detectionROI);
    if (!preprocessDetectionImage(d_bgr_roi, d_processed_roi)) {
        std::cerr << "Preprocessing on ROI failed.\n";
        return false;
    }
    d_processed_roi.download(this->processed);

	// auto t2 = high_resolution_clock::now();
	// std::cout << "[TIMER] iProbe ROI and Preprocess image: " << duration_cast<milliseconds>(t2 - t1).count() << " ms\n";

    return true;
}

void SmartCamera::cleanupCudaMemory() {
    if (d_y) { cudaFree(d_y); d_y = nullptr; }
    if (d_u) { cudaFree(d_u); d_u = nullptr; }
    if (d_v) { cudaFree(d_v); d_v = nullptr; }
	if (d_green) { cudaFree(d_green); d_green = nullptr; }

	if (pinnedY) { cudaFreeHost(pinnedY); pinnedY = nullptr; }
	if (pinnedU) { cudaFreeHost(pinnedU); pinnedU = nullptr; }
	if (pinnedV) { cudaFreeHost(pinnedV); pinnedV = nullptr; }

    if (stream != nullptr) {
        cudaStreamDestroy(stream);
        stream = nullptr;
    }
}

void SmartCamera::initCudaMemory(int width, int height) {
    cleanupCudaMemory();

    size_t y_size = width * height;
    size_t uv_size = (width / 2) * (height / 2);
    size_t bgr_size = width * height * sizeof(uchar3);
	
    cudaMallocPitch(&d_y, &pitch_y, width, height);
    cudaMallocPitch(&d_u, &pitch_u, width / 2, height / 2);
    cudaMallocPitch(&d_v, &pitch_v, width / 2, height / 2);
	
	cudaMallocPitch(&d_green, &pitch_green, width * sizeof(uchar), height);

	d_green_gpu = cv::cuda::GpuMat(height, width, CV_8UC1, d_green, pitch_green);

    cudaStreamCreate(&stream);

    bgr_width = width;
    bgr_height = height;
}

void SmartCamera::initPinnedHostMemory(int width, int height, size_t y_step, size_t uv_step) {
    size_t y_bytes = height * y_step;
    size_t uv_bytes = (height / 2) * uv_step;

    cudaHostAlloc(&pinnedY, y_bytes, cudaHostAllocDefault);
    cudaHostAlloc(&pinnedU, uv_bytes, cudaHostAllocDefault);
    cudaHostAlloc(&pinnedV, uv_bytes, cudaHostAllocDefault);
}

bool SmartCamera::updateTriggerROI(const cv::Size& imageSize,
                                   float roiRange) {
    double range = roiRange * 1;
	// std::cout << "Range: " << range << std::endl;
	// std::cout << "SMR Coord: " << smrCoord.x << ", " << smrCoord.y << std::endl;

    int x_start = std::max(static_cast<int>(smrCoord.x - range), 0);
    int y_start = std::max(static_cast<int>(smrCoord.y - range), 0);
    int x_end   = std::min(static_cast<int>(smrCoord.x + range), imageSize.width - 1);
    int y_end   = std::min(static_cast<int>(smrCoord.y + range), imageSize.height - 1);

    int width  = x_end - x_start + 1;
    int height = y_end - y_start + 1;

	// std::cout << "Trigger ROI: " << x_start << ", " << y_start << ", " << width << ", " << height << std::endl;

    if (width <= 0 || height <= 0) {
        this->triggerROI = cv::Rect();
        return false;
    }

    cv::Rect roi(x_start, y_start, width, height);
    this->triggerROI = roi;

	CV_Assert(roi.x >= 0 && roi.y >= 0 &&
          	  roi.x + roi.width  <= imageSize.width &&
          	  roi.y + roi.height <= imageSize.height);

    return true;
}

bool SmartCamera::detectTriggerLED(const cv::cuda::GpuMat& d_g) {
	this->d_green_trigger = d_g(triggerROI);
	
    openFilter->apply(d_green_trigger, d_eroded);
    cv::cuda::threshold(d_eroded, d_binary_trigger, 180, 255, cv::THRESH_BINARY);
	
    trigger_binary.create(d_binary_trigger.size(), d_binary_trigger.type());
	d_binary_trigger.download(this->trigger_binary);
	CV_Assert(!trigger_binary.empty());

	int w = trigger_binary.cols;
	int h = trigger_binary.rows;
	int box_size = w / 3;
	int cx = w / 2;
	int cy = h / 2;

	int x_start = std::max(0, cx - box_size / 2);
	int y_start = std::max(0, cy - box_size / 2);
	x_start = std::min(x_start, w - box_size);
	y_start = std::min(y_start, h - box_size);

	cv::Rect center_roi(x_start, y_start, box_size, box_size);
	trigger_binary(center_roi).setTo(0);
	
    float currentMinArea = iprobeParams.Centroid_area * 0.15f;
    float currentMaxArea = iprobeParams.Centroid_area * 1.5f;
	
    if (currentMinArea != prevMinArea || currentMaxArea != prevMaxArea) {
		blobParams.minArea = currentMinArea;
        blobParams.maxArea = currentMaxArea;
        blobDetector = cv::SimpleBlobDetector::create(blobParams);
        prevMinArea = currentMinArea;
        prevMaxArea = currentMaxArea;
    }
	
    blobDetector->detect(this->trigger_binary, keypoints_trigger);
	
	std::vector<cv::KeyPoint> filteredKeypoints;
	for (const auto& kp : keypoints_trigger) {
		cv::Point2f globalPt = kp.pt + cv::Point2f(static_cast<float>(triggerROI.x), static_cast<float>(triggerROI.y));
        if (cv::norm(globalPt - this->smrCoord) > iprobeParams.LED_range * 0.2) {
			filteredKeypoints.push_back(kp);
        }
    }
	
    if (filteredKeypoints.size() == 1) {
		// cv::imwrite("debug_trigger_binary_img_success.png", trigger_binary);
		this->triggerLed = filteredKeypoints[0].pt + cv::Point2f(static_cast<float>(triggerROI.x), static_cast<float>(triggerROI.y));
        return true;
    }
	// std::cout << "Incorrect blob size: " << filteredKeypoints.size() << std::endl;
	// cv::imwrite("debug_trigger_binary_img_fail.png", trigger_binary);

    return false;
}

bool SmartCamera::updateIprobeROI(const cv::Size& imageSize,
                                  float roiRange) {
    cv::Point2f direction = triggerLed - smrCoord;
	
    float norm = std::sqrt(direction.x * direction.x + direction.y * direction.y);
    if (norm < 1e-6f) {
        std::cerr << "Invalid direction vector: triggerLed == smrCoord.\n";
        return false;
    }
    direction.x /= norm;
    direction.y /= norm;
	
    cv::Point2f center = smrCoord + direction * roiRange / 2;

    int x_start = static_cast<int>(center.x - roiRange / 2);
    int y_start = static_cast<int>(center.y - roiRange / 2);
    int x_end   = static_cast<int>(center.x + roiRange / 2);
    int y_end   = static_cast<int>(center.y + roiRange / 2);

    x_start = std::max(x_start, 0);
    y_start = std::max(y_start, 0);
    x_end   = std::min(x_end, imageSize.width - 1);
    y_end   = std::min(y_end, imageSize.height - 1);

    int width  = x_end - x_start + 1;
    int height = y_end - y_start + 1;

    if (width <= 0 || height <= 0) {
        this->detectionROI = cv::Rect();
        return false;
    }

    cv::Rect roi(x_start, y_start, width, height);
    this->detectionROI = roi;

    CV_Assert(roi.x >= 0 && roi.y >= 0 &&
              roi.x + roi.width <= imageSize.width &&
              roi.y + roi.height <= imageSize.height);

    return true;
}

bool SmartCamera::preprocessDetectionImage(const cv::cuda::GpuMat& d_g, cv::cuda::GpuMat& d_out) {
	cv::cuda::GpuMat d_blur;
	cv::cuda::GpuMat d_edges;

    CV_Assert(gaussFilter);
	gaussFilter->apply(d_g, d_blur);

	CV_Assert(cannyDetector);
	cannyDetector->detect(d_blur, d_edges);
	// cannyDetector->detect(d_blur, d_out);
	
    CV_Assert(dilateFilter);
	dilateFilter->apply(d_edges, d_out);

	// {
	// 	std::cout << "gauss" << std::endl;
    //     cv::Mat blur_debug;
    //     d_blur.download(blur_debug);
    //     cv::imwrite("debug_step2_blur.png", blur_debug);
    // }

	// {
	// 	std::cout << "canny" << std::endl;
	// 	cv::Mat edge_debug;
	// 	d_out.download(edge_debug);
	// 	cv::imwrite("debug_step3_canny_edges.png", edge_debug);
	// }

	// {
	// 	std::cout << "morph" << std::endl;
    //     cv::Mat morph_debug;
    //     d_out.download(morph_debug);
    //     cv::imwrite("debug_step4_morph_close.png", morph_debug);
    // }

    return true;
}

bool SmartCamera::updateIprobeCentroids() {
	CV_Assert(this->processed.type() == CV_8UC1);
	
	std::vector<std::vector<cv::Point>> contours;
    cv::findContours(this->processed, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    contours.erase(std::remove_if(contours.begin(), contours.end(), [&](const std::vector<cv::Point>& contour) {
        cv::Moments M = cv::moments(contour);
        if (M.m00 != 0) {
            double cx = M.m10 / M.m00 + detectionROI.x;
            double cy = M.m01 / M.m00 + detectionROI.y;
            double distance = cv::norm(cv::Point2f(static_cast<float>(cx), static_cast<float>(cy)) - this->triggerLed);
            return distance < 10.0;
        }
        return false;
    }), contours.end());

    ContourFilterParams defaultParams;
    std::vector<std::vector<cv::Point>> filteredContours;
	if (!filterContours(contours, filteredContours, defaultParams)) {
		return false;
	}

    std::vector<cv::Point2f> keypoints;
    for (const auto& contour : filteredContours) {
        cv::Moments M = cv::moments(contour);
        if (M.m00 != 0) {
            double cx = M.m10 / M.m00 + detectionROI.x;
            double cy = M.m01 / M.m00 + detectionROI.y;
            keypoints.push_back(cv::Point2f(static_cast<float>(cx), static_cast<float>(cy)));
        }
    }

    if (keypoints.size() != 17) {
        std::cerr << "Expected 17 centroids, but found: " << keypoints.size() << std::endl;
        return false;
    }

    std::vector<cv::Point2f> centroids_sorted;
	std::vector<bool> used(keypoints.size(), false);

	auto calculate_distance = [](const cv::Point2f& p1, const cv::Point2f& p2) {
		return cv::norm(p1 - p2);
	};
	
	auto p1_it = std::max_element(keypoints.begin(), keypoints.end(),
		[&](const auto& a, const auto& b) {
			return cv::norm(a - smrCoord) < cv::norm(b - smrCoord);
		});
	size_t p1_index = std::distance(keypoints.begin(), p1_it);
	const cv::Point2f& p1 = keypoints[p1_index];
	centroids_sorted.push_back(p1);
	used[p1_index] = true;
	
	float min_dist = std::numeric_limits<float>::max();
	size_t p2_index = 0;
	for (size_t i = 0; i < keypoints.size(); ++i) {
		if (used[i]) continue;
		float d = cv::norm(keypoints[i] - smrCoord);
		if (d < min_dist) {
			min_dist = d;
			p2_index = i;
		}
	}
	const cv::Point2f& p2 = keypoints[p2_index];
	centroids_sorted.push_back(p2);
	used[p2_index] = true;

	cv::Point2f v = p1 - smrCoord;
	float v_norm = std::sqrt(v.dot(v));

	std::vector<cv::Point2f> ref_pts = {p1, p2};
	float max_left_dist = -1.0f, max_right_dist = -1.0f;
	int left_idx = -1, right_idx = -1;

	for (size_t i = 0; i < keypoints.size(); ++i) {
		if (used[i]) continue;
		const auto& pt = keypoints[i];
		float cross = (pt.x - smrCoord.x) * v.y - (pt.y - smrCoord.y) * v.x;
		float perp_dist = std::abs(cross) / v_norm;

		if (cross > 0 && perp_dist > max_left_dist) {
			max_left_dist = perp_dist;
			left_idx = static_cast<int>(i);
		} else if (cross < 0 && perp_dist > max_right_dist) {
			max_right_dist = perp_dist;
			right_idx = static_cast<int>(i);
		}
	}

	if (left_idx != -1) {
		const auto& p3 = keypoints[left_idx];
		centroids_sorted.push_back(p3);
		ref_pts.push_back(p3);
		used[left_idx] = true;
	}
	if (right_idx != -1) {
		const auto& p4 = keypoints[right_idx];
		centroids_sorted.push_back(p4);
		ref_pts.push_back(p4);
		used[right_idx] = true;
	}

	for (const auto& ref_pt : ref_pts) {
		std::vector<std::pair<float, size_t>> dist_idx;
		for (size_t i = 0; i < keypoints.size(); ++i) {
			if (used[i]) continue;
			float d = calculate_distance(ref_pt, keypoints[i]);
			dist_idx.emplace_back(d, i);
		}
		std::sort(dist_idx.begin(), dist_idx.end());

		for (size_t j = 0; j < std::min<size_t>(3, dist_idx.size()); ++j) {
			size_t idx = dist_idx[j].second;
			centroids_sorted.push_back(keypoints[idx]);
			used[idx] = true;
		}
	}
	
	for (size_t i = 0; i < keypoints.size(); ++i) {
		if (!used[i]) {
			centroids_sorted.push_back(keypoints[i]);
			break;
		}
	}

	auto centroids_undistorted = camCalib.undistortPoints(centroids_sorted, this->imageSize);

	this->centroids = centroids_undistorted;
	return true;
}

bool SmartCamera::filterContours(const std::vector<std::vector<cv::Point>>& inputContours,
                                 std::vector<std::vector<cv::Point>>& outputContours,
                                 const ContourFilterParams& params) {
    std::vector<std::vector<cv::Point>> filtered;
    std::vector<double> areas;
    std::vector<double> circularities;

    for (const auto& contour : inputContours) {
        double area = cv::contourArea(contour);
        if (area < iprobeParams.Centroid_area * 0.2 || area > iprobeParams.Centroid_area * 1.2) continue;

        double perimeter = cv::arcLength(contour, true);
        if (perimeter == 0) continue;

        double circularity = (4 * CV_PI * area) / (perimeter * perimeter);
        if (circularity < params.min_circularity || circularity > params.max_circularity) continue;

        std::vector<cv::Point> hull;
        cv::convexHull(contour, hull);
        double hull_area = cv::contourArea(hull);
        if (hull_area == 0) continue;

        double convexity = area / hull_area;
        if (convexity < params.min_convexity || convexity > params.max_convexity) continue;

        cv::RotatedRect minRect = cv::minAreaRect(contour);
        float width = minRect.size.width;
        float height = minRect.size.height;
        double inertia_ratio = std::min(width, height) / std::max(width, height);
        if (inertia_ratio < params.min_inertia_ratio) continue;

        filtered.push_back(contour);
        areas.push_back(area);
        circularities.push_back(circularity);
    }

	if (filtered.size() > 17) {
		std::vector<double> areas_copy = areas;
		const size_t M = areas_copy.size();
		if (M > 0) {
			double median_area = 0.0;
			if (M % 2 == 1) {
				std::nth_element(areas_copy.begin(), areas_copy.begin() + M/2, areas_copy.end());
				median_area = areas_copy[M/2];
			} else {
				std::nth_element(areas_copy.begin(), areas_copy.begin() + M/2 - 1, areas_copy.end());
				double lo = areas_copy[M/2 - 1];
				std::nth_element(areas_copy.begin(), areas_copy.begin() + M/2, areas_copy.end());
				double hi = areas_copy[M/2];
				median_area = 0.5 * (lo + hi);
			}
	
			std::vector<std::vector<cv::Point>> temp_filtered;
			temp_filtered.reserve(filtered.size());
			// std::vector<double> temp_circularities; temp_circularities.reserve(circularities.size());
	
			for (size_t i = 0; i < areas.size(); ++i) {
				if (areas[i] >= 0.3 * median_area && areas[i] <= 2.0 * median_area) {
					temp_filtered.push_back(filtered[i]);
					// temp_circularities.push_back(circularities[i]);
				}
			}
			filtered = std::move(temp_filtered);
			// circularities.swap(temp_circularities);
		}
	}

	if (filtered.size() > 17) {
        const size_t N = filtered.size();
		
        std::vector<cv::Point2f> centers;
        centers.reserve(N);
        for (const auto& c : filtered) {
            cv::Moments m = cv::moments(c);
            if (m.m00 != 0.0) {
                centers.emplace_back(static_cast<float>(m.m10 / m.m00),
                                     static_cast<float>(m.m01 / m.m00));
            } else {
                cv::RotatedRect rr = cv::minAreaRect(c);
                centers.emplace_back(rr.center);
            }
        }
		
        std::vector<double> nearest_dist(N, std::numeric_limits<double>::infinity());
        for (size_t i = 0; i < N; ++i) {
            double best = std::numeric_limits<double>::infinity();
            for (size_t j = 0; j < N; ++j) {
                if (i == j) continue;
                double dx = centers[i].x - centers[j].x;
                double dy = centers[i].y - centers[j].y;
                double dist = std::sqrt(dx * dx + dy * dy);
                if (dist < best) best = dist;
            }
            nearest_dist[i] = best;
        }
		
        std::vector<size_t> idx(N);
        std::iota(idx.begin(), idx.end(), 0);
        std::stable_sort(idx.begin(), idx.end(),
                         [&](size_t a, size_t b) {
                             return nearest_dist[a] > nearest_dist[b];
                         });
						 
        const size_t remove_count = N - 17;
		
        std::vector<char> keep(N, 1);
        for (size_t k = 0; k < remove_count; ++k) {
            keep[idx[k]] = 0;
        }
		
        std::vector<std::vector<cv::Point>> compact;
        compact.reserve(17);
        for (size_t i = 0; i < N; ++i) {
            if (keep[i]) compact.push_back(std::move(filtered[i]));
        }
        filtered.swap(compact);
    }

    // if (filtered.size() > 17) {
    //     std::vector<size_t> indices(filtered.size());
    //     std::iota(indices.begin(), indices.end(), 0);
    //     std::sort(indices.begin(), indices.end(), [&circularities](size_t i1, size_t i2) {
    //         return circularities[i1] > circularities[i2];
    //     });

    //     std::vector<std::vector<cv::Point>> final_filtered;
    //     for (size_t i = 0; i < 17; ++i) {
    //         final_filtered.push_back(filtered[indices[i]]);
    //     }
    //     filtered = std::move(final_filtered);
    // }

    outputContours = std::move(filtered);
    return !outputContours.empty();
}

void SmartCamera::drawCentroids(const cv::Mat& image, cv::Point2f smrCoord) {
    cv::Mat img_with_keypoints = image.clone();

    for (size_t i = 0; i < centroids.size(); ++i) {
        cv::Point2f pt = centroids[i];
        std::string text = std::to_string(i + 1);
		pt = pt - cv::Point2f(static_cast<float>(detectionROI.x), static_cast<float>(detectionROI.y));
        cv::circle(img_with_keypoints, pt, 5, cv::Scalar(0, 0, 255), -1);
        cv::putText(img_with_keypoints, text, pt + cv::Point2f(5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }
	cv::circle(image, smrCoord, 5, cv::Scalar(0, 0, 255), -1);

    std::cout << "Image with keypoints dumped." << std::endl;
    cv::imwrite("img_with_keypoints.png", img_with_keypoints);
}


bool SmartCamera::addTeachTarget(float imgX, float imgY) {
		cv::Point2f imgAzEl = camCalib.singleAzElFromXY(imgX, imgY, cam.getResolution());

		SMRData sd;
		if (tracker.smrNear(imgAzEl.x, imgAzEl.y, sd)) {
			targeter.addTarget(sd.getAz(), sd.getEl(), -1);
			return true;
		}
		return false;
}

bool SmartCamera::saveTeachFile() {
	return targeter.saveTargetsToFile();
}

bool SmartCamera::loadTeachFile() {
	return targeter.loadTargetsFromFile();
}





bool SmartCamera::watchForShakingSMR() {
	SMRData sd;
	if (tracker.smrShaking(sd)) {
		setSingleSMRTarget(sd.getAz(), sd.getEl(), -1);
		return true;
	}
	return false;
}





Movement SmartCamera::findManualMovement(bool setTarget, float imgX, float imgY) {

	if (setTarget) {
		cv::Size camRes = cam.getResolution();
		imgX *= camRes.width;
		imgY *= camRes.height;
		cv::Point2f imgAzEl = camCalib.singleAzElFromXY(imgX, imgY, camRes);

		SMRData sd;
		if (tracker.smrNear(imgAzEl.x, imgAzEl.y, sd)) {
			targeter.clearTargets();
			//std::cout << "Found SMR near click, setting target." << std::endl;
			setSingleSMRTarget(sd.getAz(), sd.getEl(), -1);
			return findTrackingMovement();
		}
		else {
			//std::cout << "Did not find SMR near click, moving to position." << std::endl;
			targeter.clearTargets();
			cv::Point2f offset = moveCalc.getLaserPoint(10.0f);
			Movement m;
			m.type = MoveBy;
			m.az = RadianToDegree(-(imgAzEl.x-offset.x));
			m.el = RadianToDegree(imgAzEl.y - offset.y);
			m.radius = 0;
			return m;
		}
	}
	else {
		if (targeter.numTargets() == 1) {
			// // if(searcher.lockedOnSMR)
			// // 	sd.locked = true;
			// searcher.updateSearch(sd.locked, sd.found, sd.laserHit, laserPoint.x, laserPoint.y, sd.getImgAz(), sd.getImgEl(), distance);	
			// if (searcher.jogRequired()) {
			// 	//TODO: figure out more legit way to calculate jog angles
			// 	float test_distance = searcher.getLaserObservedDistance();
			// 	std::cout<< "test_distance: " << test_distance << std::endl;
			// 	float jog_distance = 0.5f / test_distance;
			// 	tracker.clearTracking();
			// 	Movement temp;
			// 	temp.type = MoveBy;
			// 	temp.az = jog_distance;
			// 	temp.el = jog_distance;
			// 	temp.radius = 0.2;
			// 	std::cout << "Searching in Progress" << std::endl;
			// 	return temp;
			// }
			
			// else if(sd.found && !sd.locked) {
			// 	std::cout << "Get Movement" << std::endl;
			// 	return moveCalc.getMovement(trkAngles.x, trkAngles.y, sd.getImgAz(), sd.getImgEl(), searcher.getCurrentSearchDistance(), sd.laserHit);
			// }
			return findTrackingMovement();
		}
		else {
			Movement m;
			m.type = NoMove;
			m.az = 0;
			m.el = 0;
			m.radius = 0;
			return m;
		}
	}
}



//static int circle_counter = 0;

Movement SmartCamera::findTrackingMovement() {
    
	//initialize all the parameters required to start moving
    Movement result;
	cv::Point2f trkAngles = tracker.getLastTrkAngles();
	TargetData target = targeter.findTargetData(tracker.getCurrentTrackedSMRs());
	float last_dist_command = ((firmware_->currentDistance() / 1000.0) > 0.0 )? firmware_->currentDistance() / 1000.0 : 1.0;
	//std::cout << "Last good dist: " << last_dist_command << std::endl;
	// std::cout << "current search dist: " << searcher.getCurrentSearchDistance() << std::endl;
	spiral_dist = searcher.getLaserObservedDistance();
	cv::Point2f laserPoint = moveCalc.getLaserPoint(searcher.getCurrentSearchDistance());
	//diff required to calculate the spiral az el
	float diffAz = laserPoint.x -  target.imgAz;
	float diffEl = laserPoint.y -  target.imgEl;

	//update search every iteration 
    searcher.updateSearch(target.trkLocked, target.found, target.hitByLaser, laserPoint.x, laserPoint.y, target.imgAz, target.imgEl, last_dist_command);
	auto jog_comm = moveCalc.getMovement(trkAngles.x, trkAngles.y, target.imgAz, target.imgEl, last_dist_command, target.hitByLaser, back_cam, camCalib.pixelAngle(cv::Size(3264, 2464)));

	//std::cout << "targetNew: " << target.isNew << " target found:" << target.found <<std::endl;
	//std::cout << "current State: " << currentState << std::endl;
	//state manager for manging tracking movement
    while (true) {

        switch (currentState) {
            case INIT: {
                //std::cout << "INIT state" << std::endl;
				if (target.found)
					currentState = REGULAR_JOG;
				else if (target.hitByLaser || tracker.laserHitSMR)
					currentState = SPIRAL_MOVEMENT;
                else {
                    currentState = SEARCH_IN_PROGRESS;
                }
                break;
            }


			case FIRSTJOG: {
				//std::cout << "FIRSTJOG state; Last good dist: " << last_dist_command <<std::endl;
				currentState = REGULAR_JOG;
				return jog_comm;
			}


            case SEARCH_IN_PROGRESS:
			//std::cout << "SEARCH_IN_PROGRESS state" << std::endl;
                if (firmware_->is_Spiral()) {
                    result = { NoMove, 0, 0, 0 };
                    return result;
                } else if (target.hitByLaser || tracker.laserHitSMR) {
                    no_mv_cnt_ang = 0;
                    no_mv_cnt_img = 0;
                    Spiral_counter = 0;
                    currentState = SPIRAL_MOVEMENT;
                } else if (target.found && !target.hitByLaser && !tracker.laserHitSMR) {
                    currentState = REGULAR_JOG;
                } else {
                    currentState = TARGET_NOT_FOUND;
                }
                break;
                

            case SPIRAL_MOVEMENT:
			// std::cout << "SPIRAL_MOVEMENT state" << std::endl;
			
			if ((!target.hitByLaser && (fabs(jog_comm.az - firmware_->currentAzimuth()) >= spiral_thresh) || (fabs(jog_comm.el - firmware_->currentElevation()) >= spiral_thresh) || Spiral_counter >= spiral_timeout) && firmware_->is_Spiral()){
					//std::cout <<"Stopping spiral search due to timeout or distance threshold, Timeout:" << Spiral_counter << " Distance:(" << (fabs(jog_comm.az - firmware_->currentAzimuth()) >= spiral_thresh) << "," << (fabs(jog_comm.el - firmware_->currentElevation()) >= spiral_thresh) << ")" << std::endl;
					Spiral_counter = 0;
					tracker.laserHitSMR = false;
					firmware_->StopSearch();
					currentState = INIT;
				}
			else {
				std::cout << "Checking Spiral Status: Spiral Cntr:" << Spiral_counter << " Trgt hit by Lsr:" << target.hitByLaser <<" Az dist comp to thresh: " << fabs(jog_comm.az - firmware_->currentAzimuth()) << " EL dist comp to Thresh: " << fabs(jog_comm.el - firmware_->currentElevation()) << std::endl;
                result = { Spiral, RadianToDegree(trkAngles.x + diffAz), RadianToDegree(trkAngles.y - diffEl), 0.2 };
			}
			return result;
                

            case REGULAR_JOG:
			std::cout << "REGULAR_JOG state" << std::endl;
            //std::cout << "Laser hit? : " << target.hitByLaser << std::endl;
			//std::cout << "Tracker Laser hit? : " << tracker.laserHitSMR << std::endl;
				if(target.hitByLaser || tracker.laserHitSMR) {
					currentState = SPIRAL_MOVEMENT;
				}
				
				else if (target.found) {
					result = moveCalc.getMovement(trkAngles.x, trkAngles.y, target.imgAz, target.imgEl, searcher.getCurrentSearchDistance(), target.hitByLaser, back_cam, camCalib.pixelAngle(cv::Size(3264, 2464)));
					//std::cout << "Regular Jog MovTo StepWise" << std::endl;
					result.type = MoveToStep;
					return result;
				}


            case TARGET_NOT_FOUND:
			//std::cout << "TARGET_NOT_FOUND state" << std::endl;
				
				if (target.isNew && target.found)
						currentState = FIRSTJOG;

				else if (target.found && !target.hitByLaser) {
                    currentState = REGULAR_JOG;
                }

                no_mv_cnt_img++;
                if (no_mv_cnt_img == 5) {
                    result = { NoMove, 0, 0.05, 0 };
                    return result;
                }
				else if (no_mv_cnt_img >= 10){
					currentState = INIT;
					no_mv_cnt_img = 0;
				}
				else
                	result = { NoMove, 0, 0, 0 };
				
                return result;
                
        }
    }
}

void SmartCamera::reset_tracking_state(){
	currentState = INIT;
}
//inform targeter that tracker has locked onto SMR (so that it can move to the next target and record the current target's last distance)
void SmartCamera::reportTrackerLock(bool locked, float az, float el, bool updateCalibration, float smrDistance, int stableDuration) {
	//smrAnglesFromLock(az, el, smrDistance);
	float az_tmp = az;
	float el_tmp = el;
	smrAnglesFromLock(az_tmp, el_tmp, smrDistance);
	targeter.reportLock(locked, az_tmp, el_tmp, smrDistance, DegreeToRadian(az), DegreeToRadian(el));
	if(locked){
		reset_tracking_state();
		tracker.laserHitSMR = false; //reset the laser hit SMR flag
	}
	searcher.reportLock();
	
	if (updateCalibration) { // if the user wants  to enable auto calibration, one can do so from the cam_calibration.json file
		//std::cout << "Stable Duration " << stableDuration << std::endl;
		if (locked && stableDuration > 1)  { //number of cycles
			cv::Point3f laser = getLaserCoordinates();
			if (laser.z >= 0) {
				if (moveCalc.updateParallaxCalibrationPoint(smrDistance, laser.x, laser.y)) {
					std::cout << "Updated calibration point at distance " << smrDistance << std::endl;
				}
			}
		}
	}
}





void SmartCamera::clearSMRTargets() {
	targeter.clearTargets();
}

void SmartCamera::resetTargetLoop() {
	targeter.resetTargetLoop();
}

void SmartCamera::addSMRTarget(float az, float el, float dist) {
	targeter.addTarget(az, el, dist);
}

int SmartCamera::addAllObservedSMRTargets(bool curr_locked, float curr_az, float curr_el, float curr_dist) {
	float az_tmp = curr_az;
	float el_tmp = curr_el;
	smrAnglesFromLock(az_tmp, el_tmp, curr_dist);
	if (curr_locked) {
		std::cout << "Added pre-locked target: " << az_tmp << ", " << el_tmp << std::endl;
		targeter.addTarget(az_tmp, el_tmp, curr_dist, curr_locked, DegreeToRadian(curr_az), DegreeToRadian(curr_el));
	}
	std::vector<SMRData> observedSmrs = tracker.getCurrentTrackedSMRs();
	//std::cout << "Tracked SMRs: " << observedSmrs.size() << std::endl;
	//int addedCount = 0;
	for (int i = 0; i < observedSmrs.size(); i++) {
		if (targeter.addTarget(observedSmrs[i].getAz(), observedSmrs[i].getEl(), 0))
			std::cout << "Added target: " << observedSmrs[i].getAz() << ", " << observedSmrs[i].getEl() << std::endl;
		//	addedCount++;
	}
	//return addedCount;
	return targeter.numTargets();
}

void SmartCamera::finalizeSMRTargets() {
	targeter.finalizeTargets();
}

bool SmartCamera::finishedAllSMRTargets() {
	return targeter.finishedAllTargets();
}

bool SmartCamera::finishedAllSMRTargetsExceptOne() {
	return targeter.finishedAllTargetsExceptOne();
}

void SmartCamera::setSingleSMRTarget(float az, float el, float dist) {
	targeter.addTarget(az, el, dist);
	targeter.finalizeTargets();
}

int SmartCamera::getCurrentTarget() {
	return targeter.currentTargetIndex();
}

int SmartCamera::getNumTargets() {
	return targeter.numTargets();
}

bool SmartCamera::findBasePoint(float &az, float &el) {
	cv::Point2f offset = moveCalc.getLaserPoint(10.0f);
	cv::Point2f point = targeter.findBasePosition(offset.x, offset.y);
	az = point.x;
	el = point.y;
	return true;
}

int SmartCamera::lockedOnTargetIndex(float az, float el, float distance) {
	smrAnglesFromLock(az, el, distance);
	return targeter.getLockedIndex(az, el);
}

void SmartCamera::incrementSMRTarget() {
	targeter.incrementTarget();
}


bool SmartCamera::lockedOntoCurrentTarget(float az, float el, float distance) {
	smrAnglesFromLock(az, el, distance);
	return targeter.currentTargetIndex() == targeter.getLockedIndex(az, el);
}

//TODO: 3.6 is temporary for testing, needs to be distance once distance is reliable
void SmartCamera::smrAnglesFromLock(float &az, float &el, float distance) {
	cv::Point2f offset;
	if (distance > 0.4f)
		offset = moveCalc.getLaserPoint(distance);
	else
		//offset = moveCalc.getLaserPoint(3.6f);
		offset = moveCalc.getLaserPoint(distance);
	//std::cout << "Offset: " << offset << ", distance: " << distance << std::endl;
	az = DegreeToRadian(az) - offset.x;
	el = DegreeToRadian(el) + offset.y;
}

void SmartCamera::setOutdoorExp() {
	outdoor_mode = true;
	indoor_mode = false;
	//std::cout << "Outdoor Mode: " << std::endl;

}

void SmartCamera::setIndoorExp() {
	outdoor_mode = false;
	indoor_mode = true;
	//std::cout << "Indoor Mode: " << std::endl;
}

//load calibration data from file and initialize undistortion maps for quicker undistortion
//also load SMR angle/distance calibration
//also set image processing brightness threshold, Camera Properties and LED Flash settings
bool SmartCamera::loadCalibration() {
	if(!camCalib.loadCalibrationFromFile())	 
		return false;
	//camCalib.initializeUndistortionMaps(cam.getResolution());
	if (!moveCalc.loadParallaxCalibrationFile())
		return false;
	CalibData cd;
	cHandle.getUpdatedCalibData(cd);
	imProc.setBrightThreshold(cd.bright_threshold); //setting brightness threshold for image detection from config file cam_calibration.json
	imProc.setRedThresholdY(cd.red_thresh_y);
	imProc.setRedThresholdCr(cd.red_thresh_cr);
	imProc.setGreenThresholdCr(cd.green_thresh_cr);
	imProc.setGreenThresholdCb(cd.green_thresh_cb);
	setTargetInt_Thresh(cd.target_intensity_threshold_high, cd.target_intensity_threshold_low);
	setTargetIntensity_forAutoExp(cd.target_intensity_for_auto_exp);
	setAutoExpResetInterval(cd.auto_exp_reset_interval);
	if(!getIprobeMode() || !getIprobeLocked()) {
		setAutoLockCameraProp();
	}
	firmware_->setFlashBrightness(cd.flash_brightness);
	if(outdoor_mode){
		//std::cout << "Outdoor Mode: " << outdoor_mode << std::endl;
	}
	else if (indoor_mode){
		//std::cout << "Indoor Mode: " << indoor_mode << std::endl;
	}
	Set_back_cam(cd.back_side_detection); // back camera detection control from config file
	Set_Spiral_Thresh(cd.spiral_threshold); //set the threshold for spiral search to stop(when away from the tracker)
	Set_Spiral_freq(cd.spiral_freq); //spiral_freq
	Set_Spiral_timeout(cd.spiral_timeout); // spiral_timeout
	Set_auto_calib(cd.cam_auto_calib); // set auto calib for camera parallax
	//setting flash control from config file cam_calibration.json
	
	firmware_->setFlashDuration(cd.flash_duration);
	firmware_->setFlashOffset(cd.flash_offset);
	firmware_->setStepSize_for_iVisionMove(cd.iv_mv_step_size);
	return true;
}

void SmartCamera::Set_back_cam(bool set_cam){
	back_cam = set_cam;
}

void SmartCamera::Set_Spiral_Thresh(float set_spiral_Thresh){
	spiral_thresh = set_spiral_Thresh;
}

void SmartCamera::Set_Spiral_timeout(float set_spiral_timeout){
	spiral_timeout = set_spiral_timeout;
}

void SmartCamera::Set_Spiral_freq(float Set_Spiral_freq){
	spiral_freq = Set_Spiral_freq;
}

void SmartCamera::Set_auto_calib(bool set_auto_calib) {
	cam_auto_calib = set_auto_calib;
}

bool SmartCamera:: Filter_calibration(){
	return moveCalc.filterCalibData();
}


void SmartCamera::resetParallaxCalibration() {
	if(camCalib.loadCalibrationFromFile()) {
		camCalib.initializeUndistortionMaps(cam.getResolution());
		moveCalc.resetParallaxCalibrationProcess();
	}
	else {
		std::cout << "Must do camera calibration before parallax calibration." << std::endl;
	}
}

bool SmartCamera::parallaxCalibrationStep(float smrDistance) {
	if (moveCalc.distanceNeedsCalibration(smrDistance)) {
		cv::Point3f laser = getLaserCoordinates();
		if (/*laser.x >= 0 && laser.y >= 0 && */laser.z >= 0) {
			//cv::Point2f imgAzEl = camCalib.singleAzElFromXY(laser.x, laser.y, cam.getResolution());
			if (moveCalc.addParallaxCalibrationPoint(smrDistance, laser.x, laser.y) || true) {
				auto ret = moveCalc.saveParallaxCalibrationFile();
				std::cout << "Writing parallax calibration file status: " << ret << '\n';
			}
			return false;
		}
	}
	return false;
}




//reset the camera calibration process
void SmartCamera::setupCameraCalibration() {
	camCalib.resetCalibrationProcess();
}


//get a new image and use it for calibration
bool SmartCamera::cameraCalibrationStep() {
	cv::Mat img = grabNextImageBW();
	//cv::imwrite("camera_calibration.png", img);
	//std::cout << "Stored calibration image." << std::endl;
	return camCalib.performCalibrationStep(img);
}


//update calibration data and store it to file
bool SmartCamera::updateCameraCalibration(cv::Mat cameraMatrix, cv::Mat distortionCoefficients, cv::Size imageSize) {
	camCalib.setCalibrationValues(cameraMatrix, distortionCoefficients, imageSize);
	return camCalib.saveCalibrationToFile();
}






//get next processing image in RGBA format, copy image to object detector's buffer
bool SmartCamera::updateDetectionImage() {
	if (detector.isActive()) {
		cv::Mat y, u, v, rgba;
		unsigned long imgTime;
		if (!cam.nextProcessImage(y, false, u, false, v, false, rgba, true, false, imgTime)) {
			return false;
		}
		return detector.copyImageToBuffer(rgba);
	}
	return false;
}

//initialize detector network with the given type
bool SmartCamera::setupObjectDetector(detectNet::NetworkType type) {
	return detector.setup(cam.getResolution(), type);
}


//clean up the object detector
bool SmartCamera::closeObjectDetector() {
	return detector.close();
}


//set object detector active/inactive
void SmartCamera::setObjectDetectorActive(bool a) {
	detector.setActive(a);
}


//is object detector active?
bool SmartCamera::objectDetectorIsActive() {
	return detector.isActive();
}


//process image in buffer to find target objects
bool SmartCamera::detectObjects() {
	return detector.processFromBuffer();
}


//return list of most recent detected objects
std::vector<DetectedObject> SmartCamera::getDetectedObjects() {
	return detector.getDetectedObjects();
}



//find the SMR that is reflecting the red laser
// cv::Point3f SmartCamera::getLaserCoordinates() {

// 	// importing Parallax calibration data to determine the ROI for auto calib 
// 	CalibData data;
// 	cHandle.getUpdatedCalibData(data);
// 	float sum_az = 0.0;
// 	float sum_el = 0.0;
// 	float avg_az = 0.0;
// 	float avg_el = 0.0;
// 	int table_size = data.parallaxTable.distance.size();

// 	//averaging out the realtime Parallax calibration points to have a bounding box ROI
// 	for (int i = 0; i < table_size; i++) {	
// 			sum_az += data.parallaxTable.offset[i].first  + data.parallaxTable.auto_calib_x_offset;
// 			sum_el += data.parallaxTable.offset[i].second + data.parallaxTable.auto_calib_y_offset;
// 		}
// 	avg_az = sum_az/data.parallaxTable.offset.size();
// 	avg_el = sum_el/data.parallaxTable.offset.size();
	
	
//     // Reverse the normalization to the pixel range [0, 3264] in AZ and [0, 2464] in EL
//     int az_norm = 0.5 * ((avg_az + 1.0) * (3264.0 - 0.0));
// 	int el_norm = 0.5 * ((avg_el + 1.0) * (2464.0 - 0.0));
	
// 	cv::Mat y_roi, u_roi, v_roi, rgba;
// 	int distance = firmware_->currentDistance() / 1000; // in meter

// 	int az_left = 0;
// 	int az_right = 0;
// 	int el_top = 0;
// 	int el_bot = 0; 
// 	//changing ROI range depending on the distance of 2m or more
// 	if(distance > 2) {

// 		/*multipliers for ROI size in accordance the distance*/
// 		const int min_az_left = 1500; 
// 		const int min_az_right = 2250; 
// 		const int min_el_top = 1500;
// 		const int min_el_bot = 3000;

// 		/* Calculate parameters inversely proportional to distance*/
// 		az_left = static_cast<int>(min_az_left / distance );
// 		az_right = static_cast<int>(min_az_right / distance );
// 		el_top = static_cast<int>(min_el_top / distance);
// 		el_bot = static_cast<int>(min_el_bot / distance );
// 		/* Calculate parameters inversely proportional to distance*/
// 		az_left = static_cast<int>(min_az_left / distance );
// 		az_right = static_cast<int>(min_az_right / distance );
// 		el_top = static_cast<int>(min_el_top / distance);
// 		el_bot = static_cast<int>(min_el_bot / distance );

// 		/*carve out the ROI from the center*/
// 		cv::Rect roi(az_norm - az_left, el_norm - el_top, az_right, el_bot);
// 		/*extract the ROI from the real time Image*/ 
// 		if (!cam.nextProcessImage(y_roi, true, u_roi, true, v_roi, true, rgba, false, true, roi)) {
// 			return cv::Point3f(-1, -1, -1);
// 		}
// 		//cv::imwrite("ROI Yimage.png", y_roi);
// 	}
// 	else 
// 		return cv::Point3f(-1, -1, -1);
	
// 	cv::Point3f roi_coordinates = imProc.findRedLaser(y_roi, u_roi, v_roi);
	
// 	if (!cam.releaseBuffers())
// 		std::cout << "Failed to release buffers." << std::endl;

// 	std::cout << "ROI coordinates with offset " << roi_coordinates.x + (az_norm - az_left) << " " << roi_coordinates.y + (el_norm - el_top) << std::endl;

// 	cv::Point2f imgAzEl = camCalib.singleAzElFromXY(roi_coordinates.x + (az_norm - az_left), roi_coordinates.y + (el_norm - el_top) , cam.getResolution());
// 	cv::Point3f angles (imgAzEl.x, imgAzEl.y, roi_coordinates.z);

// 	return angles;
// }

//find the SMR that is reflecting the red laser
cv::Point3f SmartCamera::getLaserCoordinates() {

	cv::Mat y, u, v, rgba, src1Y, src1U, src1V,src2Y, src2U, src2V;

	unsigned long imgTime, firstImgTime, secondImgTime;
	if (!cam.nextProcessImage(y, true, u, true, v, true, rgba, false, true, imgTime)) {
		return cv::Point3f(-1, -1, -1);
	}

	// if (!cam.lastTwoProcessImages(src1Y, src1U, src1V, src2Y, src2U, src2V, firstImgTime, secondImgTime)) {
	// 	return cv::Point3f(-1, -1, -1);
	// }
	//cv::imwrite("imgdump.png", src1Y);
	cv::Point3f coordinates = imProc.findRedLaser(y, u, v);	

	if (!cam.releaseBuffers())
		std::cout << "Failed to release buffers." << std::endl;

	//std::cout << "Found laser point: " << coordinates << std::endl;

	cv::Point2f imgAzEl = camCalib.singleAzElFromXY(coordinates.x, coordinates.y, cam.getResolution());
	cv::Point3f angles (imgAzEl.x, imgAzEl.y, coordinates.z);

	return angles;
}


float SmartCamera::getRedSum(cv::Point2f lastCoordinates, float approximateDistance) {

	cv::Mat y, u, v, rgba;
	unsigned long imgTime;
	if (!cam.nextProcessImage(y, true, u, true, v, true, rgba, false, true, imgTime)) {
		return -1;
	}

	cv::Size imgRes = cam.getResolution();

	int aoiSize = std::max((int)(imgRes.height / (20 + approximateDistance * 10)), 1);

	int aoiX = std::min(std::max((int)(imgRes.width*lastCoordinates.x) - aoiSize, 0), (int)imgRes.width - aoiSize);
	int aoiY = std::min(std::max((int)(imgRes.height*lastCoordinates.y) - aoiSize, 0), (int)imgRes.height - aoiSize);

	cv::Rect redAoi(aoiX / 2, aoiY / 2, aoiSize, aoiSize);

	int redSum = imProc.getRedSum(v, redAoi);

	if (!cam.releaseBuffers())
		std::cout << "Failed to release buffers." << std::endl;

	return ((float)redSum) / (aoiSize*aoiSize);
}


cv::Mat SmartCamera::grabNextImageRGBA() {
	cv::Mat y, u, v, rgba;
	unsigned long imgTime;
	cam.nextProcessImage(y, false, u, false, v, false, rgba, true, false, imgTime);
	return rgba;
}

//get Y component of next processing image, which is just the black and white image
cv::Mat SmartCamera::grabNextImageBW() {
	cv::Mat y, u, v, rgba;
	unsigned long imgTime;
	if (!cam.nextProcessImage(y, true, u, false, v, false, rgba, false, false, imgTime)) {
		return y;
	}
	return y;
}

#ifdef LT_IVISION_USE_FAKE_IMG_TAKEN_FLAG
void SmartCamera::AttachTimestampObserver(api::common::Observer<ImageCaptureTimestamp>* obs) {
	if (obs)
		this->cam.AttachTimestampObserver(obs);
}
void SmartCamera::DetachTimestampObserver(api::common::Observer<ImageCaptureTimestamp>* obs) {
	if (obs)
		this->cam.DetachTimestampObserver(obs);
}
#endif