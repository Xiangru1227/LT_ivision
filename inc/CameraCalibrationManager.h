#pragma once

#include <opencv2/core.hpp>
#include "CamCalibReader.h"

class CameraCalibrationManager {
public:

	//constructor/destructor
	CameraCalibrationManager();
	~CameraCalibrationManager();

	void setCalibHandle(CalibHandle* ch) { cHandle = ch; }

	//get intrinsic parameters
	cv::Mat getCameraMatrix() {return cameraMatrix;}
	cv::Mat getScaledCameraMatrix(cv::Size imageSize);

	//get distortion parameters
	cv::Mat getDistortionCoefficients() {return distortionCoeffs;}

	//set camera/distortion parameters
	void setCalibrationValues(cv::Mat cm, cv::Mat dc, cv::Size imSize);


	//fix camera distortion
	void initializeUndistortionMaps(cv::Size imageSize);	//set up undistortion functions
	bool undistortCameraImage(cv::Mat input, cv::Mat& output);	//undistort an image
	std::vector<cv::Point2f> undistortPoints(std::vector<cv::Point2f> pts, cv::Size imageSize);	//undistort a vector of points
	cv::Point2f undistortPoint(cv::Point2f pt, cv::Size imageSize);


	//cv::Point2f distortPoint(cv::Point2f pt, cv::Size imageSize);


	//get azimuth and elevation (relative to calibrated image center) from x/y image coordinates
	std::vector<cv::Point2f> azElFromXY(std::vector<cv::Point2f> coordinates, cv::Size imageSize, bool undistort = false);
	cv::Point2f singleAzElFromXY(float imgX, float imgY, cv::Size imageSize, bool undistort = false);

	std::vector<cv::Point2f> xyFromAzEl(std::vector<cv::Point2f> angles, cv::Size imageSize);
	cv::Point2f singleXYFromAzEl(float az, float el, cv::Size imageSize);

	//get approximate size of a single pixel in radians
	cv::Point2f pixelAngle(cv::Size imageSize);


	//store/load calibration data
	bool saveCalibrationToFile();
	bool loadCalibrationFromFile();


	//camera calibration process
	void resetCalibrationProcess();	//resets calibration data before starting calibration process
	bool performCalibrationStep(cv::Mat img);	//analyze one image for calibration process


	//is there valid calibration data?
	bool hasCalibration() {return isCalibrated;}

private:
	//camera calibration parameters
	cv::Mat cameraMatrix;
	cv::Mat distortionCoeffs;

	//calibration file reader
	CalibHandle* cHandle;
	bool saveCalibrationToTxtFile();
	bool saveCalibrationToJsonFile();
	bool loadCalibrationFromTxtFile();
	bool loadCalibrationFromJsonFile();

	//pixel size as angles
	cv::Size pixel_ang_img_size;
	cv::Point2f pixel_ang;
	cv::Point2f getCenter();

	//undistortion maps
	cv::Mat undistortMap1, undistortMap2;


	//object has valid calibration data
	bool isCalibrated;


	//undistortion maps have been initialized
	bool mapsInitialized;


	//calibration process data
	std::vector<std::vector<cv::Point3f>> objectPoints;	//3D points corresponding to image points
	std::vector<std::vector<cv::Point2f>> imagePoints;	//2D calibration pattern points detected in image
	cv::Point boardSize;					//size of calibration pattern
	std::vector<cv::Mat> rvecs;				//rotation matrices used in calibration
	std::vector<cv::Mat> tvecs;				//translation matrices used in calibration

};
