#include "CameraCalibrationManager.h"
#include <iostream>
#include <fstream>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>

#include "Debug.h"

//default constructor - all calibration data uninitialized
CameraCalibrationManager::CameraCalibrationManager() {
	cameraMatrix = cv::Mat(3, 3, CV_64FC1, cv::Scalar(0));
	distortionCoeffs = cv::Mat(5, 1, CV_64FC1, cv::Scalar(0));
	FC_in_FB = cv::Mat::eye(3, 3, CV_64F);

	isCalibrated = false;
	mapsInitialized = false;

	pixel_ang = cv::Point2f(-1, -1);
	pixel_ang_img_size = cv::Size(0,0);
}


//destructor
CameraCalibrationManager::~CameraCalibrationManager() {

}


//returns the camera matrix scaled by image resolution
cv::Mat CameraCalibrationManager::getScaledCameraMatrix(cv::Size imageSize) {
	cv::Mat cm = cameraMatrix.clone();
	cm.at<double>(0,0) *= imageSize.width;
	cm.at<double>(0,2) *= imageSize.width;
	cm.at<double>(1,1) *= imageSize.height;
	cm.at<double>(1,2) *= imageSize.height;
	return cm;
}


//clears all data used for performing camera calibration
void CameraCalibrationManager::resetCalibrationProcess() {
	objectPoints.clear();
	imagePoints.clear();
	distortionCoeffs = cv::Mat::zeros(8, 1, CV_64FC1);
	//clear cameraMatrix?
	rvecs.clear();
	tvecs.clear();
	boardSize = cv::Point(9, 6);
}


//processes image to find calibration pattern, and if pattern has been found enough times, actually performs calibration
bool CameraCalibrationManager::performCalibrationStep(cv::Mat img) {
	//calculate current imagePoints and add it to list if pattern is found
	//cv::imwrite("img.png", img);
	std::vector<cv::Point2f> points;
	bool found = cv::findCirclesGrid(img, cv::Size(boardSize.x, boardSize.y), points);
	if (found) {
		std::cout << "Found pattern." << std::endl;

		cv::Mat colorImg;
		cv::cvtColor(img, colorImg, cv::COLOR_GRAY2BGR);
		cv::drawChessboardCorners(colorImg, cv::Size(boardSize.x, boardSize.y), cv::Mat(points), found);
		cv::imwrite(createFilename("camCalib", imagePoints.size()), colorImg);

		imagePoints.push_back(points);

		//if pattern is found, calculate current objectPoints and add it to list
		std::vector<cv::Point3f> objp;
		for (int i = 0; i < boardSize.y; i++) {
			for (int j = 0; j < boardSize.x; j++) {
				objp.push_back(cv::Point3f(j, i, 0));
			}
		}
		objectPoints.push_back(objp);
	}
	else {
		std::cout << "Pattern not found." << std::endl;
	}

	std::cout << "Captured " << imagePoints.size() << " of 50 points." << std::endl;

	//once you have enough valid images, perform calibration, then save calibration values
	if (imagePoints.size() > 50) {
		double error = cv::calibrateCamera(objectPoints, imagePoints, img.size(), cameraMatrix, distortionCoeffs, rvecs, tvecs);

		std::cout << "Camera matrix: " << cameraMatrix << std::endl;
		std::cout << "Distortion coefficients: " << distortionCoeffs << std::endl;
		std::cout << "Re-projection error: " << error << std::endl;

		isCalibrated = true;
		setCalibrationValues(cameraMatrix, distortionCoeffs, img.size());
		saveCalibrationToFile();
		return true;
	}
	return false;
}


//updates camera matrix and distortion data
void CameraCalibrationManager::setCalibrationValues(cv::Mat cm, cv::Mat dc, cv::Size imSize) {
	cameraMatrix = cm.clone();
	cameraMatrix.at<double>(0,0) /= imSize.width;
	cameraMatrix.at<double>(0,2) /= imSize.width;
	cameraMatrix.at<double>(1,1) /= imSize.height;
	cameraMatrix.at<double>(1,2) /= imSize.height;
	distortionCoeffs = dc.clone();
	isCalibrated = true;
}


//set up undistortion maps so undistort operations are easier
void CameraCalibrationManager::initializeUndistortionMaps(cv::Size imageSize) {
	if (!mapsInitialized) {
		cv::Mat cm = getScaledCameraMatrix(imageSize);
		cv::initUndistortRectifyMap(cm, distortionCoeffs, cv::Mat(), cm, imageSize, CV_16SC2, undistortMap1, undistortMap2);
		mapsInitialized = true;
	}
}


//undistort entire image
bool CameraCalibrationManager::undistortCameraImage(cv::Mat input, cv::Mat& output) {
	if (!isCalibrated)
		return false;
	else if (!mapsInitialized)
		initializeUndistortionMaps(input.size());
	cv::remap(input, output, undistortMap1, undistortMap2, cv::INTER_LINEAR);
	return true;
}


//undistort list of points
std::vector<cv::Point2f> CameraCalibrationManager::undistortPoints(std::vector<cv::Point2f> pts, cv::Size imageSize) {
	std::vector<cv::Point2f> undistPts;
	//std::cout << "Scaled camera matrix: " << getScaledCameraMatrix(imageSize) << std::endl;
	cv::Mat cm = getScaledCameraMatrix(imageSize);
	cv::undistortPoints(pts, undistPts, cm, distortionCoeffs, cv::noArray(), cm);
	return undistPts;
}


//undistort single point
cv::Point2f CameraCalibrationManager::undistortPoint(cv::Point2f pt, cv::Size imageSize) {
	std::vector<cv::Point2f> distPts;
	distPts.push_back(pt);
	std::vector<cv::Point2f> undistPts = undistortPoints(distPts, imageSize);
	return undistPts[0];
}

//get angles relative to camera center based on x/y coordinates
std::vector<cv::Point2f> CameraCalibrationManager::azElFromXY(std::vector<cv::Point2f> coordinates, cv::Size imageSize, bool undistort) {
	std::vector<cv::Point2f> angles;
	if (!coordinates.empty()) {
		std::vector<cv::Point2f> processedPoints = undistort ? undistortPoints(coordinates, imageSize) : coordinates;
		
		cv::Point2f imageCenter(imageSize.width / 2.0f, imageSize.height / 2.0f);

		for (const auto& pt : processedPoints) {
			// Compute the raw pixel displacement from the image center
			float dx = pt.x - imageCenter.x;
			float dy = pt.y - imageCenter.y;
			// Compute the angles in radians
			angles.push_back(cv::Point2f(dx, dy));
		}
	return angles;
	}
}


//get single set of angles relative to camera center based on x/y coordinates
cv::Point2f CameraCalibrationManager::singleAzElFromXY(float imgX, float imgY, cv::Size imageSize, bool undistort) {
	std::vector<cv::Point2f> xy(1, cv::Point2f(imgX,imgY));
	return azElFromXY(xy, imageSize, undistort)[0];
}


//get image x/y coordinates based on camera angles
std::vector<cv::Point2f> CameraCalibrationManager::xyFromAzEl(std::vector<cv::Point2f> angles, cv::Size imageSize) {
	cv::Mat cm = getScaledCameraMatrix(imageSize);
	std::vector<cv::Point2f> xys;
	if (angles.empty()) return xys;

    // Define the image center as the reference point
    cv::Point2f imageCenter(imageSize.width / 2.0f, imageSize.height / 2.0f);

    for (const auto& angle : angles) {
        float x = imageCenter.x + angle.x;
        float y = imageCenter.y + angle.y;

        // Store the calculated pixel coordinates
        xys.push_back(cv::Point2f(x, y));
    }
	return xys;
}

cv::Point2f CameraCalibrationManager::singleXYFromAzEl(float az, float el, cv::Size imageSize) {
	std::vector<cv::Point2f> azel(1, cv::Point2f(az,el));
	return xyFromAzEl(azel, imageSize)[0];
}

cv::Point2f CameraCalibrationManager::pixelAngle(cv::Size imageSize) {
	if (imageSize == pixel_ang_img_size) {
		return pixel_ang;
	}
	else {
		pixel_ang_img_size = imageSize;
		std::vector<cv::Point2f> tmp_pix, tmp_ang;
		cv::Point2f center_point = getCenter();
		center_point.x *= imageSize.width;
		center_point.y *= imageSize.height;
		tmp_pix.push_back(center_point);
		tmp_pix.push_back(center_point + cv::Point2f(1,-1));
		tmp_ang = azElFromXY(tmp_pix, pixel_ang_img_size, true);
		pixel_ang = tmp_ang[1] - tmp_ang[0];
		//std::cout << "Pix ang: " << pixel_ang << ", " << tmp_ang[1] << " - " << tmp_ang[0] << std::endl;
		return pixel_ang;
	}
}


cv::Point2f CameraCalibrationManager::getCenter() {
	return cv::Point2f(cameraMatrix.at<double>(0,2), cameraMatrix.at<double>(1,2));
}


//store camera matrix and distortion data in file
bool CameraCalibrationManager::saveCalibrationToFile() {
	return saveCalibrationToJsonFile();
}


//retrieve camera matrix and distortion data from file
bool CameraCalibrationManager::loadCalibrationFromFile() {
	return loadCalibrationFromJsonFile();
}


bool CameraCalibrationManager::saveCalibrationToTxtFile() {
	std::ofstream calibFile;
	calibFile.open("cam_calibration.txt");

	double f_x = cameraMatrix.at<double>(0,0);
	double c_x = cameraMatrix.at<double>(0,2);
	double f_y = cameraMatrix.at<double>(1,1);
	double c_y = cameraMatrix.at<double>(1,2);	

	double k_1 = distortionCoeffs.at<double>(0,0);
	double k_2 = distortionCoeffs.at<double>(1,0);
	double p_1 = distortionCoeffs.at<double>(2,0);
	double p_2 = distortionCoeffs.at<double>(3,0);
	double k_3 = distortionCoeffs.at<double>(4,0);

	calibFile << f_x << " " << c_x << " " << f_y << " " << c_y << std::endl;
	calibFile << k_1 << " " << k_2 << " " << p_1 << " " << p_2 << " " << k_3 << std::endl;

	calibFile.close();

	return true;
}

bool CameraCalibrationManager::saveCalibrationToJsonFile() {
		CalibData cd;
		cHandle->getUpdatedCalibData(cd);
		cd.camCalib.cam_fx = cameraMatrix.at<double>(0,0);
		cd.camCalib.cam_cx = cameraMatrix.at<double>(0,2);
		cd.camCalib.cam_fy = cameraMatrix.at<double>(1,1);
		cd.camCalib.cam_cy = cameraMatrix.at<double>(1,2);

		cd.camCalib.dist_k1 = distortionCoeffs.at<double>(0,0);
		cd.camCalib.dist_k2 = distortionCoeffs.at<double>(1,0);
		cd.camCalib.dist_p1 = distortionCoeffs.at<double>(2,0);
		cd.camCalib.dist_p2 = distortionCoeffs.at<double>(3,0);
		cd.camCalib.dist_k3 = distortionCoeffs.at<double>(4,0);

		cd.FC_in_FB = FC_in_FB.clone();

		cHandle->updateCalibData(cd);
		return cHandle->writeCalibFile("cam_calibration.json") == 0;
}

bool CameraCalibrationManager::loadCalibrationFromTxtFile() {
	std::ifstream calibFile;
	calibFile.open("cam_calibration.txt");

	if(calibFile.is_open()) {

		double f_x, f_y, c_x, c_y, k_1, k_2, p_1, p_2, k_3;

		calibFile >> f_x;
		calibFile >> c_x;
		calibFile >> f_y;
		calibFile >> c_y;
		calibFile >> k_1;
		calibFile >> k_2;
		calibFile >> p_1;
		calibFile >> p_2;
		calibFile >> k_3;	

		cameraMatrix = cv::Mat(3, 3, CV_64FC1, cv::Scalar(0));
		cameraMatrix.at<double>(0,0) = f_x;
		cameraMatrix.at<double>(0,2) = c_x;
		cameraMatrix.at<double>(1,1) = f_y;
		cameraMatrix.at<double>(1,2) = c_y;
		cameraMatrix.at<double>(2,2) = 1;
		distortionCoeffs = cv::Mat(5, 1, CV_64FC1);
		distortionCoeffs.at<double>(0,0) = k_1;
		distortionCoeffs.at<double>(1,0) = k_2;
		distortionCoeffs.at<double>(2,0) = p_1;
		distortionCoeffs.at<double>(3,0) = p_2;
		distortionCoeffs.at<double>(4,0) = k_3;

		calibFile.close();

		isCalibrated = true;

		return true;
	}
	else {
		std::cout << "Couldn't open calibration file." << std::endl;
		return false;
	}
}

bool CameraCalibrationManager::loadCalibrationFromJsonFile() {
	if (cHandle->readCalibFile("cam_calibration.json") == 0) {
		CalibData cd;
		cHandle->getUpdatedCalibData(cd);
		cameraMatrix = cv::Mat(3, 3, CV_64FC1, cv::Scalar(0));
		cameraMatrix.at<double>(0,0) = cd.camCalib.cam_fx;
		cameraMatrix.at<double>(0,2) = cd.camCalib.cam_cx;
		cameraMatrix.at<double>(1,1) = cd.camCalib.cam_fy;
		cameraMatrix.at<double>(1,2) = cd.camCalib.cam_cy;
		cameraMatrix.at<double>(2,2) = 1;
		distortionCoeffs = cv::Mat(5, 1, CV_64FC1);
		distortionCoeffs.at<double>(0,0) = cd.camCalib.dist_k1;
		distortionCoeffs.at<double>(1,0) = cd.camCalib.dist_k2;
		distortionCoeffs.at<double>(2,0) = cd.camCalib.dist_p1;
		distortionCoeffs.at<double>(3,0) = cd.camCalib.dist_p2;
		distortionCoeffs.at<double>(4,0) = cd.camCalib.dist_k3;
		FC_in_FB = cd.FC_in_FB.clone();
		// std::cout << "FC_in_FB Matrix: " << std::endl;
        // std::cout << FC_in_FB << std::endl;
		isCalibrated = true;
		return true;
	}
	std::cout << "Couldn't open calibration file." << std::endl;
	return false;
}