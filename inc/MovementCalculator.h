#pragma once

#include <opencv2/core.hpp>
#include <vector>
#include <random>
#include "CamCalibReader.h"
#include "Utility.hpp"
#include "iVisionClient.h"
#include "CameraCalibrationManager.h"

using namespace std::chrono;
enum MovementType { NoMove, MoveBy, MoveTo, MoveToStep, Spiral};

struct Movement {
	MovementType type;
	float az;
	float el;
	float radius;
};

// struct CalibrationPoint {
// 	float distance;
// 	float az;
// 	float el;
// };

struct AutoCalibration {
	double x_offset = 0.0;
	double y_offset = 0.0;
};

class MovementCalculator {
public:
	MovementCalculator();
	~MovementCalculator();

	void setCalibHandle(CalibHandle* ch) { cHandle = ch; }

	//will need to add functions for calibration
	//parallax calibration process
	void resetParallaxCalibrationProcess();	//resets calibration data before starting calibration process
	bool addParallaxCalibrationPoint(float distance, float imgAz, float imgEl);	//add a calibration point to stored list
	bool addParallaxCalibrationPoint(CalibrationPoint cp);	//add a calibration point to stored list

	bool updateParallaxCalibrationPoint(float distance, float imgAz, float imgEl);
	int buf = 0;
	void reset_buf();
	std::vector<float>x_offset_val;
	std::vector<float>y_offset_val;
	double findMedian(std::vector<float> a, int n);

	bool saveParallaxCalibrationFile();
	bool loadParallaxCalibrationFile();
	bool filterCalibData();

	std::vector<CalibrationPoint>& getParallaxCalibPoints();

	cv::Point2f getLaserPoint(float searchDistance);

	float getLaserDistance(cv::Point2f imgAngles);

	Movement getMovement(float trkAz, float trkEl, float smrImgAz, float smrImgEl, float searchDistance, bool smrHitByLaser, bool back_cam, cv::Point2f pixelAngle);

	bool distanceNeedsCalibration(float distance);
	iVisionClient visionclient;
	bool SpiralStart = false;
	const double degrees_per_pixel = 0.004486532723144;
private:

	cv::Point2f interpolateCalibPoints(CalibrationPoint cp1, CalibrationPoint cp2, float searchDistance);



	//list of calibration points
	std::vector<CalibrationPoint> parallaxCalibPoints;

	AutoCalibration value;
	float lastCalibDistance;


	//calibrated rotation and translation of camera relative to laser
	cv::Mat camR;
	cv::Mat camT;

	//from current parallax calibration points, estimate camera rotation and translation
	//(i.e. calculate camR and camT based on parallaxCalibPoints) - also private for now
	double calibrateParallax(cv::Size imageSize);


	std::random_device rd;
	std::mt19937 gen;
	std::uniform_real_distribution<> dis;

	CalibHandle* cHandle;
	std::shared_ptr<CameraCalibrationManager> camCalibManager;
	bool saveCalibrationToTxtFile();
	bool saveCalibrationToJsonFile();
	bool loadCalibrationFromTxtFile();
	bool loadCalibrationFromJsonFile();

	//float Rad2Deg(float r);
};
