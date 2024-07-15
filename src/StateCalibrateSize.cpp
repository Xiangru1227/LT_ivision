#include "StateCalibrateSize.h"
#include <iostream>
#include <fstream>
//#include <opencv2/imgproc.hpp>
//#include <opencv2/imgcodecs.hpp>
#include <unistd.h>

StateCalibrateSize::StateCalibrateSize() : IVisionState() {

	lastDistance = 1000;
}

StateCalibrateSize::StateCalibrateSize(SmartCamera* ca, iVisionClient* fi, SDKCommunicator* sc, StateEnum* st, MovementCalculator* mo) : IVisionState(ca, fi, sc, st, mo) {

	lastDistance = 1000;
}

StateCalibrateSize::~StateCalibrateSize() {

}

bool StateCalibrateSize::enterState() {
	std::cout << "Entering Calibrate Size State." << std::endl;

	//calibPoints.clear();
	currentSizes.clear();

	usingCamera = true;
	cam->startVideo();

	//sleep(2);
	//cv::Point3f laserPosition = cam->getLaserCoordinates();
	sleep(4);
	std::cout << "Ready for calibration." << std::endl;

	return true;
}

bool StateCalibrateSize::stateAction() {
	//std::cout << "Performing Calibrate Size State action." << std::endl;

/*	float distance = firmware->currentDistance();

	if (firmware->hasSMRLock()) {
		if (firmware->SMRStableDuration() > 6) {
			currentSizes.clear();
			lastDistance = distance;
		}
	}
	else if (lastDistance > 1000) {
		std::vector<cv::Point2f> coordinates = cam->getSMRCoordinates(firmware);
		//for (int i  = 0; i < coordinates.size() / 2; i++) {
			//std::cout << "SMR size: " << coordinates[2*i+1].x << std::endl;
		//}
		if (coordinates.size() / 2 == 1) {
			if (currentSizes.size() < 11)
				currentSizes.push_back(coordinates[1].x);
			if (currentSizes.size() == 10) {
				float avg = 0;
				for (int i = 0; i < currentSizes.size(); i++) {
					avg += currentSizes[i];
				}
				avg /= currentSizes.size();
				std::cout << "Average SMR size: " << avg << std::endl;
				CalibrationPoint cp;
				cp.distance = lastDistance;
				cp.x = 0;
				cp.y = 0;
				cp.size = avg;
				//calibPoints.push_back(cp);
				//if (distance > 21500) {
				if (lastDistance > 17500) {
					if (!writeCalibrationToFile()) {
						std::cout << "Couldn't write calibration data to file." << std::endl;
					}
					std::cout << "Finished calibration." << std::endl;
					(*nextState) = End;
				}
			}
		}
	}
*/
	return true;
}

bool StateCalibrateSize::leaveState() {
	std::cout << "Leaving Calibrate Size State." << std::endl;
	return true;
}

//write short and long range distance and positions to file
bool StateCalibrateSize::writeCalibrationToFile() {

	//open file
	std::ofstream calibFile;
	calibFile.open("calibration2.txt");
	
	//for (int i = 0; i < calibPoints.size(); i++) {
	//	calibFile << calibPoints[i].distance << " " << calibPoints[i].size << std::endl;
	//}

	//close file
	calibFile.close();

	return true;
}
