#include "MovementCalculator.h"
#include <fstream>
#include <iostream>
#include <utility>
#include <unistd.h>
#include <cstdlib>

MovementCalculator::MovementCalculator() {
	gen = std::mt19937(rd());
	dis = std::uniform_real_distribution<>(-0.0002, 0.0002);
	cHandle = new CalibHandle();
	camCalibManager = std::make_shared<CameraCalibrationManager>();
	camCalibManager->setCalibHandle(this->cHandle);
	lastCalibDistance = 0;
}

MovementCalculator::~MovementCalculator() {

}

void MovementCalculator::resetParallaxCalibrationProcess() {
	parallaxCalibPoints.clear();
	lastCalibDistance = 0;
}

bool MovementCalculator::addParallaxCalibrationPoint(float distance, float imgAz, float imgEl) {
	CalibrationPoint cp;
	cp.distance = distance;
	cp.az = imgAz;
	cp.el = imgEl;
	return addParallaxCalibrationPoint(cp);
}

bool MovementCalculator::addParallaxCalibrationPoint(CalibrationPoint cp) {
	parallaxCalibPoints.push_back(cp);
	lastCalibDistance = cp.distance;

	std::cout << "Added calibration point at distance " << cp.distance << std::endl;
	std::cout << "New size of calibration points: " << parallaxCalibPoints.size() << '\n';

	//probably adjust this to better ensure we have good calibration data, but this is the bare minimum
	//return parallaxCalibPoints.size() > 8;

	//old way for now
	return cp.distance > 20;
}


bool MovementCalculator::updateParallaxCalibrationPoint(float distance, float imgAz, float imgEl) {
	float minDiff = 100;
	int minIndex = -1;
	
	if(parallaxCalibPoints.size() < 5) {
		std::cout << "Did not find enough points, adding this to the table" << std::endl;
		return addParallaxCalibrationPoint(distance, imgAz, imgEl);
	}
	
	cv::Point2f laser = getLaserPoint(distance);
	float x_offset_obs = imgAz - laser.x;
	float y_offset_obs = imgEl - laser.y;
	buf++;
	// std::cout << "x_offset: " << x_offset_obs << " y_offset: " << y_offset_obs << std::endl;
	for (int i = 0; i < parallaxCalibPoints.size(); i++) {
		float otherDiff;
		if (parallaxCalibPoints.size() > 1) {
			if (i == 0) {
				otherDiff = std::abs(parallaxCalibPoints[i+1].distance - distance);
			}
			else if (i == parallaxCalibPoints.size() - 1) {
				otherDiff = std::abs(parallaxCalibPoints[i-1].distance - distance);
			}
			else {
				otherDiff = std::min(std::abs(parallaxCalibPoints[i+1].distance - distance), std::abs(parallaxCalibPoints[i-1].distance - distance));
			}
		}
		else {
			otherDiff = 0;
		}
		if (otherDiff >= 1) {
			float distDiff = std::abs(parallaxCalibPoints[i].distance - distance);
			if (distDiff < minDiff) {
				minDiff = distDiff;
				minIndex = i;
			}
		}
	}
	x_offset_val.push_back(x_offset_obs);
	y_offset_val.push_back(y_offset_obs);
	double med_result_x;
	double med_result_y;
	if(buf == 10){
		//std::cout << "X_buf_size: " << x_offset_val.size() << " y_buf_size: " << y_offset_val.size() << std::endl;;
		
		buf = 0;
		med_result_x = findMedian(x_offset_val, x_offset_val.size());
		med_result_y = findMedian(y_offset_val, y_offset_val.size());
		x_offset_val.clear();
		y_offset_val.clear();
		std::cout << "Added to File - X Median: " << med_result_x << " Y Median: " << med_result_y << std::endl;
		
		value.x_offset = med_result_x;
		value.y_offset = med_result_y;
		
		return saveParallaxCalibrationFile();
	}
	
	// std::cout << "Calib Point Worth Updating: " << minIndex << " at dist " << distance << " with AZ: " << imgAz << " with EL: " << imgEl <<  std::endl;
	// std::cout << "Diff between new Calib Point and Existing Data - AZ: " << fabs(parallaxCalibPoints[minIndex].az - imgAz) << " EL: " << fabs(parallaxCalibPoints[minIndex].el -  imgEl) << std::endl;
	if (minIndex >= 0) {
		parallaxCalibPoints[minIndex].distance = distance;
		parallaxCalibPoints[minIndex].az = imgAz;
		parallaxCalibPoints[minIndex].el = imgEl;
		//return saveParallaxCalibrationFile();
	}
	return false;
}

void MovementCalculator::reset_buf(){
	buf = 0;
	x_offset_val.clear();
	y_offset_val.clear();
}


bool MovementCalculator::saveParallaxCalibrationFile() {
	return saveCalibrationToJsonFile();
}

bool MovementCalculator::loadParallaxCalibrationFile() {
	return loadCalibrationFromJsonFile();
}

//TODO: check angles to figure out sudden pointing upward issue
Movement MovementCalculator::getMovement(float trkAz, float trkEl, float smrImgAz, float smrImgEl, float searchDistance, bool smrHitByLaser, bool back_cam, cv::Point2f pixelAngle) {

	//TODO: maybe adjust based on absolute AZ/EL instead of just assuming everyting works relatively

	cv::Point2f laser = getLaserPoint(searchDistance);
	//cv::Point2f laser = getLaserPoint(3.44154f);
	//std::cout << "Mvmt " << laser << " - " << smrImgAz << ", " << smrImgEl << "; trk ang: " << trkAz << ", " << trkEl << std::endl;
	float diffAz = laser.x - smrImgAz;
	float diffEl = laser.y - smrImgEl;
	//std::cout << "Mvmt diff: " << diffAz << ", " << diffEl << std::endl;
	float search_radius;

	
	if (smrHitByLaser) {
		//std::cout << "hit by lsr." << std::endl;
		search_radius = 0.05f / searchDistance;
		//usleep(2000000);
		//mil = mil*2;
		//mil.count();
	
		//search_radius = 0;
	}
	else {
		search_radius = 0.0f;
	}

	Movement m;
	m.type = MoveTo;
	if(back_cam){
		m.az = trkAz - (degrees_per_pixel * diffAz);
		m.el = trkEl - (degrees_per_pixel * diffEl);
	}
	else{
		m.az = trkAz + (degrees_per_pixel * diffAz);
		m.el = trkEl + (degrees_per_pixel * diffEl);
	}
	//std::cout << "Pix ang" << pixelAngle << std::endl;
	
	//std::cout <<"moveTO: " << m.az << " " << m.el << std::endl;
	// for (int i = 0; i < numSteps; ++i) {
    //     // Calculate velocity based on position in the trajectory
    //     double velocity = initialVelocity + (finalVelocity - initialVelocity) * (i / static_cast<double>(numSteps - 1));
    //     // Move with velocity in the direction of the destination
    //     m.az += RadianToDegree(velocity * cos(angle));
    //     m.el += RadianToDegree(velocity * sin(angle));
	// 	m.radius = search_radius;
	// 	std::cout << "numSteps: " << numSteps << " Velocity: " << velocity << " target step: (" << m.az << "," << m.el << ")" << std::endl; 
	// 	//std::cout << "Current position: (" << currentPos.x << ", " << currentPos.y << ")\n";
	// 	return m;
	// }
    //     // Output current position
	m.radius = search_radius;
	//std::cout << "sending move to  " << m.az << " " << m.el << std::endl;
	return m;
}



bool MovementCalculator::distanceNeedsCalibration(float distance) {
	float dist_increment = 0.1;
	return distance > lastCalibDistance + dist_increment;
}


cv::Point2f MovementCalculator::interpolateCalibPoints(CalibrationPoint cp1, CalibrationPoint cp2, float searchDistance) {
	float cx = (cp1.az - cp2.az) / ((1 / cp1.distance) - (1 / cp2.distance));
	float x0 = cp1.az - cx / cp1.distance;
	float cy = (cp1.el - cp2.el) / ((1 / cp1.distance) - (1 / cp2.distance));
	float y0 = cp1.el - cy / cp1.distance;
	return cv::Point2f(x0 + cx / searchDistance, y0 + cy / searchDistance);
}

cv::Point2f MovementCalculator::getLaserPoint(float searchDistance) {
	searchDistance = std::max(std::min(searchDistance, 30.0f), 1.0f);
	cv::Point2f target = cv::Point2f(-1.0f, -1.0f);
	if (parallaxCalibPoints.size() > 0) {
		for (int i = 1; i < parallaxCalibPoints.size(); i++) {
			int prevIndex = i - 1;
			if (searchDistance >= parallaxCalibPoints[prevIndex].distance && searchDistance < parallaxCalibPoints[i].distance)
				target = interpolateCalibPoints(parallaxCalibPoints[prevIndex], parallaxCalibPoints[i], searchDistance);
		}
		if (parallaxCalibPoints.size() > 0) {
			if (searchDistance < parallaxCalibPoints[0].distance)
				target = cv::Point2f(parallaxCalibPoints[0].az, parallaxCalibPoints[0].el);
			if (searchDistance >= parallaxCalibPoints[parallaxCalibPoints.size() - 1].distance)
				target = cv::Point2f(parallaxCalibPoints[parallaxCalibPoints.size() - 1].az, parallaxCalibPoints[parallaxCalibPoints.size() - 1].el);
				//target = interpolateCalibPoints(parallaxCalibPoints[parallaxCalibPoints.size() - 2], parallaxCalibPoints[parallaxCalibPoints.size() - 1], searchDistance);
		}
		//std::cout << "Target point: " << target << " Distance: " << searchDistance << std::endl;
	}
	else {
		//TODO: needs to be modified for current camera, although shouldn't really be used anyway
		target = cv::Point2f(.5f, .5f + .05f / (searchDistance * std::tan(21.81f * 3.14157 / 180)));
	}
	return target;
}

float MovementCalculator::getLaserDistance(cv::Point2f imgAngles) {
	float minDist = 5000.0f;
	int minIndex = -1;
	for (int i = 0; i < parallaxCalibPoints.size(); i++) {
		float diffAz = imgAngles.x - parallaxCalibPoints[i].az;
		float diffEl = imgAngles.y - parallaxCalibPoints[i].el;
		float dist = diffAz * diffAz + diffEl * diffEl;
        // std::cout << "imgAngles: " << imgAngles.x << ", " << imgAngles.y << " Parallax Point: " << parallaxCalibPoints[i].az << ", " << parallaxCalibPoints[i].el << std::endl;
		// std::cout << "Distance to point " << i << ": " << dist << " diffAZ & diffEL: " << diffAz << " " << diffEl << std::endl;
		if (dist < minDist) {
			minDist = dist;
			minIndex = i;
		}
	}
	if (minIndex >= 0) {
		return parallaxCalibPoints[minIndex].distance;
	}
	else {
		return -1;
	}
}


//TODO: either need to adjust this for camera-calibrated az/el or also store raw image x/y coordinates, for now I'm just commenting this out
double MovementCalculator::calibrateParallax(cv::Size imageSize) {
	/*std::vector<std::vector<cv::Vec3f>> objPoints;
	std::vector<std::vector<cv::Point2f>> imgPoints1, imgPoints2;
	for (int i = 0; i < parallaxCalibPoints.size(); i++) {
		objPoints.push_back(std::vector<cv::Vec3f>(1, cv::Vec3f(0, 0, parallaxCalibPoints[i].distance)));
		imgPoints1.push_back(std::vector<cv::Point2f>(1, cv::Point2f(0, 0)));
		imgPoints2.push_back(std::vector<cv::Point2f>(1, cv::Point2f(parallaxCalibPoints[i].x, parallaxCalibPoints[i].y)));
	}

	cv::Mat realCameraMatrix = getScaledCameraMatrix(imageSize);
	cv::Mat realDistCoeffs = distortionCoeffs.clone();

	cv::Mat fakeCameraMatrix = realCameraMatrix.clone();
	fakeCameraMatrix.at<double>(0,2) = imageSize.width / 2 - .5;
	fakeCameraMatrix.at<double>(1,2) = imageSize.height / 2 - .5;
	cv::Mat fakeDistCoeffs(5, 1, CV_64FC1, cv::Scalar(0));

	return cv::stereoCalibrate(objPoints, imgPoints1, imgPoints2, fakeCameraMatrix, fakeDistCoeffs, realCameraMatrix, realDistCoeffs, imageSize, camR, camT, cv::Mat(), cv::Mat());*/
	return 0;
}

//float MovementCalculator::Rad2Deg(float r) {
//	return 180 * r / 3.1415926536;
//}




bool MovementCalculator::saveCalibrationToTxtFile() {
	std::ofstream calibFile;
	calibFile.open("cam_parallax_calibration.txt");

	calibFile << parallaxCalibPoints.size() << std::endl;
	for (int i = 0; i < parallaxCalibPoints.size(); i++) {
		calibFile << parallaxCalibPoints[i].distance << " " << parallaxCalibPoints[i].az << " " << parallaxCalibPoints[i].el << std::endl;
	}

	calibFile.close();

	return true;
}

bool MovementCalculator::saveCalibrationToJsonFile() {
		CalibData cd;
		cHandle->getUpdatedCalibData(cd);
		
		cd.parallaxTable = ParallaxTableStruct();

		for (int i = 0; i < parallaxCalibPoints.size(); i++) {
			cd.parallaxTable.distance.push_back(parallaxCalibPoints[i].distance);
			cd.parallaxTable.offset.push_back(std::make_pair(parallaxCalibPoints[i].az,parallaxCalibPoints[i].el));
		}
		cd.parallaxTable.auto_calib_x_offset = value.x_offset;
		cd.parallaxTable.auto_calib_y_offset = value.y_offset;
		cd.parallaxTable.Normalized_Pixels = false;

		cHandle->updateCalibData(cd);
		return cHandle->writeCalibFile("cam_calibration.json") == 0;
}

bool MovementCalculator::loadCalibrationFromTxtFile() {
	std::ifstream calibFile;
	calibFile.open("cam_parallax_calibration.txt");

	if(calibFile.is_open()) {

		parallaxCalibPoints.clear();

		int n;
		calibFile >> n;
		for (int i = 0; i < n; i++) {
			float d, a, e;
			calibFile >> d;
			calibFile >> a;
			calibFile >> e;
			CalibrationPoint cp;
			cp.distance = d;
			cp.az = a;
			cp.el = e;
			parallaxCalibPoints.push_back(cp);
		}

		calibFile.close();

		return true;
	}
	else {
		std::cout << "Couldn't open parallax calibration file." << std::endl;
		return false;
	}
}

bool MovementCalculator::loadCalibrationFromJsonFile() {
	if (cHandle->readCalibFile("cam_calibration.json") == 0) {
	// if (cHandle->readCalibFile("cam_calibration_smoothed.json") == 0) {
		parallaxCalibPoints.clear();
		CalibData cd;
		cHandle->getUpdatedCalibData(cd);
		int table_size = cd.parallaxTable.distance.size();
		for (int i = 0; i < table_size; i++) {
			CalibrationPoint cp;
			cp.distance = cd.parallaxTable.distance[i];
			cp.az = cd.parallaxTable.offset[i].first + cd.parallaxTable.auto_calib_x_offset;
			cp.el = cd.parallaxTable.offset[i].second + cd.parallaxTable.auto_calib_y_offset;
			parallaxCalibPoints.push_back(cp);
		}
		std::cout << "Normalized pixels: " << cd.parallaxTable.Normalized_Pixels << " Parallax Calib Points Size: " << parallaxCalibPoints.size() << std::endl;
		if (!camCalibManager) {
				std::cout << "camCalibManager pntr is null!" << std::endl;
				return false;
			}
		bool reloadCalib = camCalibManager->loadCalibrationFromFile();
		if (!reloadCalib) {
				std::cout << "Failed to load camera calibration from file." << std::endl;
				return false;
		}	
		if (cd.parallaxTable.Normalized_Pixels == true) {
			
			cv::Size imageSize(3264, 2464);
			float center_x = imageSize.width / 2.0f;
			float center_y = imageSize.height / 2.0f;
			std::vector<CalibrationPoint> normalizedPoints;
			for(int j = 0; j < parallaxCalibPoints.size(); j++){
				cv::Point2f normXY = camCalibManager->Norm_singleXYFromAzEl(parallaxCalibPoints[j].az, parallaxCalibPoints[j].el, cv::Size(3264, 2464));
				CalibrationPoint normCp = parallaxCalibPoints[j];
				normCp.az = (normXY.x - center_x);
    			normCp.el = (-1) * (normXY.y - center_y); // negative because y-axis is inverted in image coordinates
				normalizedPoints.push_back(normCp);
			}
			parallaxCalibPoints = normalizedPoints;
		}
		return true;
	}
	std::cout << "Couldn't open calibration file." << std::endl;
	return false;
}

bool MovementCalculator::filterCalibData() {
    std::cout << "In Filtering Func" << std::endl;
    if (cHandle->readCalibFile("cam_calibration.json") == 0) {
        cHandle->writeCalibFile("cam_calibration_original.json");
        std::cout << "Filtering the parallax calibration" << std::endl;
        const char* pythonCommand = "python3";
        const char* pythonScript = "ILT_iVision_filter_track_calibration.py";
        const char* jsonFilePath = "/home/radian/LT_ivision_iLT/src/cam_calibration.json";

        std::string command = pythonCommand;
        command += " ";
        command += pythonScript;
        command += " ";
        command += jsonFilePath;

        int result = system(command.c_str());

        if (result == 0) {
            std::cout << "Successfully filtered the calib data" << std::endl;
            loadCalibrationFromJsonFile();
            std::cout << "Successfully loaded back the filtered calib data" << std::endl;
        } else {
            std::cout << "Filtering data failed" << std::endl;
        }
        return true;
    } else {
        std::cout << "Cannot read json file to filter" << std::endl;
        return false;
    }
}

std::vector<CalibrationPoint>& MovementCalculator::getParallaxCalibPoints() {
    return parallaxCalibPoints;
}

double MovementCalculator::findMedian(std::vector<float> a, int n) {
    if (n % 2 == 0) {
        nth_element(a.begin(), a.begin() + n / 2, a.end());
        nth_element(a.begin(), a.begin() + (n - 1) / 2, a.end());
        return (a[(n - 1) / 2] + a[n / 2]) / 2.0;
    } else {
        nth_element(a.begin(), a.begin() + n / 2, a.end());
        return a[n / 2];
    }
}