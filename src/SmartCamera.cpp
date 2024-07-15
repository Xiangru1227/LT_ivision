#include "SmartCamera.h"
#include <iostream>
#include <fstream>
#include <cstdlib>
#include "ImageProcessor.h"
#include "Debug.h"
#include <opencv2/core.hpp>

//constructor
SmartCamera::SmartCamera(iVisionClient *firmware): firmware_(firmware) {
	camCalib.setCalibHandle(&cHandle);
	moveCalc.setCalibHandle(&cHandle);
	tracker.setCalibrationManager(&camCalib);
	tracker.setCameraInterface(&cam);
}

//destructor (don't need to do anything now)
SmartCamera::~SmartCamera() {

}

//initializes the camera
bool SmartCamera::setup() {
	return cam.initCamera();
}

//starts running the video (and currently starts the detector pipeline in full mode, not sure where that should actually go)
bool SmartCamera::startVideo() {
	//std::cout << "Start Cam Status: " << cam.startCamera() << std::endl;
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
	prop.exposure = 5.0;
	prop.analog_gain = 1.5f;
	prop.digital_gain = 1.0f;
	setProperties(prop);
}

void SmartCamera::setRegCamProp(){
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
	prop.exposure = 25.0;
	prop.analog_gain = 2.8f;
	prop.digital_gain = 2.3f;
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




//return the most recent image from the camera (block until new image is received)
bool SmartCamera::grabNextJpeg(char** buf, unsigned long& data_size) {
	return cam.nextVideoImage(buf, data_size);
}

uint64_t SmartCamera::getTimeStamp() {
	return cam.present_time; 
}

std::vector<SMRData> SmartCamera::getTrackedSMRs() {
	return tracker.getCurrentTrackedSMRs();
}


//wait for new images for SMR tracking, update buffers, and get image timestamps
bool SmartCamera::updateSMRImages() {

	unsigned long firstTime, secondTime;
	if (!cam.updateProcessImages(firstTime, secondTime))
		return false;

	return true;
}

//process last two images to find SMRs, update the SMR tracker
bool SmartCamera::updateSMRTracking(float img1AzTop, float img1ElTop, float img1AzBot, float img1ElBot, float img2AzTop, float img2ElTop, float img2AzBot, float img2ElBot ) {
	
	//clearing the auto calibration buffer in Movement Calculator when the beam is broken
	moveCalc.reset_buf();
	
	float rad1AzTop = DegreeToRadian(img1AzTop);
	float rad1ElTop = DegreeToRadian(img1ElTop);
	float rad1AzBot = DegreeToRadian(img1AzBot);
	float rad1ElBot = DegreeToRadian(img1ElBot);

	float rad2AzTop = DegreeToRadian(img2AzTop);
	float rad2ElTop = DegreeToRadian(img2ElTop);
	float rad2AzBot = DegreeToRadian(img2AzBot);
	float rad2ElBot = DegreeToRadian(img2ElBot);
	
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

	cv::Point2f imgXY1Top = camCalib.singleXYFromAzEl(rad1AzTop - rad2AzTop, rad1ElTop - rad2ElTop, cam.getResolution());
	cv::Point2f imgXY1Bot = camCalib.singleXYFromAzEl(rad1AzBot - rad2AzBot, rad1ElBot - rad2ElBot, cam.getResolution());
	cv::Point2f imgXY2 = camCalib.singleXYFromAzEl(0, 0, cam.getResolution());

	ImgDelta id;
	id.topX = imgXY1Top.x - imgXY2.x;
	id.topY = imgXY1Top.y - imgXY2.y;
	id.botX = imgXY1Bot.x - imgXY2.x;
	id.botY = imgXY1Bot.y - imgXY2.y;
	std::vector<cv::Point2f> redSMRs;
	ImageSMRs imgsmrs = imProc.getCoordinates(src1Y, src1U, src1V, src2Y, src2U, src2V, false , id, redSMRs);

	if (!cam.releaseAllBuffers())
		std::cout << "Releasing buffers failed." << std::endl;

	for (int i = 0; i < redSMRs.size(); i++) {
		//std::cout << "Red SMR: " << redSMRs[i].x << ", " << redSMRs[i].y << std::endl;
	}

	//use information from image processing to determine which image/time/angles the observed SMRs correspond to
	std::vector<SMRData> observedSMRs;
	//unsigned long lastRelevantTime = 0;
	float camAz = 0;
	float camEl = 0;
	if (!imgsmrs.observed.empty()) {
	//if (false) {

		float azTop, elTop, azBot, elBot;

		if (imgsmrs.inFirstImage) {
			//lastRelevantTime = firstImgTime;
			camAz = (rad1AzTop + rad1AzBot) / 2;
			camEl = (rad1ElTop + rad1ElBot) / 2;
			azTop = rad1AzTop;
			elTop = rad1ElTop;
			azBot = rad1AzBot;
			elBot = rad1ElBot;
		}

		
		else {
			//lastRelevantTime = secondImgTime;
			camAz = (rad2AzTop + rad2AzBot) / 2;
			camEl = (rad2ElTop + rad2ElBot) / 2;
			azTop = rad2AzTop;
			elTop = rad2ElTop;
			azBot = rad2AzBot;
			elBot = rad2ElBot;
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
			float interp = imgsmrs.observed[i].imgY / imgRes.height;
			float azInterp = azBot * interp + azTop * (1 - interp);
			float elInterp = elBot * interp + elTop * (1 - interp);
			//std::cout << "SMR cam angles: " << smrCamAngles[i].x << ", " << smrCamAngles[i].y << std::endl;
			//std::cout << "Camera angles: " << camAz << ", " << camEl << " - SMR angles: " << azInterp << ", " << elInterp << std::endl;
			data.setAz(azInterp - data.getImgAz());
			data.setEl(elInterp + data.getImgEl());
			//std::cout << "AZ: " << azInterp << " - " << data.getImgAz() << " = " << data.getAz() <<  ", EL: " << elInterp << " + " << data.getImgEl() << " = " << data.getEl() << std::endl;
			data.setImgCoordinates(imgsmrs.observed[i].imgX, imgsmrs.observed[i].imgY, imgRes);
			observedSMRs.push_back(data);
		}
	}
	else {
		
		//lastRelevantTime = secondImgTime;
		camAz = (rad2AzTop + rad2AzBot) / 2;
		camEl = (rad2ElTop + rad2ElBot) / 2;

		//changin this to first angle set.
		camAz = (rad1AzTop + rad1AzBot) / 2;
		camEl = (rad1ElTop + rad1ElBot) / 2;
	}

	//std::cout << "Cam angles: " << camAz << ", " << camEl << std::endl;

	//TODO: convert red SMR coordinates to angles, remove any that aren't possibly the laser
	redSMRs = camCalib.azElFromXY(redSMRs, cam.getResolution());
	std::vector<cv::Point2f> laserSMRs;
	for (int i = 0; i < redSMRs.size(); i++) {
		float laser_distance = moveCalc.getLaserDistance(redSMRs[i]);
		if (laser_distance >= 0) {
			//std::cout << "Laser Dist: " << laser_distance << std::endl;
			laserSMRs.push_back(redSMRs[i]);
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

	//get current tracked SMRs and give data to targeter
	cv::Point2f trkAngles = tracker.getLastTrkAngles();
	TargetData target = targeter.findTargetData(tracker.getCurrentTrackedSMRs());
	
	//RT Last Valid Track distance from firmware
	float distance = firmware_->currentDistance() / 1000.0;
	//give targeter data to search control
	//std::cout << "target new?! " << target.isNew << "target hit? "<< target.hitByLaser << std::endl;

	auto jog_comm= moveCalc.getMovement(trkAngles.x, trkAngles.y, target.imgAz, target.imgEl, distance, target.hitByLaser, back_cam); 

	if (target.isNew && target.found){
		no_mv_cnt_ang = 0;
		no_mv_cnt_img = 0;
		searcher.startSearch(distance);
		SpiralInAction = false;
		//std::cout << "New Target distance " << distance << std::endl;
		std::cout << "Before Jog: Centroid: (" << target.imgAz << ", " << target.imgEl << "), Distance: (" << distance << "), Jog angles: (" << jog_comm.az << ", " << jog_comm.el << ")\n";
		return jog_comm;
	}

	//stop the spiral if it goes far from the target
	std::cout << "AZ: " << fabs(jog_comm.az - firmware_->currentAzimuth()) << " EL: " << fabs(jog_comm.el - firmware_->currentElevation()) << " Spiral _Thresh: " << spiral_thresh << " Spiral Cnt & Timoout: " << Spiral_counter << ", " << spiral_timeout <<std::endl;
	if (!target.hitByLaser && ((fabs(jog_comm.az - firmware_->currentAzimuth()) >= spiral_thresh) || (fabs(jog_comm.el - firmware_->currentElevation()) >= spiral_thresh) || Spiral_counter >= spiral_timeout) && firmware_->is_Spiral()){
		Spiral_counter = 0;
		firmware_->StopSearch();
	}

	//calculating the position of the laser pointer using the current search distance
	cv::Point2f laserPoint = moveCalc.getLaserPoint(searcher.getCurrentSearchDistance());
	//diff required to calculate the spiral az el
	float diffAz = laserPoint.x -  target.imgAz;
	float diffEl = laserPoint.y -  target.imgEl;

	searcher.updateSearch(target.trkLocked, target.found, target.hitByLaser, laserPoint.x, laserPoint.y, target.imgAz, target.imgEl, distance);

	spiral_dist = searcher.getLaserObservedDistance();
	//continue only if the difference between the destination jog and current jog are very close
	//if ((fabs(jog_comm.az - firmware_->currentAzimuth()) <= 0.05) && (fabs(jog_comm.el - firmware_->currentElevation()) <= 0.05)) {
		
		
		if(firmware_->is_Spiral()){
			Movement temp;
			temp.type = NoMove;
			temp.az = 0;
			temp.el = 0;
			temp.radius = 0;
			prev_trkAngles = trkAngles;
			//std::cout << "Searching in Progress, Az: " << temp.az << "EL: " << temp.el << std::endl;
			return temp;
		}	
		//Laser inside the SMR
		//std::cout << "Sprl reqd: " << searcher.jogRequired() << " sprl in progress: " << firmware_->SpiralInProgress << std::endl;
		else if (searcher.jogRequired() && target.hitByLaser) {
			no_mv_cnt_ang = 0;
			no_mv_cnt_img = 0;
			//TODO: figure out more legit way to calculate jog angles

			//std::cout<< "In Spiral Loop with dist: " << spiral_dist << std::endl;
			tracker.clearTracking();		
			Spiral_counter = 0;
			Movement temp;
			temp.type = Spiral;
			temp.az = RadianToDegree(trkAngles.x + diffAz);
			temp.el = RadianToDegree(trkAngles.y - diffEl);
			temp.radius = 0.2;
			prev_trkAngles = trkAngles;
			//std::cout << "Searching in Progress, Az: " << temp.az << "EL: " << temp.el << std::endl;
			return temp;
		}

		else if(target.found) {
			no_mv_cnt_ang = 0;
			no_mv_cnt_img = 0;
			std::cout << "Regular Jog" << std::endl;
			prev_trkAngles = trkAngles;
			return moveCalc.getMovement(trkAngles.x, trkAngles.y, target.imgAz, target.imgEl, searcher.getCurrentSearchDistance(), target.hitByLaser, back_cam);
		}


		else { //when target is not found
			std::cout << "tgt_fnd:" << target.found << " tgt_lkd:" << target.trkLocked << " Spiral:" << firmware_->is_Spiral() << " No_mov_img: " << no_mv_cnt_img << std::endl;
			no_mv_cnt_img++;
			Movement temp2;
			if(no_mv_cnt_img >= 5) {
				no_mv_cnt_img = 0;
				temp2.type = MoveBy;
				std::cout << "Curr: " << trkAngles.x << ", " << trkAngles.y << " Prev Ang: " << prev_trkAngles.x << ", " << prev_trkAngles.y << std::endl;
				if(trkAngles.y > prev_trkAngles.y)
					temp2.el = 0.05;
				else if (trkAngles.x > prev_trkAngles.x)
					temp2.az = 0.05;
				else if (trkAngles.y < prev_trkAngles.y)
					temp2.el = -0.05;
				else if (trkAngles.x < prev_trkAngles.x)
					temp2.az = -0.05;
				else {
					temp2.az = 0;
					temp2.el = 0.05;

				}
				temp2.radius = 0;
				prev_trkAngles = trkAngles;
				return temp2;
			}
			temp2.type = NoMove;
			temp2.az = 0; 
			temp2.el = 0;
			temp2.radius = 0.0;
			prev_trkAngles = trkAngles;
			return temp2;
		}
	//}
	// else {
	// 	std::cout << "Not reached tgt agl, Diff (" << fabs(jog_comm.az - firmware_->currentAzimuth())  << ", " << fabs(jog_comm.el - firmware_->currentElevation()) << ")" << std::endl;
	// 	usleep(1000);
	// 	no_mv_cnt_ang++;
	// 	Movement temp2;
	// 	Movement spi;
	// 	if(no_mv_cnt_ang == 5){ // to push back the laser towards the detected target if it is stuck 
	// 		if ((target.hitByLaser || searcher.redBeamCount > 3) && !firmware_->SpiralInProgress){
	// 			no_mv_cnt_ang = 0;
	// 			spi.type = Spiral;
	// 			spi.az = RadianToDegree(trkAngles.x + diffAz);
	// 			spi.el = RadianToDegree(trkAngles.y - diffEl);
	// 			spi.radius = 0.2;
	// 			std::cout << "Spiral needed: " << spi.az << ", " << spi.el << std::endl;
	// 			prev_trkAngles = trkAngles;
	// 			return spi;
	// 		}
	// 		else if(target.found && !firmware_->SpiralInProgress){
	// 			no_mv_cnt_ang = 0;
	// 			prev_trkAngles = trkAngles;
	// 			return moveCalc.getMovement(trkAngles.x, trkAngles.y, target.imgAz, target.imgEl, searcher.getCurrentSearchDistance(), target.hitByLaser, back_cam);
	// 		}
			
	// 	}
	// 	else if(no_mv_cnt_ang >= 7) { // in case the red beam is blocking the image detection for differential
	// 		no_mv_cnt_ang = 0;
	// 		temp2.type = MoveBy;
	// 		std::cout << "Curr: " << trkAngles.x << ", " << trkAngles.y << " Prev Ang: " << prev_trkAngles.x << ", " << prev_trkAngles.y << std::endl;
	// 		if(trkAngles.y > prev_trkAngles.y)
	// 			temp2.el = 0.05;
	// 		else if (trkAngles.x > prev_trkAngles.x)
	// 			temp2.az = 0.05;
	// 		else if (trkAngles.y < prev_trkAngles.y)
	// 			temp2.el = -0.05;
	// 		else if (trkAngles.x < prev_trkAngles.x)
	// 			temp2.az = -0.05;
	// 		else {
	// 			temp2.az = 0;
	// 			temp2.el = 0.05;

	// 		}
	// 		temp2.radius = 0;
	// 		prev_trkAngles = trkAngles;
	// 		return temp2;
	// 	}
	// 	temp2.type = NoMove; // do not move if the target angle is not reached
	// 	temp2.az = 0;
	// 	temp2.el = 0;
	// 	temp2.radius = 0.0;
	// 	prev_trkAngles = trkAngles;
	// 	return temp2;
	// }
}

//inform targeter that tracker has locked onto SMR (so that it can move to the next target and record the current target's last distance)
void SmartCamera::reportTrackerLock(bool locked, float az, float el, bool updateCalibration, float smrDistance, int stableDuration) {
	//smrAnglesFromLock(az, el, smrDistance);
	float az_tmp = az;
	float el_tmp = el;
	smrAnglesFromLock(az_tmp, el_tmp, smrDistance);
	targeter.reportLock(locked, az_tmp, el_tmp, smrDistance, DegreeToRadian(az), DegreeToRadian(el));
	if (locked)
		searcher.reportLock();
	if (updateCalibration) { // if the user wants  to enable auto calibratin, one can do so from the cam_calibration.json file
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
	std::cout << "Tracked SMRs: " << observedSmrs.size() << std::endl;
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







//load calibration data from file and initialize undistortion maps for quicker undistortion
//also load SMR angle/distance calibration
//also set image processing brightness threshold, Camera Properties and LED Flash settings
bool SmartCamera::loadCalibration() {
	if(!camCalib.loadCalibrationFromFile())
		return false;
	camCalib.initializeUndistortionMaps(cam.getResolution());
	if (!moveCalc.loadParallaxCalibrationFile())
		return false;
	CalibData cd;
	cHandle.getUpdatedCalibData(cd);
	imProc.setBrightThreshold(cd.bright_threshold); //setting brightness threshold for image detection from config file cam_calibration.json
	imProc.setRedThresholdY(cd.red_thresh_y);
	imProc.setRedThresholdCr(cd.red_thresh_cr);
	imProc.setGreenThresholdCr(cd.green_thresh_cr);
	imProc.setGreenThresholdCb(cd.green_thresh_cb);
	setProperties(cd.CamProp); // setting camera properties from config file cam_calibration.json
	Set_back_cam(cd.back_side_detection); // back camera detection control from config file
	Set_Spiral_Thresh(cd.spiral_threshold); //set the threshold for spiral search to stop(when away from the tracker)
	Set_Spiral_freq(cd.spiral_freq); //spiral_freq
	Set_Spiral_timeout(cd.spiral_timeout); // spiral_timeout
	Set_auto_calib(cd.cam_auto_calib); // set auto calib for camera parallax
	//setting flash control from config file cam_calibration.json
	firmware_->setFlashBrightness(cd.flash_brightness);
	firmware_->setFlashDuration(cd.flash_duration);
	firmware_->setFlashOffset(cd.flash_offset);
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
		if (!cam.nextProcessImage(y, false, u, false, v, false, rgba, true, false)) {
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

	cv::Mat y, u, v, rgba;
	if (!cam.nextProcessImage(y, true, u, true, v, true, rgba, false, true)) {
		return cv::Point3f(-1, -1, -1);
	}

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
	if (!cam.nextProcessImage(y, true, u, true, v, true, rgba, false, true)) {
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
	cam.nextProcessImage(y, false, u, false, v, false, rgba, true, false);
	return rgba;
}

//get Y component of next processing image, which is just the black and white image
cv::Mat SmartCamera::grabNextImageBW() {
	cv::Mat y, u, v, rgba;
	if (!cam.nextProcessImage(y, true, u, false, v, false, rgba, false, false)) {
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