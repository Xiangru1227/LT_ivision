#include "SMRTracker.h"
#include <math.h>
#include <iostream>


SMRTracker::SMRTracker() {
	lastCamAz = 0;
	lastCamEl = 0;
}

SMRTracker::~SMRTracker() {

}



bool SMRTracker::smrNear(float imgAz, float imgEl, SMRData& smr) {

	std::lock_guard<std::mutex> guard(smrGuard);

	float minDistance = 9999.0f;
	int minIndex = -1;

	for (int i = 0; i < trackedSMRs.size(); i++) {
		float distAz = trackedSMRs[i].data.getImgAz() - imgAz;
		float distEl = trackedSMRs[i].data.getImgEl() - imgEl;
		float dist = std::sqrt(distAz * distAz + distEl * distEl);
		if (dist < minDistance) {
			minDistance = dist;
			minIndex = i;
			smr = trackedSMRs[i].data;
		}
	}
	return minIndex >= 0;
}


bool SMRTracker::smrShaking(SMRData& smr) {

	std::lock_guard<std::mutex> guard(smrGuard);

	for (int i = 0; i < trackedSMRs.size(); i++) {
		//get total recent distance traveled and the range it has traveled over
		cv::Point2f td = trackedSMRs[i].shake.GetTotalDistance();
		cv::Point2f size = trackedSMRs[i].shake.GetSize();
		float maxTd;
		float maxSize;
		if (td.x > td.y) {
			maxTd = td.x;
			maxSize = size.x;
		}
		else {
			maxTd = td.y;
			maxSize = size.y;
		}
		//std::cout << "TD: " << td << ", Size: " << size << std::endl;

		//if has traveled far enough but isn't just going in one direction, SMR is probably shaking, so start tracking it
		if ( maxTd > .5f && maxSize < .5f*maxTd ) {
			//std::cout << "SMR " << i << " is shaking with TD " << maxTd << "." << std::endl;
			smr = trackedSMRs[i].data;
			return true;
		}
	}
	return false;
}

//TODO: handle red SMRs better
//TODO: only allow matching to one tracked SMR; actually this is all kind of related, I think the minDistance below was smaller to avoid going to the wrong SMR when the correct SMR is obscured by red laser
void SMRTracker::updateTracking(std::vector<SMRData>& currentSMRs, float camAz, float camEl, cv::Point2f pixAng, std::vector<cv::Point2f> redSMRs, bool back_cam) {

	std::lock_guard<std::mutex> guard(smrGuard);

	float diffAz = camAz - lastCamAz;
	float diffEl = camEl - lastCamEl;
	
	clearSMRUpdates();
	for (int i = 0; i < currentSMRs.size(); i++) {
		float minDistance = 9999.0f;
		int minIndex = -1;
		//std::cout << "Current SMR: " << currentSMRs[i].getAz() << ", " << currentSMRs[i].getEl() << std::endl;
		for (int j = 0; j < trackedSMRs.size(); j++) {
			//std::cout << "Tracked SMR: " << trackedSMRs[j].data.getAz() << ", " << trackedSMRs[j].data.getEl() << std::endl;
			float dist = trackedSMRs[j].data.validDistance(currentSMRs[i], pixAng);	
			//std::cout << "Valid distance: " << dist << std::endl;
			if (dist >= 0 && dist < minDistance) {
				minIndex = j;
				minDistance = dist;
			}
		}
		if (minIndex >= 0) {
			//std::cout << "Updating SMR " << minIndex << " with " << i << std::endl;
			updateSMR(minIndex, currentSMRs[i]);
		}
		else {
			//std::cout << "Adding SMR " << i << std::endl;
			addSMR(currentSMRs[i]);
		}

	}

	if (!redSMRs.empty()) {
		//std::cout << "Handling red SMRs" << std::endl;
		blind_redlaserCount +=1;
		if (blind_redlaserCount > 10) {
			laserHitSMR = true; //don't let this get too high
			blind_redlaserCount = 0;
		}
	}
	handleUnmatched(diffAz, diffEl, redSMRs, back_cam);
	
	
	lastCamAz = camAz;
	lastCamEl = camEl;

	//std::cout << "Number of tracked SMRs: " << trackedSMRs.size() << std::endl;
	//int maxScoreIndex = -1;
	//float maxScore = -1;
	for (int i = 0; i < trackedSMRs.size(); i++) {
		//if (trackedSMRs[i].data.obscuredByLaser())
			//std::cout << "SMR " << i << " obscured by laser." << std::endl;
		
		//update SMR scores, probably make this more advanced at some point
		trackedSMRs[i].data.setScore((trackedSMRs[i].matchedCount - trackedSMRs[i].missingCount * 0.05f) * 0.2f);
		//if (trackedSMRs[i].data.getScore() > maxScore) {
		//	maxScore = trackedSMRs[i].data.getScore();
		//	maxScoreIndex = i;
		//}
		//std::cout << "Tracked: " << trackedSMRs[i].data.getAz() << ", " << trackedSMRs[i].data.getEl() << " - " << trackedSMRs[i].data.getScore() << std::endl;
	}
	//std::cout << "Tracked: " << trackedSMRs[maxScoreIndex].data.getAz() << ", " << trackedSMRs[maxScoreIndex].data.getEl() << " - " << trackedSMRs[maxScoreIndex].data.getScore() << std::endl;
}

void SMRTracker::clearTracking() {
	std::lock_guard<std::mutex> guard(smrGuard);

	trackedSMRs.clear();
}

std::vector<SMRData> SMRTracker::getCurrentTrackedSMRs() {

	std::lock_guard<std::mutex> guard(smrGuard);

	std::vector<SMRData> data;
	//std::cout << "Tracked SMRs: " << trackedSMRs.size() << std::endl;
	for (int i = 0; i < trackedSMRs.size(); i++) {
		//std::cout << "Matched Count: " << trackedSMRs[i].matchedCount << std::endl;
		if (trackedSMRs[i].matchedCount >= 2) {
			data.push_back(trackedSMRs[i].data);	
			//std::cout << "SMRs hit: " << trackedSMRs[i].data.obscuredByLaser() << std::endl;
			// std::cout << "Tracked SMR: Img (AZ, El) = (" << trackedSMRs[i].data.getImgAz() << ", " << trackedSMRs[i].data.getImgEl() << ")" 
			// 		  << " Abs (AZ,El() = (" << trackedSMRs[i].data.getAz()  << ", " << trackedSMRs[i].data.getEl()  << ")"
			// 		  << " Pixels:" << trackedSMRs[i].data.getImgPix() << ", Score:"<< trackedSMRs[i].data.getScore() << std::endl;
		}
	}
	return data;
}

cv::Point2f SMRTracker::getLastTrkAngles() {
	std::lock_guard<std::mutex> guard(smrGuard);
	return cv::Point2f(lastCamAz, lastCamEl);
}

void SMRTracker::updateSMR(int index, SMRData& latest) {
	trackedSMRs[index].data.updateTo(latest);
	if (!trackedSMRs[index].updated) {
		if (trackedSMRs[index].matchedCount < 5) {
			trackedSMRs[index].matchedCount++;
		}
		else if (trackedSMRs[index].missingCount > 0) {
			trackedSMRs[index].missingCount -= 10;
			if (trackedSMRs[index].missingCount < 0)
				trackedSMRs[index].missingCount = 0;
		}
		trackedSMRs[index].shake.AddAzEl(cv::Point2f(trackedSMRs[index].data.getAz(), trackedSMRs[index].data.getEl()));
	}
	trackedSMRs[index].updated = true;
}

void SMRTracker::clearSMRUpdates() {
	for (int i = 0; i < trackedSMRs.size(); i++) {
		trackedSMRs[i].updated = false;
	}
}

void SMRTracker::addSMR(SMRData& observed) {
	TrackedSMR ts;
	ts.data = observed;
	ts.missingCount = 0;
	ts.matchedCount = 1;
	ts.updated = true;
	ts.shake = ShakeSMR(observed.getAz(), observed.getEl());
	trackedSMRs.push_back(ts);
}
void SMRTracker::add_redSMR(std::vector<cv::Point2f>& red_observed) {
	TrackedSMR ts_red;
	
	trackedSMRs.push_back(ts_red);
}

void SMRTracker::handleUnmatched(float diffAz, float diffEl, std::vector<cv::Point2f>& redSMRs, bool back_cam) {
	for (int i = trackedSMRs.size() - 1; i >= 0; i--) {
		//std::cout << "Handling Unmatched SMRs"<< std::endl;
		if (!trackedSMRs[i].updated) {
			// std::cout << "Tracked SMR " << i << " missing count: " << trackedSMRs[i].missingCount << std::endl;
			// std::cout << "Tracked SMR " << i << " matched count: " << trackedSMRs[i].matchedCount << std::endl;
			//std::cout << "Can't see SMR, diff az: " << diffAz << ", diff el: " << diffEl << std::endl;
			//TODO: if red SMR is close, update data to say it is being hit by laser
			if (!trackedSMRs[i].data.obscuredByLaser()){
				trackedSMRs[i].missingCount += 5;
			}
			else 
				trackedSMRs[i].missingCount++;
			if (trackedSMRs[i].missingCount > 1 * trackedSMRs[i].matchedCount) {
				trackedSMRs.erase(trackedSMRs.begin() + i);
				//std::cout << "This SMR's missed count: " << trackedSMRs[i].missingCount << "  Matched Count: " << trackedSMRs[i].matchedCount << std::endl;
				//std::cout << "erase tracked SMR" << std::endl;
			}
			else {
				if(back_cam){
					trackedSMRs[i].data.setImgAz(trackedSMRs[i].data.getImgAz() - diffAz); // The sign is reversed on the Az only to enable back side image detection
					trackedSMRs[i].data.setImgEl(trackedSMRs[i].data.getImgEl() - diffEl);
				}
				else {
					
					trackedSMRs[i].data.setImgAz(trackedSMRs[i].data.getImgAz() + diffAz);
					trackedSMRs[i].data.setImgEl(trackedSMRs[i].data.getImgEl() - diffEl);
					//std::cout << "Tracked SMR Img Az: " << trackedSMRs[i].data.getImgAz() << " diff az: " << diffAz << std::endl;
					//std::cout << "Tracked SMR Img El: " << trackedSMRs[i].data.getImgEl() << " diff el: " << diffEl << std::endl;
				}
			}	

			for (int j = 0; j < redSMRs.size(); j++) {
				float redDiffAz = trackedSMRs[i].data.getImgAz() - redSMRs[j].x;
				float redDiffEl = trackedSMRs[i].data.getImgEl() - redSMRs[j].y;
				float redDist = redDiffAz * redDiffAz + redDiffEl * redDiffEl;
				// std::cout << "1. SMR: " << trackedSMRs[i].data.getImgAz() << ", " << trackedSMRs[i].data.getImgEl() << std::endl;
				// std::cout << "Distance to red reflection: " << redDiffAz << ", " << redDiffEl << " = " << redDist << std::endl;
				if (redDist < 40000.0f) {
					trackedSMRs[i].data.setLaserHit(true);
				}
			}
			
		}
		else {
			trackedSMRs[i].data.setLaserHit(false);
		}
	}
	// std::cout << "Tracked SMRs after handling unmatched: " << trackedSMRs.size() << std::endl;
	// std::cout << "Red SMRs: " << redSMRs.size() << " TrackedSMRs: " <<  trackedSMRs.size() << std::endl;
	if (redSMRs.size() > trackedSMRs.size() ){

		for (int j = 0; j < redSMRs.size(); j++) {
					float redDiffAz = /*trackedSMRs[i].data.getImgAz() -*/ redSMRs[j].x;
					float redDiffEl = /*trackedSMRs[i].data.getImgEl() -*/ redSMRs[j].y;
					float redDist = redDiffAz * redDiffAz + redDiffEl * redDiffEl;
					// std::cout << "SMR: " << trackedSMRs[j].data.getImgAz() << ", " << trackedSMRs[j].data.getImgEl() << std::endl;
					// std::cout << "Distance to red reflection: " << redDiffAz << ", " << redDiffEl << " = " << redDist << std::endl;
					if (redDist < .005f) {
						add_redSMR(redSMRs);
						//std::cout << "Adding Red SMRs to tracked SMRs" << std::endl;
						// redSMRs[j].red_data.setLaserHit(true);
						//trackedSMRs[j].data.setLaserHit(true);

					}
					//else
						//trackedSMRs[j].data.setLaserHit(false);
		}

	}
}