#include "SMRSearchControl.h"

#include <algorithm>
#include <math.h>
#include <iostream>

//TODO: basically everything here is tentative, so refine it more as I become more sure of how it should work, but also will need to do testing

SMRSearchControl::SMRSearchControl() {
	searchCount = 0;
	minSearchDistance = 1.2f;
	maxSearchDistance = 35.0f;
	searchDistance = maxSearchDistance;
	wantTarget = false;
	//targetMissingCount = 0;
	//shouldUseRed = false;
	wasHitByLaser = false;
	searchRadius = 0;
	allowJog = true;
	lockedOnSMR = false;
}

SMRSearchControl::~SMRSearchControl() {

}

void SMRSearchControl::startSearch() {
	//std::cout << "Restarting search." << std::endl;
	searchCountELMinus = 5;
	searchCount = 5;
	searchCountELPlus = 5;
	searchDistance = maxSearchDistance;
	wantTarget = true;
	//targetMissingCount = 0;
	redBeamCount = 0;
	wasHitByLaser = false;
	searchRadius = 0;
	allowJog = true;
	lockedOnSMR = false;
}

void SMRSearchControl::startSearch(float distanceGuess) {
	//std::cout << "Starting search from guess: " << distanceGuess << std::endl;
	searchCountELMinus = (distanceGuess > minSearchDistance) ? 0 : 5;
	searchCount = (distanceGuess > minSearchDistance) ? 0 : 5;
	searchCountELPlus = (distanceGuess > minSearchDistance) ? 0 : 5;
	searchDistance = (distanceGuess > minSearchDistance) ? distanceGuess * 1.04f : maxSearchDistance;
	wantTarget = true;
	//targetMissingCount = 0;
	redBeamCount = 0;
	wasHitByLaser = false;
	searchRadius = 0;
	allowJog = true;
	lockedOnSMR = false;
}

void SMRSearchControl::reportLock() {
	//std::cout << "Reported lock to search control." << std::endl;
	wasHitByLaser = false;
	redBeamCount = 0;
	allowJog = true;
	lockedOnSMR = true;
}

//TODO: basically everything that happens when the target is not found, figure out what works for the red algorithm and if there are any special conditions for jogging or anything else
//laserImg angles are the point where the laser should be given the search distance, so you have to get the current search distance, give that to the movement calculator, get laser coordinates/angles, and then feed them back here...kinda messy but idk
void SMRSearchControl::updateSearch(bool locked, bool targetFound, bool smrHitByLaser, float laserImgAz, float laserImgEl, float smrImgAz, float smrImgEl, float smrDistance) {
	/*if (!locked) {
		if (targetFound) {
			//not sure if these will be used, one more thing to figure out later
			float laserDistAz = std::abs(smrImgAz - laserImgAz);
			float laserDistEl = std::abs(smrImgEl - laserImgEl);
			float minDist = (smrHitByLaser) ? .01f : .005f;
			//std::cout << "SMR img EL: " << smrImgEl << ", laser img EL: " << laserImgEl << std::endl;
			//std::cout << "AZ dist: " << laserDistAz << ", EL dist: " << laserDistEl << std::endl;
			if (laserDistAz < minDist && laserDistEl < minDist && !wasHitByLaser) {
				updateSearchDistance(smrDistance, smrHitByLaser);
			}
			lastSMRAz = smrImgAz;
			lastSMREl = smrImgEl;
			targetMissingCount = 0;
			//this probably shouldn't go here, should be more of a fixed number of red detections, but figure that out later
			shouldUseRed = false;
			wasHitByLaser = smrHitByLaser;
		}
		else {
			if (targetMissingCount < 9)
				targetMissingCount++;
			//TODO: change condition here, should check that smr image angles are possibly close to laser (or maybe just don't check them at all)
			if (targetMissingCount > 4 && smrImgAz == smrImgEl) {
				shouldUseRed = true;
			}
		}
	}
	else {
		//std::cout << "Updated search with SMR locked." << std::endl;
		wasHitByLaser = false;
	}
	if (wasHitByLaser) {
		searchRadius = 0.1f;
	}
	else {
		searchRadius = 0;
	}*/
	lockedOnSMR = locked;
	if (!locked) {
		float laserDistAz = std::abs(smrImgAz - laserImgAz);
		float laserDistEl = std::abs(smrImgEl - laserImgEl);
		float minDist = .04f;
		//std::cout << "SMR img EL: " << smrImgEl << ", laser img EL: " << laserImgEl << std::endl;
		//std::cout << "AZ dist: " << laserDistAz << ", EL dist: " << laserDistEl << std::endl;
		

		// if(smrImgEl > laserImgEl)
		// 	updateSearchDistance(smrDistance, smrHitByLaser, false, true); //guiding which direction the laser should jog to search for SMR
		// else if (smrImgEl < laserImgEl)
		// 	updateSearchDistance(smrDistance, smrHitByLaser, true, false);

		if (laserDistAz < minDist && laserDistEl < minDist) {
			//std::cout << "Updated search distance with smrdistance: " << smrDistance << std::endl;
			updateSearchDistance(smrDistance, smrHitByLaser);
		}
		else 
			searchDistance = smrDistance;
	

		if (smrHitByLaser)
			wasHitByLaser = true;

		// if (wasHitByLaser) {
		// 	redBeamCount++;
		// }
		// else {
		// 	redBeamCount = 0;
		// }
	}
	else {
		wasHitByLaser = false;
		redBeamCount = 0;
	}
}


void SMRSearchControl::laserObserved() {
	if (/*allowJog && */!lockedOnSMR) {
		redBeamCount++;
		//std::cout<< "redBeamCount: " << redBeamCount << std::endl;
	}
}

void SMRSearchControl::smrObserved() {
	allowJog = true;
}


bool SMRSearchControl::jogRequired() {
	//if (wasHitByLaser)
	//	std::cout << "Hit by laser and missing count = " << targetMissingCount << std::endl;
	//else
	//	std::cout << "Not hit by laser and missing count = " << targetMissingCount << std::endl;
	int countThresh = (wasHitByLaser) ? 4 : 8;
	if (redBeamCount > countThresh && allowJog && !lockedOnSMR) {
		redBeamCount = 0;
		allowJog = false;
		return true;
	}
	return false;
	//return wasHitByLaser && targetMissingCount > 0 && targetMissingCount % 4 == 0;
}

float SMRSearchControl::getCurrentSearchDistance() {
	return searchDistance;
}

float SMRSearchControl::getCurrentSearchRadius() {
	return searchRadius;
}

float SMRSearchControl::DistEstimatePow(int x, int nmode )
{
	//testing the function to get an estimated distance, I know a lot of constants, figuring out what those are. eventually needs to be replaced by actual formula of calculating distance depending on the reflection diameter
	float dis = -1;
	//std::cout << "Nm Pix: " << x << std::endl;
	if (x>0)
	{
		switch (nmode)  // not sure what these numbers mean. Testing from the old iVision
		{
		case 0: // use hollow SMR prediction equation  
			dis = (float) 1000*pow(x,-0.853);
			break;
		case 1: // use Solid SMR prediction equation
			dis = (float) 1000*pow(x,-0.755);
			break;
		case 2: // use Average prediction equation
			dis = (float) 1000*pow(x,-1.3);
			break;
		case 3: // this case is when using the core pixels with solid
			dis = (float) 1000*pow(x,-0.903);
			break;
		//default: // use hollow SMR prediction equation  
		//	dis = (int) 19426*pow(x,-0.853);
		//	break;
		}
		// distance estimate cannot be more than the farmost desined calibration distance which is 30 meters
		if (dis > MAXIMUM_EST_DISTANCE)
		{
			dis = MAXIMUM_EST_DISTANCE;
		}
		else if (dis < m_NearCalibDistance)
			dis = m_NearCalibDistance;
	}
	else
	{
		std::cout << "Number of pixels cannot be zero or negative value" << std::endl;
	}
	//std::cout << "Dist Est: " << dis << std::endl;
	return dis;
}

//TODO: will probably need adjustments, but won't know until testing
void SMRSearchControl::updateSearchDistance(float estimatedDistance, bool smrClose, bool dist_inc, bool dist_dec) {
	float searchConstant = (smrClose) ? .12f : 1.2f;
	float searchScalar = (smrClose) ? .02f : .06f;

	// if(dist_dec){ //if the laser pointer is below the smr
	// 	std::cout << "Incr search dist, elMinus cnt: " << searchCountELMinus << std::endl;
	// 	if (estimatedDistance > 0 && (searchCountELMinus == 0 || searchCountELMinus == 1)) {
	// 		if (searchDistance > estimatedDistance - 2.0f && searchDistance > minSearchDistance) {
	// 			searchDistance = std::max(searchDistance - searchConstant  - searchScalar*searchDistance, estimatedDistance - 2.0f);
	// 			searchDistance = std::max(searchDistance, minSearchDistance);
	// 			searchCountELMinus++;
	// 		}
	// 		else {
	// 			searchCountELMinus++;
	// 		}
	// 	}
	// 	else if (estimatedDistance > 0 && (searchCountELMinus == 2 || searchCountELMinus == 3)) {
	// 		if (searchDistance > estimatedDistance - 4.0f && searchDistance > minSearchDistance) {
	// 			searchDistance = std::max(searchDistance - searchConstant  - searchScalar*searchDistance, estimatedDistance - 4.0f);
	// 			searchDistance = std::max(searchDistance, minSearchDistance);
	// 			searchCountELMinus++;
	// 		}
	// 		else {
	// 			searchCountELMinus++;
	// 		}
	// 	}
	// 	else {
	// 		if (searchDistance > minSearchDistance) {
	// 			searchDistance = std::max(searchDistance - searchConstant - searchScalar*searchDistance, estimatedDistance - 6.0f);
	// 			searchDistance = std::max(searchDistance, minSearchDistance);
	// 		}
	// 	}

	// }

	// else if(dist_inc){ // if the laser pointer is above the SMR
	// std::cout << "Decr search dist, elPlus cnt: " << searchCountELMinus << std::endl;
	// 	if (estimatedDistance > 0 && (searchCountELPlus == 0 || searchCountELPlus == 1)) {
	// 		if (searchDistance < estimatedDistance + 2.0f && searchDistance < maxSearchDistance) {
	// 			searchDistance = std::min(searchDistance + searchConstant + searchScalar*searchDistance, estimatedDistance + 2.0f);
	// 			searchDistance = std::min(searchDistance, maxSearchDistance);
	// 			searchCountELPlus++;
	// 		}
	// 		else {
	// 			searchCountELPlus++;
	// 		}
	// 	}
	// 	else if (estimatedDistance > 0 && (searchCountELPlus == 2 || searchCountELPlus == 3)) {
	// 		if (searchDistance < estimatedDistance + 4.0f && searchDistance < maxSearchDistance) {
	// 			searchDistance = std::min(searchDistance + searchConstant + searchScalar*searchDistance, estimatedDistance + 4.0f);
	// 			searchDistance = std::min(searchDistance, maxSearchDistance);
	// 			searchCountELPlus++;
	// 		}
	// 		else {
	// 			searchCountELPlus++;
	// 		}
	// 	}
	// 	else{
	// 		if (searchDistance < maxSearchDistance) {
	// 			searchDistance = std::min(searchDistance + searchConstant + searchScalar*searchDistance, estimatedDistance + 6.0f);
	// 			searchDistance = std::max(searchDistance, minSearchDistance);
	// 		}
			
	// 	}

	//}

	if (estimatedDistance > 0 && (searchCount == 0 || searchCount == 2)) {
		if (searchDistance > estimatedDistance - 2.0f && searchDistance > minSearchDistance) {
			searchDistance = std::max(searchDistance - searchConstant  - searchScalar*searchDistance, estimatedDistance - 2.0f);
			searchDistance = std::max(searchDistance, minSearchDistance);
		}
		else {
			searchCount++;
		}
	}
	else if (estimatedDistance > 0 && (searchCount == 1 || searchCount == 3)) {
		if (searchDistance < estimatedDistance + 2.0f && searchDistance < maxSearchDistance) {
			searchDistance = std::min(searchDistance + searchConstant + searchScalar*searchDistance, estimatedDistance + 2.0f);
			searchDistance = std::min(searchDistance, maxSearchDistance);
		}
		else {
			searchCount++;
		}
	}
	else if (estimatedDistance > 0 && (searchCount == 4 || searchCount == 6)) {
		if (searchDistance > estimatedDistance - 4.0f && searchDistance > minSearchDistance) {
			searchDistance = std::max(searchDistance - searchConstant  - searchScalar*searchDistance, estimatedDistance - 4.0f);
			searchDistance = std::max(searchDistance, minSearchDistance);
		}
		else {
			searchCount++;
		}
	}
	else if (estimatedDistance > 0 && (searchCount == 5 || searchCount == 7)) {
		if (searchDistance < estimatedDistance + 4.0f && searchDistance < maxSearchDistance) {
			searchDistance = std::min(searchDistance + searchConstant + searchScalar*searchDistance, estimatedDistance + 4.0f);
			searchDistance = std::min(searchDistance, maxSearchDistance);
		}
		else {
			searchCount++;
		}
	}

	else if (searchCount % 2 == 0) {
		//float searchConstant = -.8f;
		//float searchScalar = -.03f;
		if (searchDistance > minSearchDistance) {
			searchDistance = std::max(searchDistance - searchConstant - searchScalar*searchDistance, minSearchDistance);
			searchDistance = std::max(searchDistance, minSearchDistance);
		}
		else {	
			searchDistance = std::min(searchDistance + searchConstant + searchScalar*searchDistance, maxSearchDistance);
			searchDistance = std::min(searchDistance, maxSearchDistance);
			searchCount++;
		}
	}
	else {
		//float searchConstant = .8f;
		//float searchScalar = .03f;
		if (searchDistance < maxSearchDistance) {
			searchDistance = std::min(searchDistance + searchConstant + searchScalar*searchDistance, maxSearchDistance);
			searchDistance = std::max(searchDistance, minSearchDistance);
		}
		else {
			searchDistance = std::max(searchDistance - searchConstant - searchScalar*searchDistance, minSearchDistance);
			searchDistance = std::min(searchDistance, maxSearchDistance);
			searchCount++;
		}
	}
	//std::cout << "Search count: " << searchCount << ", search distance: " << searchDistance << std::endl;
}
