#include "SMRSearchControl.h"

#include <algorithm>
#include <math.h>
#include <iostream>

//TODO: basically everything here is tentative, so refine it more as I become more sure of how it should work, but also will need to do testing

SMRSearchControl::SMRSearchControl() {
	searchCount = 0;
	minSearchDistance = 0.1f;
	maxSearchDistance = 80.0f;
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
	isFirstIteration = true;
	wasHitByLaser = false;
	redBeamCount = 0;
	allowJog = true;
	lockedOnSMR = true;
}

//TODO: basically everything that happens when the target is not found, figure out what works for the red algorithm and if there are any special conditions for jogging or anything else
//laserImg angles are the point where the laser should be given the search distance, so you have to get the current search distance, give that to the movement calculator, get laser coordinates/angles, and then feed them back here...kinda messy but idk
void SMRSearchControl::updateSearch(bool locked, bool targetFound, bool smrHitByLaser, float laserImgAz, float laserImgEl, float smrImgAz, float smrImgEl, float smrDistance) {
	
	lockedOnSMR = locked;
	if (!locked) {
		//std::cout << "SMRSearchControl: search Dist: " << searchDistance <<  std::endl;
		updateSearchDistance(smrDistance, smrHitByLaser); //smr is blinded by laser, I still want the dist to be updated.

		if (smrHitByLaser)
			wasHitByLaser = true;

	}
	else {
		wasHitByLaser = false;
		redBeamCount = 0;
		isFirstIteration = true;
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

void SMRSearchControl::updateSearchDistance(float estimatedDistance, bool targetHit) {
    static float lowerBound = minSearchDistance; // Dynamic lower bound
    static float upperBound = maxSearchDistance; // Dynamic upper bound
    static int iterationCount = 0;               // Tracks number of iterations
    static int resetCount = 0;                   // Tracks number of resets
    static bool reverseDirection = false;        // Tracks the direction of bounds adjustment
    // Make epsilon distance-dependent
    float epsilon = std::max(0.5f, searchDistance * 0.2f); // 5% of distance, but at least 0.2
    const int maxIterations = 6;               // Maximum iterations before reset

    // Perform binary search: Narrow down bounds
    searchDistance = (lowerBound + upperBound) / 2.0f;

    // Debugging output
    // std::cout << "Iteration " << iterationCount
    //           << ": Search Distance = " << searchDistance
    //           << ", Lower Bound = " << lowerBound
    //           << ", Upper Bound = " << upperBound
    //           << ", Epsilon = " << epsilon
    //           << ", Reverse Direction = " << reverseDirection << std::endl;

    // Adjust bounds
    if (upperBound - lowerBound > epsilon) {
        // Narrow the bounds based on the midpoint
        if (!reverseDirection) {
            if (searchDistance > lowerBound) {
                lowerBound = searchDistance; // Shrinking the lower bound
            } else {
                upperBound = searchDistance; // Shrinking the upper bound
            }
        } else {
            // Reverse bounds adjustment logic


            if (searchDistance < upperBound) {
                upperBound = searchDistance; // Shrinking the upper bound
            } else {
                lowerBound = searchDistance; // Shrinking the lower bound
            }
        }
    } else {
       // std::cout << "Bounds too close! Attempting reset." << std::endl;

        // Reset bounds if they are too close or max iterations reached
        if (iterationCount >= maxIterations || upperBound - lowerBound < epsilon) {
            resetCount++;
            lowerBound = minSearchDistance;
            upperBound = maxSearchDistance;
            searchDistance = (lowerBound + upperBound) / 2.0f;
            iterationCount = 0; // Reset iteration counter

            // Reverse direction after every two resets
            if (resetCount % 2 == 0) {
                reverseDirection = !reverseDirection;
                //std::cout << "Reversing direction of bounds adjustment after reset #" << resetCount << std::endl;
            }

            // std::cout << "Bounds reset to -> "
            //           << "Lower = " << lowerBound
            //           << ", Upper = " << upperBound
            //           << ", Reverse Direction = " << reverseDirection << std::endl;
            return;
        }
    }

    // Increment iteration counter
    iterationCount++;
}