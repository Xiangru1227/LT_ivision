#include "SMRTargeter.h"

#include <math.h>
#include <fstream>
#include <iostream>
#include "Utility.hpp"



StoredTarget::StoredTarget() {
	az = 0;
	el = 0;
	distance = -1;
	radius = 0.017f;
	data_from_lock = false;
	reached_in_cycle = false;
}

StoredTarget::StoredTarget(float a, float e, float d, float r) {
	az = a;
	el = e;
	distance = d;
	radius = r;
	data_from_lock = false;
	reached_in_cycle = false;
}

StoredTarget::~StoredTarget() {

}

void StoredTarget::setLockData(float a, float e, float d) {
	//update camera-relative az/el
	if (reached_in_cycle) {
		float diff_az = a - real_az;
		float diff_el = e - real_el;
		az += diff_az;
		el += diff_el;
	}
	real_az = a;
	real_el = e;
	//TODO: change this once weird ADM data is resolved
	if (d > 0.5f)
		distance = d;
	data_from_lock = true;
	reached_in_cycle = true;
}

bool StoredTarget::isSameAs(StoredTarget& other) {
	//float maxRadius = std::max(radius, other.getRadius());
	float distAz = az - other.getAz();
	float distEl = el - other.getEl();
	float angleDistance = std::sqrt(distAz * distAz + distEl * distEl);
	float ret_val = angleDistance < radius;
	//std::cout << "Stored Target - ang_dist: " << angleDistance << " return: " << ret_val << "\n"; 
	return angleDistance < radius;
}

void StoredTarget::updateTo(StoredTarget& other) {
	az = other.getAz();
	el = other.getEl();
	//if (distance <= 0)
	if (other.getRadius() > radius)
		radius = other.getRadius();
	if (!data_from_lock) {
		data_from_lock = other.isDataFromLock();
		distance = other.getDistance();
		real_az = other.real_az;
		real_el = other.real_el;
	}
}



SMRTargeter::SMRTargeter() {
	currentTarget = 0;
	targetNew = true;
	smrLocked = false;
	targetingEnabled = true;
	finishedAll = true;
	lastSingleSMRDistance = -1;
	targetsFinal = false;
}

SMRTargeter::~SMRTargeter() {

}


void SMRTargeter::clearTargets() {
	targets.clear();
	currentTarget = 0;
	targetNew = true;
	finishedAll = true;
	smrLocked = false;
	lastSingleSMRDistance = -1;
	targetsFinal = false;
}

void SMRTargeter::resetTargetLoop() {
	for (int i = 0; i < targets.size(); i++) {
		targets[i].setReachedInCycle(false);
	}
}


bool SMRTargeter::addTarget(float az, float el, float dist, bool locked, float ra, float re) {

	if (targetsFinal)
		return false;
	
	StoredTarget st (az, el, dist);
	std::cout << "Added " << az << ", " << el << std::endl;
	if (locked) {
		st.setLockData(ra, re, dist);
	}

	std::cout << "Targets: " << targets.size() << std::endl;
	for (int i = 0; i < targets.size(); i++) {
		if (st.isSameAs(targets[i])) {
			targets[i].updateTo(st);
			return false;
		}
	}

	targets.push_back(st);
	return true;
}


void SMRTargeter::finalizeTargets() {
	for (int i = 0; i < targets.size(); i++) {
		float minDistance = 10000;
		for (int j = 0; j < targets.size(); j++) {
			if (i != j) {
				float diffAz = targets[i].getAz() - targets[j].getAz();
				float diffEl = targets[i].getEl() - targets[j].getEl();
				float distance = std::sqrt(diffAz * diffAz + diffEl * diffEl);
				if (distance < minDistance) {
					minDistance = distance;
				}
			}
		}
		if (minDistance < 8000) {
			targets[i].setRadius(minDistance / 12);
		}
	}
	if (targets.size() > 0) {
		finishedAll = false;
		targetsFinal = true;
	}
		
}

void SMRTargeter::incrementTarget() {
	if (targets.size() > 0) {
		float min_dist = 100;
		int min_index = -1;
		for (int i = 0; i < targets.size(); i++) {
			float dist_az = targets[i].getAz() - targets[currentTarget].getAz();
			float dist_el = targets[i].getEl() - targets[currentTarget].getEl();
			float dist = dist_az * dist_az + dist_el * dist_el;
			if (!targets[i].hasBeenReachedInCycle() && dist < min_dist) {
				min_dist = dist;
				min_index = i;
			}
		}
		if (min_index >= 0) {
			currentTarget = min_index;
		}
		else {
			finishedAll = true;
		}
	}
	targetNew = true;
}

cv::Point2f SMRTargeter::findBasePosition(float az_offset, float el_offset) {
	//find centroid
	cv::Point2f centroid(0,0);
	for (int i = 0; i < targets.size(); i++) {
		std::cout << targets[i].getAz() << ", " << targets[i].getEl() << std::endl;
		centroid += cv::Point2f(targets[i].getAz(), targets[i].getEl());
	}
	centroid.x /= targets.size();
	centroid.y /= targets.size();
	centroid.x += az_offset;
	centroid.y -= el_offset;
	//std::cout << "Centroid: " << centroid << std::endl;

	int loop_count = 0;
	bool not_close = false;
	while (!not_close && loop_count < 10) {
		loop_count++;
		not_close = true;
		for (int i = 0; i < targets.size(); i++) {
			float diffaz = centroid.x - targets[i].getAz();
			float diffel = centroid.y - targets[i].getEl();
			float distance = std::sqrt(diffaz * diffaz + diffel * diffel);
			if (distance < targets[i].getRadius()) {
				not_close = false;
				centroid += cv::Point2f(diffaz * targets[i].getRadius() / distance, diffel * targets[i].getRadius() / distance);
				continue;
			}
		}
	}

	if (loop_count >= 10) {
		//std::cout << "Found bad base position." << std::endl;
	}
	else {
		//std::cout << "Found good base position." << std::endl;
	}
	
	return RadianToDegree(centroid);
}


bool SMRTargeter::finishedAllTargets() {
	int num_unfinished = 0;
	if (targets.size() > 0) {
		float min_dist = 100;
		for (int i = 0; i < targets.size(); i++) {
			float dist_az = targets[i].getAz() - targets[currentTarget].getAz();
			float dist_el = targets[i].getEl() - targets[currentTarget].getEl();
			float dist = dist_az * dist_az + dist_el * dist_el;
			if (!targets[i].hasBeenReachedInCycle() && dist < min_dist) {
				++num_unfinished;
			}
		}
	}
	std::cout << "Unfinished targets: " << num_unfinished << '\n';
	return num_unfinished == 0;
}

bool SMRTargeter::finishedAllTargetsExceptOne() {
	int num_unfinished = 0;
	if (targets.size() > 0) {
		float min_dist = 100;
		for (int i = 0; i < targets.size(); i++) {
			float dist_az = targets[i].getAz() - targets[currentTarget].getAz();
			float dist_el = targets[i].getEl() - targets[currentTarget].getEl();
			float dist = dist_az * dist_az + dist_el * dist_el;
			if (!targets[i].hasBeenReachedInCycle() && dist < min_dist) {
				++num_unfinished;
			}
		}
	}
	std::cout << "Unfinished targets: " << num_unfinished << '\n';
	return num_unfinished == 1;
}


int SMRTargeter::numTargets() {
	return targets.size();
}


void SMRTargeter::reportLock(bool locked, float az, float el, float distance, float real_az, float real_el) {
	/*if (smrLocked) {
		if (locked) {
			//update current target distance
			if (targets.size() > 0)
				targets[currentTarget].setDistance(distance);
			else
				lastSingleSMRDistance = distance;
		}
		else {
			smrLocked = false;
			//update target index
			if (targets.size() > 0) {
				currentTarget++;
				if (currentTarget >= targets.size()) {
					clearTargets();
				}
			}
			targetNew = true;
		}
	}
	else {
		smrLocked = locked;
	}*/
	if (smrLocked && !locked) {
		targetNew = true;
	}
	smrLocked = locked;
	if (locked) {
		//update current target distance
		if (targets.size() > 0) {
			targets[getLockedIndex(az, el)].setLockData(real_az, real_el, distance);
		}
		else {
			lastSingleSMRDistance = distance;
		}
	}
}


int SMRTargeter::getLockedIndex(float curr_az, float curr_el) {

	float min_dist = 100;
	int min_ind = -1;

	for (int i = 0; i < targets.size(); i++) {
		float diff_az = curr_az - targets[i].getAz();
		float diff_el = curr_el - targets[i].getEl();
		float dist = diff_az * diff_az + diff_el * diff_el;
		//std::cout << "Target: " << targets[i].getAz() << ", " << targets[i].getEl() << " - curr: " << curr_az << ", " << curr_el << std::endl;
		if (dist < min_dist) {
			min_dist = dist;
			min_ind = i;
		}
	}
	return min_ind;
}

//TODO: this if want to allow moving SMRs
void SMRTargeter::updateTargets(std::vector<SMRData> smrs) {

	std::vector<bool> target_done(targets.size(), false);
	std::vector<bool> smr_available(smrs.size(), true);

	//for each target, find closest SMRs in valid range
	std::vector<int> closest_smrs(targets.size(), -1);
	for (int i = 0; i < targets.size(); i++) {
		float min_dist = 0.034f;
		int min_idx = -1;
		for (int j = 0; j < smrs.size(); j++) {
			float distaz = targets[i].getAz() - smrs[j].getAz();
			float distel = targets[i].getEl() - smrs[j].getEl();
			float dist = std::sqrt(distaz * distaz + distel * distel);
			if (dist < min_dist) {
				min_dist = dist;
				min_idx = j;
			}
		}
		if (min_idx >= 0) {
			closest_smrs[i] = min_idx;
		}
	}

	bool all_targets_done = false;

	while (!all_targets_done) {

		for (int i = 0; i < targets.size(); i++) {
			if (!target_done[i]) {
				if (closest_smrs[i] < 0) {
					target_done[i] = true;
				}
				else {
					bool conflict = false;
					for (int j = 0; j < targets.size(); j++) {
						if (i != j && closest_smrs[i] == closest_smrs[j]) {
							conflict = true;
						}
					}
					if (!conflict) {
						target_done[i] = true;
						smr_available[closest_smrs[i]] = false;
					}
				}
			}
		}

		for (int i = 0; i < targets.size(); i++) {
			if (!target_done[i]) {
				if (closest_smrs[i] >= 0) {
					for (int j = 0; j < targets.size(); j++) {
						if (i != j && closest_smrs[i] == closest_smrs[j]) {
							int second_closest1 = closestSMR(smrs, i, closest_smrs[i], smr_available);
							int second_closest2 = closestSMR(smrs, j, closest_smrs[j], smr_available);
							if (second_closest1 < 0 && second_closest2 < 0) {
								float distaz1 = targets[i].getAz() - smrs[closest_smrs[i]].getAz();
								float distel1 = targets[i].getEl() - smrs[closest_smrs[i]].getEl();
								float dist1 = std::sqrt(distaz1 * distaz1 + distel1 * distel1);

								float distaz2 = targets[j].getAz() - smrs[closest_smrs[j]].getAz();
								float distel2 = targets[j].getEl() - smrs[closest_smrs[j]].getEl();
								float dist2 = std::sqrt(distaz2 * distaz2 + distel2 * distel2);

								if (dist1 < dist2) {
									closest_smrs[j] = -1;
								}
								else {
									closest_smrs[i] = -1;
								}
							}
							else if (second_closest1 < 0) {
								closest_smrs[j] = second_closest2;
							}
							else if (second_closest2 < 0) {
								closest_smrs[i] = second_closest1;
							}
							else {
								float distaz1_1 = targets[i].getAz() - smrs[closest_smrs[i]].getAz();
								float distel1_1 = targets[i].getEl() - smrs[closest_smrs[i]].getEl();
								float dist1_1 = std::sqrt(distaz1_1 * distaz1_1 + distel1_1 * distel1_1);

								float distaz1_2 = targets[i].getAz() - smrs[second_closest1].getAz();
								float distel1_2 = targets[i].getEl() - smrs[second_closest1].getEl();
								float dist1_2 = std::sqrt(distaz1_2 * distaz1_2 + distel1_2 * distel1_2);

								float distaz2_1 = targets[j].getAz() - smrs[closest_smrs[j]].getAz();
								float distel2_1 = targets[j].getEl() - smrs[closest_smrs[j]].getEl();
								float dist2_1 = std::sqrt(distaz2_1 * distaz2_1 + distel2_1 * distel2_1);

								float distaz2_2 = targets[j].getAz() - smrs[second_closest2].getAz();
								float distel2_2 = targets[j].getEl() - smrs[second_closest2].getEl();
								float dist2_2 = std::sqrt(distaz2_2 * distaz2_2 + distel2_2 * distel2_2);

								if (dist1_1 + dist2_2 < dist1_2 + dist2_1) {
									closest_smrs[j] = second_closest2;
								}
								else {
									closest_smrs[i] = second_closest1;
								}
							}
						}
					}
				}
			}
		}

		//check for completion
		all_targets_done = true;
		for (int i = 0; i < target_done.size(); i++) {
			if (!target_done[i]) {
				all_targets_done = false;
				break;
			}
		}
	}

	for (int i = 0; i < targets.size(); i++) {
		if (closest_smrs[i] >= 0) {
			StoredTarget st (smrs[closest_smrs[i]].getAz(), smrs[closest_smrs[i]].getEl(), -1);
			targets[i].updateTo(st);	
		}
	}

	//if none, don't update target

	//for all with no conflict, associate target with SMR and remove SMR from list

	//for each conflict, find second closest SMRs, compare total distance for each combination
	//(if one has no second SMR in valid range, only one configuration possible)
	//if neiher has second SMR in valid range, either don't update anything, associate the closest pair, or base on movement of other targets
	//if second closest is the same for both, do the same as normal, so maybe nvm

	//continue until all targets assigned (or skipped according to algorithm)




	// std::vector<std::vector<int>> target_to_smr(targets.size(), std::vector<int>());
	// for (int i = 0; i < smrs.size(); i++) {
	// 	StoredTarget st (smrs[i].getAz(), smrs[i].getEl(), -1);
	// 	for (int j = 0; j < targets.size(); j++) {
	// 		if (targets[j].isSameAs(st)) {
	// 			//std::cout << "Old target " << j << ": " << targets[j].getAz() << ", " << targets[j].getEl() << std::endl;
	// 			//targets[j].updateTo(st);
	// 			//std::cout << "New target " << j << ": " << targets[j].getAz() << ", " << targets[j].getEl() << std::endl;
	// 			target_to_smr[j].push_back(i);
	// 		}
	// 	}
	// }
	// bool finished = false;
	// bool no_ones, all_zeros;
	// while (!finished) {
	// 	int tgt_idx = -1;
	// 	int smr_idx = -1;
	// 	no_ones = true;
	// 	for (int i = 0; i < target_to_smr.size(); i++) {
	// 		if (target_to_smr[i].size() == 1) {
	// 			tgt_idx = i;
	// 			smr_idx = target_to_smr[i][0];
	// 			no_ones = false;
	// 			break;
	// 		}
	// 	}
	// 	if (no_ones) {

	// 	}
	// 	else {
	// 		for (int i = 0; i < target_to_smr.size(); i++) {
	// 			for (int j = 0; j < target_to_smr[i].size(); j++) {
	// 				//assumes each SMR index can only appear once for each target index, which should always be true, but make sure
	// 				if (target_to_smr[i][j] == smr_idx) {
	// 					target_to_smr[i].erase(target_to_smr[i].begin() + j);
	// 					break;
	// 				}
	// 			}
	// 		}
	// 	}
	// }
	for (int j = 0; j < targets.size(); j++) {
		//std::cout << "New target " << j << ": " << targets[j].getAz() << ", " << targets[j].getEl() << std::endl;
	}
}


int SMRTargeter::closestSMR(std::vector<SMRData>& smrs, int target_num, int ignore_num, std::vector<bool>& smr_available) {
	float min_dist = 0.034f;
	int min_idx = -1;
	for (int j = 0; j < smrs.size(); j++) {
		float distaz = targets[target_num].getAz() - smrs[j].getAz();
		float distel = targets[target_num].getEl() - smrs[j].getEl();
		float dist = std::sqrt(distaz * distaz + distel * distel);
		if (dist < min_dist && j != ignore_num && smr_available[j]) {
			min_dist = dist;
			min_idx = j;
		}
	}
	return min_idx;
}



float SMRTargeter::totalDistance(std::vector<SMRData>& smrs, std::vector<int> index) {
	float sum = 0;
	int num = 0;
	for (int i = 0; i < index.size(); i++) {
		if (index[i] >= 0) {
			float diffaz = smrs[index[i]].getAz() - targets[i].getAz();
			float diffel = smrs[index[i]].getEl() - targets[i].getEl();
			sum += diffaz * diffaz + diffel * diffel;
			num++;
		}
	}
	if (num > 0)
		return sum / num;
	else
		return 1000;
}


//TODO: figure out the isNew stuff?  but probably lots of changes, really
//TODO: could add ability to update az/el of targets once you've matched the target
TargetData SMRTargeter::findTargetData(std::vector<SMRData> smrs) {

	if (!targetingEnabled) {
		TargetData temp;
		temp.found = false;
		temp.imgAz = 0;
		temp.imgEl = 0;
		temp.distance = -1;
		temp.isNew = false;
		temp.trkLocked = smrLocked;
		temp.hitByLaser = false;
		temp.numPix = 0;
		return temp;	
	}

	if (targets.empty()) {
		//std::cout << "target empty" << "\n";
		if (smrs.empty()) {
			TargetData temp;
			temp.found = false;
			temp.imgAz = 0;
			temp.imgEl = 0;
			temp.distance = -1;
			temp.isNew = false;
			temp.trkLocked = smrLocked;
			temp.hitByLaser = false;
			temp.numPix = 0;
			//std::cout << "SMRs empty" << "\n";
			return temp;
		}
		else {
			//std::cout << "SMRs not empty" << "\n";
			float maxScore = -100.0f;
			int maxIndex = 0;
			for (int i = 0; i < smrs.size(); i++) {
				float score = smrs[i].getScore();
				if (score > maxScore) {
					maxScore = score;
					maxIndex = i;
				}
			}
			TargetData temp;
			temp.found = true;
			temp.imgAz = smrs[maxIndex].getImgAz();
			temp.imgEl = smrs[maxIndex].getImgEl();
			temp.distance = lastSingleSMRDistance;
			temp.isNew = targetNew;
			temp.trkLocked = smrLocked;
			temp.hitByLaser = smrs[maxIndex].obscuredByLaser();
			temp.numPix = smrs[maxIndex].getImgPix();
			if (targetNew) {
				targetNew = false;
			}
			return temp;
		}
	}

	//std::cout << "target not empty" << " SMRs size: " << smrs.size() << std::endl;
	float minDistance = 2.0;
	int minIndex = -1;

	for (int i = 0; i < smrs.size(); i++) {
		float distAz = smrs[i].getAz() - targets[currentTarget].getAz();
		float distEl = smrs[i].getEl() - targets[currentTarget].getEl();
		float angleDist = std::sqrt(distAz * distAz + distEl * distEl);
		//std::cout << "AngleDist: " << angleDist << std::endl;
		if (angleDist < minDistance) {
			minDistance = angleDist;
			minIndex = i;
		}
	}

	//std::cout << "Min index: " << minIndex << std::endl;
	if (minIndex >= 0) {
		TargetData temp;
		temp.found = true;
		temp.imgAz = smrs[minIndex].getImgAz();
		temp.imgEl = smrs[minIndex].getImgEl();
		temp.distance = targets[currentTarget].getDistance();
		temp.isNew = targetNew;
		if (targetNew) {
			targetNew = false;
		}
		temp.trkLocked = smrLocked;
		temp.hitByLaser = smrs[minIndex].obscuredByLaser();
		return temp;
	}
	else {
		TargetData temp;
		temp.found = false;
		temp.imgAz = 0;
		temp.imgEl = 0;
		temp.distance = -1;
		temp.isNew = false;
		temp.trkLocked = smrLocked;
		temp.hitByLaser = false;
		return temp;
	}
}


bool SMRTargeter::saveTargetsToFile() {
	//open file
	std::ofstream teachFile;
	teachFile.open("teachingPoints.txt");
	
	//write teaching point values to file
	for (int i = 0; i < targets.size(); i++) {
		teachFile << targets[i].getAz() << " " << targets[i].getAz() << " " << targets[i].getDistance() << " " << targets[i].getRadius() << std::endl;
	}

	//close file
	teachFile.close();
	return true;
}

bool SMRTargeter::loadTargetsFromFile() {
	clearTargets();

	//open file
	std::ifstream teachFile;
	teachFile.open("teachingPoints.txt");

	//read distances and image coordinates
	if(teachFile.is_open()) {
		float az, el, dist, rad;
		while (!teachFile.eof()) {
			teachFile >> az >> el >> dist >> rad;
			targets.push_back(StoredTarget(az, el, dist, rad));
		}
		teachFile.close();
		return true;
	}
	else {
		std::cout << "Couldn't open teaching file." << std::endl;
		return false;
	}
}
