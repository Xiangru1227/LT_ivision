#pragma once

#include <vector>
#include "SMRData.h"



struct TargetData {
	bool found;
	float imgAz;
	float imgEl;
	float distance;
	bool isNew;
	bool trkLocked;
	bool hitByLaser;
	int numPix;
};



class StoredTarget {
public:
	StoredTarget();
	StoredTarget(float a, float e, float d = -1, float r = 0.017f);
	~StoredTarget();

	float getAz() { return az; };
	float getEl() { return el; };
	float getRealAz() { return real_az; };
	float getRealEl() { return real_el; };
	float getDistance() { return distance; };
	float getRadius() { return radius; };
	bool isDataFromLock() { return data_from_lock; };
	bool hasBeenReachedInCycle() { return reached_in_cycle; };

	void setAz(float a) { az = a; };
	void setEl(float e) { el = e; };
	void setDistance(float d) { distance = d; };
	void setRadius(float r) { radius = r; };
	void setDataFromLock(bool l) { data_from_lock = l; };
	void setReachedInCycle(bool c) { reached_in_cycle = c; };

	void setLockData(float a, float e, float d);
	bool isSameAs(StoredTarget& other);
	void updateTo(StoredTarget& other);
private:
	float az;
	float el;
	float real_az;
	float real_el;
	float distance;
	float radius;
	bool data_from_lock;
	bool reached_in_cycle;
};



class SMRTargeter {
public:
	SMRTargeter();
	~SMRTargeter();

	//need functions for configuring targeter
	void clearTargets();
	void resetTargetLoop();
	bool addTarget(float az, float el, float dist, bool locked = false, float ra = 0, float re = 0);
	void finalizeTargets();
	cv::Point2f findBasePosition(float az_offset, float el_offset);
	int numTargets();
	void incrementTarget();

	void updateTargets(std::vector<SMRData> smrs);

	//function for reporting SMR lock
	void reportLock(bool locked, float az, float el, float distance, float real_az, float real_el);
	int getLockedIndex(float curr_az, float curr_el);
	
	void enable() { targetingEnabled = true; };
	void disable() { targetingEnabled = false; };

	TargetData findTargetData(std::vector<SMRData> smrs);
	int currentTargetIndex() { return currentTarget; };
	bool finishedAllTargets();
	bool finishedAllTargetsExceptOne();

	bool saveTargetsToFile();
	bool loadTargetsFromFile();
private:
	std::vector<StoredTarget> targets;
	int currentTarget;
	bool targetNew;
	bool smrLocked;
	bool targetingEnabled;
	bool finishedAll;
	int prev_SMR_Size;
	std::vector<int>SMR_size;
	int buf = 0;
	int prev_min_index;

	float lastSingleSMRDistance;

	bool targetsFinal;

	float totalDistance(std::vector<SMRData>& smrs, std::vector<int> index);

	int closestSMR(std::vector<SMRData>& smrs, int target_num, int ignore_num, std::vector<bool>& smr_available);

};
