#pragma once

#include "SMRData.h"
#include "CameraCalibrationManager.h"
#include "CameraInterface.h"
#include <opencv2/core.hpp>
#include <vector>
#include <mutex>


//struct containing all data for observed SMRs
struct TrackedSMR {
	SMRData data;		//current SMR data from images
	int missingCount;	//determine if SMR has actually disappeared
	int matchedCount;
	bool updated;
	ShakeSMR shake;		//history of SMR movement to determine if it is shaking
};


//class for tracking SMRs
//main jobs are to consolidate all SMR data and continue tracking SMRs even when SMR isn't found every frame
class SMRTracker {
public:
	SMRTracker();
	~SMRTracker();

	void setCalibrationManager(CameraCalibrationManager* cm) { calibManager = cm; }
	void setCameraInterface(CameraInterface* cm) { cam = cm; }

	//find SMR near given angles (relative to camera)
	bool smrNear(float imgAz, float imgEl, SMRData& smr);

	//return SMR data if an SMR is shaking
	bool smrShaking(SMRData& smr);

	//given data from images, update list of tracked SMRs
	void updateTracking(std::vector<SMRData>& currentSMRs, float camAz, float camEl, cv::Point2f pixAng, std::vector<cv::Point2f> redSMRs, bool back_cam);

	//get current list of tracked SMRs
	std::vector<SMRData> getCurrentTrackedSMRs();

	//get camera angles at last update time
	cv::Point2f getLastTrkAngles();

	//clear list of tracked SMRs
	//(basically call this whenever you've stopped constantly tracking, so you don't have old SMRs hanging around)
	void clearTracking();
	int blind_redlaserCount;
	bool laserHitSMR;
private:

	CameraCalibrationManager* calibManager = nullptr;
	CameraInterface* cam = nullptr;
	//list of currently tracked SMRs
	std::vector<TrackedSMR> trackedSMRs;

	//camera angles at last update
	float lastCamAz, lastCamEl;

	void updateSMR(int index, SMRData& latest);
	void clearSMRUpdates();
	void handleUnmatched(float diffAz, float diffEl, std::vector<cv::Point2f>& redSMRs, bool back_cam);
	void addSMR(SMRData& observed);
	void add_redSMR(std::vector<cv::Point2f>& red_observed);

	//mutex in case you want to do tracking in a separate thread
	//probably can be removed since I don't think I'll use a separate thread, but idk
	std::mutex smrGuard;
};
