#pragma once

#include <opencv2/core.hpp>


//class for storing data about SMRs being observed and tracked
class SMRData {
public:
	SMRData();
	SMRData(float a, float e, float ar, float w, float h);
	SMRData(const SMRData& d);
	~SMRData();

	//getters for private data
	float getImgAz() { return imgAz; }
	float getImgEl() { return imgEl; }
	float getAz() { return az; };
	float getEl() { return el; };
	float getImgPix() { return imgA; };
	float getImgWidth() { return imgW; };
	float getImgHeight() { return imgH; };
	float getScore() { return score; };
	float getImgX() { return imgX; };
	float getImgY() { return imgY; };

	//setters for private data
	void setImgAz(float a) { imgAz = a; }
	void setImgEl(float e) { imgEl = e; }
	void setAz(float a) { az = a; };
	void setEl(float e) { el = e; };
	void setImgArea(float a) { imgA = a; };
	void setImgWidth(float w) { imgW = w; };
	void setImgHeight(float h) { imgH = h; };
	void setScore(float s) { score = s; };
	void setImgCoordinates(float x, float y, cv::Size imgRes);

	bool obscuredByLaser() { return laserHit; }
	bool setLaserHit(bool laser) { laserHit = laser; }

	//if other SMR is close enough to be considered the same SMR
	bool isSame(SMRData d);

	float validDistance(SMRData d, cv::Point2f pixelAngle);

	//update this SMR's data with data from other SMR
	void updateTo(const SMRData& d);

private:
	//angle offset relative to camera center
	float imgAz;
	float imgEl;

	//estimated absolute angles
	float az;
	float el;

	//pixel area
	float imgA;
	//pixel width
	float imgW;
	//pixel height
	float imgH;

	//normalized image coordinates
	float imgX;
	float imgY;

	//score estimating how certain that an observed SMR is actually a real SMR
	float score;

	//is it likely that the laser is currently hitting this SMR
	bool laserHit;
};




//class to help detect SMRs that are shaking
class ShakeSMR {
public:
	ShakeSMR();
	ShakeSMR(float a, float e);
	~ShakeSMR();

	//add newest SMR angles
	void AddAzEl(cv::Point2f azel);

	//total angle distance traveled by SMR in recent history
	cv::Point2f GetTotalDistance() {return totalDistance;};

	//size of region that contains all SMR angles in recent history
	cv::Point2f GetSize();

	//clear recent history
	void Clear();
private:

	//list of recent SMR angles (recent history)
	std::vector<cv::Point2f> lastAzEls;

	//total distance traveled
	cv::Point2f totalDistance;

	//upper corner of all angles
	cv::Point2f upperBound;
	//lower corner of all angles
	cv::Point2f lowerBound;
};
