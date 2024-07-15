#pragma once

#include <vector>
#include <opencv2/imgproc.hpp>
//struct grouping SMR data for convenience
struct ObservedSMR {
	float imgX;
	float imgY;
	float imgW;
	float imgH;
	float imgA;
};

//struct associating SMR data with bool indicating in which image SMRs were actually observed
struct ImageSMRs {
	std::vector<ObservedSMR> observed;
	bool inFirstImage;
};

struct ImgDelta
{
	float topX;
	float topY;
	float botX;
	float botY;
};


//class for accumulating data about observed SMRs
class SMR {
public:
	SMR();
	SMR(float cx, float cy);
	SMR(SMR one, SMR two);
	void AddPixel(float x, float y);
	float GetCenterX(){return centerX;};
	float GetCenterY(){return centerY;};
	float GetMinX(){return minX;};
	float GetMaxX(){return maxX;};
	float GetMinY(){return minY;};
	float GetMaxY(){return maxY;};
	int GetNumPix(){return pixelNum;};
	//int GetOriginalNumPix(){return originalPixelNum;};
	void SetAvgV(float v) { avgV = v; };
	float GetAvgV() { return avgV; };
	void Clear();

private:
	float centerX;
	float centerY;
	float minX;
	float maxX;
	float minY;
	float maxY;
	int pixelNum;
	int originalPixelNum;
	float avgV;
};



class ImageProcessor {
public:
	//constructor/destructor
	ImageProcessor();
	~ImageProcessor();

	//get the coordinates of an SMR based on the difference between 2 images, can run 2 different forms of the algorithm, one that just looks for a green difference and one that also looks for red in one of the images - used for SMR tracking
	ImageSMRs getCoordinates(cv::Mat src1Y, cv::Mat src1U, cv::Mat src1V, cv::Mat src2Y, cv::Mat src2U, cv::Mat src2V, bool includeRed, ImgDelta delta, std::vector<cv::Point2f>& redSMRs);

	//just find the red laser reflected off the SMR - used for calibration
	cv::Point3f findRedLaser(cv::Mat srcY, cv::Mat srcU, cv::Mat srcV);
	//cv::Point3f findRedLaser(cv::Mat srcY, cv::Mat srcU, cv::Mat srcV, cv::Mat srcY2, cv::Mat srcU2, cv::Mat srcV2);

	//get total redness of pixels withiin AOI
	int getRedSum(cv::Mat srcV, cv::Rect& aoi);

	void setBrightThreshold(uint8_t thresh) { bright_thresh = thresh; }
	void setRedThresholdY(uint8_t red_threshY) {red_thresh_y = red_threshY; }
	void setRedThresholdCr(uint8_t red_threshCr) {red_thresh_cr = red_threshCr; }
	void setGreenThresholdCr (uint8_t grn_threshCr) {green_thresh_cr = grn_threshCr; }
	void setGreenThresholdCb (uint8_t grn_threshCb) { green_thresh_cb = grn_threshCb; }
private:
	//produce the list of found SMRs and store them in list
	void produceSMRList(cv::Mat src1Y, cv::Mat src1U, cv::Mat src1V, cv::Mat src2Y, cv::Mat src2U, cv::Mat src2V, bool includeRed, bool highRes, ImgDelta delta, bool& smrInSrc1, std::vector<cv::Point2f>& observedRedSMRs);
	//produce a list of found laser SMRs and store them in list
	void produceLaserList(cv::Mat srcY, cv::Mat srcU, cv::Mat srcV);
	//void produceLaserList(cv::Mat srcY, cv::Mat srcU, cv::Mat srcV, cv::Mat srcY2, cv::Mat srcU2, cv::Mat srcV2);

	//filter out bright spots which are not actual SMRs
	void MatchSMRs(std::vector<SMR>& src1List, std::vector<SMR>& src2List, float& offsetX, float& offsetY, ImgDelta delta, int& matches, int distanceThreshold, float imgHeight);
	void MatchSplitSMRs(std::vector<SMR>& src1List, std::vector<SMR>& src2List, float& offsetX, float& offsetY, ImgDelta delta, int& matches, int distanceThreshold, float imgHeight);
	void MatchEdgeSMRs(std::vector<SMR>& src1List, std::vector<SMR>& src2List, float& offsetX, float& offsetY, ImgDelta delta, int& matches, cv::Mat src1Y, cv::Mat src2Y, int distanceThreshold, float imgHeight);
	void RemoveBadSMRs(std::vector<SMR>& src1List, cv::Mat src1Y, cv::Mat srcU, cv::Mat src1V, float& offsetX, float& offsetY, ImgDelta delta, int distanceThreshold, float imgHeight);
	cv::Mat YUV_to_RGB(cv::Mat src1Y, cv::Mat src1U, cv::Mat src1V);
	//determine if there is a red laser reflection within a given region of interest
	bool IsColorReflection(cv::Rect roi, cv::Mat srcY, cv::Mat srcU, cv::Mat srcV, float centerX, float centerY, int size, int redThreshold, bool laser);
	//determine the color detection for differential
	bool IsColorReflection_for_diff(cv::Rect roi, cv::Mat srcY, cv::Mat srcU, cv::Mat srcV, float centerX, float centerY, int size, int redThreshold, bool laser);

	//add a pixel to the closest SMR (creates a new SMR if no SMR is close enough)
	static void AssignPixelToSMR(double x, double y, std::vector<SMR>& list, int& lastIndex);

	//list used to store detected SMRs
	std::vector<SMR> list;
	int avg_counter = 0;
	int counter_latch = 0;
	std::vector<cv::Point3f> avg_list;


	//keeps track of the number of detections performed, just used for debugging
	int detectCounter;

	std::vector<double> lastTimes;

	static uint8_t bright_thresh;
	static uint8_t red_thresh_y;
	static uint8_t red_thresh_cr;
	static uint8_t green_thresh_cr;
	static uint8_t green_thresh_cb;
	
	//static uint8_t bright_thresh_diff;

	void LaserSearch(std::vector<SMR>& laserList, cv::Mat srcY);
	static void PartialSMRSearch(cv::Rect area, std::vector<SMR>& list, cv::Mat src1);
	static void PartialSMR_Diff_search(cv::Rect area, std::vector<SMR>& list, cv::Mat src3);
	std::vector<SMR> combineSMRs(std::vector<SMR>& list);
	bool SMRsTouching(SMR& a, SMR& b);

	cv::Point2f interpolateDelta(ImgDelta d, float y, float h);
};