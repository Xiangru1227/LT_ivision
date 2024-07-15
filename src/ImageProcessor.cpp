#include "ImageProcessor.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <string>
#include <sstream>
#include <algorithm>
#include <thread>
#include <unistd.h>
#include "Debug.h"
#include "IVisionState.h"

uint8_t ImageProcessor::bright_thresh = 150;
uint8_t ImageProcessor::red_thresh_y = 70;
uint8_t ImageProcessor::red_thresh_cr = 25;
uint8_t ImageProcessor::green_thresh_cr = 22;
uint8_t ImageProcessor::green_thresh_cb = 22;


//default constructor
SMR::SMR() {
	centerX = 0;
	centerY = 0;
	minX = 100000;
	maxX = 0;
	minY = 100000;
	maxY = 0;
	originalPixelNum = -1;
	pixelNum = 0;
}

//constructor given a single starting pixel location
SMR::SMR(float cx, float cy) {

	//center is just at the single pixel location
	centerX = cx;
	centerY = cy;

	//minimums and maximums are all just the starting location
	minX = cx;
	maxX = cx;
	minY = cy;
	maxY = cy;

	//starts with just one pixel
	pixelNum = 1;

	originalPixelNum = -1;
}

SMR::SMR(SMR one, SMR two) {
	int onePix = one.GetNumPix();
	int twoPix = two.GetNumPix();

	pixelNum = onePix + twoPix;
	centerX = (onePix * one.GetCenterX() + twoPix * two.GetCenterX()) / pixelNum;
	centerY = (onePix * one.GetCenterY() + twoPix * two.GetCenterY()) / pixelNum;

	maxX = std::max(one.GetMaxX(), two.GetMaxX());
	minX = std::min(one.GetMinX(), two.GetMinX());

	maxY = std::max(one.GetMaxY(), two.GetMaxY());
	minY = std::min(one.GetMinY(), two.GetMinY());

	originalPixelNum = -1;
}

//add pixel to SMR
void SMR::AddPixel(float x, float y) {

	//update center, which is just the average, so can update it by using the stored number of pixels and the existing center
	centerX = (pixelNum*centerX + x) / (pixelNum + 1);
	centerY = (pixelNum*centerY + y) / (pixelNum + 1);

	//update maximums and minimums
	if (x > maxX) {
		maxX = x;
	}
	if (x < minX) {
		minX = x;
	}
	if (y > maxY) {
		maxY = y;
	}
	if (y < minY) {
		minY = y;
	}

	//update number of pixels
	pixelNum++;
}

void SMR::Clear() {
	centerX = 0;
	centerY = 0;
	minX = 100000;
	maxX = 0;
	minY = 100000;
	maxY = 0;
	originalPixelNum = pixelNum;
	pixelNum = 0;
}





//constructor
ImageProcessor::ImageProcessor() {
	detectCounter = 0;
}

//destructor (nothing to do right now)
ImageProcessor::~ImageProcessor() {
	
}


int ImageProcessor::getRedSum(cv::Mat srcV, cv::Rect& aoi) {
	int left = std::max(aoi.x, 0);
	int up = std::max(aoi.y, 0);
	int right = std::min(aoi.x + aoi.width, srcV.cols);
	int down = std::min(aoi.y + aoi.height, srcV.rows);
	aoi = cv::Rect(left, up, right - left, down - up);
	//imwrite(createFilename("test",detectCounter), srcV(aoi));
	cv::Scalar srcSum = cv::sum(srcV(aoi));
	return srcSum.val[0];
}

//takes two images and uses differential algorithm to detect SMRs and returns vector of points representing SMR locations
ImageSMRs ImageProcessor::getCoordinates(cv::Mat src1Y, cv::Mat src1U, cv::Mat src1V, cv::Mat src2Y, cv::Mat src2U, cv::Mat src2V, bool includeRed, ImgDelta delta, std::vector<cv::Point2f>& redSMRs) {
	

	lastTimes.push_back(cv::getTickCount() / cv::getTickFrequency());

	if (lastTimes.size() > 4) {
		lastTimes.erase(lastTimes.begin());
		//std::cout << "Average speed: " << 3.0 / (lastTimes.back() - lastTimes.front()) << std::endl;
	}
	
		//initialize SMR and point lists
		ImageSMRs imgsmr;
		list.clear();

		bool smrInSrc1 = false;
		//produce list of detected SMRs
		produceSMRList(src1Y, src1U, src1V, src2Y, src2U, src2V, includeRed, true, delta, smrInSrc1, redSMRs);
		//double min, max;
		//cv::minMaxLoc(src1Y, &min, &max);
		//std::cout << "min: " << min << ", max: " << max << std::endl;
		//cv::minMaxLoc(src2Y, &min, &max);
		//std::cout << "min: " << min << ", max: " << max << std::endl;

		//produce point list from SMR list - produces 2 points per detected SMR, first is the SMR coordinates (x, y), second is the SMR size (width, number of pixels)
		for (int i = list.size() - 1; i >= 0; i--) {

			ObservedSMR osmr;
			osmr.imgX = list[i].GetCenterX();
			osmr.imgY = list[i].GetCenterY();
			osmr.imgA = list[i].GetNumPix();
			osmr.imgW = list[i].GetMaxX() - list[i].GetMinX();
			osmr.imgH = list[i].GetMaxY() - list[i].GetMinY();

			imgsmr.observed.push_back(osmr);

			//cv::drawMarker(src1Y, cv::Point(list[i].GetCenterX(), list[i].GetCenterY()), cv::Scalar(0));
			//cv::drawMarker(src2Y, cv::Point(list[i].GetCenterX(), list[i].GetCenterY()), cv::Scalar(0));
		}
		//if (std::abs(delta.topX) > 50 || std::abs(delta.topY) > 50) {
		//imwrite(createFilename("imageY",detectCounter, 2), src1Y);
		//imwrite(createFilename("imageY",detectCounter, 1), src2Y);
		//}

		//std::cout << "Observed " << imgsmr.observed.size() << " SMRs." << std::endl;

		if (smrInSrc1) {
			imgsmr.inFirstImage = true;
			//std::cout << "1st image" << std::endl;
		}
		else {
			imgsmr.inFirstImage = false;
			//std::cout << "2nd image" << std::endl;
		}

		//detectCounter really just keeps track of the number of detections for debugging/performance measurement purposes
		detectCounter++;
		return imgsmr;
	
}
//find coordinates of red laser reflected by SMR
cv::Point3f ImageProcessor::findRedLaser(cv::Mat srcY, cv::Mat srcU, cv::Mat srcV) {

	//clear list so not affected by previous detections
	list.clear();

	//imwrite(createFilename("image",detectCounter), src);

	//find all detected red SMRs (hopefully there's only one)
	produceLaserList(srcY, srcU, srcV);

	detectCounter++;

	for (int i = 0; i < list.size(); i++) {
		cv::drawMarker(srcY, cv::Point((int)list[i].GetCenterX(), (int)list[i].GetCenterY()), cv::Scalar(0), cv::MARKER_TILTED_CROSS, 20, 2, cv::LINE_8);
	}

	//imwrite(createFilename("image",detectCounter), srcY);

	//could add something to make a guess in case there is more than one "laser" found, perhaps based on the location in the image, but maybe it's not necessary to be that smart
	if (list.size() == 1) {
		return cv::Point3f(list[0].GetCenterX(), list[0].GetCenterY(), (float)list[0].GetNumPix());
	}
	else {
		return cv::Point3f(-1, -1, -1);
	}
}



//given two images, detects SMRs and adds them to the SMR list field
void ImageProcessor::produceSMRList(cv::Mat src1Y, cv::Mat src1U, cv::Mat src1V, cv::Mat src2Y, cv::Mat src2U, cv::Mat src2V, bool includeRed, bool highRes, ImgDelta delta, bool& smrInSrc1, std::vector<cv::Point2f>& observedRedSMRs) {

	std::vector<SMR> src1List;
	std::vector<SMR> src2List;
	std::vector<SMR> src3List;
	std::vector<SMR> src4List;
	std::vector<SMR> src5List;
	std::vector<SMR> src6List;

	cv::Mat src3Y, src4Y, src3U, src3V;
	 
	// imwrite("src2Y.png", src2Y);
	
	//differential of y channel
	cv::absdiff(src1Y , src2Y , src3Y);
	//cv::absdiff(src2Y , src1Y , src4Y);

	//differential of U channel
	cv::absdiff(src1U, src2U, src3U);
	
	//differential of V channel
	cv::absdiff(src1V, src2V, src3V);

	//imwrite("src3Y.png", src3Y);
	// imwrite("src3U.png", src3U);
	// imwrite("src3V.png", src3V);

	//std::cout << "Produce SMR List" << std::endl;
	  if (src3Y.cols * src3Y.rows > 1000000) {
		std::vector<SMR> src3List2, src3List3, src3List4, src4List2, src4List3, src4List4, src5List2, src5List3, src5List4;

		std::thread first (PartialSMRSearch, cv::Rect(0, 0, src3Y.cols, src3Y.rows/4), std::ref(src3List2),/*, std::ref(src4List2)*/ src3Y/*, src4Y*/);
		std::thread second (PartialSMRSearch, cv::Rect(0, src3Y.rows/4, src3Y.cols, src3Y.rows/4), std::ref(src3List3), /*std::ref(src4List3)*/ src3Y /*, src4Y*/);
		std::thread third (PartialSMRSearch, cv::Rect(0, src3Y.rows/2, src3Y.cols, src3Y.rows/4), std::ref(src3List4), /*std::ref(src4List4)*/ src3Y/*, src4Y*/);
		PartialSMRSearch(cv::Rect(0, 3*src3Y.rows/4, src3Y.cols, src3Y.rows/4), src3List, /*src4List*/ src3Y/*, src4Y*/);

		first.join();
		second.join();
		third.join();
     
	    src3List.insert(src3List.end(), src3List2.begin(), src3List2.end());
		src3List.insert(src3List.end(), src3List3.begin(), src3List3.end());
		src3List.insert(src3List.end(), src3List4.begin(), src3List4.end());

		std::thread fifth (PartialSMRSearch, cv::Rect(0, 0, src2Y.cols, src2Y.rows/4), std::ref(src5List2)/*, std::ref(src6List2)*/, src2Y/*, src2Y*/);
		std::thread sixth (PartialSMRSearch, cv::Rect(0, src2Y.rows/4, src2Y.cols, src2Y.rows/4), std::ref(src5List3), /*std::ref(src6List3)*/ src2Y/*, src2Y*/);
		std::thread seventh (PartialSMRSearch, cv::Rect(0, src2Y.rows/2, src2Y.cols, src2Y.rows/4), std::ref(src5List4), /*std::ref(src6List4)*/ src2Y/*, src2Y*/);
		PartialSMRSearch(cv::Rect(0, 3*src2Y.rows/4, src2Y.cols, src2Y.rows/4), src5List, /*src6List*/ src2Y/*, src2Y*/);

		fifth.join();
		sixth.join();
		seventh.join();

		
		src5List.insert(src5List.end(), src5List2.begin(), src5List2.end());
		src5List.insert(src5List.end(), src5List3.begin(), src5List3.end());
		src5List.insert(src5List.end(), src5List4.begin(), src5List4.end());
		
	}
	else {
		PartialSMRSearch(cv::Rect(0, 0, src3Y.cols, src3Y.rows), src3List/*, src4List*/, src3Y /*,src4Y*/);
		PartialSMRSearch(cv::Rect(0, 0, src2Y.cols, src2Y.rows), src5List/*, src6List*/, src2Y/*, src2Y*/);
		
	}
	
	src1List = combineSMRs(src3List); // differential
	//src2List = combineSMRs(src4List); // differential

	//for u and v channels to detect red smrs
	src5List = combineSMRs(src5List);
	// src6List = combineSMRs(src6List);
	
	float offsetX = 0;
	float offsetY = 0;
	int matches = 0;

	//std::cout << "src1 list " << src1List.size() << std::endl;
	// std::cout << "src5 list " << src5List.size() << std::endl;
	// std::cout << "src6 list " << src6List.size() << std::endl;
	
	if (includeRed) {
	//if (true) {
		//std::cout << "Checking for red reflection." << std::endl;
		std::vector<SMR> redSMRs;		
		for (int i = 0; i < src5List.size(); i++) {
			//std::cout << "List 1 SMR: (" << src1List[i].GetMinX() << "," << src1List[i].GetMinY() << "," << src1List[i].GetMaxX() - src1List[i].GetMinX() << "," << src1List[i].GetMaxY() - src1List[i].GetMinY() << ") - " << src1List[i].GetNumPix() << " pixels" << std::endl;
			
			int size = (int)std::max(src5List[i].GetMaxX() - src5List[i].GetMinX() + 1, src5List[i].GetMaxY() - src5List[i].GetMinY() + 1);
			int left = std::max((int)src5List[i].GetMinX() - size, 0);
			int right = std::min((int)src5List[i].GetMaxX() + size, src1Y.cols);
			int up = std::max((int)src5List[i].GetMinY() - size, 0);
			int down = std::min((int)src5List[i].GetMaxY() + size, src1Y.rows);

			cv::Rect smrROI(left, up, right - left, down - up);

			float centerX = src5List[i].GetCenterX();
			float centerY = src5List[i].GetCenterY();
			if (IsColorReflection(smrROI, src1Y, src1U, src1V, centerX, centerY, size, 170, true)) {
				redSMRs.push_back(src5List[i]);
			}
		}

		for (int i = 0; i < src6List.size(); i++) {
			//std::cout << "List 2 SMR: (" << src2List[i].GetMinX() << "," << src2List[i].GetMinY() << "," << src2List[i].GetMaxX() - src2List[i].GetMinX() << "," << src2List[i].GetMaxY() - src2List[i].GetMinY() << ") - " << src2List[i].GetNumPix() << " pixels" << std::endl;
			
			int size = (int)std::max(src6List[i].GetMaxX() - src6List[i].GetMinX() + 1, src6List[i].GetMaxY() - src6List[i].GetMinY() + 1);
			int left = std::max((int)src6List[i].GetMinX() - size, 0);
			int right = std::min((int)src6List[i].GetMaxX() + size, src1Y.cols);
			int up = std::max((int)src6List[i].GetMinY() - size, 0);
			int down = std::min((int)src6List[i].GetMaxY() + size, src1Y.rows);

			cv::Rect smrROI(left, up, right - left, down - up);

			float centerX = src6List[i].GetCenterX();
			float centerY = src6List[i].GetCenterY();
			if (IsColorReflection(smrROI, src2Y, src2U, src2V, centerX, centerY, size, 170, true)) {
				redSMRs.push_back(src6List[i]);
			}
		}

		int maxPix = 0;
		int maxIndex = -1;
		for (int i = 0; i < redSMRs.size(); i++) {
			if (redSMRs[i].GetNumPix() > maxPix) {
				maxPix = redSMRs[i].GetNumPix();
				maxIndex = i;
			}
		}
		if (maxIndex >= 0) {
			list.push_back(redSMRs[maxIndex]);
			//std::cout << "List SMR: (" << list[0].GetMinX() << "," << list[0].GetMinY() << "," << list[0].GetMaxX() - list[0].GetMinX() << "," << list[0].GetMaxY() - list[0].GetMinY() << ") - " << list[0].GetNumPix() << " pixels" << std::endl;
		}
	}
	else {
		//std::cout << "After finding SMRs:" << std::endl;
		for (int i = 0; i < src1List.size(); i++) {
			cv::Rect aoi = cv::Rect(src1List[i].GetMinX()/2-10, src1List[i].GetMinY()/2-10, (src1List[i].GetMaxX()-src1List[i].GetMinX())/2+20, (src1List[i].GetMaxY()-src1List[i].GetMinY())/2+20);
			int sum = getRedSum(src1V, aoi);
			float avg = ((float)sum) / (aoi.width * aoi.height);
			src1List[i].SetAvgV(avg);
			//std::cout << "List 1 SMR: (" << src1List[i].GetMinX() << "," << src1List[i].GetMinY() << "," << src1List[i].GetMaxX() - src1List[i].GetMinX() << "," << src1List[i].GetMaxY() - src1List[i].GetMinY() << ") - " << src1List[i].GetNumPix() << " pixels, " << src1List[i].GetAvgV() << std::endl;
			//std::cout << "List 1 SMR: (" << src1List[i].GetCenterX() << "," << src1List[i].GetCenterY() << "," << src1List[i].GetMaxX() - src1List[i].GetMinX() << "," << src1List[i].GetMaxY() - src1List[i].GetMinY() << ") - " << src1List[i].GetNumPix() << " pixels" << std::endl;
			//int sum = getRedSum(src1V, cv::Rect(src1List[i].GetMinX()/2-10, src1List[i].GetMinY()/2-10, (src1List[i].GetMaxX()-src1List[i].GetMinX())/2+20, (src1List[i].GetMaxY()-src1List[i].GetMinY())/2+20));
			//float avg = (4.0f*sum) / ((src1List[i].GetMaxX()-src1List[i].GetMinX()+40)*(src1List[i].GetMaxY()-src1List[i].GetMinY()+40));
			//std::cout << "List 1 SMR red sum: " << sum << ", avg: " << avg << std::endl;
			if (avg > 160 && sum > 90000)
				observedRedSMRs.push_back(cv::Point2f(src3List[i].GetCenterX(), src3List[i].GetCenterY()));
				//redSMRs1.push_back(cv::Point2f(src1List[i].GetCenterX(), src1List[i].GetCenterY()));
		}

		for (int i = 0; i < src5List.size(); i++) {
			cv::Rect aoi = cv::Rect(src5List[i].GetMinX()/2-10, src5List[i].GetMinY()/2-10, (src5List[i].GetMaxX()-src5List[i].GetMinX())/2+20, (src5List[i].GetMaxY()-src5List[i].GetMinY())/2+20);
			int sum = getRedSum(src2V, aoi);
			float avg = ((float)sum) / (aoi.width * aoi.height);
			src5List[i].SetAvgV(avg);
			//std::cout << "List 2 SMR: (" << src2List[i].GetMinX() << "," << src2List[i].GetMinY() << "," << src2List[i].GetMaxX() - src2List[i].GetMinX() << "," << src2List[i].GetMaxY() - src2List[i].GetMinY() << ") - " << src2List[i].GetNumPix() << " pixels, " << src2List[i].GetAvgV() << std::endl;
			//std::cout << "List 2 SMR: (" << src2List[i].GetCenterX() << "," << src2List[i].GetCenterY() << "," << src2List[i].GetMaxX() - src2List[i].GetMinX() << "," << src2List[i].GetMaxY() - src2List[i].GetMinY() << ") - " << src2List[i].GetNumPix() << " pixels" << std::endl;
			//int sum = getRedSum(src1V, cv::Rect(src2List[i].GetMinX()/2-10, src2List[i].GetMinY()/2-10, (src2List[i].GetMaxX()-src2List[i].GetMinX())/2+20, (src2List[i].GetMaxY()-src2List[i].GetMinY())/2+20));
			//float avg = (4.0f*sum) / ((src2List[i].GetMaxX()-src2List[i].GetMinX()+40)*(src2List[i].GetMaxY()-src2List[i].GetMinY()+40));
			//std::cout << "List 2 SMR red sum: " << sum << ", avg: " << avg << std::endl;
			if (avg > 160 && sum > 90000)
				//redSMRs2.push_back(cv::Point2f(src2List[i].GetCenterX(), src2List[i].GetCenterY()));
				observedRedSMRs.push_back(cv::Point2f(src5List[i].GetCenterX(), src5List[i].GetCenterY()));
		}

		int distanceThreshold = 50;
		int edgeThreshold = 2 * distanceThreshold;
		//std::cout << "dxt=" << delta.topX << ", dyt=" << delta.topY << std::endl;
		//std::cout << "dxb=" << delta.botX << ", dyb=" << delta.botY << std::endl;
		MatchSMRs(src1List, src2List, offsetX, offsetY, delta, matches, distanceThreshold, src1Y.rows);
		MatchSplitSMRs(src1List, src2List, offsetX, offsetY, delta, matches, distanceThreshold, src1Y.rows);
		MatchSplitSMRs(src2List, src1List, offsetX, offsetY, delta, matches, distanceThreshold, src1Y.rows);
		MatchEdgeSMRs(src1List, src2List, offsetX, offsetY, delta, matches, src1Y, src2Y, edgeThreshold, src1Y.rows);

		//std::cout << "After matching SMRs:" << std::endl;

		if (matches > 0) {
			offsetX = offsetX / matches;
			offsetY = offsetY / matches;
		}
		RemoveBadSMRs(src1List, src3Y, src3U, src3V, offsetX, offsetY, delta, edgeThreshold, src1Y.rows);
		//std::cout << "Removing bad SMRs from list 2." << std::endl;
		//RemoveBadSMRs(src2List, src2Y, src2V, offsetX, offsetY, delta, edgeThreshold, src1Y.rows);
		//removing bad smr from differential algorithm.
		//RemoveBadSMRs(src3List, src3Y, src3V, offsetX, offsetY, delta, edgeThreshold, src1Y.rows);
		//std::cout << "After removing bad SMRs:" << std::endl;
		//std::cout << "SMRs remaining in list 1: " << src1List.size() << std::endl;
		// std::cout << "SMRs remaining in list 2: " << src2List.size() << std::endl;
		//std::cout << "SMRs remaining in list 3: " << src3List.size() << std::endl;
		//std::cout << "Good SMRs: " << std::max(src1List.size(), src2List.size()) << std::endl;

		int totalPix1 = 0;
		int totalPix2 = 0;	
		//int totalPix3 = 0;
		for (int i = 0; i < src1List.size(); i++) {
			//std::cout << "List 1 SMR: (" << src1List[i].GetMinX() << "," << src1List[i].GetMinY() << "," << src1List[i].GetMaxX() - src1List[i].GetMinX() << "," << src1List[i].GetMaxY() - src1List[i].GetMinY() << ") - " << src1List[i].GetNumPix() << " pixels, " << src1List[i].GetAvgV() << std::endl;
			totalPix1 += src1List[i].GetNumPix();
		}

		// for (int i = 0; i < src2List.size(); i++) {
		// 	//std::cout << "List 2 SMR: (" << src2List[i].GetMinX() << "," << src2List[i].GetMinY() << "," << src2List[i].GetMaxX() - src2List[i].GetMinX() << "," << src2List[i].GetMaxY() - src2List[i].GetMinY() << ") - " << src2List[i].GetNumPix() << " pixels, " << src2List[i].GetAvgV() << std::endl;
		// 	totalPix2 += src2List[i].GetNumPix();
		// }


		if (totalPix1 >= totalPix2) {
			list = src1List;
			smrInSrc1 = true;
		}
		else {
			list = src2List;
			smrInSrc1 = false;
		}
			
	}
}


//produce the list of SMRs reflecting the red laser (there should only be one, but it's possible it could mistakenly detect more)
void ImageProcessor::produceLaserList(cv::Mat srcY, cv::Mat srcU, cv::Mat srcV) {

	std::vector<SMR> tempList;
	LaserSearch(tempList, srcY);
	tempList = combineSMRs(tempList);

	for (int i = 0; i < tempList.size(); i++) {
		//std::cout << "List 1 SMR: (" << tempList[i].GetMinX() << "," << tempList[i].GetMinY() << "," << tempList[i].GetMaxX() - tempList[i].GetMinX() << "," << tempList[i].GetMaxY() - tempList[i].GetMinY() << ") - " << tempList[i].GetNumPix() << " pixels" << std::endl;
		
		int width = tempList[i].GetMaxX() - tempList[i].GetMinX();
		int height = tempList[i].GetMaxY() - tempList[i].GetMinY();
		float sizeRatio = ((float)width)/height;
		int size = (int)std::max(width + 1, height + 1);
		int left = std::max((int)tempList[i].GetMinX() - size, 0);
		int right = std::min((int)tempList[i].GetMaxX() + size, srcY.cols);
		int up = std::max((int)tempList[i].GetMinY() - size, 0);
		int down = std::min((int)tempList[i].GetMaxY() + size, srcY.rows);
		cv::Rect smrROI(left, up, right - left, down - up);
		float centerX = tempList[i].GetCenterX();
		float centerY = tempList[i].GetCenterY();
		if (IsColorReflection(smrROI, srcY, srcU, srcV, centerX, centerY, size, 160, true) && sizeRatio < 2.0f && sizeRatio > .5f/* && centerX < .6f*srcY.cols && centerX > .4f*srcY.cols && centerY > .3f*srcY.rows*/) {
			list.push_back(tempList[i]);
		}
	}
}

// //produce the list of SMRs reflecting the red laser (there should only be one, but it's possible it could mistakenly detect more)
// void ImageProcessor::produceLaserList(cv::Mat srcY, cv::Mat srcU, cv::Mat srcV, cv::Mat srcY2, cv::Mat srcU2, cv::Mat srcV2) {

// 	std::vector<SMR> tempList;
// 	cv::Mat srcY3;
// 	cv::absdiff(srcY , srcY2 , srcY3);
// 	cv::imwrite("src3Y.png", srcY3);
	
// 	LaserSearch(tempList, srcY3);
// 	tempList = combineSMRs(tempList);

// 	for (int i = 0; i < tempList.size(); i++) {
// 		//std::cout << "List 1 SMR: (" << tempList[i].GetMinX() << "," << tempList[i].GetMinY() << "," << tempList[i].GetMaxX() - tempList[i].GetMinX() << "," << tempList[i].GetMaxY() - tempList[i].GetMinY() << ") - " << tempList[i].GetNumPix() << " pixels" << std::endl;
		
// 		int width = tempList[i].GetMaxX() - tempList[i].GetMinX();
// 		int height = tempList[i].GetMaxY() - tempList[i].GetMinY();
// 		float sizeRatio = ((float)width)/height;
// 		//std::cout << "sizeRatio: " << sizeRatio << "\n";
// 		int size = (int)std::max(width + 1, height + 1);
// 		int left = std::max((int)tempList[i].GetMinX() - size, 0);
// 		int right = std::min((int)tempList[i].GetMaxX() + size, srcY.cols);
// 		int up = std::max((int)tempList[i].GetMinY() - size, 0);
// 		int down = std::min((int)tempList[i].GetMaxY() + size, srcY.rows);
// 		cv::Rect smrROI(left, up, right - left, down - up);
// 		float centerX = tempList[i].GetCenterX();
// 		float centerY = tempList[i].GetCenterY();
// 		if (IsColorReflection(smrROI, srcY3, srcU2, srcV2, centerX, centerY, size, 160, false) && sizeRatio < 2.0f && sizeRatio > .5f/* && centerX < .6f*srcY.cols && centerX > .4f*srcY.cols && centerY > .3f*srcY.rows*/) {
// 			list.push_back(tempList[i]);
// 		}
// 	}
// }


void ImageProcessor::MatchSMRs(std::vector<SMR>& src1List, std::vector<SMR>& src2List, float& offsetX, float& offsetY, ImgDelta delta, int& matches, int distanceThreshold, float imgHeight) {

	for (int i = src1List.size() - 1; i >= 0; i--) {
		for (int j = src2List.size() - 1; j >= 0; j--) {
			cv::Point2f delt = interpolateDelta(delta, (src1List[i].GetCenterY() + src2List[j].GetCenterY())/2, imgHeight);
			float diffX = (float)(src1List[i].GetCenterX() - src2List[j].GetCenterX()) - delt.x;
			float diffY = (float)(src1List[i].GetCenterY() - src2List[j].GetCenterY()) - delt.y;
			float distance = std::sqrt(diffX*diffX + diffY*diffY);
			float width1 = (float)(src1List[i].GetMaxX() - src1List[i].GetMinX() + 1);	
			float height1 = (float)(src1List[i].GetMaxY() - src1List[i].GetMinY() + 1);
			float width2 = (float)(src2List[j].GetMaxX() - src2List[j].GetMinX() + 1);
			float height2 = (float)(src2List[j].GetMaxY() - src2List[j].GetMinY() + 1);
			float widthRatio = width1 / width2;
			float heightRatio = height1 / height2;
			float pixRatio = (float)src1List[i].GetNumPix() / src2List[j].GetNumPix();
			int pixDiff = src1List[i].GetNumPix() - src2List[j].GetNumPix();
			float minDimension = std::min(std::min(width1, width2), std::min(height1, height2));
			float maxDimension = std::max(std::max(width1, width2), std::max(height1, height2));
			float thresh = std::max((float)distanceThreshold, .2f*minDimension);
			//bool colorMatch = ((src1List[i].GetAvgV() - 130) * (src2List[j].GetAvgV() - 130)) > 0;
			
			if (distance < thresh && ((maxDimension < 2*distanceThreshold && (widthRatio < 3.0f && widthRatio > .33f && heightRatio < 3.0f && heightRatio > .33f)) || (widthRatio < 2.0f && widthRatio > .5f && heightRatio < 2.0f && heightRatio > .5f) || (pixRatio > .6f && pixRatio < 1.67f) || (pixDiff < 8 && pixDiff > -8))) {
				src1List.erase(src1List.begin() + i);
				src2List.erase(src2List.begin() + j);
				//std::cout << "Found match " << i << "-" << j << " with offset: " << diffX << "," << diffY << std::endl;
				offsetX += diffX + delt.x;
				offsetY += diffY + delt.y;
				matches++;
				break;
			}
		}
	}
}

void ImageProcessor::MatchSplitSMRs(std::vector<SMR>& src1List, std::vector<SMR>& src2List, float& offsetX, float& offsetY, ImgDelta delta, int& matches, int distanceThreshold, float imgHeight) {

	for (int i = src1List.size() - 1; i >= 0; i--) {
		for (int j = src2List.size() - 1; j >= 0; j--) {
			bool found = false;
			for (int k = 0; k < src2List.size(); k++) {
				if (k != j) {
					int pix1 = src2List[j].GetNumPix();
					int pix2 = src2List[k].GetNumPix();
					int totalPix = pix1 + pix2;
					float centerX = (float)(pix1 * src2List[j].GetCenterX() + pix2 * src2List[k].GetCenterX()) / totalPix;
					float centerY = (float)(pix1 * src2List[j].GetCenterY() + pix2 * src2List[k].GetCenterY()) / totalPix;
					cv::Point2f delt = interpolateDelta(delta, (src1List[i].GetCenterY() + centerY)/2, imgHeight);
					float diffX = (float)(src1List[i].GetCenterX() - centerX) - delt.x;
					float diffY = (float)(src1List[i].GetCenterY() - centerY) - delt.y;
					float distance = std::sqrt(diffX*diffX + diffY*diffY);
					float width1 = (float)(src1List[i].GetMaxX() - src1List[i].GetMinX() + 1);	
					float height1 = (float)(src1List[i].GetMaxY() - src1List[i].GetMinY() + 1);
					float width2 = (float)(std::max(src2List[j].GetMaxX(), src2List[k].GetMaxX()) - std::min(src2List[j].GetMinX(), src2List[k].GetMinX()) + 1);
					float height2 = (float)(std::max(src2List[j].GetMaxY(), src2List[k].GetMaxY()) - std::min(src2List[j].GetMinY(), src2List[k].GetMinY()) + 1);
					float widthRatio = width1 / width2;
					float heightRatio = height1 / height2;
					float pixRatio = (float)src1List[i].GetNumPix() / (src2List[j].GetNumPix() + src2List[k].GetNumPix());
					int pixDiff = src1List[i].GetNumPix() - src2List[j].GetNumPix() - src2List[k].GetNumPix();
					float minDimension = std::min(std::min(width1, width2), std::min(height1, height2));
					float maxDimension = std::max(std::max(width1, width2), std::max(height1, height2));
					float thresh = std::max((float)distanceThreshold, .2f*minDimension);
					//float avgColor = (src2List[j].GetAvgV() + src2List[k].GetAvgV()) / 2;
					//std::cout << "avg clr " << src2List[j].GetAvgV()<< "-" << src2List[k].GetAvgV() << std::endl;
					//bool colorMatch = ((src1List[i].GetAvgV() - 130) * (avgColor - 130)) > 0;
					std::cout << "distance " << distance << "-" << std::endl;
					if (distance < thresh && ((maxDimension < 2*distanceThreshold && (widthRatio < 3.0f && widthRatio > .33f && heightRatio < 3.0f && heightRatio > .33f && pixRatio > .33f && pixRatio < 3.0f)) || (widthRatio < 2.0f && widthRatio > .5f && heightRatio < 2.0f && heightRatio > .5f && pixRatio > .33f && pixRatio < 3.0f) || (pixRatio > .6f && pixRatio < 1.67f) || (pixDiff < 8 && pixDiff > -8))) {
						src1List.erase(src1List.begin() + i);
						int maxIndex = std::max(j, k);
						int minIndex = std::min(j, k);
						src2List.erase(src2List.begin() + maxIndex);
						src2List.erase(src2List.begin() + minIndex);
						//std::cout << "Found match " << i << "-" << j << "-" << k << " with offset: " << diffX << "," << diffY << std::endl;
						offsetX += diffX + delt.x;
						offsetY += diffY + delt.y;
						matches++;
						std::cout << " MatchSplitSMR matches is   " << matches<< std::endl;
						found = true;
						break;
					}
				}
			}
			if (found)
				break;
		}
	}
}

void ImageProcessor::MatchEdgeSMRs(std::vector<SMR>& src1List, std::vector<SMR>& src2List, float& offsetX, float& offsetY, ImgDelta delta, int& matches, cv::Mat src1Y, cv::Mat src2Y, int distanceThreshold, float imgHeight) {

	for (int i = src1List.size() - 1; i >= 0; i--) {
		for (int j = src2List.size() - 1; j >= 0; j--) {

			cv::Point2f delt = interpolateDelta(delta, (src1List[i].GetCenterY() + src2List[j].GetCenterY())/2, imgHeight);
			float diffX = (float)(src1List[i].GetCenterX() - src2List[j].GetCenterX()) - delt.x;
			float diffY = (float)(src1List[i].GetCenterY() - src2List[j].GetCenterY()) - delt.y;
			float distance = std::sqrt(diffX*diffX + diffY*diffY);
			if (distance < distanceThreshold) {

				bool matching = false;
				if (src1List[i].GetMinX() == 0) {
					float widthRatio = ((float)(src1List[i].GetMaxX() + 1)) / (src2List[j].GetMaxX() - src2List[j].GetMinX() + 1);
					float sizeRatio = (widthRatio * src2List[j].GetNumPix()) / src1List[i].GetNumPix();
					float diffMinY = (float)std::abs(src1List[i].GetMinY() - src2List[j].GetMinY());
					float diffMaxY = (float)std::abs(src1List[i].GetMaxY() - src2List[j].GetMaxY());
					matching = (sizeRatio <= 2.0f && sizeRatio >= .5f && diffMinY < distanceThreshold && diffMaxY < distanceThreshold);
				}
				else if (src1List[i].GetMaxX() == src1Y.cols - 1) {
					float widthRatio = ((float)(src1Y.cols - src1List[i].GetMinX() - 1)) / (src2List[j].GetMaxX() - src2List[j].GetMinX() + 1);
					float sizeRatio = (widthRatio * src2List[j].GetNumPix()) / src1List[i].GetNumPix();
					float diffMinY = (float)std::abs(src1List[i].GetMinY() - src2List[j].GetMinY());
					float diffMaxY = (float)std::abs(src1List[i].GetMaxY() - src2List[j].GetMaxY());
					matching = (sizeRatio <= 2.0f && sizeRatio >= .5f && diffMinY < distanceThreshold && diffMaxY < distanceThreshold);
				}
				else if (src1List[i].GetMinY() == 0) {
					float heightRatio = ((float)(src1List[i].GetMaxY() + 1)) / (src2List[j].GetMaxY() - src2List[j].GetMinY() + 1);
					float sizeRatio = (heightRatio * src2List[j].GetNumPix()) / src1List[i].GetNumPix();
					float diffMinX = (float)std::abs(src1List[i].GetMinX() - src2List[j].GetMinX());
					float diffMaxX = (float)std::abs(src1List[i].GetMaxX() - src2List[j].GetMaxX());
					matching = (sizeRatio <= 2.0f && sizeRatio >= .5f && diffMinX < distanceThreshold && diffMaxX < distanceThreshold);
				}
				else if (src1List[i].GetMaxY() == src1Y.rows - 1) {
					float heightRatio = ((float)(src1Y.rows - src1List[i].GetMinY() - 1)) / (src2List[j].GetMaxY() - src2List[j].GetMinY() + 1);
					float sizeRatio = (heightRatio * src2List[j].GetNumPix()) / src1List[i].GetNumPix();
					float diffMinX = (float)std::abs(src1List[i].GetMinX() - src2List[j].GetMinX());
					float diffMaxX = (float)std::abs(src1List[i].GetMaxX() - src2List[j].GetMaxX());
					matching = (sizeRatio <= 2.0f && sizeRatio >= .5f && diffMinX < distanceThreshold && diffMaxX < distanceThreshold);
				}
				else if (src2List[j].GetMinX() == 0) {
					float widthRatio = ((float)(src2List[j].GetMaxX() + 1)) / (src1List[i].GetMaxX() - src1List[i].GetMinX() + 1);
					float sizeRatio = (widthRatio * src1List[i].GetNumPix()) / src2List[j].GetNumPix();
					float diffMinY = (float)std::abs(src1List[i].GetMinY() - src2List[j].GetMinY());
					float diffMaxY = (float)std::abs(src1List[i].GetMaxY() - src2List[j].GetMaxY());
					matching = (sizeRatio <= 2.0f && sizeRatio >= .5f && diffMinY < distanceThreshold && diffMaxY < distanceThreshold);
				}
				else if (src2List[j].GetMaxX() == src2Y.cols - 1) {
					float widthRatio = ((float)(src2Y.cols - src2List[j].GetMinX() - 1)) / (src1List[i].GetMaxX() - src1List[i].GetMinX() + 1);
					float sizeRatio = (widthRatio * src1List[i].GetNumPix()) / src2List[j].GetNumPix();
					float diffMinY = (float)std::abs(src1List[i].GetMinY() - src2List[j].GetMinY());
					float diffMaxY = (float)std::abs(src1List[i].GetMaxY() - src2List[j].GetMaxY());
					matching = (sizeRatio <= 2.0f && sizeRatio >= .5f && diffMinY < distanceThreshold && diffMaxY < distanceThreshold);
				}
				else if (src2List[j].GetMinY() == 0) {
					float heightRatio = ((float)(src2List[j].GetMaxY() + 1)) / (src1List[i].GetMaxY() - src1List[i].GetMinY() + 1);
					float sizeRatio = (heightRatio * src1List[i].GetNumPix()) / src2List[j].GetNumPix();
					float diffMinX = (float)std::abs(src1List[i].GetMinX() - src2List[j].GetMinX());
					float diffMaxX = (float)std::abs(src1List[i].GetMaxX() - src2List[j].GetMaxX());
					matching = (sizeRatio <= 2.0f && sizeRatio >= .5f && diffMinX < distanceThreshold && diffMaxX < distanceThreshold);
				}
				else if (src2List[j].GetMaxY() == src2Y.rows - 1) {
					float heightRatio = ((float)(src2Y.rows - src2List[j].GetMinY() - 1)) / (src1List[i].GetMaxY() - src1List[i].GetMinY() + 1);
					float sizeRatio = (heightRatio * src1List[i].GetNumPix()) / src2List[j].GetNumPix();
					float diffMinX = (float)std::abs(src1List[i].GetMinX() - src2List[j].GetMinX());
					float diffMaxX = (float)std::abs(src1List[i].GetMaxX() - src2List[j].GetMaxX());
					matching = (sizeRatio <= 2.0f && sizeRatio >= .5f && diffMinX < distanceThreshold && diffMaxX < distanceThreshold);
				}

				if (matching) {
					src1List.erase(src1List.begin() + i);
					src2List.erase(src2List.begin() + j);
					//std::cout << "Found match " << i << "-" << j << " with offset: " << diffX << "," << diffY << std::endl;
					offsetX += diffX + delt.x;
					offsetY += diffY + delt.y;
					matches++;
					std::cout << " MatchEdgeSMR matches is   " << matches<< std::endl;
					break;
				}
				
			}
		}
	}
}

//checks SMR list and removes SMRs that are incorrect shape or color (will probably need to modify several things here for new camera)
void ImageProcessor::RemoveBadSMRs(std::vector<SMR>& src1List, cv::Mat src1Y,  cv::Mat src1U, cv::Mat src1V, float& offsetX, float& offsetY, ImgDelta delta, int distanceThreshold, float imgHeight) {

	int ignoreBorder = 3 * distanceThreshold / 4;
	float narrowRatio = 1.8f;
	//float avgDX = (offsetX + deltaX) / 2;
	//float avgDY = (offsetY + deltaY) / 2;
	for (int i = src1List.size() - 1; i >= 0; i--) {
		cv::Point2f delt = interpolateDelta(delta, src1List[i].GetCenterY(), imgHeight);
		float avgDX = (offsetX + delt.x) / 2;
		float avgDY = (offsetY + delt.y) / 2;
		//std::cout << "List 1 SMR: (" << src1List[i].GetMinX() << "," << src1List[i].GetMinY() << "," << src1List[i].GetMaxX() - src1List[i].GetMinX() << "," << src1List[i].GetMaxY() - src1List[i].GetMinY() << ") - " << src1List[i].GetNumPix() << " pixels" << std::endl;
		if (src1List[i].GetMaxX() < ignoreBorder + avgDX || src1List[i].GetMinX() > src1Y.cols - ignoreBorder + avgDX || src1List[i].GetMaxY() < ignoreBorder + avgDY || src1List[i].GetMinY() > src1Y.rows - ignoreBorder + avgDY || src1List[i].GetNumPix() < 5) {
			//std::cout << "Too small or close to edge." << std::endl;
			src1List.erase(src1List.begin() + i);
			continue;
		}
		float width = (float)(src1List[i].GetMaxX() - src1List[i].GetMinX() + 1);
		float height = (float)(src1List[i].GetMaxY() - src1List[i].GetMinY() + 1);
		float proportion = ((float)src1List[i].GetNumPix()) / (width * height);
		//std::cout << "Num Pix: " << src1List[i].GetNumPix() << " Proportions: " << proportion << " width: " << width << " height: " << height << std::endl;
		if (proportion < .3f || width > narrowRatio * height || height > narrowRatio * width) {
			//std::cout << "Num Pix: " << src1List[i].GetNumPix() << " Proportions: " << proportion << " width: " << width << " height: " << height << std::endl;
			src1List.erase(src1List.begin() + i);
			continue;
		}
		int size = (int)std::max(width, height);
		int left = std::max((int)src1List[i].GetMinX() - size, 0);
		int right = std::min((int)src1List[i].GetMaxX() + size, src1Y.cols);
		int up = std::max((int)src1List[i].GetMinY() - size, 0);
		int down = std::min((int)src1List[i].GetMaxY() + size, src1Y.rows);

		cv::Rect smrROI(left, up, right - left, down - up);
		cv::Rect smallSmrROI(smrROI.x/2, smrROI.y/2, smrROI.width/2, smrROI.height/2);

		float centerX = src1List[i].GetCenterX();
		float centerY = src1List[i].GetCenterY();

		if (!IsColorReflection(smrROI, src1Y, src1U, src1V, centerX, centerY, size, 50, false)) {
			src1List.erase(src1List.begin() + i);
			//std::cout << "Wrong color." << std::endl;
			continue;
		}

	}
}

//check for a bright white center surrounded by color against a dark background
bool ImageProcessor::IsColorReflection(cv::Rect roi, cv::Mat srcY, cv::Mat srcU, cv::Mat srcV, float centerX, float centerY, int size, int redThreshold, bool laser) {


		//cv::imwrite("srcU.png", srcU);
		//cv::imwrite("srcV.png", srcV);

		//set up variables tracking each category (color) of pixel
		uchar *y, *cr, *cb;
		int numWhite = 0;
		int numColor = 0;
		int numDark = 0;
		float avgWhitePosX = 0.0f;
		float avgWhitePosY = 0.0f;
		float avgWhiteDistance = 0.0f;
		float avgColorPosX = 0.0f;
		float avgColorPosY = 0.0f;
		float avgColorDistance = 0.0f;
		float avgDarkPosX = 0.0f;
		float avgDarkPosY = 0.0f;
		float avgDarkDistance = 0.0f;

		//check pixels in area around detected SMR
		for (int v = roi.y; v < roi.y + roi.height; v++) {
			y = srcY.ptr<uchar>(v);
			cr = srcV.ptr<uchar>(v/2);
			cb = srcU.ptr<uchar>(v/2);
			for (int h = roi.x; h < roi.x + roi.width; h++) {

				//marks whether pixel is actually part of the SMR
				bool inSMR = false;

				//very bright, so count this pixel as white
				if ((y[h] > bright_thresh && laser) || (y[h] > bright_thresh && !laser)) {
					inSMR = true;
					numWhite++;
					avgWhitePosX += h;
					avgWhitePosY += v;
					float diffX = h - centerX;
					float diffY = v - centerY;
					avgWhiteDistance += std::sqrt(diffX * diffX + diffY * diffY);
				}

				//somewhat bright and either red or green, so count as color pixel. Thresholds set from calib file
				if ((y[h] > red_thresh_y && cr[h/2] > red_thresh_cr && laser) || (cr[h/2] < green_thresh_cr && cb[h/2] < green_thresh_cb && !laser)) {
					inSMR = true;
					numColor++;
					avgColorPosX += h;
					avgColorPosY += v;
					float diffX = h - centerX;
					float diffY = v - centerY;
					avgColorDistance += std::sqrt(diffX * diffX + diffY * diffY);
				}

				//if didn't fit into either of the previous 2 categories, consider it a dark background pixel
				if (!inSMR)  {
					numDark++;
					avgDarkPosX += h;
					avgDarkPosY += v;
					float diffX = h - centerX;
					float diffY = v - centerY;
					avgDarkDistance += std::sqrt(diffX * diffX + diffY * diffY);
				}
			}
		}

		//calculates average positions and distance from SMR center for each color category
		if (numWhite > 0) {
			avgWhitePosX /= numWhite;
			avgWhitePosY /= numWhite;
			avgWhiteDistance /= numWhite;
		}
		if (numColor > 0) {
			avgColorPosX /= numColor;
			avgColorPosY /= numColor;
			avgColorDistance /= numColor;
		}
		if (numDark > 0) {
			avgDarkPosX /= numDark;
			avgDarkPosY /= numDark;
			avgDarkDistance /= numDark;
		}

		//determine how close the average color center is to the average white center (if the color pixels are distributed evenly around the white center, the average positions for each should be close to each other)
		float c2wDiffX = avgColorPosX - avgWhitePosX;
		float c2wDiffY = avgColorPosY - avgWhitePosY;
		float c2wDistance = std::sqrt(c2wDiffX * c2wDiffX + c2wDiffY * c2wDiffY);
		float maxCenterDistance;
		if (laser)
			maxCenterDistance = (numColor > 4) ? size : 1.5f*size;
		else
			maxCenterDistance = (numColor > 4) ? ((float)size) / 1.5f : 1.0f*size;

		//reject if there aren't enough color pixels, the white pixels are farther from the center than the color pixels, the color and white pixel centers are too far apart, or the color pixels are farther from the center than the dark pixels
		if (/*(numWhite > 500 && numColor <= (int)(.5f*numWhite)) ||*/ numColor <= (int)(.3f * numWhite)) /*|| avgColorDistance < avgWhiteDistance || c2wDistance > maxCenterDistance || avgColorDistance > avgDarkDistance)*/ {
			//std::cout << "white: " << numWhite << " " << "color: " << numColor << " " << "W Dist: " << avgWhiteDistance << " " << "C Dist: " << avgColorDistance << " " << "D Dist: " << avgDarkDistance << " " << " c2W: " << c2wDistance << " " << "maxCenter D: " << maxCenterDistance << std::endl;
			return false;
		}
		return true;
}

//assigns pixels in part of both images to SMRs based on brightness
void ImageProcessor::PartialSMRSearch(cv::Rect area, std::vector<SMR>& list, cv::Mat src1) {
	
	int index1 = -1;
	//int index2 = -1;
	
	//std::cout << "Partial SMR" << std::endl;
	for (int i = area.y; i < (area.y + area.height); i++) {
		
		uchar* s1 = src1.ptr<uchar>(i);
		//uchar* s2 = src2.ptr<uchar>(i);

		for (int j = area.x; j < (area.x + area.width); j++) {
			if ((uint8_t)s1[j] > bright_thresh) {
				//std::cout << "1: " << (int)s3[j] << " - " << i << "," << j << std::endl;
				AssignPixelToSMR((double)j, (double)i, list, index1);
			}
			else {
				index1 = -1;
			}
			// if ((uint8_t) s2[j] > bright_thresh) {
			// 	//std::cout << "2: " << (int)s4[j] << " - " << i << ", " << j << std::endl;
			// 	AssignPixelToSMR((double)j, (double)i, list2, index2);
			// }
			// else {
			// 	index2 = -1;
			// }
		}
	}
}

//assigns pixels in one image to SMRs based on brightness
void ImageProcessor::LaserSearch(std::vector<SMR>& laserList, cv::Mat srcY) {

	//std::cout << "Laser Search" << std::endl;
	uchar *sy;
	int index1 = -1;
	for (int i = 0; i < srcY.rows; i++) {
		sy = srcY.ptr<uchar>(i);
		for (int j = 0; j < srcY.cols; j++) {
			if (sy[j] > red_thresh_y) {
				AssignPixelToSMR((double)j, (double)i, laserList, index1);
			}
			else {
				index1 = -1;
			}
		}
	}
}

//assign a pixel to the nearest SMR (or create new SMR if none are within max SMR radius)
void ImageProcessor::AssignPixelToSMR(double x, double y, std::vector<SMR>& list, int& lastIndex) {

	//std::cout << "Assign Pixel" << std::endl;
	if (lastIndex < 0) {

		//search for touching SMR
		int minIndex = -1;
		for (int i = 0; i < list.size(); i++) {
			if (x <= list[i].GetMaxX() + 2 && x >= list[i].GetMinX() - 2 && y <= list[i].GetMaxY() + 2 && y >= list[i].GetMinY() - 2) {
				minIndex = i;
			}
		}
	
		//if close enough, assign pixel to nearest SMR, otherwise create new SMR from pixel and add to list
		if (minIndex >= 0) {
			list[minIndex].AddPixel(x, y);
			lastIndex = minIndex;
		}
		else {
			list.push_back(SMR(x, y));
			lastIndex = list.size() - 1;
		}

	}
	else {
		list[lastIndex].AddPixel(x, y);
	}

}
//have tried to make this function more efficient below

//combine SMRs that are close enough (mostly for combining partial SMRs from different search regions, but can also cluster slightly malformed SMRs)
// std::vector<SMR> ImageProcessor::combineSMRs(std::vector<SMR>& list) {
// 	std::vector<SMR> combined;
// 	while (!list.empty()) {
// 		SMR current = list[0];
// 		list.erase(list.begin());
// 		bool noMatches = false;
// 		while (!noMatches) {
// 			noMatches = true;
// 			for (int i = 0; i < list.size(); i++) {
// 				if (SMRsTouching(current, list[i])) {
// 					noMatches = false;
// 					current = SMR(current, list[i]);
// 					list.erase(list.begin() + i);
// 					break;
// 				}
// 			}
// 		}
// 		noMatches = false;
// 		while (!noMatches) {
// 			noMatches = true;
// 			for (int i = 0; i < combined.size(); i++) {
// 				if (SMRsTouching(current, combined[i])) {
// 					noMatches = false;
// 					current = SMR(current, combined[i]);
// 					combined.erase(combined.begin() + i);
// 					break;
// 				}
// 			}
// 		}
// 		if (current.GetNumPix() >= 3)
// 			combined.push_back(current);
// 	}
// 	return combined;
// }

// Iterate over SMRs and check if current SMR can be combined with that of in the list
std::vector<SMR> ImageProcessor::combineSMRs(std::vector<SMR>& list) {
    std::vector<SMR> combined;

    //std::cout << "In combined SMR" << std::endl;
    for (auto& current : list) {
        bool combinedWithExisting = false;
        
        for (auto& existing : combined) {
            if (SMRsTouching(current, existing)) {
                combinedWithExisting = true;
                existing = SMR(current, existing);
                break;
            }
        }

        // If the current SMR couldn't be combined with any existing SMR, add it to combined list
        if (!combinedWithExisting && current.GetNumPix() >= 3)
            combined.push_back(current);
    }

    return combined;
}


//determines if partial SMRs are close enough to be considered part of the same true SMR
bool ImageProcessor::SMRsTouching(SMR& a, SMR& b) {
	//std::cout << "SMR Touching" << std::endl;
	if (std::min(a.GetNumPix(), b.GetNumPix()) < 3) {	
		return (a.GetMinX() - 1 <= b.GetMaxX() + 1 && a.GetMaxX() + 1 >= b.GetMinX() - 1 && a.GetMinY() - 1 <= b.GetMaxY() + 1 && a.GetMaxY() + 1 >= b.GetMinY() - 1);
	}
	else {
		int maxSize = std::max(std::max(a.GetMaxX() - a.GetMinX(), a.GetMaxY() - a.GetMinY()), std::max(b.GetMaxX() - b.GetMinX(), b.GetMaxY() - b.GetMinY()));
		int touchDistance = std::min(std::max(maxSize/3, 2), 30);
		return (a.GetMinX() - touchDistance <= b.GetMaxX() + touchDistance && a.GetMaxX() + touchDistance >= b.GetMinX() - touchDistance && a.GetMinY() - touchDistance <= b.GetMaxY() + touchDistance && a.GetMaxY() + touchDistance >= b.GetMinY() - touchDistance);
	}
}


cv::Point2f ImageProcessor::interpolateDelta(ImgDelta d, float y, float h) {
	float c1 = y / h;
	float c2 = 1 - c1;
	return cv::Point2f(c1*d.botX+c2*d.topX, c1*d.botY+c2*d.topY);
}