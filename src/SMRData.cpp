#include "SMRData.h"
#include <math.h>
#include <iostream>

//default constructor
SMRData::SMRData() {
	imgAz = 0;
	imgEl = 0;
	az = 0;
	el = 0;
	imgA = 0;
	imgW = 0;
	imgH = 0;
	score = 0;
	laserHit = false;
	imgX = 0;
	imgY = 0;
}

//constructor with initial image data
SMRData::SMRData(float a, float e, float ar, float w, float h) {
	imgAz = a;
	imgEl = e;
	az = a;
	el = e;
	imgA = ar;
	imgW = w;
	imgH = h;
	score = 0;
	laserHit = false;
	imgX = 0;
	imgY = 0;
}

//copy constructor
SMRData::SMRData(const SMRData& d) {
	imgA = d.imgA;
	imgW = d.imgW;
	imgH = d.imgH;
	updateTo(d);
}

//destructor
SMRData::~SMRData() {

}

void SMRData::setImgCoordinates(float x, float y, cv::Size imgRes) {
	imgX = x / imgRes.width;
	imgY = y / imgRes.height;
}

//checks that angles are close and size is roughly similar
bool SMRData::isSame(SMRData d) {
	float azDist = std::abs(az - d.getAz());
	float elDist = std::abs(el - d.getEl());
	float sizeRatio = imgA / d.getImgPix();
	return azDist < 0.085f && elDist < 0.085f && sizeRatio > .5f && sizeRatio < 2.0f;
}

//TODO: change distance threshold based on size
float SMRData::validDistance(SMRData d, cv::Point2f pixelAngle) {
	float azDist = std::abs(az - d.az);
	float elDist = std::abs(el - d.el);
	float sizeRatio = imgA / d.imgA;
	//std::cout << imgA << " " << d.getImgPix() << " " << azDist << " " << elDist << std::endl;
	float maxDim = std::max(imgW, imgH);
	float azThresh = 4.0f * maxDim * pixelAngle.x;
	float elThresh = 4.0f * maxDim * pixelAngle.y;
	if (sizeRatio > .5f && sizeRatio < 2.0f && azDist < azThresh && elDist < elThresh)
	//std::cout << "AZ Thresh: " << azThresh << ", EL Thresh: " << elThresh << ", Max Dim: " << maxDim << std::endl;
		return azDist * azDist + elDist * elDist;
	return -1;
}

//set all data to data from other SMRData
void SMRData::updateTo(const SMRData& d) {
	/*imgAz = d.getImgAz();
	imgEl = d.getImgEl();
	az = d.getAz();
	el = d.getEl();
	imgA = d.getImgArea();
	imgW = d.getImgWidth();
	imgH = d.getImgHeight();
	*/
	
	imgAz = d.imgAz;
	imgEl = d.imgEl;
	az = d.az;
	el = d.el;
	imgA = (imgA + d.imgA) / 2;
	imgW = (imgW + d.imgW) / 2;
	imgH = (imgH + d.imgH) / 2;
	score = d.score;
	laserHit = d.laserHit;
	imgX = d.imgX;
	imgY = d.imgY;
}





//start with no history by default
ShakeSMR::ShakeSMR() {
	Clear();
}

//start with one angle point
ShakeSMR::ShakeSMR(float a, float e) {
	Clear();
	AddAzEl(cv::Point2f(a,e));
}

//destructor
ShakeSMR::~ShakeSMR() {

}

//add new angle point to history, updates bounds and total distance
void ShakeSMR::AddAzEl(cv::Point2f azel) {
	if (lastAzEls.size() > 20) {
		float diffX = std::abs(lastAzEls[1].x - lastAzEls[0].x);
		float diffY = std::abs(lastAzEls[1].y - lastAzEls[0].y);
		totalDistance.x -= diffX;
		totalDistance.y -= diffY;
		lastAzEls.erase(lastAzEls.begin());
	}
	lastAzEls.push_back(azel);
	int numPoints = lastAzEls.size();
	if (lastAzEls.size() > 1) {
		float diffX = std::abs(lastAzEls[numPoints - 1].x - lastAzEls[numPoints - 2].x);
		float diffY = std::abs(lastAzEls[numPoints - 1].y - lastAzEls[numPoints - 2].y);
		totalDistance.x += diffX;
		totalDistance.y += diffY;
	}
	float maxX = -10000;
	float maxY = -10000;
	float minX = 10000;
	float minY = 10000;
	for (int i = 0; i < numPoints; i++) {
		if (lastAzEls[i].x > maxX) {
			maxX = lastAzEls[i].x;
		}
		if (lastAzEls[i].x < minX) {
			minX = lastAzEls[i].x;
		}
		if (lastAzEls[i].y > maxY) {
			maxY = lastAzEls[i].y;
		}
		if (lastAzEls[i].y < minY) {
			minY = lastAzEls[i].y;
		}
	}
	upperBound = cv::Point2f(maxX, maxY);
	lowerBound = cv::Point2f(minX, minY);
}

//get size based on angle bounds
cv::Point2f ShakeSMR::GetSize() {
	return cv::Point2f(upperBound.x - lowerBound.x, upperBound.y - lowerBound.y);
}

//clear all history
void ShakeSMR::Clear() {
	lastAzEls.clear();
	totalDistance = cv::Point2f(0,0);
	upperBound = cv::Point2f(0,0);
	lowerBound = cv::Point2f(0,0);
}
