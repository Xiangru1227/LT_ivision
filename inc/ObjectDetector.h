#pragma once

#include "detectNet.h"
#include "cudaMappedMemory.h"
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <mutex>
#include <thread>

//object class ID, confidence that object of that class is observed, and bounding box of observed object
struct DetectedObject {
	int classNum;
	float confidence;
	float boxLeft;
	float boxRight;
	float boxUp;
	float boxDown;
};

//class detects objects in images using a neural network run on the GPU
class ObjectDetector {
public:

	//constructor/destructor
	ObjectDetector();
	~ObjectDetector();

	//setup and shut down object detector
	bool setup(cv::Size resolution, detectNet::NetworkType type);
	bool close();

	//control/determine if object detector is running (set before you call setup, not while it's running)
	void setActive(bool a) { active = a;};
	bool isActive() { return active; };

	//find objects in new image
	bool processImage(cv::Mat img);
	//store new image in buffer
	bool copyImageToBuffer(cv::Mat img);
	//find objects in stored image
	bool processFromBuffer();

	//return the most recently detected objects
	std::vector<DetectedObject> getDetectedObjects();

private:
	//is object detector supposed to run
	bool active;

	//neural network object that performs object detection
	detectNet* net;

	//image buffer pointers
	float* imgCPU;
	float* imgCUDA;

	//image size
	int imgWidth;
	int imgHeight;

	//reports if there is a new image for detection
	bool hasNewDetectionImage;

	//protects access to image buffers
	std::mutex imgAccess;

	//list of objects detected in the last captured image
	std::vector<DetectedObject> currentDetectedObjects;

	//protects access to list
	std::mutex listAccess;
};
