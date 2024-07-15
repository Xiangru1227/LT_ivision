#include "ObjectDetector.h"


//constructor
ObjectDetector::ObjectDetector() {
	active = true;
	net = NULL;
	imgCPU    = NULL;
	imgCUDA    = NULL;
	imgWidth  = 0;
	imgHeight = 0;
}

//destructor (don't need to do anything now)
ObjectDetector::~ObjectDetector() {

}




//create net, save image size, allocate image buffer
bool ObjectDetector::setup(cv::Size resolution, detectNet::NetworkType type) {

	//don't do anything if detector is not active
	if (!active)
		return false;

	//set up neural network (will take several seconds)
	net = detectNet::Create(type);
	if( !net ) {
		std::cout << "detectnet-console:   failed to initialize detectNet" << std::endl;
		return false;
	}

	//store image size for future use
	imgWidth = (int)resolution.width;
	imgHeight = (int)resolution.height;

	//allocate buffer accessible by both CPU and GPU
	if( !cudaAllocMapped((void**)&imgCPU, (void**)&imgCUDA, imgWidth * imgHeight * sizeof(float) * 4) ) {
		std::cout << "failed to allocate " << imgWidth * imgHeight * sizeof(float) * 4 << " bytes" << std::endl;
		return false;
	}

	return true;
}

//delete net, free buffer
bool ObjectDetector::close() {
	SAFE_DELETE(net);
	CUDA(cudaFreeHost(imgCPU));
	return true;
}

//copy image data into image buffer, notify detector of new image
bool ObjectDetector::copyImageToBuffer(cv::Mat img) {
	//make sure buffer and image data are valid
	if (imgCPU == NULL || img.empty())
		return false;
	//make sure only one thread accesses image buffer at a time
	std::lock_guard<std::mutex> guard(imgAccess);
	hasNewDetectionImage = true;
	memcpy(imgCPU, img.data, img.cols * img.rows * sizeof(float) * 4);
	return true;
}

//perform detection, update detected object list
bool ObjectDetector::processFromBuffer() {

	//detect objects
	detectNet::Detection* detections = NULL;
	int numDetections;
	//only do one detection for each new image
	if (hasNewDetectionImage) {
		std::lock_guard<std::mutex> guardImg(imgAccess);
		hasNewDetectionImage = false;
		numDetections = net->Detect(imgCUDA, imgWidth, imgHeight, &detections, 0);
	}
	else {
		return false;
	}

	//make sure only one thread accesses detected object list at a time
	std::lock_guard<std::mutex> guardList(listAccess);

	//copy detected objects into list
	currentDetectedObjects.clear();
	if( numDetections > 0 ) {
		//printf("%i objects detected\n", numDetections);
		for( int n=0; n < numDetections; n++ ) {
			DetectedObject object;
			object.classNum = detections[n].ClassID;
			object.confidence = detections[n].Confidence;
			object.boxLeft = detections[n].Left / imgWidth;
			object.boxRight = detections[n].Right / imgWidth;
			object.boxUp = detections[n].Top / imgHeight;
			object.boxDown = detections[n].Bottom / imgHeight;
			currentDetectedObjects.push_back(object);
			//std::cout << "Object: " << object.classNum << " with confidence " << object.confidence << std::endl;
			//std::cout << "L = " << object.boxLeft << ", R = " << object.boxRight << ", T = " << object.boxUp << ", B = " << object.boxDown << std::endl;
		}
	}
	else if (numDetections == -1) {
		std::cout << "Detection encountered error." << std::endl;
	}

	return true;
}

//copy image into buffer and then immediately process from buffer
bool ObjectDetector::processImage(cv::Mat img) {
	copyImageToBuffer(img);
	return processFromBuffer();
}

//return detected object list while making sure only one thread accesses it at a time
std::vector<DetectedObject> ObjectDetector::getDetectedObjects() {
	std::lock_guard<std::mutex> guardList(listAccess);
	return currentDetectedObjects;
}
