

#ifndef TRACKERPRMREADER_H
#define TRACKERPRMREADER_H
#pragma once

#include <iostream>
#include <vector>
#include "json.h"
#include <fstream>
#include <string.h>
#include <utility>
#include "CameraInterface.h"


struct CamCalibStruct {
	double cam_fx;
	double cam_cx;
	double cam_fy;
	double cam_cy;

	double dist_k1;
	double dist_k2;
	double dist_p1;
	double dist_p2;
	double dist_k3;

	CamCalibStruct() {
		cam_fx = 0;
		cam_cx = 0;
		cam_fy = 0;
		cam_cy = 0;

		dist_k1 = 0;
		dist_k2 = 0;
		dist_p1 = 0;
		dist_p2 = 0;
		dist_k3 = 0;
	}
};

struct ParallaxTableStruct {
	std::vector<float> distance;
	std::vector<std::pair<float,float>> offset;
	//std::vector<float> size;
	double auto_calib_x_offset;
	double auto_calib_y_offset;

	ParallaxTableStruct() {
		distance.clear();
		offset.clear();
		//size.clear();
		auto_calib_x_offset = 0.0;
		auto_calib_y_offset = 0.0;

	}
};

struct CalibrationPoint {
	float distance;
	float az;
	float el;
};

/*
	Class holding all the informtion in prm file

*/
class CalibData {
public:

	CamCalibStruct camCalib;
	ParallaxTableStruct parallaxTable;
	unsigned int serial_num;
	uint8_t bright_threshold;
	uint8_t red_thresh_y;
	uint8_t red_thresh_cr;
	uint8_t green_thresh_cr;
	uint8_t green_thresh_cb;
	CameraProperties CamProp;
	float flash_duration;
	float flash_brightness;
	float flash_offset;
	bool back_side_detection;
	float spiral_threshold;
	float spiral_freq;
	int spiral_timeout;
	bool cam_auto_calib;

	CalibData() {
        camCalib = CamCalibStruct();
		parallaxTable = ParallaxTableStruct();
		serial_num = 0;
		bright_threshold = 160;
		red_thresh_y = 70;
		red_thresh_cr = 25;
		green_thresh_cr = 22;
		green_thresh_cb = 22;
		CamProp = CameraProperties();
		flash_brightness = 0.1;
		flash_duration = 40.0;
		flash_offset = 1.0;
		back_side_detection = 0;
		spiral_threshold = 1.0;
		spiral_timeout = 10;
		spiral_freq = 0.004f;
	}

    int clearAll(){
        camCalib = CamCalibStruct();
		parallaxTable = ParallaxTableStruct();
		serial_num = 0;
		bright_threshold = 160;
		red_thresh_y = 70;
		red_thresh_cr = 25;
		green_thresh_cr = 22;
		green_thresh_cb = 22;
		CamProp = CameraProperties();
		flash_brightness = 0.1;
		flash_duration = 40.0;
		flash_offset = 1.0;
		back_side_detection = 0;
		spiral_threshold = 1.0;
		spiral_timeout = 10;
		spiral_freq = 0.004f;
        return 0;
    }


};


class CalibHandle {
private:
	CalibData cData;
	Json::Value jsonReader;


public:
	CalibHandle();
	~CalibHandle();

	int readCalibFile(const char * filePath);
	int parseCalibFile();

	int writeCalibFile(const char *);
	int updateCalibFields();

	int getUpdatedCalibData(CalibData& val);
	int updateCalibData(const CalibData & val);

};



#endif
