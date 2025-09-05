#pragma once

#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include "json.h"
#include <algorithm>
#include <opencv2/core.hpp>

struct IprobeCalibData {
    double distance;
    double ROI_range;
    double LED_range;
    double Centroid_area;
};

class IprobeCalibHandle {
private:
    std::vector<IprobeCalibData> iprobeCalibData;
    Json::Value jsonReader;
    std::string iprobeCalibFilePath = "iprobe_calibration.json";

public:
    IprobeCalibHandle();
    ~IprobeCalibHandle();

    std::vector<IprobeCalibData> getIprobeCalibData();
    int readIprobeCalibrationFile();
    int readIprobeCalibrationFile(const std::string& filePath);
    int parseIprobeCalibrationFile();
};
