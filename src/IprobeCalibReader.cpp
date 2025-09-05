#include "IprobeCalibReader.h"
#include <string>
#include <algorithm>
#include <iostream>

IprobeCalibHandle::IprobeCalibHandle() {}

IprobeCalibHandle::~IprobeCalibHandle() {}

int IprobeCalibHandle::readIprobeCalibrationFile() {
    return readIprobeCalibrationFile(iprobeCalibFilePath);
}

int IprobeCalibHandle::readIprobeCalibrationFile(const std::string& filePath) {
    try {
        if (filePath.empty()) {
            return -1;
        }

        std::ifstream file(filePath);
        if (!file.is_open()) {
            return -2;
        }

        jsonReader.clear();

        if (file.peek() == std::ifstream::traits_type::eof()) {
            file.close();
            return -3;
        }

        Json::CharReaderBuilder builder;
        std::string errs;

        if (!Json::parseFromStream(builder, file, &jsonReader, &errs)) {
            return -4;
        }

        return parseIprobeCalibrationFile();

    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return -5;
    }
}

int IprobeCalibHandle::parseIprobeCalibrationFile() {
    if (!jsonReader.isMember("distance") ||
        !jsonReader.isMember("ROI_range") ||
        !jsonReader.isMember("LED_range") ||
        !jsonReader.isMember("Max_area")) {
        return -6;
    }

    const Json::Value& distances = jsonReader["distance"];
    const Json::Value& ROI_ranges = jsonReader["ROI_range"];
    const Json::Value& LED_ranges = jsonReader["LED_range"];
    const Json::Value& Centroid_area = jsonReader["Max_area"];

    if (distances.size() != ROI_ranges.size() ||
        distances.size() != LED_ranges.size() ||
        distances.size() != Centroid_area.size()) {
        return -7;
    }

    iprobeCalibData.clear();
    for (unsigned int i = 0; i < distances.size(); ++i) {
        IprobeCalibData data;
        data.distance = distances[i].asDouble();
        data.ROI_range = ROI_ranges[i].asDouble();
        data.LED_range = LED_ranges[i].asDouble();
        data.Centroid_area = Centroid_area[i].asDouble();
        iprobeCalibData.push_back(data);
    }

    return 0;
}

std::vector<IprobeCalibData> IprobeCalibHandle::getIprobeCalibData() {
    return iprobeCalibData;
}