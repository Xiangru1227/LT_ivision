#ifndef UTIL_H
#define UTIL_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

class Prm {
public:
    Prm(double D, double H1, double H2);
    double D;
    double H1;
    double H2;
};

cv::Mat YUV2BGR(const std::string& root_path, const std::string& sequence);
std::tuple<std::vector<cv::Point2f>, std::vector<double>, cv::Mat> color_detection(const cv::Mat& img, const cv::Scalar& hsv_min, const cv::Scalar& hsv_max, const cv::Point2f& ref_coord, double size_ts, bool draw = false);
cv::Point2f find_red_areas(const cv::Mat& img, bool draw = false);
cv::Mat filter_red(const cv::Mat& img);
double circularity(const std::vector<cv::Point>& contour);
std::vector<double> iPb_uv2pyr(const std::vector<cv::Point2f>& keypoints, const Prm& prm);
void show_image(const cv::Mat& img, const std::string& name, const cv::Size& size, const cv::Point& position);
void visualize_hsv_range(const cv::Scalar& hsv_min, const cv::Scalar& hsv_max);
std::vector<double> dist2SMR_coord(double distance, const std::string& json_file_path);
std::pair<double, double> dist2ROI_size(double distance, const std::string& json_file_path);
cv::Mat crop_image(const cv::Mat& img, const cv::Point2f& center, double length);

#endif // UTIL_H
