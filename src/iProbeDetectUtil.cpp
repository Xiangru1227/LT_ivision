#include "util.h"
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <algorithm>

Prm::Prm(double D, double H1, double H2) : D(D), H1(H1), H2(H2) {}

cv::Mat YUV2BGR(const std::string& root_path, const std::string& sequence) {
    std::string imgY_path = root_path + "/" + sequence + "/src2Y.png";
    std::string imgU_path = root_path + "/" + sequence + "/src2U.png";
    std::string imgV_path = root_path + "/" + sequence + "/src2V.png";
    
    cv::Mat imgY = cv::imread(imgY_path, cv::IMREAD_GRAYSCALE);
    cv::Mat imgU = cv::imread(imgU_path, cv::IMREAD_GRAYSCALE);
    cv::Mat imgV = cv::imread(imgV_path, cv::IMREAD_GRAYSCALE);
    
    if (imgY.empty() || imgU.empty() || imgV.empty()) {
        return cv::Mat();
    }
    
    cv::resize(imgU, imgU, imgY.size(), 0, 0, cv::INTER_LINEAR);
    cv::resize(imgV, imgV, imgY.size(), 0, 0, cv::INTER_LINEAR);
    
    std::vector<cv::Mat> yuv_img = { imgY, imgU, imgV };
    cv::Mat bgr_img;
    cv::cvtColor(yuv_img, bgr_img, cv::COLOR_YUV2BGR);
    
    // bgr_img = filter_red(bgr_img);
    
    return bgr_img;
}

std::tuple<std::vector<cv::Point2f>, std::vector<double>, cv::Mat> color_detection(const cv::Mat& img, const cv::Scalar& hsv_min, const cv::Scalar& hsv_max, const cv::Point2f& ref_coord, double size_ts, bool draw) {
    cv::Mat hsv_img, mask;
    cv::cvtColor(img, hsv_img, cv::COLOR_BGR2HSV);
    cv::inRange(hsv_img, hsv_min, hsv_max, mask);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::Mat::ones(5, 5, CV_8U));

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<std::pair<cv::Point2f, double>> centroids_and_areas;
    std::vector<std::vector<cv::Point>> filtered_contours;

    for (const auto& c : contours) {
        double area = cv::contourArea(c);
        double perimeter = cv::arcLength(c, true);
        double circ = 4 * CV_PI * (area / (perimeter * perimeter));
        if (area > size_ts && 0.7 <= circ && circ <= 1.0) {
            cv::Moments M = cv::moments(c);
            double cX = M.m10 / M.m00;
            double cY = M.m01 / M.m00;
            filtered_contours.push_back(c);
            centroids_and_areas.push_back({ cv::Point2f(cX, cY), area });
        }
    }

    std::sort(centroids_and_areas.begin(), centroids_and_areas.end(), [&ref_coord](const auto& a, const auto& b) {
        return cv::norm(a.first - ref_coord) > cv::norm(b.first - ref_coord);
    });

    std::vector<cv::Point2f> centroids;
    std::vector<double> areas;
    for (const auto& ca : centroids_and_areas) {
        centroids.push_back(ca.first);
        areas.push_back(ca.second);
    }

    cv::Mat result_img;
    if (draw) {
        result_img = img.clone();
        cv::drawContours(result_img, filtered_contours, -1, cv::Scalar(0, 0, 255), 2);
        for (const auto& centroid : centroids) {
            cv::circle(result_img, centroid, 10, cv::Scalar(0, 0, 255), -1);
        }
    }

    return { centroids, areas, result_img };
}

cv::Point2f find_red_areas(const cv::Mat& img, bool draw) {
    cv::Mat hsv_img;
    cv::cvtColor(img, hsv_img, cv::COLOR_BGR2HSV);

    cv::Mat mask1, mask2, mask;
    cv::inRange(hsv_img, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), mask1);
    cv::inRange(hsv_img, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask2);
    mask = mask1 | mask2;

    cv::Mat kernel = cv::Mat::ones(5, 5, CV_8U);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 3);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point2f> centroids;
    std::vector<std::vector<cv::Point>> selected_contours;

    for (const auto& c : contours) {
        cv::Moments M = cv::moments(c);
        if (M.m00 > 500) {
            int cX = static_cast<int>(M.m10 / M.m00);
            int cY = static_cast<int>(M.m01 / M.m00);
            centroids.push_back(cv::Point2f(cX, cY));
            selected_contours.push_back(c);
        }
    }

    if (centroids.size() == 1) {
        if (draw) {
            cv::Mat result_img = img.clone();
            cv::drawContours(result_img, selected_contours, -1, cv::Scalar(255, 0, 0), 2);
            for (const auto& centroid : centroids) {
                cv::circle(result_img, centroid, 5, cv::Scalar(255, 0, 0), -1);
            }
            return { centroids[0], result_img };
        } else {
            return centroids[0];
        }
    } else if (centroids.empty()) {
        throw std::runtime_error("No SMR detected.");
    } else {
        throw std::runtime_error("More than 1 SMR detected.");
    }
}

cv::Mat filter_red(const cv::Mat& img) {
    cv::Mat hsv_img;
    cv::cvtColor(img, hsv_img, cv::COLOR_BGR2HSV);

    cv::Mat mask1, mask2, mask;
    cv::inRange(hsv_img, cv::Scalar(0, 50, 50), cv::Scalar(30, 255, 255), mask1);
    cv::inRange(hsv_img, cv::Scalar(160, 50, 50), cv::Scalar(190, 255, 255), mask2);
    mask = mask1 + mask2;

    cv::Mat filtered_img;
    cv::bitwise_and(img, img, filtered_img, cv::bitwise_not(mask));
    return filtered_img;
}

double circularity(const std::vector<cv::Point>& contour) {
    double area = cv::contourArea(contour);
    double perimeter = cv::arcLength(contour, true);
    if (perimeter == 0) return 0;
    return 4 * CV_PI * (area / (perimeter * perimeter));
}

std::vector<double> iPb_uv2pyr(const std::vector<cv::Point2f>& keypoints, const Prm& prm) {
    std::vector<double> pyrs(4, 0.0);

    double u1 = keypoints[0].x - keypoints[1].x;
    double v1 = -(keypoints[0].y - keypoints[1].y);
    double u2 = keypoints[2].x - keypoints[1].x;
    double v2 = -(keypoints[2].y - keypoints[1].y);

    double roll = std::atan2((u1 - u2), (v1 - v2)) * 180 / CV_PI;
    double sr = std::sin(roll * CV_PI / 180);
    double cr = std::cos(roll * CV_PI / 180);

    double m = std::sqrt(((u1 - u2) * (u1 - u2) + (v1 - v2) * (v1 - v2)) / ((prm.H1 - prm.H2) * (prm.H1 - prm.H2)));
    double n = (sr * v1 - cr * u1) / prm.D;
    double k = (sr * u1 + cr * v1 - prm.H1 * m) / prm.D;

    double scale;
    if (std::abs(n) > 0.00001) {
        double temp1 = m * m + n * n + k * k;
        double temp2 = m * m * n * n;
        double ss = (temp1 - std::sqrt(temp1 * temp1 - 4 * temp2)) / (2 * temp2);
        scale = std::sqrt(ss);
    } else {
        scale = 1 / std::sqrt(m * m + k * k);
    }

    double yaw = std::asin(n * scale) * 180 / CV_PI;
    double pitch = std::asin(k * scale / std::cos((yaw / 180) * CV_PI)) * 180 / CV_PI;

    pyrs[0] = pitch;
    pyrs[1] = yaw;
    pyrs[2] = roll;
    pyrs[3] = scale;

    return pyrs;
}

void show_image(const cv::Mat& img, const std::string& name, const cv::Size& size, const cv::Point& position) {
    cv::namedWindow(name, cv::WINDOW_NORMAL);
    cv::resizeWindow(name, size.width, size.height);
    cv::moveWindow(name, position.x, position.y);
    cv::imshow(name, img);
    cv::waitKey(0);
}

void visualize_hsv_range(const cv::Scalar& hsv_min, const cv::Scalar& hsv_max) {
    cv::Mat gradient(100, 500, CV_8UC3);

    for (int i = 0; i < gradient.cols; ++i) {
        double ratio = static_cast<double>(i) / gradient.cols;
        cv::Scalar hsv_color(
            hsv_min[0] * (1 - ratio) + hsv_max[0] * ratio,
            hsv_min[1] * (1 - ratio) + hsv_max[1] * ratio,
            hsv_min[2] * (1 - ratio) + hsv_max[2] * ratio
        );
        gradient.col(i).setTo(hsv_color);
    }

    cv::Mat bgr_gradient;
    cv::cvtColor(gradient, bgr_gradient, cv::COLOR_HSV2BGR);
    cv::imshow("HSV Visualization", bgr_gradient);
    cv::waitKey(0);
}

std::vector<double> dist2SMR_coord(double distance, const std::string& json_file_path) {
    std::ifstream file(json_file_path);
    nlohmann::json data;
    file >> data;

    std::vector<double> d = data["Parallax"]["Distance"].get<std::vector<double>>();
    std::vector<double> x = data["Parallax"]["X"].get<std::vector<double>>();
    std::vector<double> y = data["Parallax"]["Y"].get<std::vector<double>>();

    auto idx_1 = std::max_element(d.begin(), d.end(), [distance](double a, double b) { return a <= distance && b > distance; });
    auto idx_2 = std::min_element(d.begin(), d.end(), [distance](double a, double b) { return a >= distance && b < distance; });

    if (idx_1 == idx_2) {
        return { x[idx_1 - d.begin()], y[idx_1 - d.begin()] };
    } else {
        double d1 = *idx_1;
        double d2 = *idx_2;
        double x1 = x[idx_1 - d.begin()];
        double x2 = x[idx_2 - d.begin()];
        double y1 = y[idx_1 - d.begin()];
        double y2 = y[idx_2 - d.begin()];

        double x_pred = (x2 - x1) * (distance - d1) / (d2 - d1) + x1;
        double y_pred = (y2 - y1) * (distance - d1) / (d2 - d1) + y1;

        return { x_pred, y_pred };
    }
}

std::pair<double, double> dist2ROI_size(double distance, const std::string& json_file_path) {
    std::ifstream file(json_file_path);
    nlohmann::json data;
    file >> data;

    std::vector<double> distances = data["Distance"].get<std::vector<double>>();
    std::vector<double> roi_sizes = data["ROI_size"].get<std::vector<double>>();
    std::vector<double> led_sizes = data["LED_size"].get<std::vector<double>>();

    if (distance < *std::min_element(distances.begin(), distances.end()) || distance > *std::max_element(distances.begin(), distances.end())) {
        throw std::runtime_error("Distance is out of the range.");
    }

    auto idx_1 = std::max_element(distances.begin(), distances.end(), [distance](double a, double b) { return a <= distance && b > distance; });
    auto idx_2 = std::min_element(distances.begin(), distances.end(), [distance](double a, double b) { return a >= distance && b < distance; });

    if (idx_1 == idx_2) {
        return { roi_sizes[idx_1 - distances.begin()], led_sizes[idx_1 - distances.begin()] };
    } else {
        double d1 = *idx_1;
        double d2 = *idx_2;
        double roi1 = roi_sizes[idx_1 - distances.begin()];
        double roi2 = roi_sizes[idx_2 - distances.begin()];
        double led1 = led_sizes[idx_1 - distances.begin()];
        double led2 = led_sizes[idx_2 - distances.begin()];

        double roi_size = (roi2 - roi1) * (distance - d1) / (d2 - d1) + roi1;
        double led_size = (led2 - led1) * (distance - d1) / (d2 - d1) + led1;

        return { roi_size, led_size };
    }
}

cv::Mat crop_image(const cv::Mat& img, const cv::Point2f& center, double length) {
    int x_start = std::max(static_cast<int>(center.x - length), 0);
    int y_start = std::max(static_cast<int>(center.y - length), 0);
    int x_end = std::min(static_cast<int>(center.x + length), img.cols);
    int y_end = std::min(static_cast<int>(center.y + length), img.rows);

    cv::Mat cropped_img = img(cv::Rect(x_start, y_start, x_end - x_start, y_end - y_start)).clone();

    return filter_red(cropped_img);
}
