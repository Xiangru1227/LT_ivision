#include "util.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>

using json = nlohmann::json;

int main() {
    double D = 53.0;
    double H1 = 125.0 / 2;
    double H2 = -125.0 / 2;
    Prm prm(D, H1, H2);
    
    // Modify these parameters to input different datasets
    std::string root_path = "7.8/2.4m";
    std::string sequence = "1";
    double distance = 2403.128;
    std::string cam_cal = "cam_calibration_50506.json";
    std::string map = "distance_mapping.json";
    
    bool _draw = true;
    cv::Scalar hsv_min(50, 50, 50);
    cv::Scalar hsv_max(100, 255, 255);
    // visualize_hsv_range(hsv_min, hsv_max);
    
    cv::Mat bgr_img = YUV2BGR(root_path, sequence);
    // cv::imwrite("bgr_img_5m.png", bgr_img);
    // show_image(bgr_img, "BGR image", cv::Size(800, 600), cv::Point(500, 200));
    
    // Find SMR by importing from cam cal file
    std::vector<double> coord_normed = dist2SMR_coord(distance / 1000, cam_cal);
    std::vector<double> SMR_coord = { (coord_normed[0] + 1) * bgr_img.cols / 2, (coord_normed[1] + 1) * bgr_img.rows / 2 };
    
    // Find SMR by searching in the image
    // cv::Mat SMR_img = bgr_img.clone();
    // cv::Point2f SMR_coord = find_red_areas(SMR_img, false);
    // bgr_img = filter_red(bgr_img);
    // auto [ROI_size, LED_filter_size] = dist2ROI_size(distance, map);
    // cv::Mat ROI_img = crop_image(bgr_img, SMR_coord, ROI_size);
    // show_image(ROI_img, "ROI", cv::Size(800, 600), cv::Point(500, 200));
    
    // auto [ROI_centroids, ROI_area, ROI_contoured] = color_detection(ROI_img, hsv_min, hsv_max, SMR_coord, LED_filter_size * 0.75, _draw);
    // for (auto& centroid : ROI_centroids) {
    //     centroid += cv::Point2f(SMR_coord[0] - ROI_size, SMR_coord[1] - ROI_size);
    // }
    
    auto [centroids, area, img_contoured] = color_detection(bgr_img, hsv_min, hsv_max, cv::Point2f(SMR_coord[0], SMR_coord[1]), 100, _draw);
    // cv::imwrite("contoured_image_5m.png", img_contoured);
    
    // std::cout << "Number of keypoints detected: " << ROI_centroids.size() << std::endl;
    // std::cout << "Coordinates and areas (P1, P3, P2):" << std::endl;
    // for (size_t i = 0; i < ROI_centroids.size(); ++i) {
    //     std::cout << ROI_centroids[i] << "\t" << ROI_area[i] << std::endl;
    // }
    
    std::cout << "Number of keypoints detected: " << centroids.size() << std::endl;
    std::cout << "Coordinates and areas (P1, P3, P2):" << std::endl;
    for (size_t i = 0; i < centroids.size(); ++i) {
        std::cout << centroids[i] << "\t" << area[i] << std::endl;
    }
    
    // auto pyrs = iPb_uv2pyr(ROI_centroids, prm);
    // std::cout << "Pitch, Yaw, Roll and Scale:" << std::endl;
    // std::cout << pyrs[0] << ", " << pyrs[1] << ", " << pyrs[2] << ", " << pyrs[3] << std::endl;
    
    // std::cout << "Crop length:" << std::endl;
    // std::cout << cv::norm(SMR_coord - centroids[0]) + 1.5 * std::sqrt(area[0] / CV_PI) << std::endl;
    // std::cout << "Min size:" << std::endl;
    // std::cout << *std::min_element(area.begin(), area.end()) << std::endl;
    
    // mark the expected position of the SMR in the image
    cv::circle(img_contoured, cv::Point(static_cast<int>(SMR_coord[0]), static_cast<int>(SMR_coord[1])), 5, cv::Scalar(255, 0, 0), 10);
    // cv::circle(ROI_contoured, cv::Point(static_cast<int>(SMR_coord[0]), static_cast<int>(SMR_coord[1])), 5, cv::Scalar(255, 0, 0), 10);
    
    if (_draw) {
        show_image(img_contoured, "Image with contour", cv::Size(800, 600), cv::Point(500, 200));
        // show_image(ROI_contoured, "ROI with contour", cv::Size(800, 600), cv::Point(500, 200));
    }

    return 0;
}
