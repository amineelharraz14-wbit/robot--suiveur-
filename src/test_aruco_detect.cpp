#include <iostream>
#include <opencv2/opencv.hpp>
#include "ArucoManager.hpp"

int main() {
    std::string image_path = "../data/test_aruco.png"; // relative from build
    cv::Mat img = cv::imread(image_path);
    if (img.empty()) {
        std::cerr << "Failed to open image: " << image_path << std::endl;
        return 1;
    }

    std::cout << "Testing image: " << image_path << " (" << img.cols << "x" << img.rows << ")" << std::endl;

    // Also check the project's ArucoManager logic specifically
    // Use default ArucoManager (detectMarkers only)
    ArucoManager am;
    cv::Mat out;
    std::vector<int> res = am.detectAruco(img, out);
    std::cout << "ArucoManager::detectAruco returned " << res.size() << " ids" << std::endl;
    if (!res.empty()) {
        std::cout << "IDs: "; for (int x:res) std::cout << x << " "; std::cout << std::endl;
    }
    std::cout << "ArucoManager::getCurrentBehavior() = " << am.getCurrentBehavior() << std::endl;

    if (!out.empty()) {
        // Save output image to help debugging
        cv::imwrite("/tmp/aruco_debug_out_manager.png", out);
        std::cout << "Wrote annotated image to /tmp/aruco_debug_out_manager.png" << std::endl;
    }

    // Generate and test small synthetic markers (internal) just to check detectMarkers
    auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    for (int testId : {0,5,10}) {
        cv::Mat marker; cv::aruco::drawMarker(dict, testId, 400, marker, 1);
        std::vector<int> ids; std::vector<std::vector<cv::Point2f>> corners; cv::aruco::detectMarkers(marker, dict, corners, ids);
        std::cout << "synthetic " << testId << ": found=" << ids.size() << std::endl;
    }

    // Also test ArucoManager directly on the generated marker files in data/
    for (int testId : {0,5,10}) {
        std::string path = "../data/marker_" + std::to_string(testId) + ".png";
        cv::Mat img2 = cv::imread(path);
        if (img2.empty()) { std::cerr << "Failed reading generated marker: " << path << std::endl; continue; }
        cv::Mat out2;
        std::vector<int> r2 = am.detectAruco(img2, out2);
            std::cout << "ArucoManager detect on " << path << " found " << r2.size() << " ids" << std::endl;
        if (!r2.empty()) {
            std::cout << "IDs: "; for (int id : r2) std::cout << id << " "; std::cout << std::endl;
            cv::imwrite((std::string)"/tmp/aruco_out_" + std::to_string(testId) + ".png", out2);
            std::cout << "Wrote /tmp/aruco_out_" << testId << ".png" << std::endl;
        }
    }
    return 0;
}
