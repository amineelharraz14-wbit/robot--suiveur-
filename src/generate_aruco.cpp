#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>

int main() {
    cv::Ptr<cv::aruco::Dictionary> dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    std::vector<int> ids = {0, 5, 10};
    std::vector<int> sizes = {200, 400, 800};
    for (int id : ids) {
        for (int s : sizes) {
            cv::Mat marker;
            cv::aruco::drawMarker(dict, id, s, marker, 1);
            std::string path = "../data/marker_" + std::to_string(id) + "_" + std::to_string(s) + ".png";
            if (!cv::imwrite(path, marker)) {
                std::cerr << "Failed to write " << path << std::endl;
                return 1;
            }
            std::cout << "Wrote " << path << std::endl;
        }
    }
    return 0;
}
