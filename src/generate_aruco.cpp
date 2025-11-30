#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>

int main() {
    cv::Ptr<cv::aruco::Dictionary> dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    std::vector<int> ids = {0, 5, 10};
    // Generate 3 clean high-resolution markers (with quiet border) - one per behavior
    int marker_size = 800; // px for good print quality
    int quiet_pixels = marker_size / 8; // add a white quiet zone around marker
    for (int id : ids) {
        cv::Mat marker;
        cv::aruco::drawMarker(dict, id, marker_size, marker, 1);

        // Create image with white quiet zone border
        cv::Mat with_border(marker_size + quiet_pixels*2, marker_size + quiet_pixels*2, marker.type(), cv::Scalar(255));
        marker.copyTo(with_border(cv::Rect(quiet_pixels, quiet_pixels, marker_size, marker_size)));

        std::string path = "../data/marker_" + std::to_string(id) + ".png";
        if (!cv::imwrite(path, with_border)) {
            std::cerr << "Failed to write " << path << std::endl;
            return 1;
        }
        std::cout << "Wrote " << path << std::endl;
    }
    return 0;
}
