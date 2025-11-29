#include "ArucoManager.hpp"
#include <opencv2/opencv.hpp>

ArucoManager::ArucoManager() : currentBehavior(0) {
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    parameters = cv::aruco::DetectorParameters::create();
}

ArucoManager::~ArucoManager() {}

int ArucoManager::detectAruco(const cv::Mat& frame, cv::Mat& outputFrame) {
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters);

    outputFrame = frame.clone();
    if (!markerIds.empty()) {
        cv::aruco::drawDetectedMarkers(outputFrame, markerCorners, markerIds);
        // Prendre le premier tag détecté (ou gérer plusieurs si nécessaire)
        int detectedId = markerIds[0];
        // Changer le comportement basé sur l'ID (persistant)
        if (detectedId == 0) {
            currentBehavior = 1; // Exploration
        } else if (detectedId == 5) {
            currentBehavior = 2; // Wall-following
        } else if (detectedId == 10) {
            currentBehavior = 0; // Manuel
        }
        // Afficher l'ID sur l'image
        cv::putText(outputFrame, "ID: " + std::to_string(detectedId), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
        return detectedId;
    }
    return -1; // Aucun tag détecté
}

int ArucoManager::getCurrentBehavior() const {
    return currentBehavior;
}