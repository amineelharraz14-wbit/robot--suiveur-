#include "ArucoManager.hpp"
#include <opencv2/opencv.hpp>

ArucoManager::ArucoManager() : currentBehavior(0) {
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    parameters = cv::aruco::DetectorParameters::create();
}

ArucoManager::~ArucoManager() {}

std::vector<int> ArucoManager::detectAruco(const cv::Mat& frame, cv::Mat& outputFrame) {
    // Pré-traitement pour améliorer robustesse
    if (frame.empty()) return {};
    cv::Mat gray;
    if (frame.channels() == 3) cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    else gray = frame;

    // simple preprocessing: blur + equalize
    cv::Mat proc;
    cv::GaussianBlur(gray, proc, cv::Size(3,3), 0);
    cv::equalizeHist(proc, proc);

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;

    // Try detectMarkers at a few scales so small tags can be found
    std::vector<double> scales = {1.0, 2.0, 3.0};
    for (double s : scales) {
        cv::Mat tryImg = (s == 1.0) ? proc : cv::Mat();
        if (s != 1.0) cv::resize(proc, tryImg, cv::Size(), s, s, cv::INTER_LINEAR);

        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;
        cv::aruco::detectMarkers(tryImg, dictionary, corners, ids, parameters);
        if (!ids.empty()) {
            // if image was scaled, remap corners back to original coordinates
            if (s != 1.0) {
                double inv = 1.0 / s;
                for (auto &c : corners) for (auto &pt : c) { pt.x *= inv; pt.y *= inv; }
            }
            markerIds = ids;
            markerCorners = corners;
            break;
        }
    }

    outputFrame = frame.clone();
    if (!markerIds.empty()) {
        cv::aruco::drawDetectedMarkers(outputFrame, markerCorners, markerIds);

        // Mettre à jour le comportement basé sur le premier marqueur (persistant)
        int detectedId = markerIds[0];
        if (detectedId == 0) currentBehavior = 1;       // Exploration
        else if (detectedId == 5) currentBehavior = 2;  // Wall-following
        else if (detectedId == 10) currentBehavior = 0; // Manuel

        // Afficher l'ID (du premier trouvé)
        cv::putText(outputFrame, "ID: " + std::to_string(detectedId), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
    }

    // No template-matching fallback; rely on detectMarkers

    return markerIds;
}

void ArucoManager::setDictionary(int dictionaryId) {
    dictionary = cv::aruco::getPredefinedDictionary(dictionaryId);
}

// no setKnownTags / setFallbackEnabled (fallback removed)

int ArucoManager::getCurrentBehavior() const {
    return currentBehavior;
}