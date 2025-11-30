#ifndef ARUCO_MANAGER_H
#define ARUCO_MANAGER_H
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>

// Classe pour gérer la détection des tags ArUco et changer le comportement
class ArucoManager {
public:
    ArucoManager();
    ~ArucoManager();

    // Détecte les tags ArUco dans l'image de la caméra
    // Retourne l'ID du tag détecté (ou -1 si aucun)
    int detectAruco(const cv::Mat& frame, cv::Mat& outputFrame);

    // Retourne le comportement actuel basé sur le dernier tag détecté
    int getCurrentBehavior() const;

private:
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> parameters;
    int currentBehavior; // 0: manuel, 1: exploration, 2: wall-following, etc.
};

#endif // ARUCO_MANAGER_H
