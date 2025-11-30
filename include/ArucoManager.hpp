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
    // Retourne la liste des IDs détectés (vide si aucun)
    std::vector<int> detectAruco(const cv::Mat& frame, cv::Mat& outputFrame);

    // Permet de configurer la dictionary (par défaut DICT_6X6_250)
    void setDictionary(int dictionaryId);

    // (no template-fallback API)

    // Retourne le comportement actuel basé sur le dernier tag détecté
    int getCurrentBehavior() const;

private:
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> parameters;
    int currentBehavior; // 0: manuel, 1: exploration, 2: wall-following, etc.
    // Known tags for fallback template matching (useful in constrained setups)
    // no extra members for fallback
};

#endif // ARUCO_MANAGER_H
