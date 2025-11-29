#include "Exploration.h"
#include <queue>
#include <opencv2/opencv.hpp>

Exploration::Exploration() : explorationDone(false), startPosition(0, 0, 0) {}

Exploration::~Exploration() {}

void Exploration::explore(const std::vector<float>& lidarMeasurements, const cv::Point3f& robotPosition, cv::Mat& occupancyGrid) {
    if (!explorationDone) {
        performExploration(lidarMeasurements, occupancyGrid);
        // Simuler la mise à jour de la grille d'occupation basée sur les mesures LiDAR
        // Ici, on suppose une grille 2D pour simplifier (extension possible à 3D)
        for (size_t i = 0; i < lidarMeasurements.size(); ++i) {
            float angle = i * 360.0f / lidarMeasurements.size();
            float distance = lidarMeasurements[i];
            if (distance > 0) {
                // Calculer la position de l'obstacle et marquer sur la grille
                int x = static_cast<int>(robotPosition.x + distance * cos(angle * CV_PI / 180.0f));
                int y = static_cast<int>(robotPosition.y + distance * sin(angle * CV_PI / 180.0f));
                if (x >= 0 && x < occupancyGrid.cols && y >= 0 && y < occupancyGrid.rows) {
                    occupancyGrid.at<uchar>(y, x) = 0; // Noir pour obstacle
                }
            }
        }
        // Logique pour vérifier si tous les coins sont explorés (simplifié)
        // Par exemple, compter les cellules explorées
        int exploredCells = cv::countNonZero(occupancyGrid);
        if (exploredCells > occupancyGrid.total() * 0.9) { // 90% exploré
            explorationDone = true;
        }
    }
}

bool Exploration::isExplorationComplete() const {
    return explorationDone;
}

void Exploration::returnToOrigin(cv::Point3f& robotPosition) {
    robotPosition = startPosition;
}

void Exploration::performExploration(const std::vector<float>& lidarMeasurements, cv::Mat& occupancyGrid) {
    // Implémentation simplifiée d'un algorithme d'exploration (ex: exploration en spirale ou BFS)
    // Ici, on simule en marquant des zones comme explorées (blanc)
    // En pratique, intégrer un path planner pour déplacer le robot
    for (int i = 0; i < occupancyGrid.rows; ++i) {
        for (int j = 0; j < occupancyGrid.cols; ++j) {
            if (occupancyGrid.at<uchar>(i, j) == 128) { // Gris pour non exploré
                occupancyGrid.at<uchar>(i, j) = 255; // Blanc pour exploré
            }
        }
    }
}