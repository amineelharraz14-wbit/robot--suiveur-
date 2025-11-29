#ifndef EXPLORATION_H
#define EXPLORATION_H

#include <vector>
#include <opencv2/core.hpp>

// Classe pour gérer l'exploration automatique de la carte
class Exploration {
public:
    Exploration();
    ~Exploration();

    // Méthode pour explorer la carte et construire la grille d'occupation
    // Prend en entrée les mesures LiDAR et la position actuelle du robot
    void explore(const std::vector<float>& lidarMeasurements, const cv::Point3f& robotPosition, cv::Mat& occupancyGrid);

    // Vérifie si l'exploration est terminée
    bool isExplorationComplete() const;

    // Retourne à la position d'origine après exploration
    void returnToOrigin(cv::Point3f& robotPosition);

private:
    bool explorationDone;
    cv::Point3f startPosition;
    // Algorithme d'exploration (ex: BFS ou DFS pour couvrir tous les coins)
    void performExploration(const std::vector<float>& lidarMeasurements, cv::Mat& occupancyGrid);
};

#endif // EXPLORATION_H