#ifndef OCCUPANCYGRID_H
#define OCCUPANCYGRID_H

#include <vector>
#include <opencv2/opencv.hpp>  // Pour visualisation

struct OccupancyGrid {
    int width, height;  // Dimensions de la grille (ex: 100x100)
    std::vector<std::vector<int>> grid;  // Grille 2D : 0=non exploré, 1=exploré, 2=obstacle

    OccupancyGrid(int w, int h) : width(w), height(h) {
        grid.assign(h, std::vector<int>(w, 0));  // Initialiser à 0 (non exploré)
    }

    // Définit la valeur d'une cellule
    void setCell(int x, int y, int value) {
        if (x >= 0 && x < width && y >= 0 && y < height) {
            grid[y][x] = value;  // Note: grid[y][x] pour accès ligne-colonne
        }
    }

    // Récupère la valeur d'une cellule
    int getCell(int x, int y) const {
        if (x >= 0 && x < width && y >= 0 && y < height) {
            return grid[y][x];
        }
        return -1;  // Erreur hors limites
    }

    // Visualise la grille avec OpenCV (gris=0, blanc=1, noir=2)
    cv::Mat visualize() const {
        cv::Mat img(height, width, CV_8UC3, cv::Scalar(128, 128, 128));  // Gris par défaut
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (grid[y][x] == 1) img.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);  // Blanc
                else if (grid[y][x] == 2) img.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);    // Noir
            }
        }
        return img;
    }
};

#endif
