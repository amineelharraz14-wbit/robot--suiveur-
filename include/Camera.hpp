#pragma once

// On inclut la librairie de vision
#include <opencv2/opencv.hpp>
#include <iostream>

// --- SIMPLIFICATION ---
// Cela nous permet d'écrire 'Mat' au lieu de 'cv::Mat'
// et 'string' au lieu de 'std::string'.
using namespace cv;
using namespace std;

class Camera {
public:
    // Le Constructeur : Prépare la caméra
    Camera();

    // Fonction pour récupérer l'image actuelle
    Mat getFrame();

    // Vérifie si ça marche
    bool estValide();

private:
    // Variable privée pour stocker l'image
    Mat image_interne;
};