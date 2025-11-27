#include "LiDAR.hpp"

LiDAR::LiDAR(string chemin_carte) {
    // On charge l'image en Noir et Blanc (GRAYSCALE)
    carte_secrete = imread(chemin_carte, IMREAD_GRAYSCALE);

    if (carte_secrete.empty()) {
        cout << "ERREUR LIDAR: Impossible de lire la carte !" << endl;
    }
}

vector<double> LiDAR::scanner(double robot_x, double robot_y, double robot_theta) {
    vector<double> mesures(360); // Tableau de 360 cases
    
    // Paramètres de simulation
    double distance_max = 10.0; // Le laser va max à 10 mètres
    double pas = 0.05;          // Il vérifie tous les 5 cm
    double pixels_par_metre = 10.0; // Echelle: 10 pixels = 1 mètre
    
    // Le centre de l'image (pour savoir où est 0,0)
    int centre_x = carte_secrete.cols / 2;
    int centre_y = carte_secrete.rows / 2;

    // BOUCLE DES 360 RAYONS
    for (int angle_degre = 0; angle_degre < 360; angle_degre++) {
        
        // Conversion Degrés -> Radians
        double angle_rayon = angle_degre * (M_PI / 180.0);
        
        // Angle total = Angle Robot + Angle Rayon - 180° (car le laser est derrière par défaut)
        double angle_final = robot_theta + angle_rayon - M_PI;

        double distance = 0.0;
        bool obstacle_touche = false;

        // ON AVANCE LE RAYON PAS A PAS
        while (distance < distance_max) {
            // 1. Calculer la position testée (en mètres)
            double test_x = robot_x + distance * cos(angle_final);
            double test_y = robot_y + distance * sin(angle_final);

            // 2. Convertir en Pixels (pour l'image)
            int pixel_x = centre_x + (int)(test_x * pixels_par_metre);
            int pixel_y = centre_y - (int)(test_y * pixels_par_metre); // Y inversé sur les images

            // 3. Vérifier si on sort de l'image
            if (pixel_x < 0 || pixel_x >= carte_secrete.cols || pixel_y < 0 || pixel_y >= carte_secrete.rows) {
                distance = distance_max; // Hors carte
                break;
            }

            // 4. Vérifier si c'est un Mur (Pixel Noir = 0)
            if (carte_secrete.at<uchar>(pixel_y, pixel_x) == 0) {
                obstacle_touche = true;
                break; // Boum ! Mur touché, on arrête ce rayon
            }

            distance = distance + pas; // On avance un peu
        }

        mesures[angle_degre] = distance;
    }

    return mesures;
}