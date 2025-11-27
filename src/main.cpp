

// On inclut les bibliothèques nécessaires
#include <iostream>            // Pour écrire du texte dans la console (cout)
#include <cmath>               // Pour les calculs mathématiques (cos, sin, sqrt)
#include <opencv2/opencv.hpp>  // Pour la vision par ordinateur et l'affichage

// On inclut nos propres modules (vos fichiers .hpp)
#include "Robot.hpp"
#include "LiDAR.hpp"
#include "Camera.hpp"

// On dit au compilateur qu'on utilise les noms standards (évite d'écrire std:: et cv:: partout)
using namespace std;
using namespace cv;

// =================================================================================
// FONCTION 1 : VERIFIER LES OBSTACLES
// Cette fonction regarde la carte et dit "VRAI" si on touche un mur, "FAUX" sinon.
// =================================================================================
bool est_un_mur(Mat& carte, double x, double y) {
    // 1. On définit l'échelle : 1 mètre dans la réalité = 10 pixels sur l'image
    double echelle = 10.0;
    
    // 2. On trouve le centre de l'image (pour savoir où est le point 0,0)
    int centre_x = carte.cols / 2;
    int centre_y = carte.rows / 2;

    // 3. On convertit la position (mètres) en position (pixels)
    int pixel_x = centre_x + (int)(x * echelle);
    int pixel_y = centre_y - (int)(y * echelle); // Le Y est inversé sur les images

    // 4. Sécurité : Si on sort de l'image, on considère que c'est un mur
    if (pixel_x < 0 || pixel_x >= carte.cols || pixel_y < 0 || pixel_y >= carte.rows) {
        return true; 
    }

    // 5. On regarde la couleur du pixel
    // Si la valeur est petite (< 128), c'est noir ou gris foncé -> C'est un MUR.
    if (carte.at<uchar>(pixel_y, pixel_x) < 128) {
        return true;
    }
    
    return false; // Sinon, c'est libre
}

// =================================================================================
// FONCTION 2 : DESSINER LA SIMULATION
// Cette fonction dessine le robot, les lasers et le but sur l'image pour qu'on puisse voir.
// =================================================================================
void dessiner_tout(Mat& image, Robot& robot, vector<double>& lasers, double but_x, double but_y) {
    double echelle = 10.0;
    int centre_x = image.cols / 2;
    int centre_y = image.rows / 2;

    // --- A. DESSINER LE BUT (Cercle Vert) ---
    int px_but = centre_x + (int)(but_x * echelle);
    int py_but = centre_y - (int)(but_y * echelle);
    circle(image, Point(px_but, py_but), 10, Scalar(0, 255, 0), -1); // Vert rempli

    // --- B. DESSINER LE ROBOT (Position en pixels) ---
    int px_robot = centre_x + (int)(robot.x * echelle);
    int py_robot = centre_y - (int)(robot.y * echelle);

    // --- C. DESSINER LES RAYONS LASER ---
    // On dessine un rayon tous les 5 degrés pour ne pas surcharger le dessin
    for (int i = 0; i < 360; i += 5) { 
        double distance = lasers[i];
        
        // Si le laser ne touche rien (trop loin), on ne le dessine pas
        if (distance >= 10.0) continue; 
        
        // Calcul de l'angle du rayon
        double angle_rayon = robot.theta + (i * M_PI / 180.0) - M_PI;
        
        // Calcul du point final du rayon
        int fin_x = centre_x + (int)((robot.x + distance * cos(angle_rayon)) * echelle);
        int fin_y = centre_y - (int)((robot.y + distance * sin(angle_rayon)) * echelle);
        
        // Dessiner la ligne verte
        line(image, Point(px_robot, py_robot), Point(fin_x, fin_y), Scalar(0, 100, 0), 1);
    }

    // --- D. DESSINER LE CORPS DU ROBOT (Rouge) ---
    circle(image, Point(px_robot, py_robot), 15, Scalar(0, 0, 255), -1);
    
    // Dessiner le "nez" (ligne blanche) pour voir où il regarde
    int nez_x = px_robot + (int)(20 * cos(robot.theta));
    int nez_y = py_robot - (int)(20 * sin(robot.theta));
    line(image, Point(px_robot, py_robot), Point(nez_x, nez_y), Scalar(255, 255, 255), 2);
}

// =================================================================================
// PROGRAMME PRINCIPAL (MAIN)
// =================================================================================
int main() {
    cout << "--- DEMARRAGE DU ROBOT ---" << endl;

    // 1. CHARGEMENT DES FICHIERS
    string nom_fichier_carte = "../data/ma_carte.png";
    
    // On charge la carte en version "Calculs" (Noir et Blanc)
    Mat carte_calcul = imread(nom_fichier_carte, IMREAD_GRAYSCALE);
    // On charge la carte en version "Affichage" (Couleur)
    Mat carte_visuelle = imread(nom_fichier_carte);

    // Vérification : est-ce que l'image existe ?
    if (carte_visuelle.empty()) {
        cout << "ERREUR : Je ne trouve pas l'image " << nom_fichier_carte << endl;
        return -1; // On arrête le programme
    }

    // 2. CREATION DES OBJETS (Nos modules)
    LiDAR monLidar(nom_fichier_carte); // Le capteur laser
    Camera maCamera;                   // Les yeux
    Robot monRobot;                    // Le corps

    // 3. REGLAGES DE DEPART
    monRobot.x = -15.0; // Départ à gauche
    monRobot.y = 0.0;   // Au centre
    monRobot.theta = 0.0; // Regarde vers la droite

    // On définit un objectif (Point B) pour la démo
    double but_x = 15.0;
    double but_y = 5.0;
    
    double temps_par_image = 0.05; // 0.05 secondes entre chaque calcul

    // 4. BOUCLE INFINIE (La vie du robot)
    while (true) {
        
        // --- ETAPE 1 : VOIR (Les Capteurs) ---
        // On récupère l'image de la caméra
        Mat image_vue = maCamera.getFrame();
        // On récupère les distances du laser
        vector<double> distances = monLidar.scanner(monRobot.x, monRobot.y, monRobot.theta);


        // --- ETAPE 2 : REFLECHIR (Cerveau) ---
        // On prépare les commandes de vitesse
        double vitesse_avance = 0.0;
        double vitesse_tourne = 0.0;

        // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

        // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

        // --- CODE DE DEMONSTRATION (Aller au point B) ---
        // Calcul des distances vers le but
        double distance_x = but_x - monRobot.x;
        double distance_y = but_y - monRobot.y;
        
        // Calcul de l'angle idéal pour viser le but
        double angle_cible = atan2(distance_y, distance_x);
        
        // Différence entre l'angle idéal et mon angle actuel
        double erreur_angle = angle_cible - monRobot.theta;

        // Petite correction mathématique pour ne pas faire des tours complets inutiles
        while (erreur_angle > 3.14159) erreur_angle -= 2 * 3.14159;
        while (erreur_angle < -3.14159) erreur_angle += 2 * 3.14159;

        // Si on est encore loin du but (> 1 mètre)
        if (sqrt(distance_x*distance_x + distance_y*distance_y) > 1.0) {
            // On tourne vers le but
            vitesse_tourne = erreur_angle * 1.0; 
            
            // Si on regarde presque dans la bonne direction, on avance
            if (abs(erreur_angle) < 0.5) {
                vitesse_avance = 2.0; 
            }
        }
        // -----------------------------------------------------------------


        // --- ETAPE 3 : SECURITE  ---
        // On prédit où sera le robot dans le futur
        double futur_x = monRobot.x + (vitesse_avance * cos(monRobot.theta) * temps_par_image);
        double futur_y = monRobot.y + (vitesse_avance * sin(monRobot.theta) * temps_par_image);

        // Si la future position est un mur, on s'arrête !
        if (est_un_mur(carte_calcul, futur_x + 1, futur_y + 1)) {
            vitesse_avance = 0.0;
        }


        // --- ETAPE 4 : AGIR (Mouvement) ---
        monRobot.deplacer(vitesse_avance, vitesse_tourne, temps_par_image);


        // --- ETAPE 5 : AFFICHER (Pour nous humains) ---
        // On fait une copie de la carte pour dessiner dessus sans l'abîmer
        Mat image_ecran = carte_visuelle.clone();
        
        // On appelle notre fonction de dessin
        dessiner_tout(image_ecran, monRobot, distances, but_x, but_y);

        // On affiche les fenêtres
        imshow("Simulation (Vue Carte)", image_ecran);
        if(!image_vue.empty()) {
            imshow("Vue Robot (Camera)", image_vue);
        }

        // Pause de 50ms. Si on appuie sur ECHAP (code 27), on quitte.
        if (waitKey(50) == 27) break;
    }

    return 0;
}