/*
 * PROJET FINAL - INTEGRATION SYSTEME
*/

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <optional> // Nécessaire pour communiquer avec BehaviorManager
#include <opencv2/opencv.hpp>

// --- 1. INCLUSION DES MODULES ---
#include "Robot.hpp"
#include "LiDAR.hpp"
#include "Camera.hpp"
#include "BehaviorManager.hpp" 
#include "ArucoManager.hpp"
#include "OccupancyGrid.hpp"
#include "Exploration.hpp"


using namespace cv;
using namespace std;

// =================================================================================
// FONCTION D'AFFICHAGE : Visualiser ce que fait le robot
// =================================================================================
void dessiner_simulation(Mat& img, Robot& rob, vector<double>& lasers) {
    double ppm = 10.0; // Echelle : 10 pixels = 1 mètre
    int cx = img.cols / 2;
    int cy = img.rows / 2;

    // Convertir la position du robot (mètres -> pixels)
    int r_px = cx + (int)(rob.x * ppm);
    int r_py = cy - (int)(rob.y * ppm);

    // A. DESSINER LES OBSTACLES DETECTES (Rayons LiDAR)
    for (int i = 0; i < 360; i += 2) { // On dessine 1 rayon sur 2 pour la performance
        double dist = lasers[i];
        
        // Si le rayon va jusqu'au max (10m), c'est qu'il n'y a pas d'obstacle
        if (dist >= 10.0) continue; 

        // Calcul de l'angle du rayon dans le monde
        double ang = rob.theta + (i * M_PI / 180.0) - M_PI;
        
        // Point d'impact (Obstacle)
        int fx = cx + (int)((rob.x + dist * cos(ang)) * ppm);
        int fy = cy - (int)((rob.y + dist * sin(ang)) * ppm);
        
        // Ligne verte du robot vers l'obstacle
        line(img, Point(r_px, r_py), Point(fx, fy), Scalar(0, 100, 0), 1);
        // Point rouge sur l'obstacle (Impact)
        circle(img, Point(fx, fy), 2, Scalar(0, 0, 255), -1);
    }

    // B. DESSINER LE ROBOT
    // Corps du robot (Cercle Bleu)
    circle(img, Point(r_px, r_py), 15, Scalar(255, 0, 0), -1);
    // Direction (Ligne Blanche)
    line(img, Point(r_px, r_py), Point(r_px + 20*cos(rob.theta), r_py - 20*sin(rob.theta)), Scalar(255,255,255), 2);
}

// =================================================================================
// PROGRAMME PRINCIPAL
// =================================================================================
int main() {
    cout << "--- DEMARRAGE DU ROBOT COMPLET ---" << endl;
    cout << "Touches de DEBUG (Simulation Vision) :" << endl;
    cout << " [1] : Simuler Tag ID 1 (Mode MANUEL)" << endl;
    cout << " [2] : Simuler Tag ID 2 (Mode EXPLORATION)" << endl;
    cout << " [3] : Simuler Tag ID 3 (Mode WALL FOLLOW)" << endl;
    cout << " [ECHAP] : Quitter" << endl;

    // --- A. INITIALISATION HARDWARE (VOTRE TRAVAIL) ---
    string map_file = "../data/ma_carte.png";
    
    // 1. Charger la carte secrète pour le LiDAR
    LiDAR monLidar(map_file);
    
    // 2. Initialiser la caméra (Image fixe ou Webcam)
    Camera maCamera;
    
    // 3. Initialiser le robot au centre
    Robot monRobot; 
    
    // 4. Charger la carte pour l'affichage à l'écran
    Mat carte_visu = imread(map_file);
    if(carte_visu.empty()) { 
        cout << "ERREUR CRITIQUE : Carte non trouvee dans ../data/" << endl; 
        return -1; 
    }

    // --- B. INITIALISATION INTELLIGENCE (TRAVAIL COLLEGUES) ---
    BehaviorManager cerveau; 

    // --- NOUVEAU: Module ArUco pour la détection de tags ---
    ArucoManager monAruco;

    // --- NOUVEAU: Occupancy grid (taille en cellules, 1 cellule = 1m)
    // On crée la grille à l'échelle de la carte (10 pixels == 1m dans la simulation)
    int pixels_per_m = 10; // doit rester cohérent avec LiDAR
    int grid_w = carte_visu.cols / pixels_per_m;
    int grid_h = carte_visu.rows / pixels_per_m;
    OccupancyGrid occupancy(grid_w, grid_h);

    // --- C. BOUCLE PRINCIPALE ---
    while (true) {
        
        // =============================================================
        // ETAPE 1 : PERCEPTION (CAPTEURS)
        // =============================================================
        // Le robot "voit"
        Mat image_actuelle = maCamera.getFrame();
        
        // Le robot "scanne" (Cette donnée sera utilisée par le cerveau)
        // Note: Le scanner est appelé implicitement dans cerveau.update(), 
        // mais on peut le rappeler ici pour l'affichage graphique.
        vector<double> scan_visu = monLidar.scanner(monRobot.x, monRobot.y, monRobot.theta);


        // =============================================================
        // ETAPE 2 : VISION (SIMULATION DU MODULE ARUCO)
        // =============================================================
        // Normalement, le module ArucoManager analyse l'image et donne un tag.
        // Si la caméra est inactive ou ne voit rien, on garde le fallback clavier.

        std::optional<int> tagDetecte = std::nullopt; // Par défaut : rien vu

        // Detection ArUco si on a une image
        cv::Mat arucoFrame;
        if (!image_actuelle.empty()) {
            int detectedId = monAruco.detectAruco(image_actuelle, arucoFrame);
            if (detectedId != -1) {
                // ArucoManager encode son comportement interne (0=MANUAL,1=EXPLORATION,2=WALLFOLLOW)
                // BehaviorManager attend les tags 1=MANUAL,2=EXPLORE,3=WALL_FOLLOW
                int a_behavior = monAruco.getCurrentBehavior();
                int mappedTag = -1;
                if (a_behavior == 0) mappedTag = 1;
                else if (a_behavior == 1) mappedTag = 2;
                else if (a_behavior == 2) mappedTag = 3;

                if (mappedTag != -1) tagDetecte = mappedTag;
            }
        }

        // Fallback clavier pour debug (toujours pris en compte, mais n'écrase pas si ArUco a trouvé un tag)
        int touche = waitKey(50); // Attend 50ms une touche
        if (!tagDetecte.has_value()) {
            if (touche == '1') tagDetecte = 1; // On simule qu'on a vu le Tag 1
            if (touche == '2') tagDetecte = 2; // On simule qu'on a vu le Tag 2
            if (touche == '3') tagDetecte = 3; // On simule qu'on a vu le Tag 3
        }
        if (touche == 27) break; // Echap pour quitter

        // On envoie l'info au cerveau
        // Si tagDetecte est vide, le cerveau garde le comportement précédent (Persistance)
        cerveau.SetBehavior(tagDetecte);


        // =============================================================
        // ETAPE 3 : REFLEXION & ACTION (CERVEAU)
        // =============================================================
        // C'est la ligne la plus importante !
        // Le BehaviorManager prend le robot, le lidar et la camera.
        // Il décide de la stratégie ET il déplace le robot lui-même.
        
        cerveau.update(monRobot, monLidar, maCamera);


        // =============================================================
        // ETAPE 4 : AFFICHAGE (VISUALISATION)
        // =============================================================
        // On clone la carte pour dessiner dessus sans l'abîmer
        Mat ecran = carte_visu.clone();
        
        // On dessine le robot et les obstacles vus par le LiDAR
        dessiner_simulation(ecran, monRobot, scan_visu);
        
        // Afficher le mode actuel à l'écran
        // (On triche un peu pour récupérer l'info et l'afficher)
        string texte_mode = "Mode: INCONNU";
        int mode_id = (int)cerveau.getCurrentBehavior();
        if (mode_id == 1) texte_mode = "Mode: MANUEL (Clavier)";
        if (mode_id == 2) texte_mode = "Mode: EXPLORATION";
        if (mode_id == 3) texte_mode = "Mode: WALL FOLLOW";
        if (mode_id == 99) texte_mode = "Mode: RETOUR BASE";
        
        putText(ecran, texte_mode, Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,0,255), 2);

        // === Occupancy grid : mettre à jour à partir du scan et afficher ===
        // On transforme les mesures en cellules de la grille
        double meters_per_cell = 1.0; // 1 cellule = 1m
        int gx_center = occupancy.width / 2;
        int gy_center = occupancy.height / 2;
        double step = 0.25; // incréments en mètres pour remplir la grille
        double max_dist = 10.0;

        for (int i = 0; i < 360; ++i) {
            double dist = scan_visu[i];
            double ang = monRobot.theta + (i * M_PI / 180.0) - M_PI;

            // marquer les cellules parcourues comme explorées (1)
            double travelled = 0.0;
            while (travelled < std::min(dist, max_dist)) {
                double tx = monRobot.x + travelled * cos(ang);
                double ty = monRobot.y + travelled * sin(ang);
                int cx = gx_center + (int)std::round(tx / meters_per_cell);
                int cy = gy_center - (int)std::round(ty / meters_per_cell);
                occupancy.setCell(cx, cy, 1);
                travelled += step;
            }

            // si obstacle touche, marquer la cellule comme obstacle (2)
            if (dist < max_dist) {
                double tx = monRobot.x + dist * cos(ang);
                double ty = monRobot.y + dist * sin(ang);
                int cx = gx_center + (int)std::round(tx / meters_per_cell);
                int cy = gy_center - (int)std::round(ty / meters_per_cell);
                occupancy.setCell(cx, cy, 2);
            }
        }

        // Afficher les fenêtres
        imshow("Simulation Robotique", ecran);

        // Vue caméra : si ArUco a annoté une image, montrer celle-ci, sinon la frame brute
        if (!arucoFrame.empty()) imshow("Vue Camera Robot", arucoFrame);
        else if(!image_actuelle.empty()) imshow("Vue Camera Robot", image_actuelle);

        // Visualisation occupancy grid (agrandie pour être lisible)
        cv::Mat occ_vis = occupancy.visualize();
        cv::resize(occ_vis, occ_vis, cv::Size(occ_vis.cols * pixels_per_m, occ_vis.rows * pixels_per_m), 0, 0, cv::INTER_NEAREST);
        imshow("Occupancy Grid", occ_vis);
    }

    return 0;
}