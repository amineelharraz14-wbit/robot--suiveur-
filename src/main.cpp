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
    int cx = 0.5;
    int cy = 0.5;

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
    // ArUcoManager: use detectMarkers only (no template fallback by design)

    // --- NOUVEAU: Occupancy grid (taille en cellules, 1 cellule = 1m)
    // On crée la grille à l'échelle de la carte (10 pixels == 1m dans la simulation)
    int pixels_per_m = 10; // doit rester cohérent avec LiDAR
    int grid_w = carte_visu.cols / pixels_per_m;
    int grid_h = carte_visu.rows / pixels_per_m;
    OccupancyGrid occupancy(grid_w, grid_h);

    // --- Positionner le robot initialement dans une zone libre (pas dans le mur)
    // Suggestion: place robot near center, slightly offset to avoid walls
    double meters_per_cell_init = 1.0; // 1 cellule = 1m (même valeur que plus loin)
    monRobot.x = -2.0; // 2m left of center
    monRobot.y = 2.0;  // 2m above center

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
            std::vector<int> detectedIds = monAruco.detectAruco(image_actuelle, arucoFrame);
                if (!detectedIds.empty()) {
                    // Si plusieurs marqueurs sont visibles, appliquer une priorité claire
                    // Priorité choisie ici : Manuel (id 10) > Exploration (id 0) > Wall follow (id 5)
                    int mappedTag = -1;
                    bool foundManual = std::find(detectedIds.begin(), detectedIds.end(), 10) != detectedIds.end();
                    bool foundExplore = std::find(detectedIds.begin(), detectedIds.end(), 0) != detectedIds.end();
                    bool foundWall = std::find(detectedIds.begin(), detectedIds.end(), 5) != detectedIds.end();

                    if (foundManual) mappedTag = 1;      // Tag 10 -> Manuelle
                    else if (foundExplore) mappedTag = 2; // Tag 0 -> Exploration
                    else if (foundWall) mappedTag = 3;    // Tag 5 -> Wall follow

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
        
        // Save previous pose so we can revert if the move caused a collision
        double prev_x = monRobot.x;
        double prev_y = monRobot.y;
        double prev_theta = monRobot.theta;

        cerveau.update(monRobot, monLidar, maCamera);

        // After update, get fresh scan to update the occupancy grid and detect collisions
        vector<double> new_scan = monLidar.scanner(monRobot.x, monRobot.y, monRobot.theta);
        // If the robot ended up too close to an obstacle (safety distance), revert
        const double safety_distance = 0.4; // meters
        double min_dist = *std::min_element(new_scan.begin(), new_scan.end());
        if (min_dist < safety_distance) {
            // revert to previous pose
            monRobot.x = prev_x;
            monRobot.y = prev_y;
            monRobot.theta = prev_theta;
            // Recompute scan at safe pose
            new_scan = monLidar.scanner(monRobot.x, monRobot.y, monRobot.theta);
            // Optional: notify
            std::cout << "[SAFETY] Move reverted: too close to wall (min_dist=" << min_dist << ")" << std::endl;
        }

        // use the fresh scan for occupancy visualization and LiDAR lines
        scan_visu = new_scan;


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

            // Compute end point (either obstacle or max range)
            double end_dist = std::min(dist, max_dist);
            double tx_end = monRobot.x + end_dist * cos(ang);
            double ty_end = monRobot.y + end_dist * sin(ang);
            int cx_end = gx_center + (int)std::round(tx_end / meters_per_cell);
            int cy_end = gy_center - (int)std::round(ty_end / meters_per_cell);

            // Compute robot cell
            int cx_robot = gx_center + (int)std::round(monRobot.x / meters_per_cell);
            int cy_robot = gy_center - (int)std::round(monRobot.y / meters_per_cell);

            // Mark all cells along the ray as free (1) using Bresenham line
            occupancy.drawFreeLine(cx_robot, cy_robot, cx_end, cy_end);

            // if obstacle was hit, mark the endpoint as obstacle (2)
            if (dist < max_dist) {
                occupancy.setCell(cx_end, cy_end, 2);
            }
        }

        // === Affichage : on affiche seulement l'Occupancy Grid (pas la carte d'origine)

        // Vue caméra : si ArUco a annoté une image, montrer celle-ci, sinon la frame brute
        if (!arucoFrame.empty()) imshow("Vue Camera Robot", arucoFrame);
        else if(!image_actuelle.empty()) imshow("Vue Camera Robot", image_actuelle);

        // Visualisation occupancy grid (agrandie pour être lisible)
        cv::Mat occ_vis = occupancy.visualize();
        cv::resize(occ_vis, occ_vis, cv::Size(occ_vis.cols * pixels_per_m, occ_vis.rows * pixels_per_m), 0, 0, cv::INTER_NEAREST);

        // Compute robot position in occupancy-visual pixels (used to draw LiDAR rays)
        int cx_cell = gx_center + (int)std::round(monRobot.x / meters_per_cell_init);
        int cy_cell = gy_center - (int)std::round(monRobot.y / meters_per_cell_init);
        // Convert to pixels on the enlarged image
        int r_px = std::clamp(cx_cell * pixels_per_m, 0, occ_vis.cols-1);
        int r_py = std::clamp(cy_cell * pixels_per_m, 0, occ_vis.rows-1);

        // Dessiner les rayons du LiDAR sur l'occupancy grid (pour debug)
        for (int i = 0; i < 360; i += 4) {
            double dist = scan_visu[i];
            if (dist >= max_dist) continue;
            double ang = monRobot.theta + (i * M_PI / 180.0) - M_PI;
            double tx = monRobot.x + dist * cos(ang);
            double ty = monRobot.y + dist * sin(ang);
            int fx_cell = gx_center + (int)std::round(tx / meters_per_cell);
            int fy_cell = gy_center - (int)std::round(ty / meters_per_cell);
            int fx_px = std::clamp(fx_cell * pixels_per_m, 0, occ_vis.cols-1);
            int fy_py = std::clamp(fy_cell * pixels_per_m, 0, occ_vis.rows-1);
            // draw ray
            cv::line(occ_vis, cv::Point(r_px, r_py), cv::Point(fx_px, fy_py), cv::Scalar(0,150,0), 1);
            cv::circle(occ_vis, cv::Point(fx_px, fy_py), 2, cv::Scalar(0,0,255), -1);
        }

        // Dessiner le robot sur l'occupancy grid (même logique de conversion que pour la grille)
        // Cercle bleu + direction
        // Cercle bleu + direction (bigger so it's clearly visible)
        int robot_radius_px = std::max(8, (int)std::round(0.3 * pixels_per_m));
        int dir_len = std::max(10, (int)std::round(0.7 * pixels_per_m));
        cv::circle(occ_vis, cv::Point(r_px, r_py), robot_radius_px, cv::Scalar(255,0,0), -1);
        cv::line(occ_vis, cv::Point(r_px, r_py), cv::Point(r_px + (int)(dir_len * cos(monRobot.theta)), r_py - (int)(dir_len * sin(monRobot.theta))), cv::Scalar(255,255,255), 2);

        imshow("Occupancy Grid", occ_vis);
    }

    return 0;
}