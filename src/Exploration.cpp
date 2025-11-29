#include "Exploration.hpp"
#include <iostream>
#include <math.h>


void Exploration::update(Robot& robot, LiDAR& lidar) {
    if (!is_exploring) {
        is_exploring = true;
        current_x = START_X;
        current_y = START_Y;
        current_line = 0;
        std::cout << "[Exploration] Démarrage..." << std::endl;
    }

    // Calculer le prochain point
    int line_num = (int)((current_y - START_Y) / GRID_STEP);
    
    if (line_num % 2 == 0) {
        // Ligne paire : gauche vers droite
        current_x += GRID_STEP;
        if (current_x > END_X) {
            current_x = END_X;
            current_y += GRID_STEP;
        }
    } else {
        // Ligne impaire : droite vers gauche
        current_x -= GRID_STEP;
        if (current_x < START_X) {
            current_x = START_X;
            current_y += GRID_STEP;
        }
    }

    // Vérifier si fin d'exploration
    if (current_y > END_Y) {
        is_exploring = false;
        exploration_complete = true;
        std::cout << "[Exploration] Terminée!" << std::endl;
        return;
    }

    // Déplacer le robot vers le prochain point
    double dx = current_x - robot.x;
    double dy = current_y - robot.y;
    double angle_target = std::atan2(dy, dx);
    double error = angle_target - robot.theta;

    // Normaliser l'erreur d'angle
    while (error > M_PI) error -= 2 * M_PI;
    while (error < -M_PI) error += 2 * M_PI;

    if (std::abs(error) > 0.1) {
        robot.deplacer(0, error, 0.05);
    } else {
        robot.deplacer(1.0, 0, 0.05);
    }
}

bool Exploration::finish() const {
    return exploration_complete;
}

void Exploration::reset() {
    is_exploring = false;
    exploration_complete = false;
    current_x = START_X;
    current_y = START_Y;
    current_line = 0;
    std::cout << "[Exploration] Réinitialisée" << std::endl;
}