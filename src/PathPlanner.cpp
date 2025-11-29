#include "PathPlanner.hpp"
#include <iostream>
#include <math.h>

void PathPlanner::resetOrigin() {
    origin_x = 0.0;
    origin_y = 0.0;
    arrived_at_origin = false;
    std::cout << "[PathPlanner] Origine réinitialisée à (0, 0)" << std::endl;
}

void PathPlanner::update(Robot& robot) {
    double dx = origin_x - robot.x;
    double dy = origin_y - robot.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    // Vérifier l'arrivée
    if (distance < ARRIVAL_THRESHOLD) {
        arrived_at_origin = true;
        std::cout << "[PathPlanner] Arrivé à l'origine!" << std::endl;
        return;
    }

    // Calculer l'angle vers l'origine
    double angle_target = std::atan2(dy, dx);
    double error = angle_target - robot.theta;

    // Normaliser l'erreur
    while (error > M_PI) error -= 2 * M_PI;
    while (error < -M_PI) error += 2 * M_PI;

    // Contrôle PID simplifié
    if (std::abs(error) > 0.1) {
        robot.deplacer(0.5, error * 0.5, 0.05);
    } else {
        robot.deplacer(1.5, 0, 0.05);
    }
}

bool PathPlanner::arrived() const {
    return arrived_at_origin;
}