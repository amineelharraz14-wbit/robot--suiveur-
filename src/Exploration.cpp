#include "Exploration.hpp"
#include <iostream>
#include <math.h>
#include <algorithm>


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

    // Use simple obstacle-aware control: if obstacle in front, enter avoidance mode
    // where we pick the best open heading reported by the LiDAR and go there.
    // Scan a small forward cone (-20..+20 degrees) for minimum distance
    auto dists = lidar.scanner(robot.x, robot.y, robot.theta);
    double min_front = 10.0;
    for (int a = 340; a < 360; ++a) min_front = std::min(min_front, dists[a]);
    for (int a = 0; a <= 20; ++a) min_front = std::min(min_front, dists[a]);

    const double SLOW_DIST = 0.7; // start slowing when obstacle below this (meters)
    const double STOP_DIST = 0.35; // consider too close, rotate instead

    // If we're already in avoidance mode, follow that plan until done.
    if (in_avoidance) {
        // compute angular error to avoid_target
        double ang_err = avoid_target_theta - robot.theta;
        while (ang_err > M_PI) ang_err -= 2*M_PI;
        while (ang_err < -M_PI) ang_err += 2*M_PI;

        // If not aligned, rotate towards target
        if (std::abs(ang_err) > 0.25) {
            double ang_vel = std::clamp(ang_err * 1.2, -1.2, 1.2);
            std::cout << "[Exploration] avoidance rotating (ang_err=" << ang_err << ")" << std::endl;
            robot.deplacer(0.0, ang_vel, 0.05);
            return;
        }

        // Aligned: move forward for a few steps
        if (avoid_steps > 0) {
            avoid_steps -= 1;
            std::cout << "[Exploration] avoidance advancing steps remaining=" << avoid_steps << std::endl;
            robot.deplacer(1.0, 0.0, 0.05);
            return;
        }

        // Done avoidance, resume exploration sweep
        in_avoidance = false;
    }

    if (min_front < STOP_DIST) {
        // Enter avoidance: pick the best open heading within a forward semicircle
        // Search angles from left(90) to right(270) (in LiDAR indexing, 180 = forward)
        int best_idx = 180;
        double best_dist = -1.0;
        for (int i = 90; i <= 270; ++i) {
            double d = dists[i];
            if (d > best_dist) { best_dist = d; best_idx = i; }
        }

        // Compute target heading in world coords using same mapping as LiDAR
        double best_ang_world = robot.theta + (best_idx * M_PI / 180.0) - M_PI;
        in_avoidance = true;
        avoid_target_theta = best_ang_world;
        // choose steps proportional to distance (move toward open space)
        int steps = std::max(10, (int)std::round(std::min(best_dist, 4.0) / 0.2));
        avoid_steps = steps;
        // We'll rotate in the next loop (function will return early until aligned/moved)
        return;
    } else if (min_front < SLOW_DIST) {
        std::cout << "[Exploration] slowing: min_front=" << min_front << std::endl;
        // Slow and adjust steering
        double forward = 0.3; // reduced forward speed near obstacles
        double angular = error; // small angular correction
        robot.deplacer(forward, angular, 0.05);
    } else {
        // Free to go faster when clear
        double forward = 1.6; // faster exploration speed
        if (std::abs(error) > 0.25) {
            // if big angular error, turn first
            robot.deplacer(0.0, error * 0.8, 0.05);
        } else {
            robot.deplacer(forward, 0.0, 0.05);
        }
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