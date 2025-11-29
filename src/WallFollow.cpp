#include "WallFollow.hpp"
#include <iostream>
#include <math.h>
#include <algorithm>

void WallFollow::update(Robot& robot, LiDAR& lidar) {
    std::vector<double> distances = lidar.scanner(robot.x, robot.y, robot.theta);
    
    // Rayons clés (en degrés)
    double dist_left = distances[90];      // À gauche
    double dist_front = distances[180];    // Devant
    double dist_right = distances[270];    // À droite

    double vitesse_avant = 0.0;
    double vitesse_tourne = 0.0;

    // Logique : main gauche sur le mur
    if (dist_left > WALL_DISTANCE + 0.5) {
        // Pas de mur à gauche → tourner pour le trouver
        vitesse_tourne = ROTATION_SPEED;
        vitesse_avant = FORWARD_SPEED * 0.5;
    } 
    else if (dist_left < WALL_DISTANCE - 0.5) {
        // Mur trop proche → s'écarter
        vitesse_tourne = -ROTATION_SPEED;
        vitesse_avant = FORWARD_SPEED * 0.5;
    } 
    else if (dist_front > WALL_THRESHOLD) {
        // Route libre → avancer droit
        vitesse_avant = FORWARD_SPEED;
        vitesse_tourne = 0.0;
    } 
    else {
        // Mur devant → tourner à droite
        vitesse_tourne = -ROTATION_SPEED;
    }

    robot.deplacer(vitesse_avant, vitesse_tourne, 0.05);
}