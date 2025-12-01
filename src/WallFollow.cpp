#include "WallFollow.hpp"
#include <iostream>
#include <math.h>
#include <algorithm>

void WallFollow::update(Robot& robot, LiDAR& lidar) {
    std::vector<double> distances = lidar.scanner(robot.x, robot.y, robot.theta);
    // Debug: print key ray distances in headless test runs
    // (This helps understand why robot might not be moving)
    std::cerr << "[WallFollow] dist_left=" << distances[90] << " dist_front=" << distances[180] << " dist_right=" << distances[270] << std::endl;
    
    // Rayons clés (en degrés)
    double dist_left = distances[90];      // À gauche
    double dist_front = distances[180];    // Devant
    double dist_right = distances[270];    // À droite

    double forward = FORWARD_SPEED;
    double angular = 0.0;

    // Emergency escape: if LiDAR reports zero or extremely small distances
    // in all directions, the robot is likely overlapping a wall in the test setup.
    // Backup and rotate to free itself.
    double min_any = std::min({dist_left, dist_front, dist_right});
    if (min_any <= 0.05) {
        // backup a bit and rotate
        robot.deplacer(-0.35, ROTATION_SPEED, 0.05);
        return;
    }

    // If there's something close in front, reduce forward speed and turn away
    if (dist_front < 0.4) {
        forward = 0.0;
        angular = -ROTATION_SPEED; // turn right
        robot.deplacer(forward, angular, 0.05);
        return;
    }

    // Proportional control to maintain a target distance to the left wall
    if (std::isfinite(dist_left) && dist_left > 0.0) {
        double error = dist_left - WALL_DISTANCE; // positive -> too far, negative -> too close
        // angular correction: positive error -> turn left (positive ang vel)
        angular = KP * error;
        // clamp angular to allowed limits
        if (angular > ROTATION_SPEED) angular = ROTATION_SPEED;
        if (angular < -ROTATION_SPEED) angular = -ROTATION_SPEED;

        // modulate forward speed based on closeness to wall
        double absErr = std::abs(error);
        if (absErr > 0.6) forward = FORWARD_SPEED * 0.5; // big error -> slow down
        else if (absErr > 0.25) forward = FORWARD_SPEED * 0.8;
        else forward = FORWARD_SPEED; // fine distance -> go normal speed
    } else {
        // No wall detected on the left: search for it by turning left slowly
        angular = ROTATION_SPEED * 0.4;
        forward = FORWARD_SPEED * 0.6;
    }

    robot.deplacer(forward, angular, 0.05);
}