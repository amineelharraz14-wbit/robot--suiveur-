#ifndef WALL_FOLLOW_HPP
#define WALL_FOLLOW_HPP

#include "Robot.hpp"
#include "LiDAR.hpp"

/**
 * @class WallFollow
 * @brief Algorithme de suivi de mur (main gauche)
 */
class WallFollow {
private:
    const double WALL_DISTANCE = 1.0;   // Desired distance to wall (meters)
    const double WALL_THRESHOLD = 0.6;  // Distance in front to consider obstacle
    const double ROTATION_SPEED = 1.0;  // max angular velocity
    const double FORWARD_SPEED = 0.7;   // forward speed while following wall
    const double KP = 0.6;              // proportional gain for distance error

public:
    WallFollow() = default;
    ~WallFollow() = default;

    /**
     * @brief Met à jour le robot pour suivre le mur
     */
    void update(Robot& robot, LiDAR& lidar);
};

#endif // WALL_FOLLOW_HPP