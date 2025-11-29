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
    const double WALL_DISTANCE = 3.0;   // Distance à garder du mur (mètres)
    const double WALL_THRESHOLD = 2.0;  // Seuil de détection
    const double ROTATION_SPEED = 0.5;
    const double FORWARD_SPEED = 1.0;

public:
    WallFollow() = default;
    ~WallFollow() = default;

    /**
     * @brief Met à jour le robot pour suivre le mur
     */
    void update(Robot& robot, LiDAR& lidar);
};

#endif // WALL_FOLLOW_HPP