#ifndef EXPLORATION_HPP
#define EXPLORATION_HPP

#include "Robot.hpp"
#include "LiDAR.hpp"
#include <vector>

/**
 * @class Exploration
 * @brief Exploration automatique en balayage serpentin
 */
class Exploration {
private:
    const double GRID_STEP = 2.0;       // Pas entre les positions (mètres)
    const double START_X = -15.0;
    const double START_Y = -10.0;
    const double END_X = 15.0;
    const double END_Y = 10.0;
    
    bool is_exploring = false;
    bool exploration_complete = false;
    int current_line = 0;
    double current_x = START_X;
    double current_y = START_Y;

public:
    Exploration() = default;
    ~Exploration() = default;

    /**
     * @brief Met à jour le robot pour explorer
     */
    void update(Robot& robot, LiDAR& lidar);

    /**
     * @brief Vérifie si l'exploration est terminée
     */
    bool finish() const;

    /**
     * @brief Réinitialise l'exploration
     */
    void reset();
};

#endif // EXPLORATION_HPP