#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include "Robot.hpp"
#include <vector>

/**
 * @class PathPlanner
 * @brief Planificateur de chemin - Retour à l'origine
 */
class PathPlanner {
private:
    double origin_x = 0.0;
    double origin_y = 0.0;
    bool arrived_at_origin = false;
    const double ARRIVAL_THRESHOLD = 1.0;  // Mètres

public:
    PathPlanner() = default;
    ~PathPlanner() = default;

    /**
     * @brief Réinitialise l'origine (à appeler au début d'une exploration)
     */
    void resetOrigin();

    /**
     * @brief Déplace le robot vers l'origine
     */
    void update(Robot& robot);

    /**
     * @brief Vérifie si le robot est arrivé à l'origine
     */
    bool arrived() const;
};

#endif // PATH_PLANNER_HPP