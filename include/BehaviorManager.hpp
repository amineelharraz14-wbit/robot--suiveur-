#ifndef BEHAVIOR_MANAGER_HPP
#define BEHAVIOR_MANAGER_HPP

#include <iostream>
#include <optional>
#include "Robot.hpp"
#include "LiDAR.hpp"
#include "Camera.hpp"
#include "KeyboardControl.hpp"
#include "Exploration.hpp"
#include "WallFollow.hpp"
#include "PathPlanner.hpp"

// Enum des comportements
enum class BehaviorType {
    MANUAL = 1,         // ID tag : 1
    EXPLORE = 2,        // ID tag : 2
    WALL_FOLLOW = 3,    // ID tag : 3
    RETURN_HOME = 99    // Interne (pas de tag)
};

// CLASS BehaviorManager
class BehaviorManager {
public:
    // Constructeur qui crée les modules comportement
    BehaviorManager();
    
    // Destructeur
    ~BehaviorManager() = default;

    // Met à jour le comportement selon un tag ArUco
    void SetBehavior(std::optional<int> tagId);

    // Exécute le comportement actif
    void update(Robot& robot, LiDAR& lidar, Camera& camera);

    // Obtenir le comportement actuel
    BehaviorType getCurrentBehavior() const;

private:
    // Comportement actif
    BehaviorType activeBehavior = BehaviorType::MANUAL;

    // Mémorise le dernier tag pour la persistance
    int lastId = -1;

    // Modules de comportement (créés dynamiquement)
    KeyboardControl keyboard;
    Exploration exploration;
    WallFollow wallFollow;
    PathPlanner pathPlanner;
};

#endif
