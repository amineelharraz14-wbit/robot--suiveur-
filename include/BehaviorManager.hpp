#ifndef BEHAVIOR_MANAGER_HPP
#define BEHAVIOR_MANAGER_HPP

#include <iostream>
#include <optional>

#include "Robot.hpp"
#include "LiDAR.hpp"
#include "Camera.hpp"
// #include "Keyboard.hpp"
// #include "Exploration.hpp"
// #include "WallFollow.hpp"
// #include "PathPlanner.hpp"

// COMPORTEMENTS
enum class BehaviorType {
    MANUAL,        // Contrôle manuel
    EXPLORE,       // Exploration automatique
    WALL_FOLLOW,   // Suivi de mur
    RETURN_HOME    // Retour au point d'origine
};


// CLASS BehaviorManager
class BehaviorManager {
public:

    // Constructeur recevant les modules (passés par référence)
    BehaviorManager(Keyboard& k, 
                    Exploration& e, 
                    WallFollow& w,
                    PathPlanner& p);

    // Met à jour le comportement selon un tag ArUco
    void SetBehavior(std::optional<int> tagId);

    // Exécute le comportement actif
    void update(Robot& robot, LiDAR& lidar, Camera& camera);

private:

    // Comportement actif
    BehaviorType activeBehavior = BehaviorType::MANUAL;

    // Mémorise le dernier tag pour la persistance
    int lastId = -1;

    // Références vers les modules externes
    Keyboard& keyboard;
    Exploration& exploration;
    WallFollow& wallFollow;
    PathPlanner& pathPlanner;
};

#endif