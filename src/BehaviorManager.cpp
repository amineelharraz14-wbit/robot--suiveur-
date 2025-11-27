#include <iostream>
#include "BehaviorManager.hpp"

// Constructeur des références
BehaviorManager::BehaviorManager(Keyboard& k, Exploration& e, WallFollow& w, PathPlanner& p) 
    : keyboard(k),
      exploration(e),
      wallFollow(w),
      pathPlanner(p),
{}


// SetBehavior : reçoit un ID et active le comportement
void BehaviorManager::SetBehavior(std::optional<int> tagId)
{
    // Aucun tag détecté 
    if (!tagId.has_value())
        return;

    int id = tagId.value();

    // Persistance
    if (id == lastId)
        return;

    // mémoriser le tag
    lastId = id;

    // Sélection du comportement selon l’ID
    switch(id)
    {
        case 1:
            activeBehavior = BehaviorType::MANUAL;
            std::cout << "mode MANUEL activé" << std::endl;
            break;

        case 2:
            activeBehavior = BehaviorType::EXPLORE;
            exploration.reset(); // restart l'exploration
            std::cout << "mode EXPLORATION activé" << std::endl;
            break;

        case 3:
            activeBehavior = BehaviorType::WALL_FOLLOW;
            std::cout << "mode WALL FOLLOW activé" << std::endl;
            break;

        default:
            // Tag inconnu
            std::cout << " Tag inconnu : " << std::endl;
            break;
    }
}


// exécute le comportement actif 
void BehaviorManager::update(Robot& robot, LiDAR& lidar, Camera& camera)
{
    switch(activeBehavior)
    {
        // Contrôle manuel : on lit le clavier
        case BehaviorType::MANUAL:
            keyboard.update(robot);
            break;


        // Exploration : le robot construit la carte + se déplace
        case BehaviorType::EXPLORE:
            exploration.update(robot, lidar);

            // Fin de l’exploration
            if (exploration.finish())
            {
                std::cout << "Exploration terminée : Retour a l'origine" << std::endl;
                activeBehavior = BehaviorType::RETURN_HOME;
                pathPlanner.resetOrigin();
            }
            break;


        // wall follow
        case BehaviorType::WALL_FOLLOW:
            wallFollow.update(robot, lidar);
            break;


        // Retour à la position d'origine
        case BehaviorType::RETURN_HOME:
            pathPlanner.update(robot);

            if (pathPlanner.arrived())
            {
                std::cout << "Le robot a atteint  l'origine : mode manuel" << std::endl;
                activeBehavior = BehaviorType::MANUAL;
            }
            break;
    }
}