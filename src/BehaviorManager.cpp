#include "BehaviorManager.hpp"
#include <iostream>

using namespace std;

// Constructeur : initialiser tous les modules
BehaviorManager::BehaviorManager()
    : activeBehavior(BehaviorType::MANUAL),
      lastId(-1),
      keyboard(),
      exploration(),
      wallFollow(),
      pathPlanner() {
    cout << "[BehaviorManager] Initialisé avec mode MANUEL" << endl;
}

// SetBehavior : reçoit un ID et active le comportement
void BehaviorManager::SetBehavior(std::optional<int> tagId) {
    // Aucun tag détecté
    if (!tagId.has_value())
        return;

    int id = tagId.value();

    // Persistance : ne change que si ID différent
    if (id == lastId)
        return;

    // Mémoriser le tag
    lastId = id;

    // Sélection du comportement selon l'ID
    switch (id) {
        case 1:
            activeBehavior = BehaviorType::MANUAL;
            cout << "[BehaviorManager] Mode MANUEL activé (Tag ID: 1)" << endl;
            break;

        case 2:
            activeBehavior = BehaviorType::EXPLORE;
            exploration.reset();  // Réinitialiser l'exploration
            cout << "[BehaviorManager] Mode EXPLORATION activé (Tag ID: 2)" << endl;
            break;

        case 3:
            activeBehavior = BehaviorType::WALL_FOLLOW;
            cout << "[BehaviorManager] Mode WALL FOLLOW activé (Tag ID: 3)" << endl;
            break;

        default:
            cout << "[BehaviorManager] Tag inconnu : " << id << endl;
            break;
    }
}

// Exécute le comportement actif
void BehaviorManager::update(Robot& robot, LiDAR& lidar, Camera& camera) {
    switch (activeBehavior) {
        // Contrôle manuel : on lit le clavier
        case BehaviorType::MANUAL:
            keyboard.update(robot);
            break;

        // Exploration : le robot construit la carte + se déplace
        case BehaviorType::EXPLORE:
            exploration.update(robot, lidar);
            // Fin de l'exploration
            if (exploration.finish()) {
                cout << "[BehaviorManager] Exploration terminée : Retour a l'origine" << endl;
                activeBehavior = BehaviorType::RETURN_HOME;
                pathPlanner.resetOrigin();
            }
            break;

        // Wall follow
        case BehaviorType::WALL_FOLLOW:
            wallFollow.update(robot, lidar);
            break;

        // Retour à la position d'origine
        case BehaviorType::RETURN_HOME:
            pathPlanner.update(robot);
            if (pathPlanner.arrived()) {
                cout << "[BehaviorManager] Le robot a atteint l'origine : mode manuel" << endl;
                activeBehavior = BehaviorType::MANUAL;
                lastId = -1;  // Réinitialiser pour pouvoir changer de comportement
            }
            break;
    }
}

BehaviorType BehaviorManager::getCurrentBehavior() const {
    return activeBehavior;
}