#ifndef KEYBOARD_CONTROL_HPP
#define KEYBOARD_CONTROL_HPP

#include "Robot.hpp"
#include <iostream>

/**
 * @class KeyboardControl
 * @brief Gère le contrôle manuel du robot via le clavier (WASD)
 */
class KeyboardControl {
private:
    const double STEP_FORWARD = 0.5;    // Distance d'avancée (mètres)
    const double STEP_BACKWARD = 0.5;   // Distance de recul
    const double ANGLE_ROTATION = 0.2;  // Angle de rotation (radians)

public:
    KeyboardControl() = default;
    ~KeyboardControl() = default;

    /**
     * @brief Met à jour le robot selon l'entrée clavier
     * @param robot Référence au robot à contrôler
     * 
     * Touches :
     * - W : Avancer
     * - S : Reculer
     * - A : Tourner à gauche
     * - D : Tourner à droite
     */
    void update(Robot& robot);
};

#endif // KEYBOARD_CONTROL_HPP