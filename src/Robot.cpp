#include "Robot.hpp"

Robot::Robot() {
    // Le robot commence au centre de la carte (0,0)
    x = 0.0;
    y = 0.0;
    theta = 0.0;
}

void Robot::deplacer(double v_lineaire, double v_angulaire, double dt) {
    // 1. On tourne
    theta = theta + (v_angulaire * dt);

    // 2. On avance
    // La formule magique : x = x + vitesse * cos(angle) * temps
    x = x + (v_lineaire * cos(theta) * dt);
    y = y + (v_lineaire * sin(theta) * dt);
}