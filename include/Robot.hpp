#pragma once
#include <cmath> // Pour les maths (cos, sin)

// On simplifie l'écriture
using namespace std;

class Robot {
public:
    // Constructeur
    Robot();

    // Fonction pour bouger
    // v_lineaire : vitesse en avant (mètres/seconde)
    // v_angulaire : vitesse de rotation (radians/seconde)
    // dt : temps écoulé (secondes)
    // Ajoutez "= 0.05" à la fin
    void deplacer(double v_lin, double v_ang, double dt = 0.05);
    // Variables publiques (c'est plus simple pour un débutant que des getX()...)
    double x;
    double y;
    double theta; // Angle en radians
};