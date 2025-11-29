#include "KeyboardControl.hpp"
#include <opencv2/highgui.hpp>

void KeyboardControl::update(Robot& robot) {
    int key = cv::waitKey(30);  // Attente de 30ms pour la touche
    
    if (key == -1) return;  // Pas de touche pressée
    
    switch (key) {
        case 'z':
        case 'Z':
            robot.deplacer(STEP_FORWARD, 0, 0.05);  // Avancer
            std::cout << "[Clavier] Avancer -> (" << robot.x << ", " << robot.y << ")" << std::endl;
            break;
            
        case 's':
        case 'S':
            robot.deplacer(-STEP_FORWARD, 0, 0.05);  // Reculer
            std::cout << "[Clavier] Reculer" << std::endl;
            break;
            
        case 'q':
        case 'Q':
            robot.deplacer(0, ANGLE_ROTATION, 0.05);  // Tourner gauche
            std::cout << "[Clavier] Tourner gauche" << std::endl;
            break;
            
        case 'd':
        case 'D':
            robot.deplacer(0, -ANGLE_ROTATION, 0.05);  // Tourner droite
            std::cout << "[Clavier] Tourner droite" << std::endl;
        break;
}
}