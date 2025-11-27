#include "Camera.hpp"

// Constructeur : Initialise la caméra
Camera::Camera() {
    
    // =================================================================================
    // NOTE POUR LE FUTUR (PASSAGE SUR ROBOT REEL / VRAIE WEBCAM)
    // =================================================================================
    // Actuellement, nous sommes sur Windows/WSL, donc nous simulons la caméra
    // avec une image fixe pour éviter les bugs de drivers.
    //
    // SI VOUS VOULEZ UTILISER UNE VRAIE WEBCAM, FAITES CECI :
    // 1. Dans 'include/Camera.hpp', ajoutez : cv::VideoCapture flux_video;
    // 2. Ici, remplacez le code ci-dessous par : 
    //      flux_video.open(0); // 0 est l'index de la webcam par défaut
    // 
    // C'est tout !
    // =================================================================================

    // --- CODE ACTUEL (SIMULATION) ---
    // Attention au chemin : ".." signifie qu'on remonte d'un dossier par rapport à l'exécutable
    string chemin = "../data/test_aruco.png";
    
    // On charge l'image fixe
    image_interne = imread(chemin);

    // Vérification
    if (image_interne.empty()) {
        cout << "ERREUR CAMERA : Impossible de trouver l'image : " << chemin << endl;
        cout << "Verifiez que le fichier est bien dans le dossier 'data'." << endl;
    } else {
        cout << "Camera : Mode Simulation active (Image fixe chargee)." << endl;
    }
}

// Fonction pour récupérer l'image actuelle
Mat Camera::getFrame() {
    
    // =================================================================================
    // NOTE POUR LE ROBOT REEL
    // =================================================================================
    // Si vous utilisez la vraie webcam (flux_video), ajoutez cette ligne ici :
    // flux_video >> image_interne; // Capture la nouvelle image en direct
    // =================================================================================

    // On retourne l'image (soit fixe, soit mise à jour par la webcam)
    return image_interne.clone(); // .clone() pour protéger l'image originale
}

// Vérifie si la caméra est prête
bool Camera::estValide() {
    // Si l'image n'est pas vide, c'est que ça marche
    return !image_interne.empty();
}