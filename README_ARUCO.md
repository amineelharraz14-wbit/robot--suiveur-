# ArUco tags & robot behaviors

Ce fichier décrit le fonctionnement de ArucoManager  et comment tester localement.

Tags attendus (par défaut) :
- Tag ID 0  -> comportement EXPLORATION automatique
- Tag ID 5  -> comportement WALL-FOLLOW (suivi de mur)
- Tag ID 10 -> comportement MANUEL (contrôle clavier)

Fonctionnement :
- `ArucoManager::detectAruco` détecte la liste des marqueurs présents dans la frame et met à jour un état interne `currentBehavior` d'après le premier marqueur détecté.
- `main.cpp` interroge `ArucoManager` et mappe les comportements internes sur les identifiants que `BehaviorManager` attend (1=MANUAL, 2=EXPLORE, 3=WALL_FOLLOW).

Notes de debug (headless CI/container) :
- Dans l'environnement CI ou conteneur sans affichage GUI, la détection peut ne pas fonctionner de la même façon qu'en local. Pour déboguer localement :
  1. Ouvrez un terminal sur votre machine avec écran.
  2. Lancez `cd build && ./robotsim`.
  3. Présentez un tag ArUco correspondant (DICT_6X6_250 par défaut) devant la webcam ou utilisez `data/test_aruco.png` qui doit contenir un marqueur valide.

