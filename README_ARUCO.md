# ArUco tags & robot behaviors

Ce fichier décrit comment les tags ArUco sont utilisés dans le projet et comment tester localement.

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

Améliorations proposées :
- Rendre la table de mapping ID→comportement configurable depuis un fichier JSON.
- Retourner et prioriser plusieurs marqueurs si plusieurs sont visibles (déjà géré par la nouvelle implémentation avec priorité).
- Ajouter estimation de pose avec `estimatePoseSingleMarkers` quand vous avez la calibration de la caméra.

Si vous voulez, je peux :
- Générer automatiquement 3 images de test ArUco (id 0, 5, 10) dans `data/` et ajouter un petit test GUI / headless qui simule leur présence pour vérifier le comportement du robot.
