cmake_minimum_required(VERSION 3.10)
project(RobotSimulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Définir les sources (les fichiers .cpp)
add_executable(robotsim
    src/main.cpp
    src/Robot.cpp
    src/LiDAR.cpp
    src/Camera.cpp
    src/BehaviorManager.cpp
    src/ArucoManager.cpp
    src/Exploration.cpp
    src/WallFollow.cpp
    src/PathPlanner.cpp
    src/test_aruco_detect.cpp
    src/KeyboardControl.cpp
    src/integration_test.cpp

)

# --- Configuration d'OpenCV ---
find_package(OpenCV REQUIRED)

# Création de l'exécutable (uniquement les fichiers .cpp)
add_executable(robotsim
    ${SOURCES}
)

# Indiquer à CMake où sont nos headers (fichiers .hpp)
# et les headers d'OpenCV
target_include_directories(robotsim PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

# Lier les bibliothèques OpenCV (la méthode la plus simple et robuste)
target_link_libraries(robotsim PUBLIC 
    ${OpenCV_LIBS}
)

message(STATUS "Configuration du projet terminée.")