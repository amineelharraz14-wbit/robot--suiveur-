cmake_minimum_required(VERSION 3.10)
project(RobotSimulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Define core library sources (shared by the main executable and tests)
set(SIM_SOURCES
    src/Robot.cpp
    src/LiDAR.cpp
    src/Camera.cpp
    src/BehaviorManager.cpp
    src/ArucoManager.cpp
    src/Exploration.cpp
    src/WallFollow.cpp
    src/PathPlanner.cpp
    src/KeyboardControl.cpp
)

add_library(robotsim_lib STATIC ${SIM_SOURCES})

# Main executable (keeps only src/main.cpp here, links to the shared library)
add_executable(robotsim src/main.cpp)
target_link_libraries(robotsim PUBLIC robotsim_lib)

# --- Configuration d'OpenCV ---
find_package(OpenCV REQUIRED)

# (Removed duplicate add_executable; the project already creates the `robotsim` target
# above with the explicit source list.)

# Indiquer à CMake où sont nos headers (fichiers .hpp)
# et les headers d'OpenCV
## Include directories and link libraries for the library (and tests)
target_include_directories(robotsim_lib PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(robotsim_lib PUBLIC ${OpenCV_LIBS})

target_include_directories(robotsim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Separate test executables (headless)
add_executable(test_aruco_detect src/test_aruco_detect.cpp)
target_include_directories(test_aruco_detect PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(test_aruco_detect PUBLIC robotsim_lib ${OpenCV_LIBS})

add_executable(integration_test src/integration_test.cpp)
target_include_directories(integration_test PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(integration_test PUBLIC robotsim_lib ${OpenCV_LIBS})

message(STATUS "Configuration du projet terminée.")