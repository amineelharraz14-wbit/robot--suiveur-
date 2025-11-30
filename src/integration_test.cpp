#include <iostream>
#include <optional>
#include <cmath>
#include "Robot.hpp"
#include "LiDAR.hpp"
#include "Camera.hpp"
#include "BehaviorManager.hpp"
#include "ArucoManager.hpp"
#include "OccupancyGrid.hpp"

int main() {
    std::cout << "Integration test: start" << std::endl;

    // 1) ArUco detection tests (headless) - check marker files
    ArucoManager am;
    bool aruco_ok = true;
    for (int id : {0,5,10}) {
        std::string path = "../data/marker_" + std::to_string(id) + ".png";
        cv::Mat img = cv::imread(path);
        if (img.empty()) {
            std::cout << "  FAIL: cannot open " << path << std::endl;
            aruco_ok = false; continue;
        }
        cv::Mat out;
        auto ids = am.detectAruco(img, out);
        std::cout << "  detect on " << path << " -> found " << ids.size() << " ids" << std::endl;
        if (ids.empty()) aruco_ok = false;
    }

    // 2) Behavior manager basic tests
    LiDAR lidar("../data/ma_carte.png");
    Camera cam; // uses fixed test image
    Robot robot;
    BehaviorManager bm;

    // Manual
    bm.SetBehavior(std::optional<int>(1));
    bool manual_ok = (bm.getCurrentBehavior() == BehaviorType::MANUAL);
    std::cout << "  Behavior MANUAL set => " << (manual_ok ? "OK" : "FAIL") << std::endl;

    // Exploration
    bm.SetBehavior(std::optional<int>(2));
    bool explore_ok = (bm.getCurrentBehavior() == BehaviorType::EXPLORE);
    std::cout << "  Behavior EXPLORE set => " << (explore_ok ? "OK" : "FAIL") << std::endl;

    // Wall follow
    bm.SetBehavior(std::optional<int>(3));
    bool wall_ok = (bm.getCurrentBehavior() == BehaviorType::WALL_FOLLOW);
    std::cout << "  Behavior WALL_FOLLOW set => " << (wall_ok ? "OK" : "FAIL") << std::endl;

    // 3) Simple dynamic test: run a few updates in EXPLORE and WALL_FOLLOW
    // Reset to exploration, run a few steps and verify robot moved
    robot.x = 0; robot.y = 0; robot.theta = 0;
    bm.SetBehavior(std::optional<int>(2));
    for (int i = 0; i < 50; ++i) bm.update(robot, lidar, cam);
    bool moved = (std::abs(robot.x) > 0.0001 || std::abs(robot.y) > 0.0001);
    std::cout << "  Exploration movement -> " << (moved ? "moved" : "stayed") << " (x=" << robot.x << ", y=" << robot.y << ")" << std::endl;

    // Now wall follow for a few steps
    robot.x = 0; robot.y = 0; robot.theta = 0;
    bm.SetBehavior(std::optional<int>(3));
    for (int i = 0; i < 50; ++i) bm.update(robot, lidar, cam);
    bool wf_moved = (std::abs(robot.x) > 0.0001 || std::abs(robot.y) > 0.0001);
    std::cout << "  WallFollow movement -> " << (wf_moved ? "moved" : "stayed") << " (x=" << robot.x << ", y=" << robot.y << ")" << std::endl;

    // 4) Occupancy grid test: simulate a single scan and ensure some cells are marked
    OccupancyGrid occ(40, 40);
    int gx_center = occ.width / 2;
    int gy_center = occ.height / 2;

    auto scan = lidar.scanner(0, 0, 0);
    for (int i = 0; i < 360; ++i) {
        double dist = scan[i];
        double ang = (i * M_PI / 180.0) - M_PI;
        double maxd = 10.0;

        // endpoint (either obstacle or max range)
        double end_dist = std::min(dist, maxd);
        double tx_end = end_dist * cos(ang);
        double ty_end = end_dist * sin(ang);
        int cx_end = gx_center + (int)std::round(tx_end);
        int cy_end = gy_center - (int)std::round(ty_end);

        // robot cell == center
        int cx_robot = gx_center;
        int cy_robot = gy_center;

        // Mark all cells along the ray as free using OccupancyGrid helper
        occ.drawFreeLine(cx_robot, cy_robot, cx_end, cy_end);

        // mark endpoint as obstacle if needed
        if (dist < maxd) {
            occ.setCell(cx_end, cy_end, 2);
        }
    }
    // Count non-zero cells
    int nonzero = 0;
    for (int y = 0; y < occ.height; ++y) for (int x = 0; x < occ.width; ++x) if (occ.getCell(x,y) != 0) ++nonzero;
    std::cout << "  Occupancy grid non-zero cells: " << nonzero << std::endl;

    bool all_ok = aruco_ok && manual_ok && explore_ok && wall_ok && moved && wf_moved && nonzero > 0;
    std::cout << "Integration test result: " << (all_ok ? "PASS" : "FAIL") << std::endl;
    return all_ok ? 0 : 2;
}
