#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <cmath>

using namespace cv;
using namespace std;

class LiDAR {
public:
    // Charge la carte
    LiDAR(string chemin_carte);

    // Calcule les 360 distances
    vector<double> scanner(double robot_x, double robot_y, double robot_theta);

private:
    Mat carte_secrete; // L'image de la carte (noire et blanche)
};