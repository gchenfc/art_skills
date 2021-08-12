/**
 * @file chebyshev_alphabet_test.cpp
 * @date August 12, 2021
 * @author JD Florez
 * @brief Unit tests for Chebyshev fitting on target geometries
 */

// Generic GTSAM includes
// #include <gtsam/3rdparty/Eigen/Eigen/Dense>
// #include <gtsam/3rdparty/Eigen/Eigen/Core>
#include <iostream>
#include <vector>
#include <cmath>

// #include <gtsam/geometry/Pose2.h>
// #include <gtsam/inference/Key.h>
// #include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
// #include <gtsam/nonlinear/Marginals.h>
// #include <gtsam/nonlinear/NonlinearFactorGraph.h>
// #include <gtsam/nonlinear/Values.h>
// #include <gtsam/slam/BetweenFactor.h>
// Chebyshev includes
// #include <gtsam/basis/Chebyshev.h>
// #include <gtsam/basis/FitBasis.h>
// namespace
using namespace std;
// using namespace gtsam;

struct trajectory
{
    char letter;
    int line_segments, distance;
};


// functions
int input_trajectory(char letter);

int main()
{
    int wave, amplitude, ang_freq, t, shift;
    cout << "Enter amplitude";
    cin >> amplitude;
    cout << "Enter ang_freq";
    cin >> ang_freq;
    cout << "Enter t";
    cin >> t;
    cout << "Enter shift";
    cin >> shift;
    
    wave = trajectory(amplitude, ang_freq, t, shift);
    cout << wave;

    return 0;
}

int input_trajectory(char letter)
{
    
    return y;
}

