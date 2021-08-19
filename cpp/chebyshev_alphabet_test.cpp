/**
 * @file chebyshev_alphabet_test.cpp
 * @date August 12, 2021
 * @author JD Florez
 * @brief Unit tests for Chebyshev1 fitting on target geometries
 */

// Generic includes
#include <iostream>
#include <vector>
#include <cmath>
// Generic GTSAM includes - Gerry
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
// Chebyshev includes
#include <gtsam/basis/Chebyshev.h>
#include <gtsam/basis/FitBasis.h>
// namespace
using namespace std;
using namespace gtsam;

struct trajectory
{
    char letter;
    int line_segments, distance;
};


// unit test functions
int input_trajectory(char letter);

int main()
{
    // test case 1
    // user defines letter for trajectory (A, C, V)
    
    
    return 0;
}

int input_trajectory(char letter)
{
    
    return y;
}