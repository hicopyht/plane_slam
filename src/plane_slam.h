#ifndef PLANE_SLAM_H
#define PLANE_SLAM_H

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include "PlaneFactor.h"

using namespace std;
using namespace gtsam;

class PlaneSlam
{
public:
    PlaneSlam();

    void planeSlam();
};

#endif // PLANE_SLAM_H
