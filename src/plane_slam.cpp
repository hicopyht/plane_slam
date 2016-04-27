#include "plane_slam.h"

PlaneSlam::PlaneSlam()
{

}

void PlaneSlam::planeSlam()
{

    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.05;
    parameters.relinearizeSkip = 1;
    parameters.print("ISAM2 parameters:");
    ISAM2 isam(parameters);

    // Create a Factor Graph and Values to hold the new data
    NonlinearFactorGraph graph; // factor graph
    Values initialEstimate; // initial guess


    // Add factors for each landmark observation
//    graph.add();

    // Add odometry factor

    // Add an initial guess for the current pose

    // If first pose, add a pose prior

    // If first observed landmark, add a landmark prior

    // do incrementally update, if not first pose


    /*
    // If this is the first iteration, add a prior on the first pose to set the coordinate frame
    // and a prior on the first landmark to set the scale
    // Also, as iSAM solves incrementally, we must wait until each is observed at least twice before
    // adding it to iSAM.
    if( i == 0)
    {
      // Add a prior on pose x0
      noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.3),Vector3::Constant(0.1))); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
      graph.push_back(PriorFactor<Pose3>(Symbol('x', 0), poses[0], poseNoise));

      // Add a prior on landmark l0
      noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(3, 0.1);
      graph.push_back(PriorFactor<Point3>(Symbol('l', 0), points[0], pointNoise)); // add directly to graph

      // Add initial guesses to all observed landmarks
      // Intentionally initialize the variables off from the ground truth
      for (size_t j = 0; j < points.size(); ++j)
        initialEstimate.insert(Symbol('l', j), points[j].compose(Point3(-0.25, 0.20, 0.15)));

    } else {
      // Update iSAM with the new factors
      isam.update(graph, initialEstimate);
      // Each call to iSAM2 update(*) performs one iteration of the iterative nonlinear solver.
      // If accuracy is desired at the expense of time, update(*) can be called additional times
      // to perform multiple optimizer iterations every step.
      isam.update();
      Values currentEstimate = isam.calculateEstimate();
      cout << "****************************************************" << endl;
      cout << "Frame " << i << ": " << endl;
      currentEstimate.print("Current estimate: ");

      // Clear the factor graph and values for the next iteration
      graph.resize(0);
      initialEstimate.clear();
    }
    */

}
