#include "plane_slam.h"

PlaneSlam::PlaneSlam() :
    isam2_parameters_()
  , graph_()
  , initial_estimate_()
  , first_pose_(true)
  , odom_poses_()
  , pose_count_(0)
  , landmakr_count_(0)
{
    isam2_parameters_.relinearizeThreshold = 0.05;
    isam2_parameters_.relinearizeSkip = 1;
    isam2_parameters_.print("ISAM2 parameters:");
    isam2_ = new ISAM2(isam2_parameters_);
}

void PlaneSlam::initialize(const Pose3 &pose, const std::vector<Eigen::VectorXd> &plane_coeffs,
                           const std::vector<Eigen::MatrixXd> &covariances)
{
    // Add a prior factor
    pose_count_ = 0;
    noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.3),Vector3::Constant(0.1))); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
    graph_.push_back(PriorFactor<Pose3>(Symbol('x', 0), poses[0], poseNoise));
    // Add an initial guess for the current pose
    initial_estimate_.insert(Symbol('x', 0), pose);
    odom_poses_.push_back(pose);
    pose_count_++;

    // Add new observation
    landmakr_count_ = 0;
    for(int i = 0; i < planes.size(); i++)
    {
        noiseModel::Gaussian::shared_ptr obs_noise = noiseModel::Gaussian::Covariance(covariances[i]);
        graph_.push_back(OrientedPlane3Factor(plane_coeffs[i], obs_noise, Symbol('x', 0), Symbol('l', i)));
        landmakr_count_ ++;
    }
    //
    first_pose_ = false;
}

void PlaneSlam::planeSlam(const Pose3 &odom, const std::vector<Eigen::VectorXd> &plane_coeffs,
                          const std::vector<Eigen::MatrixXd> &covariances)
{
    if(first_pose_)
    {
        cout << "You should initialize the map before doing slam." << endl;
        return;
    }

    // Add odometry factors
    noiseModel::Diagonal::shared_ptr odometry_noise =
            noiseModel::Diagonal::Sigmas((Vector(6) << Vector3(odom.translation().vector() * 0.2), Vector3(odom.rotation().rpy() * 0.1)));
    graph_.push_back(BetweenFactor<Pose3>(Symbol('x', pose_count_-1), Symbol('x', pose_count_), odom, odometry_noise));
    // Add pose guess
    Pose3 new_pose = *odom_poses_.back();
    new_pose.compose( odom );
    odom_poses_.push_back( new_pose );
    initial_estimate_.insert(Symbol('x', pose_count_), new_pose);
    pose_count_ ++;

    // Match local landmark with map landmarks


    // Add factor to exist landmark

    // Add new landmark


}
