#include "plane_slam.h"

PlaneSlam::PlaneSlam() :
    isam2_parameters_()
  , first_pose_(true)
  , poses_()
  , pose_count_(0)
  , landmakr_count_(0)
{
    isam2_parameters_.relinearizeThreshold = 0.05;
    isam2_parameters_.relinearizeSkip = 1;
    isam2_parameters_.print("ISAM2 parameters:");
    isam2_ = new ISAM2(isam2_parameters_);
}

void PlaneSlam::initialize(Pose3 &init_pose, std::vector<PlaneType> &planes)
{
    // Create a Factor Graph and Values to hold the new data
    NonlinearFactorGraph graph; // factor graph
    Values initial_estimate; // initial guess

    // Add a prior factor
    pose_count_ = 0;
    Vector pose_sigmas(6);
    pose_sigmas << init_pose.translation().vector()*0.2, init_pose.rotation().rpy() * 0.1;
    noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas( pose_sigmas ); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
    graph.push_back(PriorFactor<Pose3>(Symbol('x', 0), init_pose, poseNoise));
    // Add an initial guess for the current pose
    initial_estimate.insert(Symbol('x', 0), init_pose);
    poses_.insert( Symbol('x', 0), init_pose );
    pose_count_++;

    // Add new observation
    landmakr_count_ = 0;
    for(int i = 0; i < planes.size(); i++)
    {
        PlaneType &plane = planes[i];
        noiseModel::Gaussian::shared_ptr obs_noise = noiseModel::Gaussian::Covariance( plane.covariances );
        graph.push_back(OrientedPlane3Factor(plane.coefficients, obs_noise, Symbol('x', 0), Symbol('l', i)));
        landmakr_count_ ++;
    }
    isam2_->update(graph, initial_estimate);
    //
    first_pose_ = false;

    cout << GREEN << "Initialize plane slam, register first observation." << endl;
    init_pose.print(" - Initial pose: ");
    cout << RESET << endl;
}

Pose3 PlaneSlam::planeSlam(Pose3 &rel_pose, std::vector<PlaneType> &planes)
{
    if(first_pose_)
    {
        ROS_ERROR("You should initialize the map before doing slam.");
        exit(1);
    }

    // Create a Factor Graph and Values to hold the new data
    NonlinearFactorGraph graph; // factor graph
    Values initial_estimate; // initial guess

    // Add odometry factors
    Vector odom_sigmas(6);
    odom_sigmas << rel_pose.translation().vector()*0.2, rel_pose.rotation().rpy() * 0.1;
    noiseModel::Diagonal::shared_ptr odometry_noise =
            noiseModel::Diagonal::Sigmas( odom_sigmas );
    Key pose_key = Symbol('x', pose_count_);
    Key last_key = Symbol('x', pose_count_-1);
    graph.push_back(BetweenFactor<Pose3>(last_key, pose_key, rel_pose, odometry_noise));
    // Add pose guess
    Pose3 new_pose = poses_.at<Pose3>( last_key );
    new_pose.compose( rel_pose );
    initial_estimate.insert( pose_key, new_pose );
    pose_count_ ++;

    // Transform modeled landmakr to pose frame

    // Match modelled landmark with measurement

    // Add factor to exist landmark

    // Add new landmark

    // Update graph
    isam2_->update(graph, initial_estimate);

    // Update pose
    Pose3 current_estimate = isam2_->calculateEstimate(pose_key).cast<Pose3>();
    poses_.insert( pose_key, current_estimate);
    current_estimate.print("Current estimate:");

    cout << GREEN << "Do slam, pose count: " << pose_count_ << endl;
    current_estimate.print( " - Current_pose: ");
    cout << RESET << endl;

    return current_estimate;
}
