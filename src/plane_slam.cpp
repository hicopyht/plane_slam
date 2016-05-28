#include "plane_slam.h"

/*
 * Useage:
 * 1. call function 'initialize' to register first observation.
 * 2. call function 'planeSlam' to process every observation.
 * 3. publishPath() to publish estimate path
 */

PlaneSlam::PlaneSlam() :
    private_nh_("~")
  , isam2_parameters_()
  , graph_()
  , initial_estimate_()
  , first_pose_(true)
  , poses_()
  , pose_count_( 0 )
  , landmark_count_( 0 )
  , plane_match_threshold_( 0.1 )
{
    isam2_parameters_.relinearizeThreshold = 0.05;
    isam2_parameters_.relinearizeSkip = 1;
    isam2_parameters_.print( "ISAM2 parameters:" );
    isam2_ = new ISAM2( isam2_parameters_ );

    //
    path_publisher_ = nh_.advertise<nav_msgs::Path>("camera_path", 10);
    odom_path_publisher_ = nh_.advertise<nav_msgs::Path>("odom_path", 10);
    marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
}

void PlaneSlam::initialize(Pose3 &init_pose, std::vector<PlaneType> &planes)
{
    if(planes.size() == 0)
        return;

    // Add a prior factor
    pose_count_ = 0;
    Key x0 = Symbol('x', 0);
    Vector pose_sigmas(6);
    pose_sigmas << init_pose.translation().vector()*0.2, init_pose.rotation().rpy() * 0.1;
    noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas( pose_sigmas ); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
    graph_.push_back( PriorFactor<Pose3>( x0, init_pose, poseNoise ) );

    // Add an initial guess for the current pose
    initial_estimate_.insert<Pose3>( x0, init_pose );
    poses_.insert( x0, init_pose );
    pose_count_++;

    // Add a prior landmark
    Key l0 = Symbol('l', 0);
    OrientedPlane3 lm0(planes[0].coefficients);
    OrientedPlane3 glm0 = lm0.transform(init_pose.inverse());
    noiseModel::Diagonal::shared_ptr lm_noise = noiseModel::Diagonal::Sigmas( (Vector(2) << planes[0].sigmas[0], planes[0].sigmas[1]).finished() );
    graph_.push_back( OrientedPlane3DirectionPrior( l0, glm0.planeCoefficients(), lm_noise) );

    // Add odom pose to path
    odom_pose_ = init_pose;
    odom_poses_.clear();
    odom_poses_.push_back( odom_pose_ );

    landmark_count_ = 0;
    for(int i = 0; i < planes.size(); i++)
    {
        PlaneType &plane = planes[i];
        Key ln = Symbol('l', i);

        // Add observation factor
        noiseModel::Diagonal::shared_ptr obs_noise = noiseModel::Diagonal::Sigmas( plane.sigmas );
        graph_.push_back( OrientedPlane3Factor(plane.coefficients, obs_noise, x0, ln) );

        // Add initial guesses to all observed landmarks
        cout << "Key: " << ln << endl;
        OrientedPlane3 lmn( plane.coefficients );
        initial_estimate_.insert<OrientedPlane3>( ln, lmn.transform( init_pose.inverse() ) );

        //
        landmark_count_ ++;
    }
    isam2_->update( graph_, initial_estimate_ );
    last_estimate_pose_ = init_pose;
    // Clear the factor graph and values for the next iteration
    graph_.resize(0);
    initial_estimate_.clear();
    //
    first_pose_ = false;

    cout << GREEN << "Initialize plane slam, register first observation." << endl;
    initial_estimate_.print(" - Initial estimate: ");
    cout << RESET << endl;
}

Pose3 PlaneSlam::planeSlam(Pose3 &odom_pose, std::vector<PlaneType> &planes)
{
    if(first_pose_)
    {
        ROS_ERROR("You should call initialize() before doing slam.");
        exit(1);
    }
    // calculate relative pose
    Pose3 rel_pose = odom_pose_.inverse() * odom_pose;

    // convert to gtsam plane type
    std::vector<OrientedPlane3> observations;
    for( int i = 0; i < planes.size(); i++)
    {
        observations.push_back( OrientedPlane3(planes[i].coefficients) );
    }

    // Add odometry factors
    Vector odom_sigmas(6);
    odom_sigmas << rel_pose.translation().vector()*0.2, rel_pose.rotation().rpy() * 0.1;
    noiseModel::Diagonal::shared_ptr odometry_noise =
            noiseModel::Diagonal::Sigmas( odom_sigmas );
    cout << GREEN << "odom noise dim: " << odometry_noise->dim() << RESET << endl;
    Key pose_key = Symbol('x', pose_count_);
    Key last_key = Symbol('x', pose_count_-1);
    graph_.push_back(BetweenFactor<Pose3>(last_key, pose_key, rel_pose, odometry_noise));
    // Add pose guess
    Pose3 new_pose = last_estimate_pose_ * rel_pose;
    initial_estimate_.insert<Pose3>( pose_key, new_pose );
    pose_count_ ++;

    // Add odom_pose
    odom_pose_ = odom_pose;
    odom_poses_.push_back( odom_pose_ );

//    //
//    cout << RED;
//    rel_pose.print(" rel pose: ");
//    cout << RESET << endl;
//    //
//    cout << GREEN;
//    new_pose.print(" new pose: ");
//    cout << RESET << endl;

    // Transform modeled landmakr to pose frame
    std::vector<OrientedPlane3> predicted_observations;
    getPredictedObservation( new_pose, predicted_observations );

    // Match modelled landmark with measurement
    std::vector<PlanePair> pairs;
    matchPlanes( predicted_observations, observations, pairs);

    // Add factor to exist landmark
    Eigen::VectorXd unpairs = Eigen::VectorXd::Ones( planes.size() );
    for( int i = 0; i < pairs.size(); i++)
    {
        PlanePair &pair = pairs[i];
        PlaneType &obs = planes[pair.iobs];
        unpairs[pair.iobs] = 0;
        Key ln = Symbol('l', pair.ilm);
        noiseModel::Diagonal::shared_ptr obs_noise = noiseModel::Diagonal::Sigmas( obs.sigmas );
        graph_.push_back( OrientedPlane3Factor(obs.coefficients, obs_noise, pose_key, ln) );
    }
    cout << GREEN << " find pairs: " << pairs.size() << RESET << endl;

    // Add new landmark
    for( int i = 0; i < unpairs.size(); i++ )
    {
        if( unpairs[i] )
        {
            // Add factor
            PlaneType &obs = planes[i];
            Key ln = Symbol('l', landmark_count_);
            noiseModel::Diagonal::shared_ptr obs_noise = noiseModel::Diagonal::Sigmas( obs.sigmas );
            graph_.push_back( OrientedPlane3Factor(obs.coefficients, obs_noise, pose_key, ln) );

            // Add initial guess
            OrientedPlane3 lmn( obs.coefficients );
            initial_estimate_.insert<OrientedPlane3>( ln, lmn.transform( new_pose.inverse() ) );

            //
            landmark_count_ ++;
        }
    }

    cout << GREEN << " lm number: " << landmark_count_ << RESET << endl;

    // Update graph
    cout << "update " << endl;
    isam2_->update(graph_, initial_estimate_);
    // isam2->update(); // call additionally

    // Update pose
    cout << pose_count_ << " get current est " << endl;
    Pose3 current_estimate = isam2_->calculateEstimate( pose_key ).cast<Pose3>();
    poses_.insert( pose_key, current_estimate);

    // Print estimate pose:
    cout << BLUE;
    current_estimate.print("Current estimate:");
    cout << RESET << endl;

    // Clear the factor graph and values for the next iteration
    graph_.resize(0);
    initial_estimate_.clear();
    last_estimate_pose_ = current_estimate;

    return current_estimate;
}

// simple euclidian distance
void PlaneSlam::matchPlanes( std::vector<OrientedPlane3> &predicted_observations,
                             std::vector<OrientedPlane3> &observations,
                             std::vector<PlanePair> &pairs)
{
    for( int i = 0; i < observations.size(); i++)
    {
        OrientedPlane3 &obs = observations[i];
        double min_d = 1e2;
        int min_index = -1;
        for( int l = 0; l < predicted_observations.size(); l++)
        {
            OrientedPlane3 &lm = predicted_observations[l];
            Vector3 error = obs.errorVector( lm );
            double d = error.dot( error );
            cout << YELLOW << "  - " << i << "*" << l << ": " << d << RESET << endl;
            if( d < plane_match_threshold_ && d < min_d)
            {
                min_d = d;
                min_index = l;
            }
        }
        if( min_index >= 0 )
            pairs.push_back( PlanePair(i, min_index) );
    }
}

// get predicted landmarks
void PlaneSlam::getPredictedObservation( Pose3 &pose, std::vector<OrientedPlane3> &predicted_observations )
{
    Values values = isam2_->calculateBestEstimate();

    for(int i = 0; i < landmark_count_; i++)
    {
        Key ln = Symbol('l', i);
        OrientedPlane3 predicted = values.at(ln).cast<OrientedPlane3>();
        predicted_observations.push_back( predicted.transform( pose ) );
    }
}

void PlaneSlam::publishPath()
{

    // get trajectory
    Values values = isam2_->calculateBestEstimate();
    std::vector<Pose3> poses;

    for(int i = 0; i < pose_count_; i++)
    {
        Key xn = Symbol('x', i);
        Pose3 pose = values.at(xn).cast<Pose3>();
        poses.push_back( pose );
    }

    // publish trajectory
    nav_msgs::Path path;
    path.header.frame_id = "/world";
    path.header.stamp = ros::Time::now();
    for(int i = 0; i < poses.size(); i++)
    {
        Pose3 &pose = poses[i];
        geometry_msgs::PoseStamped p;
        p.pose.position.x = pose.x();
        p.pose.position.y = pose.y();
        p.pose.position.z = pose.z();
        Eigen::Vector4d quater = pose.rotation().quaternion();
        p.pose.orientation.w = quater[0];
        p.pose.orientation.x = quater[1];
        p.pose.orientation.y = quater[2];
        p.pose.orientation.z = quater[3];
        path.poses.push_back( p );
    }
    path_publisher_.publish( path );
    cout << GREEN << "Publisher path, p = " << poses.size() << RESET << endl;
}

void PlaneSlam::publishOdomPath()
{
    // publish trajectory
    nav_msgs::Path path;
    path.header.frame_id = "/world";
    path.header.stamp = ros::Time::now();
    for(int i = 0; i < odom_poses_.size(); i++)
    {
        Pose3 &pose = odom_poses_[i];
        geometry_msgs::PoseStamped p;
        p.pose.position.x = pose.x();
        p.pose.position.y = pose.y();
        p.pose.position.z = pose.z();
        Eigen::Vector4d quater = pose.rotation().quaternion();
        p.pose.orientation.w = quater[0];
        p.pose.orientation.x = quater[1];
        p.pose.orientation.y = quater[2];
        p.pose.orientation.z = quater[3];
        path.poses.push_back( p );
    }
    odom_path_publisher_.publish( path );
    cout << GREEN << "Publisher odom path, p = " << odom_poses_.size() << RESET << endl;
}

void PlaneSlam::getLandmarks( std::vector<PlaneType> &planes )
{
    // get landmarks
    Values values = isam2_->calculateBestEstimate();

    for(int i = 0; i < landmark_count_; i++)
    {
        Key ln = Symbol('l', i);
        OrientedPlane3 oplane = values.at(ln).cast<OrientedPlane3>();
        PlaneType plane;
        plane.coefficients = oplane.planeCoefficients();
        planes.push_back( plane );
    }

}

void PlaneSlam::tfToPose3( tf::Transform &trans, gtsam::Pose3 &pose )
{
    Eigen::Matrix3d m33 = matrixTF2Eigen( trans.getBasis() );
    gtsam::Rot3 rot3(m33);
    gtsam::Point3 point3;
    tf::Vector3 origin = trans.getOrigin();
    point3[0] = origin.getX();
    point3[1] = origin.getY();
    point3[2] = origin.getZ();
    pose = gtsam::Pose3( rot3, point3 );
}

void PlaneSlam::pose3ToTF( gtsam::Pose3 &pose, tf::Transform &trans )
{
    trans.setOrigin( tf::Vector3( pose.x(), pose.y(), pose.z()) );
    tf::Matrix3x3 m33;
    matrixEigen2TF( pose.rotation().matrix(), m33 );
    trans.setBasis( m33 );
}
