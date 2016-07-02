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
  , pose_count_( 0 )
  , landmark_max_count_( 0 )
  , plane_match_direction_threshold_( 0.1 )
  , plane_match_distance_threshold_( 0.05 )
  , plane_match_check_overlap_( true )
  , plane_match_overlap_alpha_( 0.5 )
  , plane_inlier_leaf_size_( 0.02f )
  , plane_hull_alpha_( 0.5 )
  , rng(12345)
{
//    isam2_parameters_.relinearizeThreshold = 0.1;
//    isam2_parameters_.relinearizeSkip = 1;
    isam2_parameters_.enableRelinearization = 0;
    isam2_parameters_.factorization = ISAM2Params::Factorization::QR;
    isam2_parameters_.print( "ISAM2 parameters:" );
    isam2_ = new ISAM2( isam2_parameters_ );

    //
    path_publisher_ = nh_.advertise<nav_msgs::Path>("estimated_path", 10);
    odom_path_publisher_ = nh_.advertise<nav_msgs::Path>("odom_path", 10);
    marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
}

bool PlaneSlam::initialize(Pose3 &init_pose, KinectFrame &frame)
{
    std::vector<PlaneType> &planes = frame.segment_planes;

    if(planes.size() == 0)
        return false;

    // clear
    pose_count_ = 0;
    landmark_max_count_ = 0;
    initial_estimate_.clear();
    estimated_poses_.clear();
    odom_poses_.clear();
    landmarks_.clear();
    estimated_planes_.clear();


    // Add a prior factor
    pose_count_ = 0;
    Key x0 = Symbol('x', 0);
    Vector pose_sigmas(6);
//    pose_sigmas << init_pose.translation().vector()*0.2, init_pose.rotation().rpy() * 0.1;
    pose_sigmas << 0.2, 0.2, 0.2, 0.1, 0.1, 0.1;
    noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas( pose_sigmas ); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
    graph_.push_back( PriorFactor<Pose3>( x0, init_pose, poseNoise ) );

    // Add an initial guess for the current pose
    initial_estimate_.insert<Pose3>( x0, init_pose );
    // Add to estimated pose
    estimated_poses_.push_back( init_pose );
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

    landmark_max_count_ = 0;
    for(int i = 0; i < planes.size(); i++)
    {
        PlaneType &plane = planes[i];
        Key ln = Symbol('l', i);

        // Add observation factor
        noiseModel::Diagonal::shared_ptr obs_noise = noiseModel::Diagonal::Sigmas( plane.sigmas );
        graph_.push_back( OrientedPlane3Factor(plane.coefficients, obs_noise, x0, ln) );

        // Add initial guesses to all observed landmarks
//        cout << "Key: " << ln << endl;
        OrientedPlane3 lmn( plane.coefficients );
        OrientedPlane3 glmn = lmn.transform( init_pose.inverse() );
        initial_estimate_.insert<OrientedPlane3>( ln,  glmn );

        // Add to estimated plane
        estimated_planes_.push_back( glmn );

        // Add to landmarks buffer
        PlaneType global_plane;
        global_plane.coefficients = glmn.planeCoefficients();
        global_plane.color.Blue = rng.uniform(0, 255);
        global_plane.color.Green = rng.uniform(0, 255);
        global_plane.color.Red = rng.uniform(0, 255);
        global_plane.color.Alpha = 255;
        Eigen::Matrix4d transform = init_pose.matrix();
        PointCloudTypePtr cloud_filtered( new PointCloudType );
        voxelGridFilter( plane.cloud, cloud_filtered, plane_inlier_leaf_size_ );
        transformPointCloud( *cloud_filtered, *global_plane.cloud, transform, global_plane.color );
//        transformPointCloud( *plane.cloud_boundary, *global_plane.cloud_boundary, transform );
//        transformPointCloud( *plane.cloud_hull, *global_plane.cloud_hull, transform );
        Eigen::Vector4f cen;
        pcl::compute3DCentroid( *global_plane.cloud, cen );
        global_plane.centroid.x = cen[0];
        global_plane.centroid.y = cen[1];
        global_plane.centroid.z = cen[2];
        landmarks_.push_back( global_plane );

        //
        landmark_max_count_ ++;
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

    return true;
}

Pose3 PlaneSlam::planeSlam(Pose3 &odom_pose, KinectFrame &frame)
{
    std::vector<PlaneType> &planes = frame.segment_planes;

    if(first_pose_)
    {
        ROS_ERROR("You should call initialize() before doing slam.");
        exit(1);
    }
    // calculate relative pose and new pose
    Pose3 rel_pose = odom_pose_.inverse() * odom_pose;
    Pose3 new_pose = last_estimate_pose_ * rel_pose;

    // convert to gtsam plane type
    std::vector<OrientedPlane3> observations;
    for( int i = 0; i < planes.size(); i++)
    {
        observations.push_back( OrientedPlane3(planes[i].coefficients) );
    }

    // Transform modeled landmakr to pose frame
    std::vector<OrientedPlane3> predicted_observations;
    predictObservation( estimated_planes_, new_pose, predicted_observations);
//    getPredictedObservation( new_pose, predicted_observations );

    // Match modelled landmark with measurement
    std::vector<PlanePair> pairs;
    matchPlanes( predicted_observations, landmarks_, observations, planes, odom_pose, pairs);

    cout << GREEN << " find pairs(obs, lm): " << pairs.size() << RESET << endl;
    // Print pairs
    for( int i = 0; i < pairs.size(); i++)
    {
        const PlanePair &pair = pairs[i];
        cout << "  - " << pair.iobs << ", " << pair.ilm << endl;
    }

//    // check pairs
//    if( pairs.size() < 3 )
//        return new_pose;

    // Add odometry factors
    Vector odom_sigmas(6);
//    odom_sigmas << rel_pose.translation().vector()*0.2, rel_pose.rotation().rpy() * 0.1;
    odom_sigmas << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    noiseModel::Diagonal::shared_ptr odometry_noise =
            noiseModel::Diagonal::Sigmas( odom_sigmas );
//    cout << GREEN << "odom noise dim: " << odometry_noise->dim() << RESET << endl;
    Key pose_key = Symbol('x', pose_count_);
    Key last_key = Symbol('x', pose_count_-1);
    graph_.push_back(BetweenFactor<Pose3>(last_key, pose_key, rel_pose, odometry_noise));
    // Add pose guess
    initial_estimate_.insert<Pose3>( pose_key, new_pose );
    pose_count_ ++;

    // Add factor to exist landmark
    Eigen::VectorXi unpairs = Eigen::VectorXi::Ones( planes.size() );
    for( int i = 0; i < pairs.size(); i++)
    {
        PlanePair &pair = pairs[i];
        PlaneType &obs = planes[pair.iobs];
        unpairs[pair.iobs] = 0;
        Key ln =  Symbol( 'l', pair.ilm);
        noiseModel::Diagonal::shared_ptr obs_noise = noiseModel::Diagonal::Sigmas( obs.sigmas );
        graph_.push_back( OrientedPlane3Factor(obs.coefficients, obs_noise, pose_key, ln) );
    }

    // Add new landmark
    for( int i = 0; i < unpairs.size(); i++ )
    {
        if( unpairs[i] )
        {
            // Add factor
            PlaneType &obs = planes[i];
            Key ln = Symbol('l', landmark_max_count_);
            noiseModel::Diagonal::shared_ptr obs_noise = noiseModel::Diagonal::Sigmas( obs.sigmas );
            graph_.push_back( OrientedPlane3Factor(obs.coefficients, obs_noise, pose_key, ln) );

            // Add initial guess
            OrientedPlane3 lmn( obs.coefficients );
            OrientedPlane3 glmn = lmn.transform( new_pose.inverse() );
            initial_estimate_.insert<OrientedPlane3>( ln, glmn );

            //
            landmark_max_count_ ++;
        }
    }

    // Update graph
    cout << "update " << endl;
    isam2_->update(graph_, initial_estimate_);
    isam2_->update(); // call additionally
    isam2_->update(); // call additionally

    // Update estimated poses and planes
    updateSlamResult( estimated_poses_, estimated_planes_ );

    // Update pose
    cout << pose_count_ << " get current est " << endl;
    Pose3 current_estimate = estimated_poses_[estimated_poses_.size() - 1];
//    Pose3 current_estimate = isam2_->calculateEstimate( pose_key ).cast<Pose3>();

    // Update landmarks
    ros::Time utime =  ros::Time::now();
    updateLandmarks( landmarks_, planes, pairs, current_estimate, estimated_planes_ );
    cout << YELLOW << " - landmarks update time: " << (ros::Time::now() - utime).toSec()*1000
         << " ms " << RESET << endl;

//    // Print estimate pose:
//    cout << BLUE;
//    current_estimate.print("Current estimate:");
//    cout << RESET << endl;

    // Clear the factor graph and values for the next iteration
    graph_.resize(0);
    initial_estimate_.clear();
    last_estimate_pose_ = current_estimate;

    if( refine_planar_map_ )
    {
        refinePlanarMap();
    }

    // Store odom_pose
    odom_pose_ = odom_pose;
    odom_poses_.push_back( odom_pose_ );

    return current_estimate;
}

Pose3 PlaneSlam::planeMapping(Pose3 &odom_pose, KinectFrame &frame)
{
    std::vector<PlaneType> &planes = frame.segment_planes;

    if(first_pose_)
    {
        ROS_ERROR("You should call initialize() before doing slam.");
        exit(1);
    }
    // calculate relative pose and new pose
    Pose3 new_pose = odom_pose;

    // convert to gtsam plane type
    std::vector<OrientedPlane3> observations;
    for( int i = 0; i < planes.size(); i++)
    {
        observations.push_back( OrientedPlane3(planes[i].coefficients) );
    }

    // Transform modeled landmakr to pose frame
    std::vector<OrientedPlane3> predicted_observations;
    predictObservation( estimated_planes_, new_pose, predicted_observations);

    // Match modelled landmark with measurement
    std::vector<PlanePair> pairs;
    matchPlanes( predicted_observations, landmarks_, observations, planes, new_pose, pairs);

    cout << GREEN << " find pairs(obs, lm): " << pairs.size() << RESET << endl;
    // Print pairs
    for( int i = 0; i < pairs.size(); i++)
    {
        const PlanePair &pair = pairs[i];
        cout << "  - " << pair.iobs << ", " << pair.ilm << endl;
    }

//    // check pairs
//    if( pairs.size() < 3 )
//        return new_pose;


    // Update landmarks
    ros::Time utime =  ros::Time::now();
    updateLandmarks( landmarks_, planes, pairs, new_pose, estimated_planes_ );
    cout << YELLOW << " - landmarks update time: " << (ros::Time::now() - utime).toSec()*1000
         << " ms " << RESET << endl;

    // Add new landmark to estimated planes
    Eigen::VectorXi unpairs = Eigen::VectorXi::Ones( observations.size() );
    for( int i = 0; i < pairs.size(); i++)
    {
        unpairs[ pairs[i].iobs ] = 0;
    }
    for( int i = 0; i < unpairs.rows(); i++)
    {
        if( unpairs[i] )
        {

            OrientedPlane3 &obs = observations[i];
            OrientedPlane3 lm = obs.transform( new_pose.inverse() );
            estimated_planes_.push_back( lm );
        }
    }


    // Refine map
    if( refine_planar_map_ )
    {
        refinePlanarMap();
    }

    last_estimate_pose_ = odom_pose;

    // Store odom_pose
    odom_pose_ = odom_pose;
    odom_poses_.push_back( odom_pose_ );

    // Store estimated pose
    estimated_poses_.push_back( odom_pose );
    pose_count_ ++;

    return last_estimate_pose_;
}

// return true if being refined.
bool PlaneSlam::refinePlanarMap()
{
    cout << RED << ", lm size: " << landmarks_.size()
         << ", pl size: " << estimated_planes_.size() << endl;

    // find co-planar landmark pair
    bool find_coplanar = false;
    const int num = landmarks_.size();
    const double direction_threshold = planar_merge_direction_threshold_;
    const double distance_threshold = planar_merge_distance_threshold_;
    for( int i = 0; i < (num - 1 ); i++)
    {
        PlaneType &p1 = landmarks_[i];
        OrientedPlane3 &lm1 = estimated_planes_[i];

        // check is alive
        if( !p1.valid )
            continue;

        // plane coordinate
        Point3 point( p1.centroid.x, p1.centroid.y, p1.centroid.z );
        Point3 col3 = lm1.normal().point3();
        Matrix32 basis = lm1.normal().basis();
        Point3 col1( basis(0,0), basis(1,0), basis(2,0) );
        Point3 col2( basis(0,1), basis(1,1), basis(2,1) );
        Rot3 rot3( col1, col2, col3 );
        Pose3 local( rot3, point);
        // Transform observation to local frame
        const OrientedPlane3 &llm1 = lm1.transform( local );

        for( int j = i+1; j < num; j++)
        {
            PlaneType &p2 = landmarks_[j];
            OrientedPlane3 &lm2 = estimated_planes_[j];

            // check if alive
            if( !p2.valid )
                continue;

            /// Match two plane
            // Transform landmark to local frame
            const OrientedPlane3 &llm2 = lm2.transform( local );
            double dr_error = acos( llm1.normal().dot( llm2.normal() ));
            double ds_error = fabs( llm1.distance() - llm2.distance() );
//            cout << CYAN << "  - " << i << "*" << j << ": " << dr_error << "("<< direction_threshold << "), "
//                 << ds_error << "(" << distance_threshold << ")" << RESET << endl;
            if( (fabs(dr_error) < direction_threshold)
                    && (ds_error < distance_threshold) )
            {
                // check if overlap
                bool overlap = false;
                if( p1.cloud->size() < p2.cloud->size() )
                    overlap = checkLandmarksOverlap( p2, p1);
                else
                    overlap = checkLandmarksOverlap( p1, p2);

                if( overlap )
                {
                    // merge, make smaller one invalid
                    if( p1.cloud->size() < p2.cloud->size() )
                    {
                        mergeLandmarkInlier( p1, p2);
                        p1.valid = false;
                        find_coplanar = true;
                        cout << RED << "  -- merge co-planar: " << i << " to " << j << RESET << endl;
                        break;
                    }
                    else
                    {
                        mergeLandmarkInlier( p2, p1);
                        find_coplanar = true;
                        cout << RED << "  -- merge co-planar: " << j << " to " << i << RESET << endl;
                        p2.valid = false;
                    }
                } // end of if overlap

            }

        } // end of for( int j = i+1; j < num; j++)

    } // end of for( int i = 0; i < (num - 1 ); i++)

    return find_coplanar;
}

// simple euclidian distance
void PlaneSlam::matchPlanes( const std::vector<OrientedPlane3> &predicted_observations,
                             const std::vector<PlaneType> &landmarks,
                             const std::vector<OrientedPlane3> &observations,
                             const std::vector<PlaneType> &observed_planes,
                             const Pose3 pose,
                             std::vector<PlanePair> &pairs)
{
//    Eigen::VectorXd paired = Eigen::VectorXd::Zero( predicted_observations.size() );
    for( int i = 0; i < observations.size(); i++)
    {
        const PlaneType &observed = observed_planes[i];
        const OrientedPlane3 &obs = observations[i];
        // plane coordinate
        Point3 point( observed.centroid.x, observed.centroid.y, observed.centroid.z );
        Point3 col3 = obs.normal().point3();
        Matrix32 basis = obs.normal().basis();
        Point3 col1( basis(0,0), basis(1,0), basis(2,0) );
        Point3 col2( basis(0,1), basis(1,1), basis(2,1) );
        Rot3 rot3( col1, col2, col3 );
        Pose3 local( rot3, point);
//        local.print("local coord: ");
        // Transform observation to local frame
        const OrientedPlane3 &lobs = obs.transform( local );
        int min_index = -1;
        int max_size = 0;
        for( int l = 0; l < predicted_observations.size(); l++)
        {
//            if( paired[l] )
//                continue;

            const PlaneType &glm = landmarks[l];
            const OrientedPlane3 &plm = predicted_observations[l];

            // check if alive
            if( !glm.valid )
                continue;

            // Transform landmark to local frame
            const OrientedPlane3 &llm = plm.transform( local );
            double cs = lobs.normal().dot( llm.normal() );
            double dr_error = acos( cs );
            double ds_error = fabs( lobs.distance() - llm.distance() );
//            cout << CYAN << "  - " << i << "*" << l << ": " << dr_error << "("<< plane_match_direction_threshold_ << "), "
//                 << ds_error << "(" << plane_match_distance_threshold_ << ")" << RESET << endl;

//            double dir_error = acos( obs.normal().dot( plm.normal() ));
//            double dis_error = fabs( obs.distance() - plm.distance() );
//            cout << YELLOW << "  - " << i << "*" << l << ": " << dir_error << "("<< plane_match_direction_threshold_ << "), "
//                 << dis_error << "(" << plane_match_distance_threshold_ << ")" << RESET << endl;
            if( (fabs(dr_error) < plane_match_direction_threshold_)
                    && (ds_error < plane_match_distance_threshold_) )
            {
                if( glm.cloud->size() > max_size )
                {
                    if( plane_match_check_overlap_ && !checkOverlap( glm.cloud, glm.coefficients, observed.cloud, pose ) )
                        continue;
    //                min_d = d;
                    min_index = l;
                    max_size = glm.cloud->size();

                }
            }
        }
        if( min_index >= 0 )
        {
//            paired[min_index] = 1;
            pairs.push_back( PlanePair(i, min_index) );
        }
    }
}

// get predicted landmarks
void PlaneSlam::getPredictedObservation( Pose3 &pose, std::vector<OrientedPlane3> &predicted_observations )
{
    Values values = isam2_->calculateBestEstimate();

    for(int i = 0; i < landmark_max_count_; i++)
    {
        Key ln = Symbol('l', i);
        if( values.exists( ln ))
        {
            OrientedPlane3 predicted = values.at(ln).cast<OrientedPlane3>();
            predicted_observations.push_back( predicted.transform( pose ) );
        }
    }
}

// get predicted landmarks
void PlaneSlam::predictObservation( std::vector<OrientedPlane3> &landmarks, Pose3 &pose,
                                    std::vector<OrientedPlane3> &predicted_observations)
{
    predicted_observations.clear();
    for(int i = 0; i < landmarks.size(); i++)
    {
        predicted_observations.push_back( landmarks[i].transform( pose ) );
    }
}

void PlaneSlam::updateSlamResult( std::vector<Pose3> &poses, std::vector<OrientedPlane3> &planes )
{
    // clear buffer
    poses.clear();
    planes.clear();

    //
    Values values = isam2_->calculateBestEstimate();
    for(int i = 0; i < pose_count_; i++)
    {
        Key xn = Symbol('x', i);
        Pose3 pose = values.at(xn).cast<Pose3>();
        poses.push_back( pose );
    }
    for(int i = 0; i < landmark_max_count_; i++)
    {

        Key ln = Symbol('l', i);
        if( values.exists(ln) )
        {
            OrientedPlane3 predicted = values.at(ln).cast<OrientedPlane3>();
            planes.push_back( predicted );
        }
        else
        {
            planes.push_back( OrientedPlane3() );
        }
    }
}

void PlaneSlam::updateLandmarks( std::vector<PlaneType> &landmarks,
                                 const std::vector<PlaneType> &observations,
                                 const std::vector<PlanePair> &pairs,
                                 const Pose3 &estimated_pose,
                                 const std::vector<OrientedPlane3> &estimated_planes)

{
//    if( landmarks.size() != estimated_planes.size() )
//    {
//        cout << RED << "[Error]: landmark.size() != estimated_planes.size()" << RESET << endl;
//        return;
//    }

    Eigen::Matrix4d transform = estimated_pose.matrix();

    for( int i = 0; i < landmarks.size(); i++)
    {
        PlaneType &lm = landmarks[i] ;
        lm.coefficients = estimated_planes[i].planeCoefficients();
    }

    Eigen::VectorXi unpairs = Eigen::VectorXi::Ones( observations.size() );

    for( int i = 0; i < pairs.size(); i++)
    {
        const int iobs = pairs[i].iobs;
        const int ilm = pairs[i].ilm;
        unpairs[iobs] = 0;
        const PlaneType &obs = observations[ iobs ];
        PlaneType &lm = landmarks[ ilm ] ;

        PointCloudTypePtr cloud( new PointCloudType ), cloud_filtered( new PointCloudType );
//        PointCloudTypePtr cloud_boundary( new PointCloudType );
//        PointCloudTypePtr cloud_hull( new PointCloudType );

        // downsample
        voxelGridFilter( obs.cloud, cloud_filtered, plane_inlier_leaf_size_ );
        // transform cloud
        transformPointCloud( *cloud_filtered, *cloud, transform, lm.color );
//        transformPointCloud( *obs.cloud_boundary, *cloud_boundary, transform );
//        transformPointCloud( *obs.cloud_hull, *cloud_hull, transform );

        // sum
        *cloud += *lm.cloud;
//        *lm.cloud_boundary += *cloud_boundary;
//        *lm.cloud_hull += *cloud_hull;

        // project
        projectPoints( *cloud, lm.coefficients, *cloud_filtered );
//        projectPoints( *lm.cloud_boundary, lm.coefficients, *cloud_boundary );
//        projectPoints( *lm.cloud_hull, lm.coefficients, *cloud_hull );

        // refresh inlier, boundary, hull
        voxelGridFilter( cloud_filtered, lm.cloud, plane_inlier_leaf_size_ );
//        cloudHull( cloud_boundary, lm.cloud_boundary );
//        cloudHull( cloud, lm.cloud_hull );

        // compute new centroid
        Eigen::Vector4d cen;
        pcl::compute3DCentroid( *lm.cloud, cen);
        lm.centroid.x = cen[0];
        lm.centroid.y = cen[1];
        lm.centroid.z = cen[2];
    }

    // Add new to landmarks buffer
    int lm_num = landmarks.size();
    for( int i = 0; i < unpairs.rows(); i++)
    {
        if( unpairs[i] )
        {
            const PlaneType &obs = observations[i];
            PlaneType global_plane;
            global_plane.coefficients = estimated_planes[lm_num].planeCoefficients();
            global_plane.color.Blue = rng.uniform(0, 255);
            global_plane.color.Green = rng.uniform(0, 255);
            global_plane.color.Red = rng.uniform(0, 255);
            global_plane.color.Alpha = 255;
            PointCloudTypePtr cloud_filtered( new PointCloudType );
            voxelGridFilter( obs.cloud, cloud_filtered, plane_inlier_leaf_size_ );
            transformPointCloud( *cloud_filtered, *global_plane.cloud, transform, global_plane.color );
//            transformPointCloud( *cloud_filtered, *global_plane.cloud, transform );
//            transformPointCloud( *obs.cloud_boundary, *global_plane.cloud_boundary, transform );
//            transformPointCloud( *obs.cloud_hull, *global_plane.cloud_hull, transform );
            Eigen::Vector4f cen;
            pcl::compute3DCentroid( *global_plane.cloud, cen );
            global_plane.centroid.x = cen[0];
            global_plane.centroid.y = cen[1];
            global_plane.centroid.z = cen[2];

            landmarks.push_back( global_plane );
            lm_num = landmarks.size();
        }
    }

}

NonlinearFactorGraph PlaneSlam::graphRekey( NonlinearFactorGraph &graph, const std::map<Key,Key>& rekey_mapping)
{
    NonlinearFactorGraph result;
    BOOST_FOREACH( boost::shared_ptr<NonlinearFactor> & f, graph)
    {
        if (f)
        {
            boost::shared_ptr<NonlinearFactor> new_factor =
                    boost::static_pointer_cast<NonlinearFactor>( boost::shared_ptr<NonlinearFactor>() );
            *new_factor = *f;
            nonlinearFactorRekey(new_factor, rekey_mapping);
            result.push_back( f );
        }
        else
        {
            result.push_back(boost::shared_ptr<NonlinearFactor>());
        }
    }

    return result;
}

void PlaneSlam::nonlinearFactorRekey(boost::shared_ptr<NonlinearFactor> &factor, const std::map<Key, Key>& rekey_mapping)
{
    for (size_t i = 0; i < factor->size(); ++i)
    {
        Key& cur_key = factor->keys()[i];
        std::map<Key, Key>::const_iterator mapping = rekey_mapping.find(cur_key);
        if (mapping != rekey_mapping.end())
            cur_key = mapping->second;
    }
}

void PlaneSlam::publishEstimatedPath()
{
    // publish trajectory
    nav_msgs::Path path;
    path.header.frame_id = "/world";
    path.header.stamp = ros::Time::now();
    for(int i = 0; i < estimated_poses_.size(); i++)
    {
        Pose3 &pose = estimated_poses_[i];
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
    cout << GREEN << "Publisher path, p = " << estimated_poses_.size() << RESET << endl;
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

void PlaneSlam::voxelGridFilter( const PointCloudTypePtr &cloud,
                                  PointCloudTypePtr &cloud_filtered,
                                  float leaf_size)
{
    // Create the filtering object
    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud ( cloud );
    sor.setLeafSize ( leaf_size, leaf_size, leaf_size );
    sor.filter ( *cloud_filtered );
}

void PlaneSlam::extractPlaneHulls(const PointCloudTypePtr &input, std::vector<PlaneType> &planes)
{
    for(int i = 0; i < planes.size(); i++)
    {
        PlaneType &plane = planes[i];
        // projected cloud
        Eigen::Vector4f model_coefficients;
        model_coefficients[0] = plane.coefficients[0];
        model_coefficients[1] = plane.coefficients[1];
        model_coefficients[2] = plane.coefficients[2];
        model_coefficients[3] = plane.coefficients[3];
        projectPoints( *input, plane.inlier, model_coefficients, *(plane.cloud) );
        // hull
        cloudHull( plane.cloud, plane.cloud_hull );
    }
}

void PlaneSlam::cloudHull( const PointCloudTypePtr &cloud, PointCloudTypePtr &cloud_hull)
{
    pcl::ConcaveHull<PointType> chull;
    chull.setInputCloud ( cloud );
    chull.setAlpha ( plane_hull_alpha_ );
    chull.reconstruct ( *cloud_hull );
}

bool PlaneSlam::checkOverlap( const PointCloudTypePtr &landmark_cloud, const OrientedPlane3 &landmark,
                              const PointCloudTypePtr &observation, const Pose3 &pose)
{
    // transform and project pointcloud
    PointCloudTypePtr cloud( new PointCloudType ), cloud_projected( new PointCloudType );
    transformPointCloud( *observation, *cloud, pose.matrix() );
    projectPoints( *cloud, landmark.planeCoefficients(), *cloud_projected );

    // build octree
    float resolution = plane_inlier_leaf_size_;
    pcl::octree::OctreePointCloud<PointType> octreeD (resolution);
    octreeD.setInputCloud( landmark_cloud );
    octreeD.addPointsFromInputCloud();

    // check if occupied
    int collision = 0;
    PointCloudType::iterator it = cloud_projected->begin();
    for( ; it != cloud_projected->end(); it++)
    {
        PointType &pt = *it;
        if( octreeD.isVoxelOccupiedAtPoint( pt ) )
            collision ++;
    }

    cout << GREEN << "  - collision: " << collision << "/" << cloud_projected->size() << RESET << endl;

    double alpha = ((float)collision) / (float)cloud_projected->size();
    if( alpha < plane_match_overlap_alpha_ )
        return false;
    else
        return true;

    return true;
}

// indices of lm1 must bigger than that of lm2
bool PlaneSlam::checkLandmarksOverlap( const PlaneType &lm1, const PlaneType &lm2)
{
    // project lm2 inlier to lm1 plane
    PointCloudTypePtr cloud_projected( new PointCloudType );
    projectPoints( *lm2.cloud, lm1.coefficients, *cloud_projected );

    // build octree from lm1
    float resolution = plane_inlier_leaf_size_;
    pcl::octree::OctreePointCloud<PointType> octreeD (resolution);
    octreeD.setInputCloud( lm1.cloud );
    octreeD.addPointsFromInputCloud();

    // check if occupied
    int collision = 0;
    PointCloudType::iterator it = cloud_projected->begin();
    for( ; it != cloud_projected->end(); it++)
    {
        PointType &pt = *it;
        if( octreeD.isVoxelOccupiedAtPoint( pt ) )
        {
            collision ++;
            if(collision > 10)
                return true;
        }
    }

//    cout << GREEN << "  - collision: " << collision << "/" << cloud_projected->size() << RESET << endl;

//    double alpha = ((float)collision) / (float)cloud_projected->size();
//    if( alpha < plane_match_overlap_alpha_ )
//        return false;
//    else
//        return true;

    return false;
}

// just do project and voxel filter
void PlaneSlam::mergeLandmarkInlier( PlaneType &from, PlaneType &to)
{
    PointCloudTypePtr cloud( new PointCloudType );
    projectPoints( *from.cloud, to.coefficients, *cloud);
    *cloud += *to.cloud;
    voxelGridFilter( cloud, to.cloud, plane_inlier_leaf_size_ );
}

