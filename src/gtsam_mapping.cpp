#include "gtsam_mapping.h"

namespace plane_slam
{

GTMapping::GTMapping(ros::NodeHandle &nh, Viewer *viewer, Tracking *tracker)
    : nh_(nh)
    , viewer_( viewer )
    , tracker_( tracker )
    , map_frame_("/world")
    , mapping_config_server_( ros::NodeHandle( nh_, "GTMapping" ) )
    , isam2_parameters_()
    , factor_graph_()
    , initial_estimate_()
    , factor_graph_buffer_()
    , initial_estimate_buffer_()
    , map_cloud_( new PointCloudType )
    , octree_map_( new octomap::OcTree( 0.025 ) )
    , use_keyframe_( true )
    , keyframe_linear_threshold_( 0.05f )
    , keyframe_angular_threshold_( 5.0f*DEG_TO_RAD )
    , isam2_relinearize_threshold_( 0.05f )
    , isam2_relinearize_skip_( 1 )
    , plane_match_direction_threshold_( 10.0*DEG_TO_RAD )   // 10 degree
    , plane_match_distance_threshold_( 0.1 )    // 0.1meter
    , plane_match_check_overlap_( true )
    , plane_match_overlap_alpha_( 0.5 )
    , plane_inlier_leaf_size_( 0.05f )  // 0.05meter
    , plane_hull_alpha_( 0.5 )
    , octomap_resolution_( 0.025f )
    , octomap_max_depth_range_( 4.0f )
    , rng_(12345)
    , next_plane_id_( 0 )
    , next_point_id_( 0 )
    , next_frame_id_( 0 )
{
    // reconfigure
    mapping_config_callback_ = boost::bind(&GTMapping::gtMappingReconfigCallback, this, _1, _2);
    mapping_config_server_.setCallback(mapping_config_callback_);

    // Pose and Map
    optimized_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("optimized_pose", 4);
    optimized_path_publisher_ = nh_.advertise<nav_msgs::Path>("optimized_path", 4);
    map_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("map_cloud", 4);
    octomap_publisher_ = nh_.advertise<octomap_msgs::Octomap>("octomap", 4);
    keypoint_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("keypoints_cloud", 4);
    predicted_keypoint_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("predicted_keypoints", 4);
    frame_keypoint_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("frame_keypoints", 4);
    matched_keypoint_frame_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("matched_keypoints_frame", 4);
    matched_keypoint_global_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("matched_keypoints_global", 4);
    marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 4);

    //
    optimize_graph_service_server_ = nh_.advertiseService("optimize_graph", &GTMapping::optimizeGraphCallback, this );
    save_graph_service_server_ = nh_.advertiseService("save_graph", &GTMapping::saveGraphCallback, this );
    save_map_service_server_ = nh_.advertiseService("save_map_pcd", &GTMapping::saveMapPCDCallback, this );
    save_map_full_service_server_ = nh_.advertiseService("save_map_full_pcd", &GTMapping::saveMapFullPCDCallback, this );
    remove_bad_inlier_service_server_ = nh_.advertiseService("remove_bad_inlier", &GTMapping::removeBadInlierCallback, this );

    // reset
    reset();
}

// Only good plane correspondences or point correspondences could be added to global map
bool GTMapping::mappingMix( Frame *frame )
{
    bool success;
    if( !optimized_poses_list_.size() )    // first frame, add prior
    {
        success = addFirstFrameMix( frame );
    }
    else    // do mapping
    {
        if( use_keyframe_ )
        {
            if( isKeyFrame( frame ))
                success = doMappingMix( frame );   // do mapping
            else
                success = false;
        }
        else
        {
            success = doMappingMix( frame );   // do mapping
        }
    }


//    // Map visualization
//    if( success )
//        updateMapViewer();

    cout << GREEN << " GTMapping, success = " << (success?"true":"false") << "." << RESET << endl;

    return success;
}

// Note from isam2:  Also, as iSAM solves incrementally,
// we must wait until each is observed at least twice before
// adding it to iSAM.
bool GTMapping::doMappingMix( Frame *frame )
{
    //
    if( !landmarks_list_.size() )
    {
        ROS_ERROR("You should call mapping() instead of doMapping().");
        exit(1);
    }

    // Set id to frame
    frame->setId( next_frame_id_ );
    next_frame_id_ ++;

    std::vector<PlaneType> &planes = frame->segment_planes_;
    gtsam::Pose3 new_pose = tfToPose3( frame->pose_ );
    gtsam::Pose3 rel_pose = last_estimated_pose_.inverse() * new_pose;

    // Convert observations to OrientedPlane3
    std::vector<OrientedPlane3> observations;
    for( int i = 0; i < planes.size(); i++)
    {
        PlaneType &plane = planes[i];
        observations.push_back( OrientedPlane3(plane.coefficients) );

        // do voxel grid filter
        voxelGridFilter( plane.cloud, plane.cloud_voxel, plane_inlier_leaf_size_ );
        if( remove_plane_bad_inlier_ )
            removePlaneBadInlier( plane.cloud_voxel );
    }

    /// 1: plane features
    // Get predicted-observation
    std::map<int, gtsam::OrientedPlane3> predicted_observations = getPredictedObservation( new_pose );

    // Match observations with predicted ones
    std::vector<PlanePair> pairs; // <lm, obs>
    matchObservationWithPredicted( predicted_observations, observations, planes, new_pose, pairs);

    // Print pairs info
    cout << GREEN << " find pairs(obs, lm): " << pairs.size() << RESET << endl;
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
//    odom_sigmas << rel_pose.translation().vector()*0.1, rel_pose.rotation().rpy() * 0.1;
    odom_sigmas << 0.05, 0.05, 0.05, 0.05, 0.05, 0.05;
    noiseModel::Diagonal::shared_ptr odometry_noise =
            noiseModel::Diagonal::Sigmas( odom_sigmas );
//    cout << GREEN << "odom noise dim: " << odometry_noise->dim() << RESET << endl;
    Key pose_key = Symbol('x', frame->id() );
    Key last_key = Symbol('x', frame->id()-1);
    factor_graph_.push_back(BetweenFactor<Pose3>(last_key, pose_key, rel_pose, odometry_noise));
    factor_graph_buffer_.push_back(BetweenFactor<Pose3>(last_key, pose_key, rel_pose, odometry_noise));
    // Add pose guess
    initial_estimate_.insert<Pose3>( pose_key, new_pose );
    initial_estimate_buffer_.insert<Pose3>( pose_key, new_pose );

    /// 3: plane features
    // Add factor to existed landmark
    Eigen::VectorXi unpairs = Eigen::VectorXi::Ones( planes.size() );
    for( int i = 0; i < pairs.size(); i++)
    {
        PlanePair &pair = pairs[i];
        PlaneType &obs = planes[pair.iobs];
        unpairs[pair.iobs] = 0;
        obs.setId( pair.ilm );
        Key ln =  Symbol( 'l', obs.id() );
        noiseModel::Diagonal::shared_ptr obs_noise = noiseModel::Diagonal::Sigmas( obs.sigmas );
        factor_graph_.push_back( OrientedPlane3Factor(obs.coefficients, obs_noise, pose_key, ln) );
        factor_graph_buffer_.push_back( OrientedPlane3Factor(obs.coefficients, obs_noise, pose_key, ln) );
    }

    // Add new landmark for unpaired observation
    for( int i = 0; i < unpairs.size(); i++ )
    {
        if( unpairs[i] )
        {
            // Add factor
            PlaneType &obs = planes[i];
            obs.setId( next_plane_id_ );
            next_plane_id_++;
            Key ln = Symbol('l', obs.id());
            noiseModel::Diagonal::shared_ptr obs_noise = noiseModel::Diagonal::Sigmas( obs.sigmas );
            factor_graph_.push_back( OrientedPlane3Factor(obs.coefficients, obs_noise, pose_key, ln) );
            factor_graph_buffer_.push_back( OrientedPlane3Factor(obs.coefficients, obs_noise, pose_key, ln) );

            // Add initial guess
            OrientedPlane3 lmn( obs.coefficients );
            OrientedPlane3 glmn = lmn.transform( new_pose.inverse() );
            initial_estimate_.insert<OrientedPlane3>( ln, glmn );
            initial_estimate_buffer_.insert<OrientedPlane3>( ln, glmn );

            // Insert landmark coefficient to list
            optimized_landmarks_list_[obs.id()] = glmn;
            // Insert global plane to list
            PlaneType *lm( new PlaneType(true) );
            lm->coefficients = glmn.planeCoefficients();
            lm->color.Blue = rng_.uniform(0, 255);
            lm->color.Green = rng_.uniform(0, 255);
            lm->color.Red = rng_.uniform(0, 255);
            lm->color.Alpha = 255;
            landmarks_list_[obs.id()] = lm;
        }
    }

    /// 2: point features
    // inlier for tracking of frame with previous
    if( frame->kp_inlier_.size() < 5 )
    {

        Frame* frame_last = frames_list_[frame->id()-1];
        unsigned int min_inlier_threshold = 10;
        double inlier_error = 1e6;
        double max_dist_m = 3.0;
        Eigen::Matrix4f transformation = transformTFToMatrix4f( frame_last->pose_.inverse() * frame->pose_);
        tracker_->computeCorrespondenceInliersAndError( frame->good_matches_, transformation, frame_last->feature_locations_3d_, frame->feature_locations_3d_,
                                                        min_inlier_threshold, frame->kp_inlier_, inlier_error, max_dist_m );
        cout << GREEN << " compute matches with previous: " << frame_last->id()
             << ", good_matches_ = " << frame->good_matches_.size()
             << ", kp_inlier = " << frame->kp_inlier_.size() << RESET << endl;
        printTransform( transformation, "  Relative TF", WHITE);
        printPose3( rel_pose, "  Relative Pose:", BLUE );
    }
    //
    if( frame->id() == 1 ) // first two frames, add bwtween factors
    {
        // Define the camera calibration parameters
        Frame* frame_last = frames_list_[frame->id()-1];
        Eigen::Matrix4d toWorldLast = transformTFToMatrix4d(frame_last->pose_);
        Eigen::Matrix4d toWorld = transformTFToMatrix4d(frame->pose_);
        const int matches_size = frame->kp_inlier_.size();
        uint64_t* descriptor_value =  reinterpret_cast<uint64_t*>(frame->feature_descriptors_.data);
        // Noise
        Vector noise_sigmas(3);
        noise_sigmas << 0.01, 0.01, 0.01;
        for(int i = 0; i < matches_size; i++)
        {
            // Point correspondence
            const cv::DMatch& m = frame->kp_inlier_[i];
            const Eigen::Vector4f& last_point = frame_last->feature_locations_3d_[m.queryIdx];
            const Eigen::Vector4f& point = frame->feature_locations_3d_[m.trainIdx];
            gtsam::Point3 p1(last_point[0], last_point[1], last_point[2]);
            gtsam::Point3 p2(point[0], point[1], point[2]);
            gtsam::Point3 p1_w = transformPoint(p1, toWorldLast);
//            gtsam::Point3 p2_w = transformPoint(p2, toWorld);
            // Add new keypoint landmark
            KeyPoint *keypoint( new KeyPoint() );
            Key kp_key = Symbol('p', next_point_id_ );
            keypoint->setId( next_point_id_ );
            keypoint->translation = p1_w;
            uint64_t* des_idx = descriptor_value + (m.trainIdx)*4;
            keypoint->descriptor[0] = *des_idx++;
            keypoint->descriptor[1] = *des_idx++;
            keypoint->descriptor[2] = *des_idx++;
            keypoint->descriptor[3] = *des_idx++;
//            keypoint->descriptor = ??;
            next_point_id_++;
            keypoints_list_[keypoint->id()] = keypoint;
            optimized_keypoints_list_[keypoint->id()] = keypoint->translation;
            keypoints_vector_.push_back( last_point );
            if( i == 0) // Add a prior on landmark p0
            {
                noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(3, 0.01);
                factor_graph_.push_back(PriorFactor<Point3>(Symbol('p', 0), p1_w, pointNoise));
            }
            // Noise
            Eigen::Matrix3d cov1 = kinectPointCov( last_point );
            Eigen::Matrix3d cov2 = kinectPointCov( point );
            noiseModel::Gaussian::shared_ptr noise1 = noiseModel::Gaussian::Covariance( cov1 );
            noiseModel::Gaussian::shared_ptr noise2 = noiseModel::Gaussian::Covariance( cov2 );
            // Define the camera observation noise model
            // Keys: pose_key, last_key, kp_key
            // BearingRange factor
            factor_graph_.push_back(BearingRangeFactor<Pose3, Point3>(last_key, kp_key, (gtsam::Unit3)p1, p1.norm(), noise1));
            factor_graph_.push_back(BearingRangeFactor<Pose3, Point3>(last_key, kp_key, (gtsam::Unit3)p2, p2.norm(), noise2));
            factor_graph_buffer_.push_back(BearingRangeFactor<Pose3, Point3>(last_key, kp_key, (gtsam::Unit3)p1, p1.norm(), noise1));
            factor_graph_buffer_.push_back(BearingRangeFactor<Pose3, Point3>(last_key, kp_key, (gtsam::Unit3)p2, p2.norm(), noise2));
            // Add initial guess
            initial_estimate_.insert<Point3>( kp_key, p1_w );
            initial_estimate_buffer_.insert<Point3>( kp_key, p1_w );
        }
    }
    else
    {
        // Keypoint matches with map
        std::vector<cv::DMatch> matches;
        matchKeypointsWithGlobal( *frame, matches );
//        matchKeypointsWithGlobal( frame->kp_inlier_, frame->pose_, frame->camera_params_,
//                                  frame->feature_descriptors_, frame->feature_locations_3d_, matches );

        // add factor to existed landmark
        std::vector<bool> is_matched;

        // add factor to unpaired landmark

        // add prior factor
    }


    // Insert optimized pose to list
    optimized_poses_list_[frame->id()] = new_pose;
    // Insert keyframe to list
    frames_list_[ frame->id() ] = frame;


    // Update graph
    isam2_->update(factor_graph_, initial_estimate_);
    isam2_->update(); // call additionally

    // Update optimized poses, planes, keypoints
    updateOptimizedResultMix();

    // Update Map
    updateLandmarksInlier();

    // Update Octomap
    updateOctoMap();

    // Clear the factor graph and values for the next iteration
    factor_graph_.resize(0);
    initial_estimate_.clear();

    // Refine map
    if( refine_planar_map_ )
    {
        refinePlanarMap();
    }

    // Semantic labelling: floor, wall, door, table
    semanticMapLabel();

    // update estimated
    last_estimated_pose_ = optimized_poses_list_[ frame->id() ];
    last_estimated_pose_tf_ = pose3ToTF( last_estimated_pose_ );
    return true;

}

bool GTMapping::addFirstFrameMix( Frame *frame )
{
    // check number of planes
    if( frame->segment_planes_.size() == 0 )
        return false;

    std::vector<PlaneType> &planes = frame->segment_planes_;
    gtsam::Pose3 init_pose = tfToPose3( frame->pose_ );

    // set ids
    frame->setId( next_frame_id_ );
    next_frame_id_++;

    //
    double radius = plane_inlier_leaf_size_ * 5;
    int min_neighbors = M_PI * radius *radius
                    / (plane_inlier_leaf_size_ * plane_inlier_leaf_size_) *  planar_bad_inlier_alpha_;
    for( int i = 0; i < planes.size(); i++)
    {
        PlaneType &plane = planes[i];
        plane.setId( next_plane_id_ );
        next_plane_id_ ++;

        // do voxel grid filter
        if( remove_plane_bad_inlier_ )
        {
            PointCloudTypePtr cloud_filtered( new PointCloudType );
            voxelGridFilter( plane.cloud, cloud_filtered, plane_inlier_leaf_size_ );
            radiusOutlierRemoval( cloud_filtered, plane.cloud_voxel, radius, min_neighbors );
        }
        else
            voxelGridFilter( plane.cloud, plane.cloud_voxel, plane_inlier_leaf_size_ );
    }


    // Add a prior factor
    Key x0 = Symbol('x', frame->id());
    Vector pose_sigmas(6);
//    pose_sigmas << init_pose.translation().vector()*0.2, init_pose.rotation().rpy() * 0.2;
    pose_sigmas << 0.001, 0.001, 0.001, 0.0001, 0.001, 0.001;
    noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas( pose_sigmas ); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
//    noiseModel::Diagonal::shared_ptr poseNoise = //
//        noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
    factor_graph_.push_back( PriorFactor<Pose3>( x0, init_pose, poseNoise ) );
    factor_graph_buffer_.push_back( PriorFactor<Pose3>( x0, init_pose, poseNoise ) );

    // Add an initial guess for the current pose
    initial_estimate_.insert<Pose3>( x0, init_pose );
    initial_estimate_buffer_.insert<Pose3>( x0, init_pose );

    /// 1: Plane features
    // Add a prior landmark
    Key l0 = Symbol('l', planes[0].id() );
    OrientedPlane3 lm0(planes[0].coefficients);
    OrientedPlane3 glm0 = lm0.transform(init_pose.inverse());
    noiseModel::Diagonal::shared_ptr lm_noise = noiseModel::Diagonal::Sigmas( (Vector(2) << planes[0].sigmas[0], planes[0].sigmas[1]).finished() );
    factor_graph_.push_back( OrientedPlane3DirectionPrior( l0, glm0.planeCoefficients(), lm_noise) );
    factor_graph_buffer_.push_back( OrientedPlane3DirectionPrior( l0, glm0.planeCoefficients(), lm_noise) );

    // Add new landmark
    for(int i = 0; i < planes.size(); i++)
    {
        PlaneType &plane = planes[i];
        Key ln = Symbol('l', plane.id() );

        cout << YELLOW << " lm" << i << " coefficents: " << plane.coefficients[0]
             << ", " << plane.coefficients[1] << ", " << plane.coefficients[2]
             << ", " << plane.coefficients[3] << ", centroid: " << plane.centroid.x
             << ", " << plane.centroid.y << ", " << plane.centroid.z << RESET << endl;

        // Add observation factor
        noiseModel::Diagonal::shared_ptr obs_noise = noiseModel::Diagonal::Sigmas( plane.sigmas );
        factor_graph_.push_back( OrientedPlane3Factor(plane.coefficients, obs_noise, x0, ln) );
        factor_graph_buffer_.push_back( OrientedPlane3Factor(plane.coefficients, obs_noise, x0, ln) );

        // Add initial guesses to all observed landmarks
//        cout << "Key: " << ln << endl;
        OrientedPlane3 lmn( plane.coefficients );
        OrientedPlane3 glmn = lmn.transform( init_pose.inverse() );
        initial_estimate_.insert<OrientedPlane3>( ln,  glmn );
        initial_estimate_buffer_.insert<OrientedPlane3>( ln,  glmn );

        // Insert landmarks coefficients to list
        optimized_landmarks_list_[plane.id()] = glmn;
        // Insert empty global plane to list
        PlaneType *lm( new PlaneType(true) );
        lm->coefficients = glmn.planeCoefficients();
        lm->color.Blue = rng_.uniform(0, 255);
        lm->color.Green = rng_.uniform(0, 255);
        lm->color.Red = rng_.uniform(0, 255);
        lm->color.Alpha = 255;
        landmarks_list_[plane.id()] = lm;
    }

    /// 2: Point features
    // For first frame, no point feature is added.

    // Insert first pose to list
    optimized_poses_list_[ frame->id() ] = init_pose;
    // Insert first frame to list
    frames_list_[ frame->id() ] = frame;  // add frame to list

    // Optimize factor graph
//    isam2_->update( graph_, initial_estimate_ );
//    updateOptimizedResult();

//    // Clear the factor graph and values for the next iteration
//    graph_.resize(0);
//    initial_estimate_.clear();
//    //
    updateLandmarksInlier();

    last_estimated_pose_ = init_pose;
    last_estimated_pose_tf_ = pose3ToTF( last_estimated_pose_ );

    cout << GREEN << " Register first frame in the map." << endl;
    printTransform( init_pose.matrix(), " - Initial pose", GREEN);
    cout << RESET << endl;

    return true;
}

bool GTMapping::mapping( Frame *frame )
{
    bool success;
    if( !optimized_poses_list_.size() )    // first frame, add prior
    {
        success = addFirstFrame( frame );
    }
    else    // do mapping
    {
        if( use_keyframe_ )
        {
            if( isKeyFrame( frame ))
                success = doMapping( frame );   // do mapping
            else
                success = false;
        }
        else
        {
            success = doMapping( frame );   // do mapping
        }
    }


//    // Map visualization
//    if( success )
//        updateMapViewer();

    cout << GREEN << " GTMapping, success = " << (success?"true":"false") << "." << RESET << endl;

    return success;
}
// Note from isam2:  Also, as iSAM solves incrementally,
// we must wait until each is observed at least twice before
// adding it to iSAM.
bool GTMapping::doMapping( Frame *frame )
{
    //
    if( !landmarks_list_.size() )
    {
        ROS_ERROR("You should call mapping() instead of doMapping().");
        exit(1);
    }

    // Set id to frame
    frame->setId( next_frame_id_ );
    next_frame_id_ ++;

    std::vector<PlaneType> &planes = frame->segment_planes_;
    gtsam::Pose3 new_pose = tfToPose3( frame->pose_ );
    gtsam::Pose3 rel_pose = last_estimated_pose_.inverse() * new_pose;

    // Convert observations to OrientedPlane3
    std::vector<OrientedPlane3> observations;
    for( int i = 0; i < planes.size(); i++)
    {
        PlaneType &plane = planes[i];
        observations.push_back( OrientedPlane3(plane.coefficients) );

        // do voxel grid filter
        voxelGridFilter( plane.cloud, plane.cloud_voxel, plane_inlier_leaf_size_ );
        if( remove_plane_bad_inlier_ )
            removePlaneBadInlier( plane.cloud_voxel );
    }

    // Get predicted-observation
    std::map<int, gtsam::OrientedPlane3> predicted_observations = getPredictedObservation( new_pose );

    // Match observations with predicted ones
    std::vector<PlanePair> pairs; // <lm, obs>
    matchObservationWithPredicted( predicted_observations, observations, planes, new_pose, pairs);

    // Print pairs info
    cout << GREEN << " find pairs(obs, lm): " << pairs.size() << RESET << endl;
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
//    odom_sigmas << rel_pose.translation().vector()*0.1, rel_pose.rotation().rpy() * 0.1;
    odom_sigmas << 0.05, 0.05, 0.05, 0.05, 0.05, 0.05;
    noiseModel::Diagonal::shared_ptr odometry_noise =
            noiseModel::Diagonal::Sigmas( odom_sigmas );
//    cout << GREEN << "odom noise dim: " << odometry_noise->dim() << RESET << endl;
    Key pose_key = Symbol('x', frame->id() );
    Key last_key = Symbol('x', frame->id()-1);
    factor_graph_.push_back(BetweenFactor<Pose3>(last_key, pose_key, rel_pose, odometry_noise));
    factor_graph_buffer_.push_back(BetweenFactor<Pose3>(last_key, pose_key, rel_pose, odometry_noise));
    // Add pose guess
    initial_estimate_.insert<Pose3>( pose_key, new_pose );
    initial_estimate_buffer_.insert<Pose3>( pose_key, new_pose );

    // Add factor to exist landmark
    Eigen::VectorXi unpairs = Eigen::VectorXi::Ones( planes.size() );
    for( int i = 0; i < pairs.size(); i++)
    {
        PlanePair &pair = pairs[i];
        PlaneType &obs = planes[pair.iobs];
        unpairs[pair.iobs] = 0;
        obs.setId( pair.ilm );
        Key ln =  Symbol( 'l', obs.id() );
        noiseModel::Diagonal::shared_ptr obs_noise = noiseModel::Diagonal::Sigmas( obs.sigmas );
        factor_graph_.push_back( OrientedPlane3Factor(obs.coefficients, obs_noise, pose_key, ln) );
        factor_graph_buffer_.push_back( OrientedPlane3Factor(obs.coefficients, obs_noise, pose_key, ln) );
    }

    // Add new landmark for unpaired observation
    for( int i = 0; i < unpairs.size(); i++ )
    {
        if( unpairs[i] )
        {
            // Add factor
            PlaneType &obs = planes[i];
            obs.setId( next_plane_id_ );
            next_plane_id_++;
            Key ln = Symbol('l', obs.id());
            noiseModel::Diagonal::shared_ptr obs_noise = noiseModel::Diagonal::Sigmas( obs.sigmas );
            factor_graph_.push_back( OrientedPlane3Factor(obs.coefficients, obs_noise, pose_key, ln) );
            factor_graph_buffer_.push_back( OrientedPlane3Factor(obs.coefficients, obs_noise, pose_key, ln) );

            // Add initial guess
            OrientedPlane3 lmn( obs.coefficients );
            OrientedPlane3 glmn = lmn.transform( new_pose.inverse() );
            initial_estimate_.insert<OrientedPlane3>( ln, glmn );
            initial_estimate_buffer_.insert<OrientedPlane3>( ln, glmn );

            // Insert landmark coefficient to list
            optimized_landmarks_list_[obs.id()] = glmn;
            // Insert global plane to list
            PlaneType *lm( new PlaneType(true) );
            lm->coefficients = glmn.planeCoefficients();
            lm->color.Blue = rng_.uniform(0, 255);
            lm->color.Green = rng_.uniform(0, 255);
            lm->color.Red = rng_.uniform(0, 255);
            lm->color.Alpha = 255;
            landmarks_list_[obs.id()] = lm;
        }
    }

    // Insert optimized pose to list
    optimized_poses_list_[frame->id()] = new_pose;
    // Insert keyframe to list
    frames_list_[ frame->id() ] = frame;


    // Update graph
    isam2_->update(factor_graph_, initial_estimate_);
    isam2_->update(); // call additionally

    // Update optimized poses and planes
    updateOptimizedResult();

    // Update Map
    updateLandmarksInlier();

    // Update Octomap
    updateOctoMap();

    // Clear the factor graph and values for the next iteration
    factor_graph_.resize(0);
    initial_estimate_.clear();

    // Refine map
    if( refine_planar_map_ )
    {
        refinePlanarMap();
    }

    // Semantic labelling: floor, wall, door, table
    semanticMapLabel();

    // update estimated
    last_estimated_pose_ = optimized_poses_list_[ frame->id() ];
    last_estimated_pose_tf_ = pose3ToTF( last_estimated_pose_ );
    return true;

}


bool GTMapping::addFirstFrame( Frame *frame )
{
    // check number of planes
    if( frame->segment_planes_.size() == 0 )
        return false;

    std::vector<PlaneType> &planes = frame->segment_planes_;
    gtsam::Pose3 init_pose = tfToPose3( frame->pose_ );

    // set ids
    frame->setId( next_frame_id_ );
    next_frame_id_++;

    //
    double radius = plane_inlier_leaf_size_ * 5;
    int min_neighbors = M_PI * radius *radius
                    / (plane_inlier_leaf_size_ * plane_inlier_leaf_size_) *  planar_bad_inlier_alpha_;
    for( int i = 0; i < planes.size(); i++)
    {
        PlaneType &plane = planes[i];
        plane.setId( next_plane_id_ );
        next_plane_id_ ++;

        // do voxel grid filter
        if( remove_plane_bad_inlier_ )
        {
            PointCloudTypePtr cloud_filtered( new PointCloudType );
            voxelGridFilter( plane.cloud, cloud_filtered, plane_inlier_leaf_size_ );
            radiusOutlierRemoval( cloud_filtered, plane.cloud_voxel, radius, min_neighbors );
        }
        else
            voxelGridFilter( plane.cloud, plane.cloud_voxel, plane_inlier_leaf_size_ );
    }


    // Add a prior factor
    Key x0 = Symbol('x', frame->id());
    Vector pose_sigmas(6);
//    pose_sigmas << init_pose.translation().vector()*0.2, init_pose.rotation().rpy() * 0.2;
    pose_sigmas << 0.001, 0.001, 0.001, 0.0001, 0.001, 0.001;
    noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas( pose_sigmas ); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
//    noiseModel::Diagonal::shared_ptr poseNoise = //
//        noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
    factor_graph_.push_back( PriorFactor<Pose3>( x0, init_pose, poseNoise ) );
    factor_graph_buffer_.push_back( PriorFactor<Pose3>( x0, init_pose, poseNoise ) );

    // Add an initial guess for the current pose
    initial_estimate_.insert<Pose3>( x0, init_pose );
    initial_estimate_buffer_.insert<Pose3>( x0, init_pose );

    // Add a prior landmark
    Key l0 = Symbol('l', planes[0].id() );
    OrientedPlane3 lm0(planes[0].coefficients);
    OrientedPlane3 glm0 = lm0.transform(init_pose.inverse());
    noiseModel::Diagonal::shared_ptr lm_noise = noiseModel::Diagonal::Sigmas( (Vector(2) << planes[0].sigmas[0], planes[0].sigmas[1]).finished() );
    factor_graph_.push_back( OrientedPlane3DirectionPrior( l0, glm0.planeCoefficients(), lm_noise) );
    factor_graph_buffer_.push_back( OrientedPlane3DirectionPrior( l0, glm0.planeCoefficients(), lm_noise) );

    // Add new landmark
    for(int i = 0; i < planes.size(); i++)
    {
        PlaneType &plane = planes[i];
        Key ln = Symbol('l', plane.id() );

        cout << YELLOW << " lm" << i << " coefficents: " << plane.coefficients[0]
             << ", " << plane.coefficients[1] << ", " << plane.coefficients[2]
             << ", " << plane.coefficients[3] << ", centroid: " << plane.centroid.x
             << ", " << plane.centroid.y << ", " << plane.centroid.z << RESET << endl;

        // Add observation factor
        noiseModel::Diagonal::shared_ptr obs_noise = noiseModel::Diagonal::Sigmas( plane.sigmas );
        factor_graph_.push_back( OrientedPlane3Factor(plane.coefficients, obs_noise, x0, ln) );
        factor_graph_buffer_.push_back( OrientedPlane3Factor(plane.coefficients, obs_noise, x0, ln) );

        // Add initial guesses to all observed landmarks
//        cout << "Key: " << ln << endl;
        OrientedPlane3 lmn( plane.coefficients );
        OrientedPlane3 glmn = lmn.transform( init_pose.inverse() );
        initial_estimate_.insert<OrientedPlane3>( ln,  glmn );
        initial_estimate_buffer_.insert<OrientedPlane3>( ln,  glmn );

        // Insert landmarks coefficients to list
        optimized_landmarks_list_[plane.id()] = glmn;
        // Insert empty global plane to list
        PlaneType *lm( new PlaneType(true) );
        lm->coefficients = glmn.planeCoefficients();
        lm->color.Blue = rng_.uniform(0, 255);
        lm->color.Green = rng_.uniform(0, 255);
        lm->color.Red = rng_.uniform(0, 255);
        lm->color.Alpha = 255;
        landmarks_list_[plane.id()] = lm;
    }


    // Insert first pose to list
    optimized_poses_list_[ frame->id() ] = init_pose;
    // Insert first frame to list
    frames_list_[ frame->id() ] = frame;  // add frame to list

    // Optimize factor graph
//    isam2_->update( graph_, initial_estimate_ );
//    updateOptimizedResult();

//    // Clear the factor graph and values for the next iteration
//    graph_.resize(0);
//    initial_estimate_.clear();
//    //
    updateLandmarksInlier();

    last_estimated_pose_ = init_pose;
    last_estimated_pose_tf_ = pose3ToTF( last_estimated_pose_ );

    cout << GREEN << " Register first frame in the map." << endl;
    printTransform( init_pose.matrix(), " - Initial pose", GREEN );
    cout << RESET << endl;

    return true;
}

bool GTMapping::isKeyFrame( Frame *frame )
{
    tf::Transform rel_tf = last_estimated_pose_tf_.inverse() * frame->pose_;
    double rad, distance;
    calAngleAndDistance( rel_tf, rad, distance );

    if( distance > keyframe_linear_threshold_ || rad > keyframe_angular_threshold_ )
        return true;
    else
        return false;
}

void GTMapping::semanticMapLabel()
{
    // Assign semantic label to every landmark
    for( std::map<int, PlaneType*>::iterator it = landmarks_list_.begin();
         it != landmarks_list_.end(); it++)
    {
        PlaneType *lm = it->second;
        if( lm->semantic_label.empty() ) // No label, labelling it
            labelPlane( lm );
    }
}

void GTMapping::labelPlane( PlaneType *plane )
{
    static int min_wall_indices = (1.6 / plane_inlier_leaf_size_) * (1.6 / plane_inlier_leaf_size_);

    Eigen::Vector3d n( plane->coefficients[0], plane->coefficients[1], plane->coefficients[2] );
    double theta = acos( plane->coefficients[2] );
    double d = plane->coefficients[3];
    int size = plane->cloud_voxel->size();

    // Print info
    cout << GREEN << "  Label: " << plane->landmark_id
         << " - angle(z): " << (theta*RAD_TO_DEG)
         << ", d: " << d
         << ", indices:" << size << RESET << endl;

    // Labelling "FLOOR"
    if( theta < (5.0*DEG_TO_RAD) && (d < 0.1) ) // theta < 5deg, d < 0.1m
    {
        plane->semantic_label = "FLOOR";
        return;
    }
    // Labelling "WALL"
    if( theta > (82.0*DEG_TO_RAD) && theta < (98.0*DEG_TO_RAD) && size > min_wall_indices )
    {
        plane->semantic_label = "WALL";
        return;
    }
    // Labelling "TABLE"
    if( theta < (5.0*DEG_TO_RAD) && (d > 0.4) && (d < 1.0) ) // theta < 5deg, 0.4m < d < 1m
    {
        plane->semantic_label = "TABLE";
        return;
    }
    // Labelling "DOOR"
    if(0)
    {
        plane->semantic_label = "DOOR";
        return;
    }
}

void GTMapping::matchKeypointsWithGlobal( const Frame &frame,
                                          std::vector<cv::DMatch> &matches)
{
    // Get prediction
    std_vector_of_eigen_vector4f predicted_feature_3d = getPredictedKeypoints( frame.pose_, keypoints_vector_ );
    std::map<int, gtsam::Point3> predicted_keypoints = getPredictedKeypoints( tfToPose3(frame.pose_), frame.camera_params_ );
    cout << GREEN << "  predicted keypoints: " << predicted_keypoints.size() << RESET << endl;

    // Publish info
    publishPredictedKeypoints( predicted_keypoints );
    publishFrameKeypoints( predicted_feature_3d );

    // Match descriptors, use all keypoints from current frame
    std::vector<cv::DMatch> good_matches;
    tracker_->matchImageFeatures( frame, keypoints_list_, predicted_keypoints, good_matches );
    cout << GREEN << "  good matches: " << good_matches.size() << RESET << endl;

    // Estimate rigid transform
    RESULT_OF_MOTION motion;
    motion.valid = tracker_->solveRelativeTransformPointsRansac( frame.feature_locations_3d_,
                                                                 predicted_feature_3d,
                                                                 good_matches, motion, matches );
    cout << GREEN << "  global matches: " << matches.size() << RESET << endl;
    printTransform( motion.transform4d(), "  Estimate motion", BLUE );

    // Publish info
    publishMatchedKeypoints( frame.feature_locations_3d_, predicted_keypoints, matches );

}

void GTMapping::matchKeypointsWithGlobal( const std::vector<cv::DMatch> &kp_inlier,
                                          const tf::Transform &pose,
                                          const CameraParameters &camera_param,
                                          const cv::Mat &features_descriptor,
                                          const std_vector_of_eigen_vector4f &feature_3d,
                                          std::vector<cv::DMatch> &matches)
{
    // Get prediction
    std_vector_of_eigen_vector4f predicted_feature_3d = getPredictedKeypoints( pose, keypoints_vector_ );
    std::map<int, gtsam::Point3> predicted_keypoints = getPredictedKeypoints( tfToPose3(pose), camera_param );
    cout << GREEN << "  predicted keypoints: " << predicted_keypoints.size() << RESET << endl;

    // Publish info
    publishPredictedKeypoints( predicted_keypoints );
    publishFrameKeypoints( predicted_feature_3d );

    // Match descriptors
    std::vector<cv::DMatch> good_matches;
    tracker_->matchImageFeatures( features_descriptor, kp_inlier, keypoints_list_, predicted_keypoints, good_matches );
    cout << GREEN << "  good matches: " << good_matches.size() << RESET << endl;


    // Estimate rigid transform
    RESULT_OF_MOTION motion;
    motion.valid = tracker_->solveRelativeTransformPointsRansac( feature_3d,
                                                                 predicted_feature_3d,
                                                                 good_matches, motion, matches );

    cout << GREEN << "  global matches: " << matches.size() << RESET << endl;
    printTransform( motion.transform4d(), "  Estimate motion", BLUE );

    // Publish info
    publishMatchedKeypoints( feature_3d, predicted_keypoints, matches );
}

// Get predicted keypoints
std_vector_of_eigen_vector4f GTMapping::getPredictedKeypoints( const tf::Transform &pose,
                                                               const std_vector_of_eigen_vector4f &feature_3d)
{
    std_vector_of_eigen_vector4f predicted_keypoints( feature_3d.size() );
    Eigen::Matrix4f trans = transformTFToMatrix4f( pose.inverse() );
    std_vector_of_eigen_vector4f::const_iterator it = feature_3d.begin();
    std_vector_of_eigen_vector4f::iterator prd_it = predicted_keypoints.begin();
    for( ; it != feature_3d.end(); it++, prd_it++)
    {
        *prd_it = transformPoint( *it, trans );
    }
    return predicted_keypoints;
}

// Get predicted keypoints in FOV
std::map<int, gtsam::Point3> GTMapping::getPredictedKeypoints( const gtsam::Pose3 &pose,
                                                               const CameraParameters &camera_param)
{
    // Define the camera calibration parameters
    Cal3_S2::shared_ptr K(new Cal3_S2(camera_param.fx, camera_param.fy, 0.0, camera_param.cx, camera_param.cy));
    gtsam::SimpleCamera camera( pose, *K );
    std::map<int, gtsam::Point3> predicted_keypoints;
    for( std::map<int, gtsam::Point3>::iterator it = optimized_keypoints_list_.begin();
         it != optimized_keypoints_list_.end(); it++)
    {
        gtsam::Point2 pc = camera.project( it->second );
        if( pc.x() > 0 && pc.x() < 639 && pc.y() > 0 && pc.y() < 479)
        {
            predicted_keypoints[ it->first ] = pose.transform_to( it->second );
        }
    }

    return predicted_keypoints;
}


// get predicted landmarks
std::map<int, gtsam::OrientedPlane3> GTMapping::getPredictedObservation( const Pose3 &pose )
{
    std::map<int, gtsam::OrientedPlane3> predicted_observations;
    for( std::map<int, gtsam::OrientedPlane3>::iterator it = optimized_landmarks_list_.begin();
         it != optimized_landmarks_list_.end(); it++)
    {
        OrientedPlane3 &plane = it->second;
        predicted_observations[ it->first ] = plane.transform( pose );
    }
    return predicted_observations;
}

// simple euclidian distance
void GTMapping::matchObservationWithPredicted( std::map<int, gtsam::OrientedPlane3> &predicted_observations,
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

        for( std::map<int, gtsam::OrientedPlane3>::iterator it = predicted_observations.begin();
             it != predicted_observations.end(); it++)
        {
            const int id = it->first;
            OrientedPlane3 &plm = it->second;
            PlaneType *glm = landmarks_list_[id];

//            // check if alive
//            if( !glm.valid )
//                continue;

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
                if( glm->cloud_voxel->size() > max_size )
                {
                    if( plane_match_check_overlap_ && !checkOverlap( glm->cloud_voxel, glm->coefficients, observed.cloud_voxel, pose ) )
                        continue;
    //                min_d = d;
                    min_index = id;
                    max_size = glm->cloud_voxel->size();

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

bool GTMapping::checkOverlap( const PointCloudTypePtr &landmark_cloud,
                            const OrientedPlane3 &landmark,
                            const PointCloudTypePtr &observation,
                            const Pose3 &pose)
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
bool GTMapping::checkLandmarksOverlap( const PlaneType &lm1, const PlaneType &lm2)
{
    // project lm2 inlier to lm1 plane
    PointCloudTypePtr cloud_projected( new PointCloudType );
    projectPoints( *lm2.cloud_voxel, lm1.coefficients, *cloud_projected );

    // build octree from lm1
    float resolution = plane_inlier_leaf_size_;
    pcl::octree::OctreePointCloud<PointType> octreeD (resolution);
    octreeD.setInputCloud( lm1.cloud_voxel );
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
            if(collision > 20)
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


// return true if being refined.
bool GTMapping::refinePlanarMap()
{
//    cout << RED << ", lm size: " << landmarks_.size()
//         << ", pl size: " << estimated_planes_.size() << endl;

    // find co-planar landmark pair
    bool find_coplanar = false;
    const double direction_threshold = planar_merge_direction_threshold_;
    const double distance_threshold = planar_merge_distance_threshold_;
    std::map<int, int> remove_list;  // removed landmarks, <from, to>
    for( std::map<int, PlaneType*>::iterator it1 = landmarks_list_.begin();
         it1 != landmarks_list_.end(); it1++)
    {
        const int idx1 = it1->first;
        PlaneType &p1 = *(it1->second);
        OrientedPlane3 &lm1 = optimized_landmarks_list_[idx1];

        // check if will be removed
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

        for( std::map<int, PlaneType*>::iterator it2 = landmarks_list_.begin();
             it2 != landmarks_list_.end(); it2++)
        {
            const int idx2 = it2->first;
            PlaneType &p2 = *(it2->second);
            OrientedPlane3 &lm2 = optimized_landmarks_list_[idx2];

            // check if will be removed
            if( !p2.valid )
                continue;

            if( idx1 == idx2)
                continue;

            /// Match two plane
            // Transform landmark to local frame
            const OrientedPlane3 &llm2 = lm2.transform( local );
            double dr_error = acos( llm1.normal().dot( llm2.normal() ));
            double ds_error = fabs( llm1.distance() - llm2.distance() );
//            cout << CYAN << "  - " << idx1 << "*" << idx2 << ": " << dr_error << "("<< direction_threshold << "), "
//                 << ds_error << "(" << distance_threshold << ")" << RESET << endl;
            if( (fabs(dr_error) < direction_threshold)
                    && (ds_error < distance_threshold) )
            {
                // check if overlap
                bool overlap = false;
                if( p1.cloud_voxel->size() < p2.cloud_voxel->size() )
                    overlap = checkLandmarksOverlap( p2, p1);
                else
                    overlap = checkLandmarksOverlap( p1, p2);

                if( overlap )
                {
                    // merge, make smaller one invalid
                    if( p1.cloud_voxel->size() < p2.cloud_voxel->size() )
                    {
                        p1.valid = false;
                        remove_list[idx1] = idx2;
                        cout << RED << "  -- find co-planar: " << idx1 << " to " << idx2 << RESET << endl;
                        break;
                    }
                    else
                    {
                        p2.valid = false;
                        remove_list[idx2] = idx1;
                        cout << RED << "  -- find co-planar: " << idx2 << " to " << idx1 << RESET << endl;
                    }
                    find_coplanar = true;
                } // end of if overlap

            }

        } // end of for( int j = i+1; j < num; j++)

    } // end of for( int i = 0; i < (num - 1 ); i++)

    cout << YELLOW << " Find co-planar pairs: " << remove_list.size() << RESET << endl;

    std::map<int, std::set<int> > merge_list;   // merge list <to, set<from>>
    for( std::map<int,int>::iterator it = remove_list.begin(); it != remove_list.end(); it++)
    {
        int from = it->first;
        int to = it->second;

        // if to in tos
        bool to_in_tos = false;
        std::map<int, std::set<int> >::iterator ttitem = merge_list.find( to );
        if( ttitem != merge_list.end() )
        {
            ttitem->second.insert( from );
            to_in_tos = true;
        }

        // if from in tos
        bool from_in_tos = false;
        std::map<int, std::set<int> >::iterator ftitem = merge_list.find( from );
        if( ftitem != merge_list.end() )
        {
            std::set<int> fset = ftitem->second;
            fset.insert( from );
            merge_list.erase( ftitem );
            merge_list[to] = fset;
            from_in_tos = true;
        }

        // if to in froms
        bool to_is_removed = false;
        for( std::map<int, std::set<int> >::iterator rit = merge_list.begin();
             rit != merge_list.end(); rit++)
        {
            std::set<int>::iterator rset_item = rit->second.find( to );
            if( rset_item != rit->second.end() ) // already be removed
            {
                rit->second.insert( from );
                to_is_removed = true;
                break;
            }
        }

        if( to_in_tos || from_in_tos || to_is_removed )
            continue;

        // default, put into merge list
        // construct empty set, add
        std::set<int> from_set;
        from_set.insert( from );
        merge_list[to] = from_set;
    }

    // Merge co-planar
    if( merge_list.size() > 0)
        mergeCoplanarLandmarks( merge_list );

    return find_coplanar;
}

void GTMapping::mergeCoplanarLandmarks( std::map<int, std::set<int> > merge_list )
{
    // print merge list
    cout << YELLOW << " Merge list: " << endl;
    for( std::map<int, std::set<int> >::iterator it = merge_list.begin(); it != merge_list.end(); it++)
    {
        std::set<int> from_set = it->second;
        cout << "  - from:";
        for( std::set<int>::iterator its = from_set.begin(); its!= from_set.end(); its++)
            cout << " " << (*its);
        cout << " to: " << it->first << endl;
    }
    cout << RESET << endl;


    // Graph variable
    Values isam2_values = isam2_->calculateBestEstimate();

    std::map<Key, Key> rekey_mapping; // old -> new
    for( std::map<int, std::set<int> >::iterator itt = merge_list.begin(); itt != merge_list.end(); itt++)
    {
        const int to = itt->first;
        const std::set<int> from_set = itt->second;
        for( std::set<int>::const_iterator itf = from_set.begin(); itf!= from_set.end(); itf++)
        {
            const int from = *itf;

            // Do Merging
            // Delete id == 'from' in the 'landmarks_list_'
            std::map<int, PlaneType*>::iterator litem = landmarks_list_.find( from );
            if( litem != landmarks_list_.end() )
                landmarks_list_.erase( litem );

            // Delete id == 'from' in the 'optimized_landmarks_list_'
            std::map<int, gtsam::OrientedPlane3>::iterator olitem = optimized_landmarks_list_.find( from );
            if( olitem != optimized_landmarks_list_.end() )
                optimized_landmarks_list_.erase( olitem );

            // Change PlaneType id in the frame,  from 'from' to 'to'
            for( std::map<int, Frame*>::iterator frame_iter = frames_list_.begin();
                 frame_iter != frames_list_.end(); frame_iter++)
            {
                Frame *frame = frame_iter->second;
                for( int idx = 0; idx < frame->segment_planes_.size(); idx++)
                {
                    PlaneType &obs = frame->segment_planes_[idx];
                    if( obs.id() == from )
                        obs.setId( to );
                }
            }

            Key key_from = Symbol( 'l', from );
            Key key_to = Symbol( 'l', to );
            // delete OrientedPlane3 id in the graph
            if( isam2_values.exists( key_from ) )
            {
                isam2_values.erase( key_from );
            }

            // Change factor id in the graph, just build rekey_mapping
            rekey_mapping[key_from] = key_to;

        }

    }

    // Change factor id in the graph, provide rekey_mapping
    for( NonlinearFactorGraph::iterator factor_iter = factor_graph_buffer_.begin();
         factor_iter != factor_graph_buffer_.end(); factor_iter++)
    {
        // OrientedPlane3Factor factor
        boost::shared_ptr<OrientedPlane3Factor> plane_factor =
                boost::dynamic_pointer_cast<OrientedPlane3Factor>(*factor_iter);
        if( plane_factor )
        {
            FastVector<Key> &keys = plane_factor->keys();
            std::map<Key, Key>::const_iterator mapping = rekey_mapping.find( keys[1] );
            if (mapping != rekey_mapping.end())
                keys[1] = mapping->second;
        }

        // OrientedPlane3DirectionPrior factor
        boost::shared_ptr<OrientedPlane3DirectionPrior> plane_prior_factor =
                boost::dynamic_pointer_cast<OrientedPlane3DirectionPrior>(*factor_iter);
        if( plane_prior_factor )
        {
            FastVector<Key> &keys = plane_prior_factor->keys();
            std::map<Key, Key>::const_iterator mapping = rekey_mapping.find( keys[0] );
            if (mapping != rekey_mapping.end())
                keys[0] = mapping->second;
        }

    }

    cout << YELLOW << " Update graph ..." << RESET << endl;
    delete isam2_;
    isam2_ = new ISAM2( isam2_parameters_ );
    isam2_->update( factor_graph_buffer_, isam2_values );
    isam2_->update();
    updateOptimizedResult();
    updateLandmarksInlier();
}

bool GTMapping::removeLandmarksBadInlier()
{
    float radius = plane_inlier_leaf_size_ * 5;
    int min_neighbors = M_PI * radius *radius
            / (plane_inlier_leaf_size_ * plane_inlier_leaf_size_) *  planar_bad_inlier_alpha_;
    cout << GREEN << " Remove bad inlier, radius = " << radius
         << ", minimum neighbours = " << min_neighbors << RESET << endl;

    for( std::map<int, PlaneType*>::const_iterator it = landmarks_list_.begin();
         it != landmarks_list_.end(); it++)
    {
        PlaneType &lm = *(it->second);
        removePlaneBadInlier( lm.cloud_voxel, radius, min_neighbors );
    }
}

void GTMapping::removePlaneBadInlier( PointCloudTypePtr &cloud_voxel, double radius, int min_neighbors )
{
    if( radius == 0)
        radius = plane_inlier_leaf_size_ * 5;
    if( min_neighbors == 0)
    {
        min_neighbors = M_PI * radius *radius
                    / (plane_inlier_leaf_size_ * plane_inlier_leaf_size_) *  planar_bad_inlier_alpha_;
    }
    // filter
    PointCloudTypePtr cloud_filtered( new PointCloudType );
    radiusOutlierRemoval( cloud_voxel, cloud_filtered, radius, min_neighbors );
    // swap
    cloud_voxel->swap( *cloud_filtered );
}

void GTMapping::updateOptimizedResultMix()
{
    //
    Values values = isam2_->calculateBestEstimate();
    for( std::map<int, gtsam::Pose3>::iterator it = optimized_poses_list_.begin();
            it != optimized_poses_list_.end(); it++)
    {
        gtsam::Pose3 pose3 = values.at( Symbol('x', it->first) ).cast<gtsam::Pose3>();
        it->second = pose3;
        frames_list_[it->first]->pose_ = pose3ToTF( pose3 );
    }
    for( std::map<int, gtsam::OrientedPlane3>::iterator it = optimized_landmarks_list_.begin();
            it != optimized_landmarks_list_.end(); it++)
    {
        gtsam::OrientedPlane3 plane = values.at( Symbol('l', it->first) ).cast<gtsam::OrientedPlane3>();
        it->second = plane;
        landmarks_list_[it->first]->coefficients = plane.planeCoefficients();
    }
    for( std::map<int, gtsam::Point3>::iterator it = optimized_keypoints_list_.begin();
            it != optimized_keypoints_list_.end(); it++)
    {
        gtsam::Point3 point = values.at( Symbol('p', it->first) ).cast<gtsam::Point3>();
        it->second = point;
        keypoints_list_.at(it->first)->translation = point;
        keypoints_vector_[it->first] = Eigen::Vector4f( point.x(), point.y(), point.z(), 1.0 );
    }
}

void GTMapping::updateOptimizedResult()
{
    //
    Values values = isam2_->calculateBestEstimate();
    for( std::map<int, gtsam::Pose3>::iterator it = optimized_poses_list_.begin();
            it != optimized_poses_list_.end(); it++)
    {
        Pose3 pose3 = values.at( Symbol('x', it->first) ).cast<Pose3>();
        it->second = pose3;
        frames_list_[it->first]->pose_ = pose3ToTF( pose3 );
    }
    for( std::map<int, gtsam::OrientedPlane3>::iterator it = optimized_landmarks_list_.begin();
            it != optimized_landmarks_list_.end(); it++)
    {
        OrientedPlane3 plane = values.at( Symbol('l', it->first) ).cast<OrientedPlane3>();
        it->second = plane;
        landmarks_list_[it->first]->coefficients = plane.planeCoefficients();
    }
}

void GTMapping::updateLandmarksInlier()
{
    // Update landmark poses, plane coefficients
    for( std::map<int, PlaneType*>::iterator it = landmarks_list_.begin();
         it != landmarks_list_.end(); it++)
    {
        PlaneType *lm = it->second;
        lm->cloud_voxel->clear();  // clear inlier
    }
    // Sum
    for( std::map<int, Frame*>::iterator it = frames_list_.begin();
         it != frames_list_.end(); it++)
    {
        Frame *frame = it->second;
        Eigen::Matrix4d trans = transformTFToMatrix4d( frame->pose_ );
        for( int idx = 0; idx < frame->segment_planes_.size(); idx++)
        {
            PlaneType &obs = frame->segment_planes_[idx];
            PlaneType *lm = landmarks_list_[ obs.id() ];
            *(lm->cloud_voxel) += transformPointCloud( *(obs.cloud_voxel), trans );
        }
    }

    // Project and Downsample
    for( std::map<int, PlaneType*>::iterator it = landmarks_list_.begin();
         it != landmarks_list_.end(); it++)
    {
        PlaneType *lm = it->second;
        PointCloudTypePtr cloud_projected( new PointCloudType );
        projectPoints( *(lm->cloud_voxel), lm->coefficients, *cloud_projected );
        voxelGridFilter( cloud_projected, lm->cloud_voxel, plane_inlier_leaf_size_ );
        setPointCloudColor( *(lm->cloud_voxel), lm->color );
        cloud_projected->clear();

        // compute 3d centroid
        Eigen::Vector4f cen;
        pcl::compute3DCentroid( *(lm->cloud_voxel), cen );
        lm->centroid.x = cen[0];
        lm->centroid.y = cen[1];
        lm->centroid.z = cen[2];
        lm->centroid.rgb = lm->color.float_value;
    }
}

void GTMapping::updateOctoMap()
{
    static int node_size = 0;

    if( !publish_octomap_ )
        return;

    // Set Resolution
    if( octree_map_->getResolution()!= octomap_resolution_ )
        octree_map_->setResolution( octomap_resolution_ );

    if( !(octomap_publisher_.getNumSubscribers()) )
        return;

    if( frames_list_.size() <= node_size )
        return;

    ros::Time start_time = ros::Time::now();
    int number = 0;
    cout << BLUE << " Octree scan size: " << node_size << RESET << endl;
    for( int idx = node_size; idx < frames_list_.size(); idx++)
    {
        std::map<int, Frame*>::iterator itf = frames_list_.find( idx );
        // Add one scan
        if( itf != frames_list_.end() )
        {
            number ++;
            const int id = itf->first;
            const Frame *frame = itf->second;
            const PointCloudTypePtr &cloud = frame->cloud_;
            const tf::Transform &pose = frame->pose_;

            // Construct a octomap::Pointcloud type
            octomap::Pointcloud * scan = new octomap::Pointcloud();
            scan->reserve( cloud->size() );
            for( PointCloudType::const_iterator it = cloud->begin();
                it != cloud->end(); ++it)
            {
                // Check if the point is invalid
                if( pcl::isFinite(*it) )
                {
                    scan->push_back(it->x, it->y, it->z);
                }
            }

            octomap::pose6d scan_pose(octomap::point3d(pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z()),
                                      octomath::Quaternion(pose.getRotation().w(), pose.getRotation().x(),
                                                           pose.getRotation().y(), pose.getRotation().z()) );
            // Construct a octomap::ScanNode
            octomap::ScanNode node( scan, scan_pose, id);
            octree_map_->insertPointCloud( node, octomap_max_depth_range_, false, false );
            node_size ++;
    //        ROS_INFO("inserted %d pt=%d ", id, (int)scan->size() );
        }

    }

    ROS_INFO(" Updated inner octree map, inserted %d (%fs)", number, (ros::Time::now() - start_time).toSec() );

}

void GTMapping::optimizeGraph( int n )
{
    while( n > 0 )
    {
        isam2_->update();
        n--;
    }
//    isam2_->print( "Map Graph");
}

void GTMapping::publishOptimizedPose()
{
    geometry_msgs::PoseStamped msg = pose3ToGeometryPose( last_estimated_pose_ );
    msg.header.frame_id = map_frame_;
    msg.header.stamp = ros::Time::now();
    optimized_pose_publisher_.publish( msg );
}

void GTMapping::publishOptimizedPath()
{
    if( !publish_optimized_path_ )
        return;

    if( optimized_path_publisher_.getNumSubscribers() )
    {
        // publish trajectory
        nav_msgs::Path path;
        path.header.frame_id = map_frame_;
        path.header.stamp = ros::Time::now();
        for( std::map<int, gtsam::Pose3>::iterator it = optimized_poses_list_.begin();
                it != optimized_poses_list_.end(); it++)
        {
            path.poses.push_back( pose3ToGeometryPose( it->second ) );
        }

        optimized_path_publisher_.publish( path );
        cout << GREEN << " Publisher optimized path, p = " << path.poses.size() << RESET << endl;
    }
}

void GTMapping::publishMapCloud()
{
    if( !publish_map_cloud_ )
        return;

    if( map_cloud_publisher_.getNumSubscribers() )
    {
        PointCloudTypePtr cloud = getMapCloud( true );

        sensor_msgs::PointCloud2 cloud2;
        pcl::toROSMsg( *cloud, cloud2);
        cloud2.header.frame_id = map_frame_;
        cloud2.header.stamp = ros::Time::now();
        cloud2.is_dense = false;

        map_cloud_publisher_.publish( cloud2 );
        cout << GREEN << " Publisher map as sensor_msgs/PointCloud2." << RESET << endl;
    }
}

void GTMapping::publishOctoMap()
{
    if( !publish_octomap_ )
        return;

    if( octomap_publisher_.getNumSubscribers() )
    {
        octomap::OcTree *octree = createOctoMap();
        octomap_msgs::Octomap msg;
        octomap_msgs::fullMapToMsg( *octree, msg);
        msg.header.frame_id = map_frame_;
        msg.header.stamp = ros::Time::now();
        octomap_publisher_.publish( msg );
        cout << GREEN << " Publisher octomap." << RESET << endl;
    }
}

void GTMapping::publishKeypointCloud()
{
    if( !publish_keypoint_cloud_ )
        return;

    if( keypoint_cloud_publisher_.getNumSubscribers() )
    {
        PointCloudTypePtr cloud = getKeypointCloud();

        sensor_msgs::PointCloud2 cloud2;
        pcl::toROSMsg( *cloud, cloud2);
        cloud2.header.frame_id = map_frame_;
        cloud2.header.stamp = ros::Time::now();
        cloud2.is_dense = false;

        keypoint_cloud_publisher_.publish( cloud2 );
        cout << GREEN << " Publisher keypoints as sensor_msgs/PointCloud2." << RESET << endl;
    }
}

void GTMapping::publishPredictedKeypoints( const std::map<int, gtsam::Point3> &predicted_keypoints )
{
    PointCloudTypePtr cloud( new PointCloudType );
    cloud->is_dense = false;
    cloud->height = 1;
    PointType pt;
    RGBValue color;
    color.Red = 0;
    color.Green = 255;
    color.Blue = 0;
    color.Alpha = 255;
    pt.rgb = color.float_value;
    for( std::map<int, gtsam::Point3>::const_iterator it = predicted_keypoints.begin();
         it != predicted_keypoints.end(); it++)
    {
        const gtsam::Point3 &point = it->second;
        pt.x = point.x();
        pt.y = point.y();
        pt.z = point.z();
        cloud->points.push_back( pt );
    }
    cloud->width = cloud->points.size();

    // To ROS message
    sensor_msgs::PointCloud2 cloud2;
    pcl::toROSMsg( *cloud, cloud2 );
    cloud2.header.frame_id = map_frame_;
    cloud2.header.stamp = ros::Time::now();

    // Publish
    predicted_keypoint_publisher_.publish( cloud2 );
}

void GTMapping::publishFrameKeypoints( const std_vector_of_eigen_vector4f &feature_3d )
{
    PointCloudXYZRGBAPtr cloud( new PointCloudXYZRGBA );
    cloud->is_dense = false;
    cloud->height = 1;
    PointType pt;
    RGBValue color;
    color.Red = 0;
    color.Green = 255;
    color.Blue = 0;
    color.Alpha = 255;
    pt.rgb = color.float_value;
    for( int i = 0; i < feature_3d.size(); i++ )
    {
        const Eigen::Vector4f &p3d = feature_3d[i];
        if( p3d(2) == 0 )
            continue;
        pt.x = p3d(0);
        pt.y = p3d(1);
        pt.z = p3d(2);
        cloud->points.push_back( pt );
    }
    cloud->width = cloud->points.size();

    // To ROS message
    sensor_msgs::PointCloud2 cloud2;
    pcl::toROSMsg( *cloud, cloud2 );
    cloud2.header.frame_id = map_frame_;
    cloud2.header.stamp = ros::Time::now();

    // Publish
    frame_keypoint_publisher_.publish( cloud2 );
}

void GTMapping::publishMatchedKeypoints( const std_vector_of_eigen_vector4f &feature_3d,
                                         const std::map<int, gtsam::Point3> &predicted_keypoints,
                                         const std::vector<cv::DMatch> &matches )
{
    cv::RNG rng(12345);
    PointCloudXYZRGBAPtr cloud( new PointCloudXYZRGBA ), cloud_global( new PointCloudXYZRGBA );
    cloud->is_dense = false;
    cloud->height = 1;
    cloud_global->is_dense = false;
    cloud_global->height = 1;
    pcl::PointXYZRGBA pt1, pt2;
    RGBValue color;
    for( int i = 0; i < matches.size(); i++ )
    {
        const cv::DMatch &m = matches[i];
        const Eigen::Vector4f &p3d = feature_3d[m.queryIdx];
        const gtsam::Point3 &pp = predicted_keypoints.at(m.trainIdx);
        color.Red = rng.uniform(0, 255);
        color.Green = rng.uniform(0, 255);
        color.Blue = rng.uniform(0, 255);
        //
        pt1.x = p3d(0);
        pt1.y = p3d(1);
        pt1.z = p3d(2);
        color.Alpha = 255;
        pt1.rgb = color.float_value;
        pt2.x = pp(0);
        pt2.y = pp(1);
        pt2.z = pp(2);
        color.Alpha = 64;
        pt2.rgb = color.float_value;
        //
        cloud->points.push_back( pt1 );
        cloud_global->points.push_back( pt2 );
    }
    cloud->width = cloud->points.size();
    cloud_global->width = cloud_global->points.size();

    // To ROS message
    sensor_msgs::PointCloud2 cloud2, cloud2_global;
    pcl::toROSMsg( *cloud, cloud2 );
    cloud2.header.frame_id = map_frame_;
    cloud2.header.stamp = ros::Time::now();
    //
    pcl::toROSMsg( *cloud_global, cloud2_global );
    cloud2_global.header.frame_id = map_frame_;
    cloud2_global.header.stamp = ros::Time::now();

    // Publish
    matched_keypoint_frame_publisher_.publish( cloud2 );
    matched_keypoint_global_publisher_.publish( cloud2_global );
}

void GTMapping::updateMapViewer()
{
    // Pose, Path, MapCloud, Octomap
    publishOptimizedPose();
    publishOptimizedPath();
    publishMapCloud();
    publishOctoMap();
    publishKeypointCloud();
    // Map in pcl visualization
    viewer_->removeMap();
    viewer_->displayMapLandmarks( landmarks_list_, "MapPlane" );
    viewer_->displayMapLandmarks( keypoints_list_, "MapPoint");
    viewer_->spinMapOnce();
}

void GTMapping::reset()
{
    // Delete old
    if( isam2_ )
        delete isam2_;

    // Construct new
    isam2_parameters_.relinearizeThreshold = isam2_relinearize_threshold_; // 0.1
    isam2_parameters_.relinearizeSkip = isam2_relinearize_skip_; // 1
    isam2_parameters_.factorization = ISAM2Params::Factorization::QR;
    isam2_parameters_.print( "ISAM2 parameters:" );
    isam2_ = new ISAM2( isam2_parameters_ );

    // clear
    factor_graph_.resize(0);
    initial_estimate_.clear();

    //
    next_plane_id_ = 0;
    next_point_id_ = 0;
    next_frame_id_ = 0;
    for( std::map<int, Frame*>::iterator it = frames_list_.begin(); it != frames_list_.end(); it++)
    {
        delete (it->second);
    }
    for( std::map<int, PlaneType*>::iterator it = landmarks_list_.begin(); it!= landmarks_list_.end(); it++)
    {
        delete (it->second);
    }
    for( std::map<int, KeyPoint*>::iterator it = keypoints_list_.begin(); it!= keypoints_list_.end(); it++)
    {
        delete (it->second);
    }
    frames_list_.clear();
    landmarks_list_.clear();
    keypoints_list_.clear();
    keypoints_vector_.clear();
    optimized_poses_list_.clear();
    optimized_landmarks_list_.clear();
    optimized_keypoints_list_.clear();
}

PointCloudTypePtr GTMapping::getMapCloud( bool force )
{
    static int iteration = 0;
    if( optimized_poses_list_.size() != iteration || force == true )
    {
        iteration = optimized_poses_list_.size();

        // update map pointcloud
        map_cloud_->clear();
        for( std::map<int, PlaneType*>::iterator it = landmarks_list_.begin();
             it != landmarks_list_.end(); it++)
        {
            PlaneType *lm = it->second;
            setPointCloudColor( *(lm->cloud_voxel), lm->color );
            *map_cloud_ += *(lm->cloud_voxel);
        }
    }

    return map_cloud_;
}

PointCloudTypePtr GTMapping::getMapFullCloud( bool colored )
{
    PointCloudTypePtr map_full_cloud( new PointCloudType );
    // Clear landmark cloud
    for( std::map<int, PlaneType*>::iterator it = landmarks_list_.begin();
         it != landmarks_list_.end(); it++)
    {
        PlaneType *lm = it->second;
        lm->cloud->clear();  // clear inlier
    }
    // Sum
    double radius = map_full_search_radius_;
//    int min_neighbors =  map_full_min_neighbor_;
    int min_neighbors =  M_PI * radius * radius / ( map_full_leaf_size_ * map_full_leaf_size_) * map_full_min_neighbor_alpha_;
    for( std::map<int, Frame*>::iterator it = frames_list_.begin();
         it != frames_list_.end(); it++)
    {
        Frame *frame = it->second;
        Eigen::Matrix4d trans = transformTFToMatrix4d( frame->pose_ );
        for( int idx = 0; idx < frame->segment_planes_.size(); idx++)
        {
            PlaneType &obs = frame->segment_planes_[idx];
            PlaneType *lm = landmarks_list_[ obs.id() ];
            PointCloudTypePtr cloud_voxeled( new PointCloudType );
            voxelGridFilter( obs.cloud, cloud_voxeled, map_full_leaf_size_ );
            if( map_full_remove_bad_inlier_ )
            {
                PointCloudTypePtr cloud_filtered( new PointCloudType );
                radiusOutlierRemoval( cloud_voxeled, cloud_filtered, radius, min_neighbors );  // remove bad inlier
                *(lm->cloud) += transformPointCloud( *(cloud_filtered), trans );
            }
            else
            {
                *(lm->cloud) += transformPointCloud( *(cloud_voxeled), trans );
            }
        }
    }

    // Project and Downsample
    for( std::map<int, PlaneType*>::iterator it = landmarks_list_.begin();
         it != landmarks_list_.end(); it++)
    {
        PlaneType *lm = it->second;
        PointCloudTypePtr cloud_projected( new PointCloudType );
        projectPoints( *(lm->cloud), lm->coefficients, *cloud_projected );
        voxelGridFilter( cloud_projected, lm->cloud, map_full_leaf_size_ );
    }


    for( std::map<int, PlaneType*>::iterator it = landmarks_list_.begin();
         it != landmarks_list_.end(); it++)
    {
        PlaneType *lm = it->second;
        if( colored )
        {
            PointCloudType cloud_color = *(lm->cloud);
            setPointCloudColor( cloud_color, lm->color );
            *map_full_cloud += cloud_color;
        }
        else
            *map_full_cloud += *(lm->cloud);
    }

    return map_full_cloud;
}

PointCloudTypePtr GTMapping::getStructureCloud()
{
    PointCloudTypePtr structure_cloud( new PointCloudType );
//    double radius = map_full_search_radius_;
//    int min_neighbors =  M_PI * radius * radius / ( map_full_leaf_size_ * map_full_leaf_size_) * map_full_min_neighbor_alpha_;
    for( std::map<int, Frame*>::iterator it = frames_list_.begin();
         it != frames_list_.end(); it++)
    {
        Frame *frame = it->second;
        Eigen::Matrix4d trans = transformTFToMatrix4d( frame->pose_ );
        PointCloudTypePtr cloud_voxeled( new PointCloudType );
        voxelGridFilter( frame->cloud_, cloud_voxeled, construct_full_leaf_size_ );
//        PointCloudTypePtr cloud_filtered( new PointCloudType );
//        radiusOutlierRemoval( cloud_voxeled, cloud_filtered, radius, min_neighbors );  // remove bad inlier
        *structure_cloud += transformPointCloud( *(cloud_voxeled), trans );
    }

    // voxel grid filter
    PointCloudTypePtr structure_cloud_voxeled( new PointCloudType );
    voxelGridFilter( structure_cloud, structure_cloud_voxeled, construct_full_leaf_size_ );

//    cout << YELLOW << " construct_full_leaf_size_ = " << construct_full_leaf_size_ << RESET << endl;

    return structure_cloud_voxeled;
}

PointCloudTypePtr GTMapping::getKeypointCloud()
{
    PointCloudTypePtr keypoint_cloud( new PointCloudType );
    keypoint_cloud->is_dense = false;
    keypoint_cloud->height = 1;

    RGBValue color;
    color.Blue = 0;
    color.Green = 0;
    color.Red = 255;
    color.Alpha = 255.0;
    PointType pt;
    pt.rgb = color.float_value;
    for( std::map<int, KeyPoint*>::iterator it = keypoints_list_.begin();
         it != keypoints_list_.end(); it++)
    {
        KeyPoint *keypoint = it->second;
        pt.x = keypoint->translation[0];
        pt.y = keypoint->translation[1];
        pt.z = keypoint->translation[2];
        keypoint_cloud->points.push_back( pt );
    }
    keypoint_cloud->width = keypoint_cloud->points.size();

    return keypoint_cloud;
}

octomap::OcTree * GTMapping::createOctoMap( double resolution )
{
    ros::Time start_time = ros::Time::now();

    if( resolution == 0) resolution = octomap_resolution_;
    octomap::OcTree * octree = new octomap::OcTree( resolution );

    int number = 0;
    for( std::map<int, Frame*>::iterator itf = frames_list_.begin();
         itf != frames_list_.end(); itf++)
    {
        number ++;
        const int id = itf->first;
        const Frame *frame = itf->second;
        const PointCloudTypePtr &cloud = frame->cloud_;
        const tf::Transform &pose = frame->pose_;

        // Construct a octomap::Pointcloud type
        octomap::Pointcloud * scan = new octomap::Pointcloud();
        scan->reserve( cloud->size() );
        for( PointCloudType::const_iterator it = cloud->begin();
            it != cloud->end(); ++it)
        {
            // Check if the point is invalid
            if( pcl::isFinite(*it) )
            {
                scan->push_back(it->x, it->y, it->z);
            }
        }

        octomap::pose6d scan_pose(octomap::point3d(pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z()),
                                  octomath::Quaternion(pose.getRotation().w(), pose.getRotation().x(),
                                                       pose.getRotation().y(), pose.getRotation().z()) );
        // Construct a octomap::ScanNode
        octomap::ScanNode node( scan, scan_pose, id);
        octree->insertPointCloud( node, octomap_max_depth_range_, true, true);
//        ROS_INFO("inserted %d pt=%d ", id, (int)scan->size() );
    }

    octree->updateInnerOccupancy();
    ROS_INFO(" Create octree, inserted %d (%fs)", number, (ros::Time::now() - start_time).toSec() );

    return octree;
}

void GTMapping::saveMapPCD( const std::string &filename )
{
    PointCloudTypePtr cloud = getMapCloud( true );
    pcl::io::savePCDFileASCII ( filename, *cloud);
}

void GTMapping::saveMapFullPCD( const std::string &filename )
{
    PointCloudTypePtr cloud = getMapFullCloud();
    pcl::io::savePCDFileASCII ( filename, *cloud);
}

void GTMapping::saveMapFullColoredPCD( const std::string &filename )
{
    PointCloudTypePtr cloud = getMapFullCloud(true);
    pcl::io::savePCDFileASCII ( filename, *cloud);
}

void GTMapping::saveStructurePCD( const std::string &filename )
{
    PointCloudTypePtr cloud = getStructureCloud();
    pcl::io::savePCDFileASCII ( filename, *cloud);
}

void GTMapping::saveMapKeypointPCD( const std::string &filename )
{
    PointCloudTypePtr cloud = getKeypointCloud();
    pcl::io::savePCDFileASCII ( filename, *cloud);
}

std::vector<geometry_msgs::PoseStamped> GTMapping::getOptimizedPath()
{
    std::vector<geometry_msgs::PoseStamped> poses;
    for( std::map<int, gtsam::Pose3>::iterator it = optimized_poses_list_.begin();
            it != optimized_poses_list_.end(); it++)
    {
        poses.push_back( pose3ToGeometryPose( it->second ) );
    }
    return poses;
}


void GTMapping::gtMappingReconfigCallback(plane_slam::GTMappingConfig &config, uint32_t level)
{
    //
    use_keyframe_ = config.use_keyframe;
    keyframe_linear_threshold_ = config.keyframe_linear_threshold;
    keyframe_angular_threshold_ = config.keyframe_angular_threshold * DEG_TO_RAD;
    //
    isam2_relinearize_threshold_ = config.isam2_relinearize_threshold;
    isam2_relinearize_skip_ = config.isam2_relinearize_skip;
    //
    plane_match_direction_threshold_ = config.plane_match_direction_threshold * DEG_TO_RAD;
    plane_match_distance_threshold_ = config.plane_match_distance_threshold;
    plane_match_check_overlap_ = config.plane_match_check_overlap;
    plane_match_overlap_alpha_ = config.plane_match_overlap_alpha;
    plane_inlier_leaf_size_ = config.plane_inlier_leaf_size;
    plane_hull_alpha_ = config.plane_hull_alpha;
    //
    refine_planar_map_ = config.refine_planar_map;
    planar_merge_direction_threshold_ = config.planar_merge_direction_threshold * DEG_TO_RAD;
    planar_merge_distance_threshold_ = config.planar_merge_distance_threshold;
    //
    remove_plane_bad_inlier_ = config.remove_plane_bad_inlier;
    planar_bad_inlier_alpha_ = config.planar_bad_inlier_alpha;
    //
    map_full_leaf_size_ = config.map_full_leaf_size;
    map_full_remove_bad_inlier_ = config.map_full_remove_bad_inlier;
    map_full_min_neighbor_ = config.map_full_min_neighbor;
    map_full_search_radius_ = config.map_full_search_radius;
    map_full_min_neighbor_alpha_ = config.map_full_min_neighbor_alpha;
    construct_full_leaf_size_ = config.construct_full_leaf_size;
    octomap_resolution_ = config.octomap_resolution;
    octomap_max_depth_range_ = config.octomap_max_depth_range;
    publish_map_cloud_ = config.publish_map_cloud;
    publish_octomap_ = config.publish_octomap;
    publish_keypoint_cloud_ = config.publish_keypoint_cloud;
    publish_optimized_path_ = config.publish_optimized_path;

    cout << GREEN <<" GTSAM Mapping Config." << RESET << endl;
}

bool GTMapping::optimizeGraphCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    if( isam2_->empty() )
    {
        res.success = false;
        res.message = " Failed, graph is empty.";
    }
    else
    {
        optimizeGraph();        // graph optimizing
        updateOptimizedResult();    // update result
        updateLandmarksInlier();    // Update Map
        if( refine_planar_map_ )    // Refine map
            refinePlanarMap();
        updateMapViewer();  // update visualization
        res.success = true;
        res.message = " Optimize graph for 10 times, and update map viewer.";
    }

    cout << GREEN << res.message << RESET << endl;
    return true;
}

bool GTMapping::saveGraphCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    if( isam2_->empty() )
    {
        res.success = false;
        res.message = " Failed, graph is empty.";
    }
    else
    {
        std::string filename = "/home/lizhi/bags/result/"+timeToStr()+"_graph.dot";
        saveGraphDot( filename );
        res.success = true;
        res.message = " Save isam2 graph as dot file: " + filename + ".";
    }

    cout << GREEN << res.message << RESET << endl;
    return true;
}

bool GTMapping::saveMapPCDCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    if( !landmarks_list_.size() )
    {
        res.success = false;
        res.message = " Failed, map is empty.";
    }
    else
    {
        std::string filename = "/home/lizhi/bags/result/" + timeToStr() + "_map.pcd";
        saveMapPCD( filename );
        res.success = true;
        res.message = " Save map as pcd file: " + filename + ".";
    }

    cout << GREEN << res.message << RESET << endl;
    return true;
}

bool GTMapping::saveMapFullPCDCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    if( !landmarks_list_.size() )
    {
        res.success = false;
        res.message = " Failed, map is empty.";
    }
    else
    {
        std::string filename = "/home/lizhi/bags/result/" + timeToStr() + "_map_full.pcd";
        std::string filename_colored = "/home/lizhi/bags/result/" + timeToStr() + "_map_full_colored.pcd";
        saveMapFullPCD( filename );
        saveMapFullColoredPCD( filename_colored );
        res.success = true;
        res.message = " Save full map as pcd file: " + filename + " and " + filename_colored +".";
    }

    cout << GREEN << res.message << RESET << endl;
    return true;
}

bool GTMapping::removeBadInlierCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    if( !landmarks_list_.size() )
    {
        res.success = false;
        res.message = " Failed, map is empty.";
    }
    else
    {
        removeLandmarksBadInlier();  // remove bad inlier in landmarks
        updateMapViewer();  // update map visualization
        //
        res.success = true;
        res.message = " Remove landmark bad inlier.";
    }

    cout << GREEN << res.message << RESET << endl;
    return true;
}

} // end of namespace plane_slam
