#include "mapping.h"

namespace plane_slam
{

Mapping::Mapping(ros::NodeHandle &nh)
    : nh_(nh)
    , map_frame_("/world")
    , mapping_config_server_( ros::NodeHandle( nh_, "Mapping" ) )
    , isam2_parameters_()
    , graph_()
    , initial_estimate_()
    , pose_count_( 0 )
    , landmark_max_count_( 0 )
    , plane_match_direction_threshold_( 10.0*DEG_TO_RAD )   // 10 degree
    , plane_match_distance_threshold_( 0.1 )    // 0.1meter
    , plane_match_check_overlap_( true )
    , plane_match_overlap_alpha_( 0.5 )
    , plane_inlier_leaf_size_( 0.05f )  // 0.05meter
    , plane_hull_alpha_( 0.5 )
    , rng_(12345)
{
    // reconfigure
    mapping_config_callback_ = boost::bind(&Mapping::mappingReconfigCallback, this, _1, _2);
    mapping_config_server_.setCallback(mapping_config_callback_);

    // ISAM2
    isam2_parameters_.relinearizeThreshold = 0.1;
    isam2_parameters_.relinearizeSkip = 1;
    isam2_parameters_.factorization = ISAM2Params::Factorization::QR;
    isam2_parameters_.print( "ISAM2 parameters:" );
    isam2_ = new ISAM2( isam2_parameters_ );

    // Pose and Map
    optimized_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("optimized_pose", 10);
    optimized_path_publisher_ = nh_.advertise<nav_msgs::Path>("optimized_path", 10);
    map_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("map_cloud", 10);
    marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
}


bool Mapping::mapping( const Frame &frame )
{
    if( !landmarks_.size() )    // first frame, add prior
    {
        return addFirstFrame( frame );
    }
    else    // do mappint
    {
        return doMapping( frame );
    }
}

bool Mapping::doMapping( const Frame &frame )
{

}

bool Mapping::addFirstFrame( const Frame &frame )
{
    if( frame.segment_planes_.size() == 0 )
        return false;

    const std::vector<PlaneType> &planes = frame.segment_planes_;
    const gtsam::Pose3 &init_pose = frame.pose_;

    // clear
    pose_count_ = 0;
    landmark_max_count_ = 0;
    initial_estimate_.clear();
    estimated_poses_.clear();
    landmarks_.clear();
    estimated_planes_.clear();

    // Add a prior factor
    pose_count_ = 0;
    Key x0 = Symbol('x', 0);
    Vector pose_sigmas(6);
//    pose_sigmas << init_pose.translation().vector()*0.2, init_pose.rotation().rpy() * 0.2;
    pose_sigmas << 0.2, 0.2, 0.2, 0.2, 0.2, 0.2;
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

    landmark_max_count_ = 0;
    for(int i = 0; i < planes.size(); i++)
    {
        const PlaneType &plane = planes[i];
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
        global_plane.color.Blue = rng_.uniform(0, 255);
        global_plane.color.Green = rng_.uniform(0, 255);
        global_plane.color.Red = rng_.uniform(0, 255);
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

    // Optimize factor graph
    isam2_->update( graph_, initial_estimate_ );
    last_estimated_pose_ = init_pose;

    // Clear the factor graph and values for the next iteration
    graph_.resize(0);
    initial_estimate_.clear();
    //

    cout << GREEN << "Register first frame in the map." << endl;
    initial_estimate_.print(" - Init pose: ");
    cout << RESET << endl;

    return true;
}

void Mapping::voxelGridFilter( const PointCloudTypePtr &cloud,
                               PointCloudTypePtr &cloud_filtered,
                               float leaf_size)
{
    // Create the filtering object
    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud ( cloud );
    sor.setLeafSize ( leaf_size, leaf_size, leaf_size );
    sor.filter ( *cloud_filtered );
}

void Mapping::publishOptimizedPose()
{
    geometry_msgs::PoseStamped msg = pose3ToGeometryPose( last_estimated_pose_ );
    msg.header.frame_id = map_frame_;
    msg.header.stamp = ros::Time::now();
    optimized_pose_publisher_.publish( msg );
}

void Mapping::publishOptimizedPath()
{
    // publish trajectory
    nav_msgs::Path path;
    path.header.frame_id = map_frame_;
    path.header.stamp = ros::Time::now();
    for(int i = 0; i < estimated_poses_.size(); i++)
    {
        path.poses.push_back( pose3ToGeometryPose( estimated_poses_[i] ) );
    }
    optimized_path_publisher_.publish( path );
    cout << GREEN << "Publisher optimized path, p = " << estimated_poses_.size() << RESET << endl;
}

void Mapping::publishMapCloud()
{
    PointCloudTypePtr cloud ( new PointCloudType );

    for( int i = 0; i < landmarks_.size(); i++)
    {
        const PlaneType &lm = landmarks_[i];
        if( !lm.valid )
            continue;
       *cloud += *lm.cloud;
    }

    sensor_msgs::PointCloud2 cloud2;
    pcl::toROSMsg( *cloud, cloud2);
    cloud2.header.frame_id = map_frame_;
    cloud2.header.stamp = ros::Time::now();
    cloud2.is_dense = false;

    map_cloud_publisher_.publish( cloud2 );
}

void Mapping::mappingReconfigCallback(plane_slam::MappingConfig &config, uint32_t level)
{
    //
    use_keyframe_ = config.use_keyframe;
    keyframe_linear_threshold_ = config.keyframe_linear_threshold;
    keyframe_angular_threshold_ = config.keyframe_angular_threshold * DEG_TO_RAD;
    //
    plane_match_direction_threshold_ = config.plane_match_direction_threshold;
    plane_match_distance_threshold_ = config.plane_match_distance_threshold;
    plane_match_check_overlap_ = config.plane_match_check_overlap;
    plane_match_overlap_alpha_ = config.plane_match_overlap_alpha;
    plane_inlier_leaf_size_ = config.plane_inlier_leaf_size;
    plane_hull_alpha_ = config.plane_hull_alpha;
    //
    refine_planar_map_ = config.refine_planar_map;
    planar_merge_direction_threshold_ = config.planar_merge_direction_threshold;
    planar_merge_distance_threshold_ = config.planar_merge_distance_threshold;
    //
    publish_optimized_path_ = config.publish_optimized_path;

    cout << GREEN <<"Mapping Config." << RESET << endl;
}

} // end of namespace plane_slam
