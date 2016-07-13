#include "g2o_mapping.h"

namespace plane_slam
{

G2OMapping::G2OMapping( ros::NodeHandle &nh, Viewer* viewer )
    : nh_(nh)
    , viewer_(viewer)
    , map_frame_("/world")
    , mapping_config_server_( ros::NodeHandle( nh_, "G2OMapping" ) )
//    , isam2_parameters_()
//    , graph_()
//    , initial_estimate_()
//    , pose_count_( 0 )
//    , landmark_max_count_( 0 )
    , use_keyframe_( true )
    , keyframe_linear_threshold_( 0.05f )
    , keyframe_angular_threshold_( 5.0f )
//    , isam2_relinearize_threshold_( 0.05f )
//    , isam2_relinearize_skip_( 1 )
    , plane_match_direction_threshold_( 10.0*DEG_TO_RAD )   // 10 degree
    , plane_match_distance_threshold_( 0.1 )    // 0.1meter
    , plane_match_check_overlap_( true )
    , plane_match_overlap_alpha_( 0.5 )
    , plane_inlier_leaf_size_( 0.05f )  // 0.05meter
    , plane_hull_alpha_( 0.5 )
    , rng_(12345)
{
    // reconfigure
    mapping_config_callback_ = boost::bind(&G2OMapping::g2oMappingReconfigCallback, this, _1, _2);
    mapping_config_server_.setCallback(mapping_config_callback_);

    // Pose and Map
    optimized_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("optimized_pose", 10);
    optimized_path_publisher_ = nh_.advertise<nav_msgs::Path>("optimized_path", 10);
    map_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("map_cloud", 10);
    marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);


    list_solvers_ = true;   // list valid solvers
    str_solver_ = "lm_var"; // select one specific solver
    // Optimizer
    global_optimizer_ = new SparseOptimizer();
    ParameterSE3Offset* odomOffset = new ParameterSE3Offset();
    odomOffset->setId(0);
    global_optimizer_->addParameter( odomOffset );
    global_optimizer_->setVerbose( true ); // verbose information during optimization
    // Solver
    OptimizationAlgorithmFactory* solverFactory = OptimizationAlgorithmFactory::instance();
    OptimizationAlgorithmProperty solverProperty;
    solver_ = solverFactory->construct( str_solver_, solverProperty );
    global_optimizer_->setAlgorithm( solver_ );
    if( list_solvers_ ){
        solverFactory->listSolvers( cerr );
    }

}

bool G2OMapping::mapping( const Frame &frame )
{
    bool success;
//    if( !landmarks_.size() )    // first frame, add prior
    if( 0 )
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

    // Map visualization
    if( success )
        updateMapViewer();

    cout << GREEN << " G2OMapping, success = " << (success?"true":"false") << "." << RESET << endl;

    return success;
}

bool G2OMapping::doMapping( const Frame &frame )
{

}

bool G2OMapping::addFirstFrame( const Frame &frame )
{
    if( frame.segment_planes_.size() == 0 )
        return false;

    const std::vector<PlaneType> &planes = frame.segment_planes_;
    const tf::Transform init_pose = frame.pose_;

    // clear
    pose_count_ = 0;
    landmark_max_count_ = 0;
    estimated_poses_.clear();
    landmarks_.clear();
    estimated_planes_.clear();

    // Add first vertex, fixed
    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId( pose_count_ );
    v->setEstimate();
    v->setFixed( true );
    global_optimizer_->addVertex( v );

    // Add to estimated pose
    estimated_poses_.push_back( v );
    pose_count_++;

    // Add plane vertex
    g2o::VertexPlane* vp = new g2o::VertexPlane();

    // Add Edge


    // Optimize factor graph
//    isam2_->update( graph_, initial_estimate_ );
    last_estimated_pose_ = init_pose;
    last_estimated_pose_tf_ = pose3ToTF( last_estimated_pose_ );


    cout << GREEN << " Register first frame in the map." << endl;
//    initial_estimate_.print(" - Init pose: ");
    cout << " - Initial pose: " << endl;
    printTransform( transformTFToMatrix4d( init_pose ); );
    cout << RESET << endl;

    return true;
}

bool G2OMapping::isKeyFrame( const Frame &frame )
{
    tf::Transform rel_tf = last_estimated_pose_tf_.inverse() * frame.pose_;
    double rad, distance;
    calAngleAndDistance( rel_tf, rad, distance );

    if( distance > keyframe_linear_threshold_ || rad > keyframe_angular_threshold_ )
        return true;
    else
        return false;
}

void G2OMapping::g2oMappingReconfigCallback(plane_slam::G2OMappingConfig &config, uint32_t level)
{
    //
    use_keyframe_ = config.use_keyframe;
    keyframe_linear_threshold_ = config.keyframe_linear_threshold;
    keyframe_angular_threshold_ = config.keyframe_angular_threshold * DEG_TO_RAD;
    //
//    isam2_relinearize_threshold_ = config.isam2_relinearize_threshold;
//    isam2_relinearize_skip_ = config.isam2_relinearize_skip;
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
    planar_bad_inlier_alpha_ = config.planar_bad_inlier_alpha;
    //
    publish_optimized_path_ = config.publish_optimized_path;

    cout << GREEN <<" G2O Mapping Config." << RESET << endl;
}

} // end of namespace plane_slam
