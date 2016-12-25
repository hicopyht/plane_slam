#include "kinect_listener.h"

namespace plane_slam
{

KinectListener::KinectListener() :
    private_nh_("~")
  , plane_slam_config_server_( ros::NodeHandle( "PlaneSlam" ) )
  , tf_listener_( nh_, ros::Duration(10.0) )
  , camera_parameters_()
  , set_init_pose_( false )
  , save_message_pcd_(0)
{
    private_nh_.param<bool>("verbose", verbose_, false );
    cout << WHITE << "  verbose = " << (verbose_?"true":"false") << RESET << endl;
    //
    nh_.setCallbackQueue(&my_callback_queue_);

    // Set initial pose
    double x, y, z, roll, pitch, yaw;
    private_nh_.param<double>("init_pose_x", x, 0);
    private_nh_.param<double>("init_pose_y", y, 0);
    private_nh_.param<double>("init_pose_z", z, 0);
    private_nh_.param<double>("init_pose_roll", roll, 0);
    private_nh_.param<double>("init_pose_pitch", pitch, 0);
    private_nh_.param<double>("init_pose_yaw", yaw, 0);
    init_pose_ = tf::Transform( tf::createQuaternionFromRPY(roll, pitch, yaw), tf::Vector3(x, y, z) );
    //
    private_nh_.param<bool>("publish_map_tf", publish_map_tf_, true );
    private_nh_.param<double>("map_tf_freq", map_tf_freq_, 50.0 );
    //
    private_nh_.param<string>("keypoint_type", keypoint_type_, "ORB");
    cout << WHITE << "  keypoint_type = " << keypoint_type_ << RESET << endl;
    //
    surf_detector_ = new DetectorAdjuster("SURF", 200);
    surf_extractor_ = new cv::SurfDescriptorExtractor();
    orb_extractor_ = new ORBextractor( 1000, 1.2, 8, 20, 7);
    line_based_plane_segmentor_ = new LineBasedPlaneSegmentor(nh_);
    organized_plane_segmentor_ = new OrganizedPlaneSegmentor(nh_);
    viewer_ = new Viewer(nh_);
    tracker_ = new Tracking(nh_, viewer_ );
    gt_mapping_ = new GTMapping(nh_, viewer_, tracker_);
    //
    tracker_->setVerbose( verbose_ );
    gt_mapping_->setVerbose( verbose_ );

    // reconfigure
    plane_slam_config_callback_ = boost::bind(&KinectListener::planeSlamReconfigCallback, this, _1, _2);
    plane_slam_config_server_.setCallback(plane_slam_config_callback_);

    private_nh_.param<int>("subscriber_queue_size", subscriber_queue_size_, 4);
    private_nh_.param<string>("topic_image_visual", topic_image_visual_, "/head_kinect/rgb/image_rect_color");
    private_nh_.param<string>("topic_image_depth", topic_image_depth_, "/head_kinect/depth_registered/image");
    private_nh_.param<string>("topic_camera_info", topic_camera_info_, "/head_kinect/depth_registered/camera_info");
    private_nh_.param<string>("topic_point_cloud", topic_point_cloud_, "");

//    private_nh_.param<int>("subscriber_queue_size", subscriber_queue_size_, 4);
//    private_nh_.param<string>("topic_image_visual", topic_image_visual_, "/camera/rgb/image_color");
////    private_nh_.param<string>("topic_image_visual", topic_image_visual_, "");
//    private_nh_.param<string>("topic_image_depth", topic_image_depth_, "/camera/depth/image");
//    private_nh_.param<string>("topic_camera_info", topic_camera_info_, "/camera/depth/camera_info");
//    private_nh_.param<string>("topic_point_cloud", topic_point_cloud_, "");

    // True path and odometry path
    true_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("true_pose", 10);
    true_path_publisher_ = nh_.advertise<nav_msgs::Path>("true_path", 10);
    odom_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("odom_pose", 10);
    odom_path_publisher_ = nh_.advertise<nav_msgs::Path>("odom_path", 10);
    visual_odometry_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("visual_odometry_pose", 10);
    visual_odometry_path_publisher_ = nh_.advertise<nav_msgs::Path>("visual_odometry_path", 10);
    update_viewer_once_ss_ = nh_.advertiseService("update_viewer_once", &KinectListener::updateViewerOnceCallback, this );
    save_slam_result_simple_ss_ = nh_.advertiseService("save_slam_result", &KinectListener::saveSlamResultSimpleCallback, this );
    save_slam_result_all_ss_ = nh_.advertiseService("save_slam_result_all", &KinectListener::saveSlamResultCallback, this );

    // config subscribers
    if( !topic_point_cloud_.empty() && !topic_image_visual_.empty() && !topic_camera_info_.empty() ) // pointcloud2
    {
        // use visual image, depth image, pointcloud2
        visual_sub_ = new image_sub_type(nh_, topic_image_visual_, subscriber_queue_size_);
        cloud_sub_ = new pc_sub_type (nh_, topic_point_cloud_, subscriber_queue_size_);
        cinfo_sub_ = new cinfo_sub_type(nh_, topic_camera_info_, subscriber_queue_size_);
        cloud_sync_ = new message_filters::Synchronizer<CloudSyncPolicy>(CloudSyncPolicy(subscriber_queue_size_),  *visual_sub_, *cloud_sub_, *cinfo_sub_),
        cloud_sync_->registerCallback(boost::bind(&KinectListener::cloudCallback, this, _1, _2, _3));
        ROS_INFO_STREAM("Listening to " << topic_image_visual_ << ", " << topic_point_cloud_ << " and " << topic_camera_info_ << ".");
    }
    else if( !topic_image_visual_.empty() && !topic_image_depth_.empty() && !topic_camera_info_.empty() )
    {
        //No cloud, use visual image, depth image, camera_info
        visual_sub_ = new image_sub_type(nh_, topic_image_visual_, subscriber_queue_size_);
        depth_sub_ = new image_sub_type (nh_, topic_image_depth_, subscriber_queue_size_);
        cinfo_sub_ = new cinfo_sub_type(nh_, topic_camera_info_, subscriber_queue_size_);
        no_cloud_sync_ = new message_filters::Synchronizer<NoCloudSyncPolicy>(NoCloudSyncPolicy(subscriber_queue_size_),  *visual_sub_, *depth_sub_, *cinfo_sub_),
        no_cloud_sync_->registerCallback(boost::bind(&KinectListener::noCloudCallback, this, _1, _2, _3));
        ROS_INFO_STREAM("Listening to " << topic_image_visual_ << ", " << topic_image_depth_ << " and " << topic_camera_info_ << ".");
    }
    else
    {
        ROS_ERROR("Can not decide subscriber type");
        exit(1);
    }

    odom_to_map_tf_.setIdentity();
    if( publish_map_tf_ )
    {
        tf_timer_ = nh_.createTimer(ros::Duration(1.0/map_tf_freq_), &KinectListener::publishTfTimerCallback, this );
        tf_timer_.start();
    }
    async_spinner_ =  new ros::AsyncSpinner( 6, &my_callback_queue_ );
    async_spinner_->start();

    std::srand( std::time(0) );
}

KinectListener::~KinectListener()
{
    async_spinner_->stop();
    cv::destroyAllWindows();
}

void KinectListener::noCloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                                const sensor_msgs::ImageConstPtr& depth_img_msg,
                                const sensor_msgs::CameraInfoConstPtr& cam_info_msg)
{
    static int skip = 0;
    camera_frame_ = depth_img_msg->header.frame_id;

    skip = (skip + 1) % skip_message_;
    if( skip )
    {
        cout << BLUE << " Skip message." << RESET << endl;
        return;
    }

    cout << BOLDRED << "#" << RESET;

    // Get odom pose
    tf::Transform odom_pose;
    if( get_odom_pose_ )
    {
        if( getTfPose( odom_pose, depth_img_msg->header.frame_id, odom_frame_, depth_img_msg->header.stamp-ros::Duration(0.033)) )
        {
            // Print info
            if( verbose_ )
            {
                cout << CYAN << " Odom pose: " << RESET << endl;
                printTransform( transformTFToMatrix4d(odom_pose) );
            }
        }
        else
        {
            return;
        }
    }

    // Get true pose
    if( get_true_pose_ )
    {
        if( getTfPose( true_pose_, depth_img_msg->header.frame_id, world_frame_, ros::Time(0)) )
        {
            // Print info
            if( verbose_ )
            {
                cout << BLUE << " True pose: " << RESET << endl;
                printTransform( transformTFToMatrix4d(true_pose_) );
            }
        }
        else
        {
            return;
        }
    }

    // Get camera parameter
    CameraParameters camera;
    cvtCameraParameter( cam_info_msg, camera);

    if( get_odom_pose_ )
    {
        trackDepthRgbImage( visual_img_msg, depth_img_msg, camera, odom_pose );
    }
    else
    {
        trackDepthRgbImage( visual_img_msg, depth_img_msg, camera );
    }

}

void KinectListener::cloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                                const sensor_msgs::PointCloud2ConstPtr& point_cloud,
                                const sensor_msgs::CameraInfoConstPtr& cam_info_msg)
{
    static int skip = 0;
    camera_frame_ = point_cloud->header.frame_id;

    skip = (skip + 1) % skip_message_;
    if( skip )
    {
        cout << BLUE << " Skip cloud message." << RESET << endl;
        return;
    }

    // Get odom pose
    tf::Transform odom_pose;
    if( get_odom_pose_ )
    {
        if( getTfPose( odom_pose, point_cloud->header.frame_id, odom_frame_, point_cloud->header.stamp-ros::Duration(0.02)) )
        {
            // Print info
            if( verbose_ )
            {
                cout << CYAN << " Odom pose: " << RESET << endl;
                printTransform( transformTFToMatrix4d(odom_pose) );
            }
        }
        else
        {
            return;
        }
    }

    // Get true pose
    if( get_true_pose_ )
    {
        if( getTfPose( true_pose_, point_cloud->header.frame_id, world_frame_, point_cloud->header.stamp-ros::Duration(0.005)) )
        {
            // Print info
            if( verbose_ )
            {
                cout << BLUE << " True pose: " << RESET << endl;
                printTransform( transformTFToMatrix4d(true_pose_) );
            }
        }
        else
        {
            return;
        }
    }

    // Get camera parameter
    CameraParameters camera;
    cvtCameraParameter( cam_info_msg, camera);

    if( get_odom_pose_ )
        trackPointCloud( point_cloud, camera, odom_pose );
    else
        trackPointCloud( point_cloud, camera, tf::Transform(tf::Quaternion(0, 0, 0, 1.0), tf::Vector3(0, 0,0)) );
}

// Only use planes from pointcloud as landmarks
void KinectListener::trackPointCloud( const sensor_msgs::PointCloud2ConstPtr &point_cloud,
                                      CameraParameters& camera,
                                      tf::Transform odom_pose )
{
    static tf::Transform last_keyframe_odom = tf::Transform::getIdentity();
    static Frame *last_frame;
    static bool last_frame_valid = false;

    cout << BOLDMAGENTA << "no cloud msg: " << point_cloud->header.seq << RESET << endl;

    // If motion is too small, stop mapping
    if( mapping_key_message_ && !last_frame->valid_
          && !isBigOdomChange(last_keyframe_odom.inverse() * odom_pose, gt_mapping_->getKeyFrameLinearThreshold(), gt_mapping_->getKeyFrameAngularThreshold()) )
    {
        return;
    }

    // Time
    const ros::Time start_time = ros::Time::now();
    ros::Time step_time = start_time;
    double frame_dura, track_dura, map_dura, display_dura;
    double total_dura;

    // Frame
    Frame *frame = pointCloudToFrame( point_cloud, camera );
    frame->odom_pose_ = odom_pose;
    //
    frame_dura = (ros::Time::now() - step_time).toSec() * 1000.0f;
    step_time = ros::Time::now();

    // Motion from odom
    trackFrameMotionOdom( last_frame, frame );
    //
    track_dura = (ros::Time::now() - step_time).toSec() * 1000.0f;
    step_time = ros::Time::now();

    // Record & publish visual odometry & odometry
    recordVisualOdometry( last_frame, frame );

    // Mapping
    if( frame->valid_ ) // always valid
    {
        if( mapping_keypoint_ )
            frame->key_frame_ = gt_mapping_->mappingMix( frame );
        else
            frame->key_frame_ = gt_mapping_->mapping( frame );
    }
    map_dura = (ros::Time::now() - step_time).toSec() * 1000.0f;
    step_time = ros::Time::now();


    // Upate odom to map tf
    calculateOdomToMapTF( frame->pose_, frame->odom_pose_ );

    // Visualization for mappint result
    displayMappingResult( frame );
    //
    display_dura = (ros::Time::now() - step_time).toSec() * 1000.0f;
    step_time = ros::Time::now();

    //
    total_dura = (step_time - start_time).toSec() * 1000.0f;
    // Print time
    cout << GREEN << "Processing total time: " << total_dura << endl;
    cout << "Time:"
         << " frame: " << frame_dura
         << ", tracking: " << track_dura
         << ", mapping: " << map_dura
         << ", display: " << display_dura
         << RESET << endl;

    // Runtimes, push new one
    if( frame->key_frame_ && last_frame_valid)
    {
        const double total = frame_dura + track_dura + map_dura;
        runtimes_.push_back( Runtime(frame->header_.seq, true, frame_dura, track_dura, map_dura, total) );
        cout << GREEN << " Runtimes size: " << runtimes_.size() << RESET << endl;
    }

    // Store key frame
    storeKeyFrame( last_frame, frame );

    // Store key frame odom pose
    if( frame->key_frame_ )
        last_keyframe_odom = odom_pose;

}


void KinectListener::trackDepthRgbImage( const sensor_msgs::ImageConstPtr &visual_img_msg,
                                         const sensor_msgs::ImageConstPtr &depth_img_msg,
                                         CameraParameters & camera,
                                         tf::Transform &odom_pose)
{
    static tf::Transform last_keyframe_odom = tf::Transform::getIdentity();
    static Frame *last_frame = new Frame();

    frame_count_++;
    if( verbose_ )
    {
        cout << BOLDMAGENTA << "no cloud msg(force odom): " << depth_img_msg->header.seq
             << MAGENTA << " use_odom_tracking_ = " << (use_odom_tracking_?"true":"false") << RESET << endl;
    }
    else{
        cout << BOLDMAGENTA << "no cloud msg: " << depth_img_msg->header.seq << RESET << endl;
    }

    // If motion is too small, stop mapping
    if( mapping_key_message_ && !last_frame->valid_
          && !isBigOdomChange(last_keyframe_odom.inverse() * odom_pose, gt_mapping_->getKeyFrameLinearThreshold(), gt_mapping_->getKeyFrameAngularThreshold()) )
    {
        return;
    }

    // Time
    const ros::Time start_time = ros::Time::now();
    ros::Time step_time = start_time;
    double frame_dura, track_dura, map_dura, display_dura;
    double total_dura;

    // Get frame
    Frame *frame = depthRgbToFrame( visual_img_msg, depth_img_msg, camera );
    frame->odom_pose_ = odom_pose;
    //
    frame_dura = (ros::Time::now() - step_time).toSec() * 1000.0f;
    step_time = ros::Time::now();

    // Debug
    debugFrame( frame );

    // Motion from odom
    trackFrameMotionOdom( last_frame, frame );
    //
    track_dura = (ros::Time::now() - step_time).toSec() * 1000.0f;
    step_time = ros::Time::now();

    // Record & publish visual odometry & odometry
    recordVisualOdometry( last_frame, frame );

    // Mapping
    if( frame->valid_ && do_slam_ ) // always valid
    {
        if( mapping_keypoint_ )
            frame->key_frame_ = gt_mapping_->mappingMix( frame );
        else
            frame->key_frame_ = gt_mapping_->mapping( frame );
    }
    map_dura = (ros::Time::now() - step_time).toSec() * 1000.0f;
    step_time = ros::Time::now();
    //
    if( frame->key_frame_ ){
        pushMappintRuntimes( frame->header_.seq, gt_mapping_->convert_duration_, gt_mapping_->plane_match_duration_,
                         gt_mapping_->keypoint_match_duration_, gt_mapping_->add_delete_duration_, gt_mapping_->optimize_duration_,
                         gt_mapping_->refine_duration_, gt_mapping_->update_lm_duration_, gt_mapping_->update_inlier_duration_,
                         gt_mapping_->update_octomap_duration_, gt_mapping_->display_duration_, gt_mapping_->total_duration_);
    }

    // Upate odom to map tf
    calculateOdomToMapTF( frame->pose_, odom_pose );

    // Map for visualization
    displayMappingResult( frame );

    //
    display_dura = (ros::Time::now() - step_time).toSec() * 1000.0f;
    step_time = ros::Time::now();

    // Throttle memory
    if( frame->key_frame_ && gt_mapping_->isThrottleMemory() )
        frame->throttleMemory();

    //
    total_dura = (step_time - start_time).toSec() * 1000.0f;
    // Print time
    if( verbose_ )
        cout << GREEN << "Segment planes = " << frame->segment_planes_.size() << RESET << endl;
    if( verbose_ )
        cout << GREEN << "Processing total time: " << total_dura << RESET << endl;
    if( frame->key_frame_ ){
        cout << GREEN << "Time:"
             << " frame: " << MAGENTA << frame_dura
             << GREEN << ", tracking: " << MAGENTA << track_dura
             << GREEN << ", mapping: " << MAGENTA << map_dura
             << GREEN << ", display: " << MAGENTA << display_dura
             << RESET << endl;
    }

    // Runtimes, push new one
    if( frame->key_frame_ )
    {
        // Runtimes
        const double total = frame_dura + track_dura + map_dura;
        runtimes_.push_back( Runtime(frame->header_.seq, true, frame_dura, track_dura, map_dura, total, gt_mapping_->isMapRefined(), gt_mapping_->isKeypointRemoved()) );
        if( verbose_ )
            cout << GREEN << " Runtimes size: " << runtimes_.size() << RESET << endl;
    }

    // Store key frame
    storeKeyFrame( last_frame, frame );

    // Store key frame odom pose
    if( frame->key_frame_ )
        last_keyframe_odom = odom_pose;
}

void KinectListener::trackDepthRgbImage( const sensor_msgs::ImageConstPtr &visual_img_msg,
                                         const sensor_msgs::ImageConstPtr &depth_img_msg,
                                         CameraParameters & camera)
{
    static Frame *last_frame = new Frame();
    static bool last_frame_valid = false;

    frame_count_++;
    if( verbose_ ){
        cout << RESET << "----------------------------------------------------------------------" << endl;
        cout << BOLDMAGENTA << "no cloud msg: " << depth_img_msg->header.seq << RESET << endl;
    }else{
        cout << BOLDMAGENTA << "no cloud msg: " << depth_img_msg->header.seq << RESET << endl;
    }

    // Time
    const ros::Time start_time = ros::Time::now();
    ros::Time step_time = start_time;
    double frame_dura, track_dura, map_dura, display_dura;
    double total_dura;

    // Get frame
    Frame *frame = depthRgbToFrame( visual_img_msg, depth_img_msg, camera );
    //
    frame_dura = (ros::Time::now() - step_time).toSec() * 1000.0f;
    step_time = ros::Time::now();

    // Motion from features
    trackFrameMotion( last_frame, frame );
    //
    track_dura = (ros::Time::now() - step_time).toSec() * 1000.0f;
    step_time = ros::Time::now();

    // Record & publish visual odometry & odometry
    recordVisualOdometry( last_frame, frame );

    // Mapping
    if( frame->valid_ && do_slam_ )
    {
        if( mapping_keypoint_ )
            frame->key_frame_ = gt_mapping_->mappingMix( frame );
        else
            frame->key_frame_ = gt_mapping_->mapping( frame );
    }
    map_dura = (ros::Time::now() - step_time).toSec() * 1000.0f;
    step_time = ros::Time::now();
    //
    if( frame->key_frame_ ){
        pushMappintRuntimes( frame->header_.seq, gt_mapping_->convert_duration_, gt_mapping_->plane_match_duration_,
                         gt_mapping_->keypoint_match_duration_, gt_mapping_->add_delete_duration_, gt_mapping_->optimize_duration_,
                         gt_mapping_->refine_duration_, gt_mapping_->update_lm_duration_, gt_mapping_->update_inlier_duration_,
                         gt_mapping_->update_octomap_duration_, gt_mapping_->display_duration_, gt_mapping_->total_duration_);
    }

    // Upate odom to map tf
//    calculateOdomToMapTF( frame->pose_, frame->odom_pose_);

    // Visualization for mapping result
    displayMappingResult( frame );
    //
    display_dura = (ros::Time::now() - step_time).toSec() * 1000.0f;
    step_time = ros::Time::now();

    // Throttle memory
    if( frame->key_frame_ && gt_mapping_->isThrottleMemory() )
        frame->throttleMemory();

    //
    total_dura = (step_time - start_time).toSec() * 1000.0f;
    // Print time
    if( verbose_ )
        cout << GREEN << "Processing total time: " << total_dura << endl;
    if( frame->key_frame_ ){
        cout << "Time:"
             << " frame: " << frame_dura
             << ", tracking: " << track_dura
             << ", mapping: " << map_dura
             << ", display: " << display_dura
             << RESET << endl;
    }

    // Runtimes, push new one, publish optimized path
    if( !do_slam_ )
    {
        // Runtimes
        const double total = frame_dura + track_dura + map_dura;
        runtimes_.push_back( Runtime(frame->header_.seq, frame->key_frame_, frame_dura, track_dura, map_dura, total) );
        if( verbose_ )
            cout << GREEN << " Runtimes size: " << runtimes_.size() << RESET << endl;
    }
    else if( frame->key_frame_ )
    {
        // Runtimes
        const double total = frame_dura + track_dura + map_dura;
        runtimes_.push_back( Runtime(frame->header_.seq, true, frame_dura, track_dura, map_dura, total, gt_mapping_->isMapRefined(), gt_mapping_->isKeypointRemoved()) );
        if( verbose_ )
            cout << GREEN << " Runtimes size: " << runtimes_.size() << RESET << endl;
    }

    // Store key frame
    storeKeyFrame( last_frame, frame );
}

Frame* KinectListener::pointCloudToFrame(const sensor_msgs::PointCloud2ConstPtr &point_cloud,
                                         CameraParameters & camera)
{
    camera_parameters_ = camera;

    // Ros message to pcl type
    PointCloudTypePtr input( new PointCloudType );
    pcl::fromROSMsg( *point_cloud, *input);

    // Compute Frame
    Frame *frame;
    if( plane_segment_method_ == LineBased )
        frame = new Frame( input, camera_parameters_, line_based_plane_segmentor_);
    else
        frame = new Frame( input, camera_parameters_, organized_plane_segmentor_);
    frame->header_ = point_cloud->header;
    frame->stamp_ = point_cloud->header.stamp;
    frame->valid_ = false;

    if( get_true_pose_ )
        frame->world_pose_ = true_pose_;

    return frame;
}

Frame* KinectListener::depthRgbToFrame(const sensor_msgs::ImageConstPtr &visual_img_msg,
                                       const sensor_msgs::ImageConstPtr &depth_img_msg,
                                       CameraParameters & camera)
{
    // Store camera parameter
    camera_parameters_ = camera;

    // Get Mat Image
    cv::Mat visual_image = cv_bridge::toCvCopy(visual_img_msg)->image; // to cv image
    cv::Mat depth_image = cv_bridge::toCvCopy(depth_img_msg)->image; // to cv image

    // Compute Frame
    Frame *frame;
    if( !keypoint_type_.compare("ORB") )
    {
        if( plane_segment_method_ == LineBased )
            frame = new Frame( visual_image, depth_image, camera_parameters_, orb_extractor_, line_based_plane_segmentor_);
        else
            frame = new Frame( visual_image, depth_image, camera_parameters_, orb_extractor_, organized_plane_segmentor_);

    }
    else if( !keypoint_type_.compare("SURF") )
    {
        if( plane_segment_method_ == LineBased )
            frame = new Frame( visual_image, depth_image, camera_parameters_, surf_detector_, surf_extractor_, line_based_plane_segmentor_);
        else
            frame = new Frame( visual_image, depth_image, camera_parameters_, surf_detector_, surf_extractor_, organized_plane_segmentor_);

    }else
    {
        ROS_ERROR_STREAM("keypoint_type_ undefined.");
        return (new Frame());
    }
    //
    frame->header_ = depth_img_msg->header;
    frame->stamp_ = depth_img_msg->header.stamp;
    frame->valid_ = false;

    if( get_true_pose_ )
        frame->world_pose_ = true_pose_;

    pushFrameRuntimes( frame->header_.seq, frame->segment_planes_.size(), frame->feature_locations_3d_.size(), frame->pointcloud_cvt_duration_, frame->pointcloud_downsample_duration_,
                       frame->plane_segment_duration_, frame->keypoint_extract_duration_, frame->total_duration_ );

    return frame;
}

bool KinectListener::isBigOdomChange( tf::Transform rel_odom, double linear_threshold, double angular_threshold )
{
    // Check odom pose change
    double linear_inc, angular_inc;
    calAngleAndDistance( rel_odom, angular_inc, linear_inc);
    if( fabs(angular_inc) < angular_threshold
          && fabs(linear_inc) < linear_threshold )
    {
        return false;
    }else{
        return true;
    }
}

bool KinectListener::trackFrameMotionOdom( Frame* last_frame, Frame* frame )
{
    tf::Transform estimated_rel_tf = tf::Transform::getIdentity();
    // First frame
    if( last_frame->valid_ )
        estimated_rel_tf = last_frame->odom_pose_.inverse()*frame->odom_pose_;
    Eigen::Matrix4d estimated_transform = transformTFToMatrix4d( estimated_rel_tf );

    // Cal motion
    RESULT_OF_MOTION motion;
    motion.valid = false;
    if( last_frame->valid_ )  // Do tracking
    {
        if( !use_odom_tracking_ )
            tracker_->trackPlanes( *last_frame, *frame, motion, estimated_transform);
        else
        {
            motion.setTransform4d( estimated_transform );
            motion.valid = true;
        }
        //
        if( motion.valid && !use_odom_tracking_ )  // success, print tracking result. Not if using odom
        {
            if( verbose_ )
            {
                printPose3( motionToPose3(motion), " estimated motion", MAGENTA );
                gtsam::Rot3 rot3( motion.rotation );
                cout << MAGENTA << " estimated motion, rmse = " << motion.rmse << endl;
                cout << "  - R(rpy): " << rot3.roll()
                     << ", " << rot3.pitch()
                     << ", " << rot3.yaw() << endl;
                cout << "  - T:      " << motion.translation[0]
                     << ", " << motion.translation[1]
                     << ", " << motion.translation[2] << RESET << endl;
            }
        }
        else if( use_odom_tracking_ )
        {
            if( verbose_ )
                cout << YELLOW << "  odom >> tracking." << RESET << endl;
        }
        else    // failed
        {
            motion.setTransform4d( estimated_transform );
            if( verbose_ )
                cout << YELLOW << "failed to estimated motion, use odom." << RESET << endl;
        }

        // estimated pose
        frame->pose_ = last_frame->pose_ * motionToTf( motion );
        frame->valid_ = true;
    }
    else
    {
        frame_count_ = 1;
        // check number of planes ???
        if( frame->segment_planes_.size() > 0)
        {
            frame->valid_ = true;   // first frame, set valid, add to mapper as first frame
            if( get_odom_pose_ )
                frame->pose_ = frame->odom_pose_;   // set odom pose as initial pose
            if( get_true_pose_ )    // Set true pose as initial pose
                frame->pose_ = true_pose_;
            if( set_init_pose_ )
                frame->pose_ = init_pose_;
        }
    }

    return motion.valid;
}

bool KinectListener::trackFrameMotion( Frame* last_frame, Frame *frame )
{
    RESULT_OF_MOTION motion;
    motion.valid = false;
    if( last_frame->valid_ )  // Do tracking
    {
        tracker_->track( *last_frame, *frame, motion );

        // print motion
        if( motion.valid )  // success, print tracking result
        {
            if( verbose_ ){
                cout << MAGENTA << " Tracking motion, rmse = " << motion.rmse << endl;
                printTransform( motion.transform4d(), "", MAGENTA);
            }
            // estimated pose
            frame->pose_ = last_frame->pose_ * motionToTf( motion );
            frame->valid_ = true;
        }
        else    // failed
        {
            frame->valid_ = false;
            if( verbose_ )
                cout << RED << "failed to estimated motion." << RESET << endl;
        }
    }
    else
    {
        // check number of planes for 1st frame ???
        if( frame->segment_planes_.size() > 0){
            frame->valid_ = true;   // first frame, set valid, add to mapper as first frame
            if( get_odom_pose_ )
                frame->pose_ = frame->odom_pose_;   // set odom pose as initial pose
            if( get_true_pose_ )    // Set true pose as initial pose
                frame->pose_ = true_pose_;
            if( set_init_pose_ )
                frame->pose_ = init_pose_;
        }
    }

    return motion.valid;
}

void KinectListener::recordVisualOdometry( Frame *last_frame, Frame *frame )
{
    // Publish visual odometry path
    if( !last_frame->valid_ && frame->valid_ ) // First frame, valid, set initial pose
    {
        frame_count_ = 1;

        // Visual odometry pose
        if( set_init_pose_ )    // First odometry pose to identity.
        {
            visual_odometry_poses_.clear();
            geometry_msgs::PoseStamped pose = tfToGeometryPose( init_pose_ );
            pose.header = frame->header_;
            visual_odometry_poses_.push_back( pose );
        }
        else
        {
            // First odometry pose to true pose.
            visual_odometry_poses_.clear();
            geometry_msgs::PoseStamped pose = tfToGeometryPose( frame->pose_ );
            pose.header = frame->header_;
            visual_odometry_poses_.push_back( pose );
        }

        // Odom pose
        odom_poses_.clear();
        if( get_odom_pose_ ){
            geometry_msgs::PoseStamped pose = tfToGeometryPose( frame->odom_pose_ );
            pose.header = frame->header_;
            odom_poses_.push_back( pose );
        }

        // True pose
        true_poses_.clear();
        if( get_true_pose_ ){
            geometry_msgs::PoseStamped pose = tfToGeometryPose( frame->world_pose_ );
            pose.header = frame->header_;
            true_poses_.push_back( pose );
        }
    }
    else if( frame->valid_ ) // not first frame, motion is valid, calculate & publish odometry pose.
    {
        // Visual odometry pose
        {
            geometry_msgs::PoseStamped pose = tfToGeometryPose( frame->pose_ );
            pose.header = frame->header_;
            visual_odometry_poses_.push_back( pose );
        }

        // Odom pose
        if( get_odom_pose_ ){
            geometry_msgs::PoseStamped pose = tfToGeometryPose( frame->odom_pose_ );
            pose.header = frame->header_;
            odom_poses_.push_back( pose );
        }

        // True pose
        if( get_true_pose_ ){
            geometry_msgs::PoseStamped pose = tfToGeometryPose( frame->world_pose_ );
            pose.header = frame->header_;
            true_poses_.push_back( pose );
        }

        publishVisualOdometryPose();
        publishVisualOdometryPath();
        publishOdomPose();
        publishOdomPath();
        publishTruePose();
        publishTruePath();
    }

}

void KinectListener::calculateOdomToMapTF( tf::Transform &map_tf, tf::Transform &odom_tf )
{
    tf::Transform odom_to_map = map_tf * odom_tf.inverse();
    double yaw = tf::getYaw( odom_to_map.getRotation() );
    // Set x, y, yaw
    map_tf_mutex_.lock();
    odom_to_map_tf_ = tf::Transform( tf::createQuaternionFromYaw(yaw),
                                     tf::Vector3(odom_to_map.getOrigin().x(), odom_to_map.getOrigin().y(), 0) );
    map_tf_mutex_.unlock();
}

void KinectListener::displayMappingResult( Frame* frame )
{
    if( frame->key_frame_)
    {
        viewer_->removeMap();
        viewer_->displayPath( true_poses_, "true_path", 0, 1.0, 0 );
        viewer_->displayPath( odom_poses_, "odom_path", 1.0, 1.0, 0 );
        viewer_->displayPath( visual_odometry_poses_, "visual_odom_path", 0, 0, 1.0 );
        gt_mapping_->updateMapViewer();
        viewer_->spinMapOnce();
    }

    if( frame->valid_ )
    {
        viewer_->removeFrames();
        viewer_->displayInputCloud( frame->cloud_, "frame_input_cloud", viewer_->vp2() );
        viewer_->displayPlanes( frame->cloud_downsampled_, frame->segment_planes_, "frame_planes", viewer_->vp4() );
        viewer_->spinFramesOnce();
    }
}

void KinectListener::storeKeyFrame( Frame* &last_frame, Frame* &frame )
{
//    cout << WHITE << " store key frame: " << BLUE << (frame->valid_?"true":"false") << RESET << endl;

    if( frame->valid_ )    // store key frame
    {
        if( !(last_frame->key_frame_) )
            delete last_frame;      // delete last frame if not keyframe

        // Store current frame
        last_frame = frame;
    }
    else
    {
        delete frame;   // delete invalid frame
    }

//    cout << YELLOW << " done." << RESET << endl;
}

void KinectListener::debugFrame( Frame* frame )
{
    // Debug frame
    if( save_message_pcd_ && save_message_pcd_ == frame->header_.seq )
    {
        std::string time_str = timeToStr();
        stringstream ss;
        ss << frame->header_.seq;
        pcl::io::savePCDFileASCII( "/home/lizhi/bags/selected/"+time_str+"_"+ss.str()+".pcd", *(frame->cloud_) );
        save_message_pcd_ = 0;

        cout << " planes size: ";
        for( int i = 0; i < frame->segment_planes_.size(); i++)
        {
            cout << " " << frame->segment_planes_[i].cloud->points.size();
        }
        cout << endl;

        ros::Rate loop_rate(10);
        viewer_->removeFrames();
        viewer_->displayInputCloud( frame->cloud_, "frame_input_cloud", viewer_->vp2() );
        viewer_->displayPlanes( frame->cloud_downsampled_, frame->segment_planes_, "frame_planes", viewer_->vp4() );
        viewer_->spinFramesOnce();
        while( ros::ok() ){
            viewer_->spinFramesOnce();
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
}

void KinectListener::savePlaneLandmarks( const std::string &filename )
{
    cout << WHITE << "Save planes landmarks: " << filename << "..." << RESET << endl;

    FILE* yaml = std::fopen( filename.c_str(), "w" );
    fprintf( yaml, "# landmarks file: %s\n", filename.c_str() );
    fprintf( yaml, "# landmarks format: ax+by+cz+d = 0");
    fprintf( yaml, "# lm(a,b,c,d,id,numOfPoints(voxel), semantic_label)");

    // Save Landmarks
    std::map<int, PlaneType*> landmarks = gt_mapping_->getLandmark();
    fprintf( yaml, "# landmarks, size %d\n\n", landmarks.size() );
    for( std::map<int, PlaneType*>::const_iterator it = landmarks.begin();
         it != landmarks.end(); it++)
    {
        PlaneType *lm = it->second;
        if( !lm->valid )
            continue;
        fprintf( yaml, "%f %f %f %f %d %d %s\n", lm->coefficients[0], lm->coefficients[1],
                lm->coefficients[2], lm->coefficients[3], it->first, lm->cloud_voxel->size(), lm->semantic_label.c_str() );
//        cout << WHITE << " - save lm: " << BLUE << it->first << WHITE << " points: " << BLUE << lm->cloud_voxel->size() << RESET << endl;
    }
    fprintf( yaml, "\n");

    // close
    fclose(yaml);
}

void KinectListener::savePathToFile( const std::vector<geometry_msgs::PoseStamped> &poses,
                                     const std::string &filename )
{
    cout << WHITE << "Save path: " << filename << "..." << RESET << endl;

    FILE* yaml = std::fopen( filename.c_str(), "w" );
    fprintf( yaml, "# %s\n", filename.c_str() );
    fprintf( yaml, "# pose format: T(xyz) Q(xyzw) seq\n" );
    fprintf( yaml, "# poses: %d\n", poses.size() );
    // Save Path
    for( int i = 0; i < poses.size(); i++)
    {
        const geometry_msgs::PoseStamped &pose = poses[i];
        fprintf( yaml, "%d %f %f %f %f %f %f %f \n", pose.header.seq, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
                 pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w );
    }

    // close
    fclose(yaml);
}

void KinectListener::saveKeypointLandmarks( const std::string &filename )
{
    cout << WHITE << "Save keypoints landmarks: " << filename << "..." << RESET << endl;

    FILE* yaml = std::fopen( filename.c_str(), "w" );
    fprintf( yaml, "# keypoints: %s\n", filename.c_str() );
    fprintf( yaml, "# gtsam::Point3: (x,y,z), key\n");
    fprintf( yaml, "# descriptor: uint64*4 = 256bits = 32bytes\n" );

    // Save location
    std::map<int, KeyPoint*> keypoints = gt_mapping_->getKeypointLandmark();
    fprintf( yaml, "# keypoints, size %d\n", keypoints.size() );
    fprintf( yaml, "# location:\n");
    for( std::map<int, KeyPoint*>::const_iterator it = keypoints.begin();
         it != keypoints.end(); it++)
    {
        KeyPoint *kp = it->second;
        if( !kp->initialized )
            continue;
        fprintf( yaml, "%f %f %f %d\n", kp->translation.x(), kp->translation.y(), kp->translation.z(), it->first );
    }
    fprintf( yaml, "\n\n");

    // Save descriptor
    fprintf( yaml, "# descriptor:\n" );
    for( std::map<int, KeyPoint*>::const_iterator it = keypoints.begin();
         it != keypoints.end(); it++)
    {
        KeyPoint *kp = it->second;
        if( !kp->initialized )
            continue;
        for( int i = 0; i < 4; i++ )
        {
            uint8_t *ch = (uint8_t *)(kp->descriptor);
            fprintf( yaml, "%d %d %d %d %d %d %d %d ", *ch, *(ch+1), *(ch+2), *(ch+3), *(ch+4), *(ch+5), *(ch+6), *(ch+7));
        }
        fprintf( yaml, "\n" );
    }
    fprintf( yaml, "\n");

    // close
    fclose(yaml);
}

void KinectListener::saveRuntimes( const std::string &filename )
{
    cout << WHITE << "Save runtimes: " << filename << "..." << RESET << endl;

    FILE* yaml = std::fopen( filename.c_str(), "w" );
    fprintf( yaml, "# runtimes file: %s\n", filename.c_str() );
    fprintf( yaml, "# format: frame tracking mapping total refined removed\n" );
    fprintf( yaml, "# frame size: %d\n", frame_count_ );
    fprintf( yaml, "# key frame size: %d\n", runtimes_.size() );

    Runtime avg_time;
    Runtime max_time( -1, true, 0, 0, 0, 0);
    Runtime min_time( -1, true, 1e6, 1e6, 1e6, 1e6);
    int count = 0;
    for( int i = 0; i < runtimes_.size(); i++)
    {
        Runtime &runtime = runtimes_[i];
//        if( !runtime.key_frame )
//            continue;
        fprintf( yaml, "%d %f %f %f %f", runtime.id, runtime.frame, runtime.tracking, runtime.mapping, runtime.total);
        if( runtime.map_refined_ )
            fprintf( yaml, " refined" );
        if( runtime.keypoint_removed_ )
            fprintf( yaml, " removed" );
        fprintf( yaml, "\n");
        avg_time += runtime;
        count ++;
        //
        max_time.getMaximum( runtime );
        min_time.getMinimum( runtime );
    }

    if( !count )
    {
        fclose(yaml);
        return;
    }

    // Average
    avg_time /= count;
    fprintf( yaml, "# average time:\n", count);
    fprintf( yaml, "%f %f %f %f\n", avg_time.frame,
             avg_time.tracking,
             avg_time.mapping,
             avg_time.total);
    fprintf( yaml, "\n");

    // Maximum
    fprintf( yaml, "# maximum time:\n");
    fprintf( yaml, "%f %f %f %f\n", max_time.frame,
             max_time.tracking, max_time.mapping, max_time.total);
    fprintf( yaml, "\n");

    // Minimum
    fprintf( yaml, "# minimum time:\n");
    fprintf( yaml, "%f %f %f %f\n", min_time.frame,
             min_time.tracking, min_time.mapping, min_time.total);
    fprintf( yaml, "\n");

    //
    Runtime error_time;
    error_time = max_time;
    error_time += min_time;
    error_time /= 2.0;
    fprintf( yaml, "# (max+min)/2.0 time:\n");
    fprintf( yaml, "%f %f %f %f\n", error_time.frame,
             error_time.tracking, error_time.mapping, error_time.total);
    fprintf( yaml, "\n");
    //
    error_time = max_time;
    error_time -= min_time;
    error_time /= 2.0;
    fprintf( yaml, "# (max-min)/2.0 time:\n");
    fprintf( yaml, "%f %f %f %f\n", error_time.frame,
             error_time.tracking, error_time.mapping, error_time.total);
    fprintf( yaml, "\n");
    //
    error_time = max_time;
    error_time -= avg_time;
    fprintf( yaml, "# (max-avg) time:\n");
    fprintf( yaml, "%f %f %f %f\n", error_time.frame,
             error_time.tracking, error_time.mapping, error_time.total);
    fprintf( yaml, "\n");
    //
    error_time = avg_time;
    error_time -= min_time;
    fprintf( yaml, "# (avg-min) time:\n");
    fprintf( yaml, "%f %f %f %f\n", error_time.frame,
             error_time.tracking, error_time.mapping, error_time.total);
    fprintf( yaml, "\n");

    // close
    fclose(yaml);
}


void KinectListener::saveFrameRuntimes(const std::string &filename)
{
    if( frame_sequences_.size() == 0 )
        return;

    cout << WHITE << "Save frame runtimes: " << filename << "..." << RESET << endl;

    FILE* yaml = std::fopen( filename.c_str(), "w" );
    fprintf( yaml, "# frame runtimes file: %s\n", filename.c_str() );
    fprintf( yaml, "# format: seq cvt downsample segment extract total nplanes nkeypoints\n" );
    fprintf( yaml, "# frame size: %d\n", (frame_sequences_.back() - frame_sequences_.front() + 1) );
    fprintf( yaml, "# size: %d\n", frame_sequences_.size() );

    for( int i = 0; i < frame_sequences_.size(); i++)
    {
        fprintf( yaml, "%d %f %f %f %f %f %d %d\n", frame_sequences_[i],
                 pointcloud_cvt_durations_[i], pointcloud_downsample_durations_[i],
                 plane_segment_durations_[i], kp_extract_durations_[i], frame_total_durations_[i],
                 frame_plane_size_[i], frame_keypoint_size_[i]);
    }

    fclose(yaml);
    return;
}

void KinectListener::saveMappingRuntimes(const std::string &filename)
{
    if( mapping_sequences_.size() == 0 )
        return;

    cout << WHITE << "Save mapping runtimes: " << filename << "..." << RESET << endl;

    FILE* yaml = std::fopen( filename.c_str(), "w" );
    fprintf( yaml, "# mapping runtimes file: %s\n", filename.c_str() );
    fprintf( yaml, "# format: seq convert p_m kp_m add&delete optimize refine lm inlier octomap display total\n" );
    fprintf( yaml, "# frame size: %d\n", (mapping_sequences_.back() - mapping_sequences_.front() + 1) );
    fprintf( yaml, "# size: %d\n", mapping_sequences_.size() );


    for( int i = 0; i < mapping_sequences_.size(); i++)
    {
        fprintf( yaml, "%d %f %f %f %f %f %f %f %f %f %f %f\n",
                 mapping_sequences_[i], mapping_convert_durations_[i], mapping_plane_match_durations_[i],
                 mapping_kp_match_durations_[i], mapping_add_delete_durations_[i], mapping_optimize_durations_[i],
                 mapping_refine_durations_[i], mapping_update_lm_durations_[i], mapping_update_inlier_durations_[i],
                 mapping_update_octomap_durations_[i], mapping_display_durations_[i], mapping_total_durations_[i] );
    }

    fclose(yaml);
    return;
}

void KinectListener::saveKeyKrameSequence( const std::string &filename )
{
    cout << WHITE << "Save key frame sequences: " << filename << "..." << RESET << endl;

    FILE* yaml = std::fopen( filename.c_str(), "w" );
    fprintf( yaml, "# key frames file: %s\n", filename.c_str() );
    fprintf( yaml, "# format: id sequence\n");

    // Save key frame sequences
    std::map<int, Frame*> frames_list = gt_mapping_->getFrames();
    fprintf( yaml, "# size: %d\n", frames_list.size());
    for( std::map<int, Frame*>::iterator it = frames_list.begin();
         it != frames_list.end(); it++)
    {
        Frame *frame = it->second;
        fprintf( yaml, "%d %d\n", frame->id(), frame->header_.seq );
    }

    // close
    fclose(yaml);
}

void KinectListener::cvtCameraParameter( const sensor_msgs::CameraInfoConstPtr &cam_info_msg,
                                         CameraParameters &camera)
{
    /* Intrinsic camera matrix for the raw (distorted) images.
         [fx  0 cx]
     K = [ 0 fy cy]
         [ 0  0  1] */
    camera.cx = cam_info_msg->K[2];
    camera.cy = cam_info_msg->K[5];
    camera.fx = cam_info_msg->K[0];
    camera.fy = cam_info_msg->K[4];

    //
    camera.scale = 1.0;
    // Additionally, organized cloud width and height.
    camera.width = cam_info_msg->width;
    camera.height = cam_info_msg->height;

//    // TUM1
////    cout << YELLOW << " Use TUM1 camera parameters." << RESET << endl;
//    camera.cx = 318.643040;
//    camera.cy = 516.469215;
//    camera.fx = 517.306408;
//    camera.fy = 516.469215;
//    //
//    camera.scale = 1.0;
//    camera.width = 640;
//    camera.height = 480;


//    // TUM2
//    cout << YELLOW << " Use TUM3 camera parameters." << RESET << endl;
//    camera.cx = 325.141442;
//    camera.cy = 521.007327;
//    camera.fx = 520.908620;
//    camera.fy = 521.007327;
//    //
//    camera.scale = 1.0;
//    camera.width = 640;
//    camera.height = 480;



//    // TUM3
////    cout << YELLOW << " Use TUM3 camera parameters." << RESET << endl;
//    camera.cx = 320.1;
//    camera.cy = 247.6;
//    camera.fx = 535.4;
//    camera.fy = 539.2;
//    //
//    camera.scale = 1.0;
//    camera.width = 640;
//    camera.height = 480;

}

//bool KinectListener::getOdomPose( tf::Transform &odom_pose, const std::string &camera_frame, const ros::Time &time)
//{
//    // get transform
//    tf::StampedTransform trans;
//    try{
//        tf_listener_.lookupTransform(camera_frame, odom_frame_, time, trans);
//    }catch (tf::TransformException &ex)
//    {
//        ROS_WARN("%s",ex.what());
//        odom_pose.setIdentity();
//        return false;
//    }
//    odom_pose.setOrigin( trans.getOrigin() );
//    odom_pose.setRotation( trans.getRotation() );

//    return true;
//}


bool KinectListener::getOdomPose( tf::Transform &odom_pose, const std::string &camera_frame, const ros::Time& t)
{
    // Identity camera pose
    tf::Stamped<tf::Pose> camera_pose = tf::Stamped<tf::Pose>(tf::Transform(tf::createQuaternionFromRPY(0,0,0),
                                                                        tf::Vector3(0,0,0)), t, camera_frame);
    // Get the camera's pose that is centered
    tf::Stamped<tf::Transform> pose;
    try
    {
        tf_listener_.transformPose(odom_frame_, camera_pose, pose);
    }
    catch(tf::TransformException e)
    {
        ROS_WARN("Failed to compute odom pose. %s", e.what());
        return false;
    }

    odom_pose = pose;

    return true;
}

bool KinectListener::getTfPose( tf::Transform &pose, const std::string &source_frame, const std::string &target_frame, const ros::Time& t)
{
    // Identity camera pose
    tf::Stamped<tf::Pose> identity_pose = tf::Stamped<tf::Pose>(tf::Transform(tf::createQuaternionFromRPY(0,0,0),
                                                                        tf::Vector3(0,0,0)), t, source_frame);
    // Get the camera's pose that is centered
    tf::Stamped<tf::Transform> target_pose;
    try
    {
        tf_listener_.transformPose(target_frame, identity_pose, target_pose);
    }
    catch(tf::TransformException e)
    {
        ROS_WARN("Failed to compute tf pose. %s", e.what());
        return false;
    }

    pose = target_pose;

    return true;
}

void KinectListener::publishTruePose()
{
    if( true_poses_.size() == 0 )
        return;

    geometry_msgs::PoseStamped msg = true_poses_.back();
    msg.header.frame_id = map_frame_;   // Publish to map frame
    msg.header.stamp = ros::Time::now();
    true_pose_publisher_.publish( msg );
}

void KinectListener::publishTruePath()
{
    if( true_path_publisher_.getNumSubscribers() <= 0 )
        return;

    nav_msgs::Path path;
    path.header.frame_id = map_frame_;  // Publish to map frame
    path.header.stamp = ros::Time::now();
    path.poses = true_poses_;
    true_path_publisher_.publish( path );
//    cout << GREEN << " Publish true path, p = " << true_poses_.size() << RESET << endl;
}

void KinectListener::publishOdomPose()
{
    if( odom_poses_.size() == 0 )
        return;

    geometry_msgs::PoseStamped msg = odom_poses_.back();
    msg.header.frame_id = map_frame_;
    msg.header.stamp = ros::Time::now();
    odom_pose_publisher_.publish( msg );
}

void KinectListener::publishOdomPath()
{
    if( odom_path_publisher_.getNumSubscribers() <= 0 )
        return;

    nav_msgs::Path path;
    path.header.frame_id = map_frame_;
    path.header.stamp = ros::Time::now();
    path.poses = odom_poses_;
    odom_path_publisher_.publish( path );
    cout << GREEN << " Publish odom path, p = " << odom_poses_.size() << RESET << endl;
}

void KinectListener::publishVisualOdometryPose()
{   
    if( visual_odometry_poses_.size() == 0 )
        return;

    geometry_msgs::PoseStamped msg = visual_odometry_poses_.back();
    msg.header.frame_id = map_frame_;
    msg.header.stamp = ros::Time::now();
    visual_odometry_pose_publisher_.publish( msg );
}

void KinectListener::publishVisualOdometryPath()
{
    if( visual_odometry_path_publisher_.getNumSubscribers() <= 0 )
        return;

    nav_msgs::Path path;
    path.header.frame_id = map_frame_;
    path.header.stamp = ros::Time::now();
    path.poses = visual_odometry_poses_;
    visual_odometry_path_publisher_.publish( path );
    cout << GREEN << " Publish odometry path, p = " << visual_odometry_poses_.size() << RESET << endl;
}

void KinectListener::planeSlamReconfigCallback(plane_slam::PlaneSlamConfig &config, uint32_t level)
{
    plane_segment_method_ = config.plane_segment_method;
    do_visual_odometry_ = config.do_visual_odometry;
    do_mapping_ = config.do_mapping;
    do_slam_ = config.do_slam;
    get_true_pose_ = config.get_true_pose;
    get_odom_pose_ = config.get_odom_pose;
    if( !get_odom_pose_ ){
        use_odom_tracking_ = false;
        config.use_odom_tracking = false;
        mapping_key_message_ = false;
        config.mapping_key_message = false;
    }
    else {
        use_odom_tracking_ = config.use_odom_tracking;
        mapping_key_message_ = config.mapping_key_message;
    }
    mapping_keypoint_ = config.mapping_keypoint;
    world_frame_ = config.world_frame;
    map_frame_ = config.map_frame;
    base_frame_ = config.base_frame;
    odom_frame_ = config.odom_frame;
    skip_message_ = config.skip_message;
    set_init_pose_ = config.set_init_pose_ || set_init_pose_;

    //
    save_map_full_pcd_ = config.save_map_full_pcd;
    save_map_full_colored_pcd_ = config.save_map_full_colored_pcd;
    save_structure_pcd_ = config.save_structure_pcd;
    save_octomap_ = config.save_octomap;
    //
    save_input_cloud_pcd_ = config.save_input_cloud_pcd;
    if( config.save_message_pcd != 0 )
    {
        save_message_pcd_ = config.save_message_pcd;
        config.save_message_pcd = 0;
    }

    // Set map frame for mapping
    if( gt_mapping_ )
        gt_mapping_->setMapFrame( map_frame_ );
    //
    cout << GREEN <<" PlaneSlam Config." << RESET << endl;
}

bool KinectListener::updateViewerOnceCallback( std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res )
{
    //
    publishTruePose();
    publishTruePath();
    publishOdomPose();
    publishOdomPath();
    publishVisualOdometryPose();
    publishVisualOdometryPath();
    // Map for visualization
    viewer_->removeMap();
    viewer_->displayPath( true_poses_, "true_path", 0, 1.0, 0 );
    viewer_->displayPath( odom_poses_, "odom_path", 1.0, 1.0, 0 );
    viewer_->displayPath( visual_odometry_poses_, "visual_odom_path", 0, 0, 1.0 );
    gt_mapping_->updateMapViewer();
    viewer_->spinMapOnce();
    res.success = true;
    res.message = " Update viewer once";
    cout << GREEN << res.message << RESET << endl;
    return true;
}

bool KinectListener::saveSlamResultSimpleCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    std::string time_str = timeToStr(); // appended time string
    std::string dir = "/home/lizhi/bags/result/"+time_str;    // directory
    std::string prefix;
    if( !boost::filesystem::create_directory(dir) )
        prefix = "/home/lizhi/bags/result/"+time_str+"_";
    else
        prefix = "/home/lizhi/bags/result/"+time_str+"/";
    //
    gt_mapping_->updateLandmarksInlierAll();
    //
    saveKeyKrameSequence( prefix + "frames.txt");
    savePlaneLandmarks( prefix + "planes.txt");  // save path and landmarks
    savePathToFile( gt_mapping_->getOptimizedPath(), prefix + "optimized_path.txt");
    savePathToFile( odom_poses_, prefix + "odom_path.txt");
    savePathToFile( visual_odometry_poses_, prefix + "visual_odometry_path.txt");
    savePathToFile( true_poses_, prefix + "true_path.txt");
    saveKeypointLandmarks( prefix + "keypoints.txt");   // save keypoints
    saveRuntimes( prefix + "runtimes.txt" );   // save runtimes
    saveFrameRuntimes( prefix + "frame_runtimes.txt");  // save frame runtimes
    saveMappingRuntimes( prefix + "mapping_runtimes.txt");
    tracker_->saveRuntimes( prefix + "tracking_runtimes.txt" ); // save tracking runtimes


    if( !do_slam_ ){
        //
        res.success = true;
        res.message = " Save slam result(landmarks&path, map(simplied, keypoints, planes, full, full colored), octomap, graph) to directory: " + dir + ".";
        cout << GREEN << res.message << RESET << endl;
        return true;
    }

    gt_mapping_->saveGraphDot( prefix + "graph.dot");      // save grapy
    gt_mapping_->saveMapKeypointPCD( prefix + "map_keypoints.pcd"); // save keypoint cloud
    gt_mapping_->saveMapPCD( prefix + "map.pcd");          // save map
    //
    if( save_map_full_pcd_ )
        gt_mapping_->saveMapFullPCD( prefix + "map_full.pcd"); // save map full
    if( save_map_full_colored_pcd_ )
        gt_mapping_->saveMapFullColoredPCD( prefix + "map_full_colored.pcd"); // save map full colored
    if( save_octomap_ )
        gt_mapping_->saveOctomap( prefix + "octomap.ot" ); // save octomap
    if( save_structure_pcd_ )
        gt_mapping_->saveStructurePCD( prefix + "structure.pcd"); // save structure cloud

    // Save every individual plane landmark as a pcd file
    if( save_map_full_pcd_ || save_map_full_colored_pcd_ )
    {
        std::string lm_dir = "/home/lizhi/bags/result/"+time_str+"/planes";    // directory
        std::string lm_prefix;
        if( !boost::filesystem::create_directory(lm_dir) )
            lm_prefix = "/home/lizhi/bags/result/"+time_str+"_planes_";
        else
            lm_prefix = "/home/lizhi/bags/result/"+time_str+"/planes/";
        gt_mapping_->saveMapPlanePCD( lm_prefix );
    }

    //
    res.success = true;
    res.message = " Save slam result(landmarks&path, map(simplied, keypoints, planes, full, full colored), octomap, graph) to directory: " + dir + ".";
    cout << GREEN << res.message << RESET << endl;
    return true;
}

bool KinectListener::saveSlamResultCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    std::string time_str = timeToStr(); // appended time string
    std::string dir = "/home/lizhi/bags/result/"+time_str;    // directory
    std::string prefix;
    if( !boost::filesystem::create_directory(dir) )
        prefix = "/home/lizhi/bags/result/"+time_str+"_";
    else
        prefix = "/home/lizhi/bags/result/"+time_str+"/";
    //
    gt_mapping_->updateLandmarksInlierAll();
    //
    saveKeyKrameSequence( prefix + "frames.txt");
    savePlaneLandmarks( prefix + "planes.txt");  // save path and landmarks
    savePathToFile( gt_mapping_->getOptimizedPath(), prefix + "optimized_path.txt");
    savePathToFile( odom_poses_, prefix + "odom_path.txt");
    savePathToFile( visual_odometry_poses_, prefix + "visual_odometry_path.txt");
    savePathToFile( true_poses_, prefix + "true_path.txt");
    saveKeypointLandmarks( prefix + "keypoints.txt");   // save keypoints
    saveRuntimes( prefix + "runtimes.txt" );   // save runtimes
    saveFrameRuntimes( prefix + "frame_runtimes.txt");  // save frame runtimes
    saveMappingRuntimes( prefix + "mapping_runtimes.txt");
    tracker_->saveRuntimes( prefix + "tracking_runtimes.txt" ); // save tracking runtimes

    if( !do_slam_ ){
        //
        res.success = true;
        res.message = " Save slam result(landmarks&path, map(simplied, keypoints, planes, full, full colored), octomap, graph) to directory: " + dir + ".";
        cout << GREEN << res.message << RESET << endl;
        return true;
    }

    gt_mapping_->saveGraphDot( prefix + "graph.dot");      // save grapy
    gt_mapping_->saveMapKeypointPCD( prefix + "map_keypoints.pcd"); // save keypoint cloud
    gt_mapping_->saveMapPCD( prefix + "map.pcd");          // save map
    //
    gt_mapping_->saveMapFullPCD( prefix + "map_full.pcd"); // save map full
    gt_mapping_->saveMapFullColoredPCD( prefix + "map_full_colored.pcd"); // save map full colored
    //
    // Save every individual plane landmark as a pcd file
    std::string lm_dir = "/home/lizhi/bags/result/"+time_str+"/planes";    // directory
    std::string lm_prefix;
    if( !boost::filesystem::create_directory(lm_dir) )
        lm_prefix = "/home/lizhi/bags/result/"+time_str+"_planes_";
    else
        lm_prefix = "/home/lizhi/bags/result/"+time_str+"/planes/";
    gt_mapping_->saveMapPlanePCD( lm_prefix );
    //
    gt_mapping_->saveOctomap( prefix + "octomap.ot" ); // save octomap
    gt_mapping_->saveStructurePCD( prefix + "structure.pcd"); // save structure cloud

    res.success = true;
    res.message = " Save slam result(landmarks&path, map(simplied, keypoints, full, full colored, structure), octomap, graph) to directory: " + dir + ".";
    cout << GREEN << res.message << RESET << endl;
    return true;
}

void KinectListener::publishTfTimerCallback(const ros::TimerEvent &event)
{
    map_tf_mutex_.lock();
    tf::Transform trans = odom_to_map_tf_;
    map_tf_mutex_.unlock();
//    tf::Quaternion quter = trans.getRotation().normalize();
//    trans.setRotation( quter );
    tf_broadcaster_.sendTransform( tf::StampedTransform(trans, ros::Time::now(), map_frame_, odom_frame_ ));
}

} // end of namespace plane_slam
