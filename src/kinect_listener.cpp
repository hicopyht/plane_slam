#include "kinect_listener.h"

namespace plane_slam
{

KinectListener::KinectListener() :
    private_nh_("~")
  , plane_slam_config_server_( ros::NodeHandle( "PlaneSlam" ) )
  , tf_listener_( nh_, ros::Duration(10.0) )
  , camera_parameters_()
  , set_init_pose_( false )
{
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
    orb_extractor_ = new ORBextractor( 1000, 1.2, 8, 20, 7);
    plane_segmentor_ = new LineBasedPlaneSegmentor(nh_);
    viewer_ = new Viewer(nh_);
    tracker_ = new Tracking(nh_, viewer_ );
    gt_mapping_ = new GTMapping(nh_, viewer_);

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
    odometry_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("odometry_pose", 10);
    odometry_path_publisher_ = nh_.advertise<nav_msgs::Path>("odometry_path", 10);
    save_path_landmarks_service_server_ = nh_.advertiseService("save_path_landmarks", &KinectListener::savePathLandmarksCallback, this );
    save_slam_result_all_ = nh_.advertiseService("save_slam_result_all", &KinectListener::saveSlamResultCallback, this );

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
    static tf::Transform last_odom_pose = tf::Transform::getIdentity();

    cout << RESET << "----------------------------------------------------------------------" << endl;
    // Get odom pose
    tf::Transform odom_pose;
    if( getOdomPose( odom_pose, depth_img_msg->header.frame_id, ros::Time(0) ) )
    {
        // Relative transform
        tf::Transform rel_tf = last_odom_pose.inverse() * odom_pose;
        gtsam::Pose3 real_r_pose = tfToPose3( rel_tf );

        cout << CYAN << " true motion: " << endl;
        cout << "  - R(rpy): " << real_r_pose.rotation().roll()
             << ", " << real_r_pose.rotation().pitch()
             << ", " << real_r_pose.rotation().yaw() << endl;
        cout << "  - T:      " << real_r_pose.translation().x()
             << ", " << real_r_pose.translation().y()
             << ", " << real_r_pose.translation().z() << RESET << endl;

        // Publish true path
        true_poses_.push_back( tfToGeometryPose(odom_pose) );
        publishTruePose();
        publishTruePath();

        // store pose
        last_odom_pose = odom_pose;
    }
    else{
        if(force_odom_)
            return;
    }

    // Get camera parameter
    CameraParameters camera;
    cvtCameraParameter( cam_info_msg, camera);

    trackDepthRgbImage( visual_img_msg, depth_img_msg, camera );

}

void KinectListener::cloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                                const sensor_msgs::PointCloud2ConstPtr& point_cloud,
                                const sensor_msgs::CameraInfoConstPtr& cam_info_msg)
{
    printf("cloud msg: %d\n", point_cloud->header.seq);

}

void KinectListener::trackDepthRgbImage( const sensor_msgs::ImageConstPtr &visual_img_msg,
                                         const sensor_msgs::ImageConstPtr &depth_img_msg,
                                         CameraParameters & camera)
{
    static Frame *last_frame;
    static bool last_frame_valid = false;
    static int skip = 0;
    static tf::Transform last_tf;

    cout << RESET << "----------------------------------------------------------------------" << endl;
    cout << BOLDMAGENTA << "no cloud msg: " << depth_img_msg->header.seq << RESET << endl;

    skip = (skip + 1) % skip_message_;
    if( skip )
    {
        cout << BLUE << " Skip message." << RESET << endl;
        return;
    }

    camera_parameters_ = camera;

    // Get Mat Image
    cv::Mat visual_image = cv_bridge::toCvCopy(visual_img_msg)->image; // to cv image
    cv::Mat depth_image = cv_bridge::toCvCopy(depth_img_msg)->image; // to cv image

    const ros::Time start_time = ros::Time::now();
    ros::Time step_time = start_time;
    double frame_dura, track_dura, map_dura, display_dura;
    double total_dura;

    // Compute Frame
    Frame *frame = new Frame( visual_image, depth_image, camera_parameters_, orb_extractor_, plane_segmentor_);
    frame->stamp_ = visual_img_msg->header.stamp;
    frame->valid_ = false;
    //
    frame_dura = (ros::Time::now() - step_time).toSec() * 1000.0f;
    step_time = ros::Time::now();

    // Tracking
    RESULT_OF_MOTION motion;
    motion.valid = false;
    if( last_frame_valid )  // Do tracking
    {
        tracker_->track( *last_frame, *frame, motion );

        // print motion
        if( motion.valid )  // success, print tracking result
        {
            gtsam::Rot3 rot3( motion.rotation );
            cout << MAGENTA << " estimated motion, rmse = " << motion.rmse << endl;
            cout << "  - R(rpy): " << rot3.roll()
                 << ", " << rot3.pitch()
                 << ", " << rot3.yaw() << endl;
            cout << "  - T:      " << motion.translation[0]
                 << ", " << motion.translation[1]
                 << ", " << motion.translation[2] << RESET << endl;

            // estimated pose
            frame->pose_ = last_frame->pose_ * motionToTf( motion );
            frame->valid_ = true;
        }
        else    // failed
        {
            frame->valid_ = false;
            cout << RED << "failed to estimated motion." << RESET << endl;
        }
    }
    else
    {
        if( frame->segment_planes_.size() > 0)
            frame->valid_ = true;   // first frame, set valid, add to mapper as first frame
    }

    //
    track_dura = (ros::Time::now() - step_time).toSec() * 1000.0f;
    step_time = ros::Time::now();

    // Publish visual odometry path
    if( !last_frame_valid && frame->valid_ ) // First frame, valid, set initial pose
    {
        // first frame
        if( !true_poses_.size() )   // No true pose, set first frame pose to identity.
        {
            frame->pose_ = init_pose_;
            frame->valid_ = true;
        }
        else    // Valid true pose, set first frame pose to it.
        {
            frame->pose_ = geometryPoseToTf( true_poses_[true_poses_.size()-1] );
            frame->valid_ = true;
        }

        if( true_poses_.size() > 0) // First odometry pose to true pose.
        {
            odometry_poses_.clear();
            odometry_poses_.push_back( true_poses_[true_poses_.size()-1]);
            last_tf = geometryPoseToTf( odometry_poses_[0] );
        }
        else if( set_init_pose_ )    // First odometry pose to identity.
        {
            odometry_poses_.clear();
            odometry_poses_.push_back( tfToGeometryPose( init_pose_ ) );
            last_tf = geometryPoseToTf( odometry_poses_[0] );
        }
        else
        {
            return;
        }
    }
    else if( motion.valid ) // not first frame, motion is valid, calculate & publish odometry pose.
    {
        tf::Transform rel_tf = motionToTf( motion );
        rel_tf.setOrigin( tf::Vector3( motion.translation[0], motion.translation[1], motion.translation[2]) );
        tf::Transform new_tf = last_tf * rel_tf;
        odometry_poses_.push_back( tfToGeometryPose(new_tf) );
        publishOdometryPose();
        publishOdometryPath();
        last_tf = new_tf;
    }
//    else    // Not first frame, motion is invalid, return.
//    {
//        return;
//    }

    // Mapping
    if( frame->valid_ )
    {
        frame->key_frame_ = gt_mapping_->mapping( frame );
    }
    map_dura = (ros::Time::now() - step_time).toSec() * 1000.0f;
    step_time = ros::Time::now();

    // Map for visualization
    if( frame->key_frame_)
        gt_mapping_->updateMapViewer();

    // Display frame
    viewer_->removeFrames();
    if( last_frame_valid )
        viewer_->displayFrame( *last_frame, "last_frame", viewer_->vp1() );
    if( frame->valid_ )
        viewer_->displayFrame( *frame, "frame", viewer_->vp2() );
    viewer_->spinFramesOnce();
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
        runtimes_.push_back( Runtime(true, frame_dura, track_dura, map_dura, total) );
        cout << GREEN << " Runtimes size: " << runtimes_.size() << RESET << endl;
    }

    if( frame->valid_ )
    {
        if( !last_frame_valid )
        {
            last_frame_valid = true;
        }
        else if( !last_frame->key_frame_ )
        {
            delete last_frame;  // not keyframe, delete data
        }

        last_frame = frame;
    }
}

void KinectListener::savePathAndLandmarks( const std::string &filename )
{
    FILE* yaml = std::fopen( filename.c_str(), "w" );
    fprintf( yaml, "# landmarks and path file: %s\n", filename.c_str() );
    fprintf( yaml, "# landmarks format: ax+by+cz+d = 0, (n,d), (a,b,c,d)\n");
    fprintf( yaml, "# pose format: T(xyz) Q(xyzw)\n\n" );

    // Save Landmarks
    std::map<int, PlaneType*> landmarks = gt_mapping_->getLandmark();
    fprintf( yaml, "# landmarks, size %d\n", landmarks.size() );
    for( std::map<int, PlaneType*>::const_iterator it = landmarks.begin();
         it != landmarks.end(); it++)
    {
        PlaneType *lm = it->second;
        if( !lm->valid )
            continue;
        fprintf( yaml, "%f %f %f %f\n", lm->coefficients[0], lm->coefficients[1],
                lm->coefficients[2], lm->coefficients[3] );
    }
    fprintf( yaml, "\n\n");

    // Save Optimized Path
    std::vector<geometry_msgs::PoseStamped> optimized_poses = gt_mapping_->getOptimizedPath();
    fprintf( yaml, "# optimized path, size %d\n", optimized_poses.size() );
    for( int i = 0; i < optimized_poses.size(); i++)
    {
        geometry_msgs::PoseStamped &pose = optimized_poses[i];
        fprintf( yaml, "%f %f %f %f %f %f %f\n", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
                 pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w );

    }
    fprintf( yaml, "\n\n");

    // Save True Path
    fprintf( yaml, "# true path, size %d\n", true_poses_.size() );
    for( int i = 0; i < true_poses_.size(); i++)
    {
        geometry_msgs::PoseStamped &pose = true_poses_[i];
        fprintf( yaml, "%f %f %f %f %f %f %f\n", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
                 pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w );
    }
    fprintf( yaml, "\n\n");

    // Save Odometry Path
    fprintf( yaml, "# odometry path, size %d\n", odometry_poses_.size() );
    for( int i = 0; i < odometry_poses_.size(); i++)
    {
        geometry_msgs::PoseStamped &pose = odometry_poses_[i];
        fprintf( yaml, "%f %f %f %f %f %f %f\n", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
                 pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w );
    }
    fprintf( yaml, "\n\n");

    // close
    fclose(yaml);
}

void KinectListener::saveRuntimes( const std::string &filename )
{
    FILE* yaml = std::fopen( filename.c_str(), "w" );
    fprintf( yaml, "# runtimes file: %s\n", filename.c_str() );
    fprintf( yaml, "# format: frame tracking mapping total\n");
    fprintf( yaml, "# key frame size: %d\n\n", runtimes_.size());

    Runtime avg_time;
    Runtime max_time(true, 0, 0, 0, 0);
    Runtime min_time( true, 1e6, 1e6, 1e6, 1e6);
    int count = 0;
    for( int i = 0; i < runtimes_.size(); i++)
    {
        Runtime &runtime = runtimes_[i];
        if( !runtime.key_frame )
            continue;
        fprintf( yaml, "%f %f %f %f\n", runtime.frame, runtime.tracking, runtime.mapping, runtime.total);
        avg_time += runtime;
        count ++;
        //
        max_time.getMaximum( runtime );
        min_time.getMinimum( runtime );
    }
    fprintf( yaml, "\n");

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

    // Maximum
    fprintf( yaml, "# maximum time:\n");
    fprintf( yaml, "%f %f %f %f\n", max_time.frame,
             max_time.tracking, max_time.mapping, max_time.total);

    // Minimum
    fprintf( yaml, "# minimum time:\n");
    fprintf( yaml, "%f %f %f %f\n", min_time.frame,
             min_time.tracking, min_time.mapping, min_time.total);

    //
    Runtime error_time;
    error_time = max_time;
    error_time += min_time;
    error_time /= 2.0;
    fprintf( yaml, "# (max+min)/2.0 time:\n");
    fprintf( yaml, "%f %f %f %f\n", error_time.frame,
             error_time.tracking, error_time.mapping, error_time.total);
    //
    error_time = max_time;
    error_time -= min_time;
    error_time /= 2.0;
    fprintf( yaml, "# (max-min)/2.0 time:\n");
    fprintf( yaml, "%f %f %f %f\n", error_time.frame,
             error_time.tracking, error_time.mapping, error_time.total);
    //
    error_time = max_time;
    error_time -= avg_time;
    fprintf( yaml, "# (max-avg) time:\n");
    fprintf( yaml, "%f %f %f %f\n", error_time.frame,
             error_time.tracking, error_time.mapping, error_time.total);
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


    // TUM3
    cout << YELLOW << " Use TUM3 camera parameters." << RESET << endl;
    camera.cx = 320.1;
    camera.cy = 247.6;
    camera.fx = 535.4;
    camera.fy = 539.2;
    //
    camera.scale = 1.0;
    camera.width = 640;
    camera.height = 480;

}

bool KinectListener::getOdomPose( tf::Transform &odom_pose, const std::string &camera_frame, const ros::Time &time)
{
    // get transform
    tf::StampedTransform trans;
    try{
        tf_listener_.lookupTransform(odom_frame_, camera_frame, time, trans);
    }catch (tf::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
        odom_pose.setIdentity();
        return false;
    }
    odom_pose.setOrigin( trans.getOrigin() );
    odom_pose.setRotation( trans.getRotation() );

    return true;
}

void KinectListener::publishTruePose()
{
    geometry_msgs::PoseStamped msg = true_poses_.back();
    msg.header.frame_id = map_frame_;
    msg.header.stamp = ros::Time::now();
    true_pose_publisher_.publish( msg );
}

void KinectListener::publishTruePath()
{
    nav_msgs::Path path;
    path.header.frame_id = map_frame_;
    path.header.stamp = ros::Time::now();
    path.poses = true_poses_;
    true_path_publisher_.publish( path );
    cout << GREEN << " Publish true path, p = " << true_poses_.size() << RESET << endl;
}

void KinectListener::publishOdometryPose()
{
    geometry_msgs::PoseStamped msg = odometry_poses_.back();
    msg.header.frame_id = map_frame_;
    msg.header.stamp = ros::Time::now();
    odometry_pose_publisher_.publish( msg );
}

void KinectListener::publishOdometryPath()
{
    nav_msgs::Path path;
    path.header.frame_id = map_frame_;
    path.header.stamp = ros::Time::now();
    path.poses = odometry_poses_;
    odometry_path_publisher_.publish( path );
    cout << GREEN << " Publish odometry path, p = " << odometry_poses_.size() << RESET << endl;
}

void KinectListener::planeSlamReconfigCallback(plane_slam::PlaneSlamConfig &config, uint32_t level)
{
    do_visual_odometry_ = config.do_visual_odometry;
    do_mapping_ = config.do_mapping;
    do_slam_ = config.do_slam;
    force_odom_ = config.force_odom;
    map_frame_ = config.map_frame;
    base_frame_ = config.base_frame;
    odom_frame_ = config.odom_frame;
    skip_message_ = config.skip_message;
    set_init_pose_ = config.set_init_pose_ || set_init_pose_;

    // Set map frame for mapping
    if( gt_mapping_ )
        gt_mapping_->setMapFrame( map_frame_ );
    //
    cout << GREEN <<" PlaneSlam Config." << RESET << endl;
}

bool KinectListener::savePathLandmarksCallback( std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res )
{
    std::string filename = "/home/lizhi/bags/result/" + timeToStr() + "_landmarks_path.txt";
    savePathAndLandmarks( filename );
    res.success = true;
    res.message = " Save landmarks and path file: " + filename + ".";
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
    savePathAndLandmarks( prefix + "landmarks_path.txt");  // save path and landmarks
    saveRuntimes( prefix + "runtimes.txt" );   // save runtimes
    gt_mapping_->saveGraphDot( prefix + "graph.dot");      // save grapy
    gt_mapping_->saveMapPCD( prefix + "map.pcd");          // save map
    gt_mapping_->saveMapFullPCD( prefix + "map_full.pcd"); // save map full
    gt_mapping_->saveMapFullColoredPCD( prefix + "map_full_colored.pcd"); // save map full colored
    gt_mapping_->saveStructurePCD( prefix + "structure.pcd"); // save structure cloud
    res.success = true;
    res.message = " Save slam result(landmarks&path, map(simplied, full, full colored, structure), graph) to directory: " + dir + ".";
    cout << GREEN << res.message << RESET << endl;
    return true;
}


} // end of namespace plane_slam
