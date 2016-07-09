#include "kinect_listener.h"

namespace plane_slam
{

KinectListener::KinectListener() :
    private_nh_("~")
  , plane_slam_config_server_( ros::NodeHandle( "PlaneSlam" ) )
  , tf_listener_( nh_, ros::Duration(10.0) )
  , camera_parameters_()
{
    nh_.setCallbackQueue(&my_callback_queue_);

    // reconfigure
    plane_slam_config_callback_ = boost::bind(&KinectListener::planeSlamReconfigCallback, this, _1, _2);
    plane_slam_config_server_.setCallback(plane_slam_config_callback_);

//    private_nh_.param<int>("subscriber_queue_size", subscriber_queue_size_, 4);
//    private_nh_.param<string>("topic_image_visual", topic_image_visual_, "/camera/rgb/image_color");
//    private_nh_.param<string>("topic_image_depth", topic_image_depth_, "/camera/depth_registered/image");
//    private_nh_.param<string>("topic_camera_info", topic_camera_info_, "/camera/rgb/camera_info");
//    private_nh_.param<string>("topic_point_cloud", topic_point_cloud_, "");

    private_nh_.param<int>("subscriber_queue_size", subscriber_queue_size_, 4);
    private_nh_.param<string>("topic_image_visual", topic_image_visual_, "/camera/rgb/image_color");
//    private_nh_.param<string>("topic_image_visual", topic_image_visual_, "");
    private_nh_.param<string>("topic_image_depth", topic_image_depth_, "/camera/depth/image");
    private_nh_.param<string>("topic_camera_info", topic_camera_info_, "/camera/depth/camera_info");
    private_nh_.param<string>("topic_point_cloud", topic_point_cloud_, "");

    //
    orb_extractor_ = new ORBextractor( 1000, 1.2, 8, 20, 7);
    plane_segmentor_ = new LineBasedPlaneSegmentor(nh_);
    viewer_ = new Viewer(nh_);
    tracker_ = new Tracking(nh_);

    // Path
    odometry_path_publisher_ = nh_.advertise<nav_msgs::Path>("odometry_path", 10);
    true_path_publisher_ = nh_.advertise<nav_msgs::Path>("true_path", 10);
    // Pose and Map
    pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("estimate_pose", 10);
    planar_map_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("planar_map", 10);

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
        publishTruePath();

        // store pose
        last_odom_pose = odom_pose;
    }

    // Get camera parameter
    CameraParameters camera;
    getCameraParameter( cam_info_msg, camera);

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
    static Frame last_frame;
    static bool last_frame_valid = false;
    static int skip = 0;
    static tf::Transform last_tf;

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
    Frame frame( visual_image, depth_image, camera_parameters_, orb_extractor_, plane_segmentor_);
    //
    frame_dura = (ros::Time::now() - step_time).toSec() * 1000.0f;
    step_time = ros::Time::now();

    // Tracking
    RESULT_OF_MOTION motion;
    motion.valid = false;
    if( last_frame_valid )  // Do tracking
    {
        tracker_->track( last_frame, frame, motion );

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
            frame.pose_ = last_frame.pose_ * motionToPose3( motion );
            frame.valid = true;
        }
        else    // failed
        {
            cout << RED << "failed to estimated motion." << RESET << endl;
        }
    }

    //
    track_dura = (ros::Time::now() - step_time).toSec() * 1000.0f;
    step_time = ros::Time::now();

    // Publish visual odometry path
    if( !last_frame_valid ) // First frame
    {
        // first frame
        if( !true_poses_.size() )   // No true pose, set first frame pose to identity.
        {
            frame.pose_ = gtsam::Pose3::identity();
            frame.valid = true;
        }
        else    // Valid true pose, set first frame pose to it.
        {
            frame.pose_ = geometryPoseToPose3( true_poses_[true_poses_.size()-1] );
            frame.valid = true;
        }

        if( true_poses_.size() > 0) // First odometry pose to true pose.
        {
            odometry_poses_.clear();
            odometry_poses_.push_back( true_poses_[true_poses_.size()-1]);
            last_tf = geometryPoseToTf( odometry_poses_[0] );
        }
        else    // First odometry pose to identity.
        {
            odometry_poses_.clear();
            odometry_poses_.push_back( pose3ToGeometryPose( gtsam::Pose3::identity() ) );
            last_tf = geometryPoseToTf( tfToGeometryPose( tf::Transform::getIdentity() ) );
        }
    }
    else if( motion.valid ) // not first frame, motion is valid, calculate & publish odometry pose.
    {
        tf::Transform rel_tf = motionToTf( motion );
        rel_tf.setOrigin( tf::Vector3( motion.translation[0], motion.translation[1], motion.translation[2]) );
        tf::Transform new_tf = last_tf * rel_tf;
        odometry_poses_.push_back( tfToGeometryPose(new_tf) );
        publishOdometryPath();
        last_tf = new_tf;
    }
    else    // Not first frame, motion is invalid, return.
    {
        return;
    }

    // Mapping
    if( frame.valid )
    {

    }
    map_dura = (ros::Time::now() - step_time).toSec() * 1000.0f;
    step_time = ros::Time::now();

    // Display frame
    viewer_->removeAll();
    if( last_frame_valid )
        viewer_->displayFrame( last_frame, "last_frame", viewer_->vp1() );
    viewer_->displayFrame( frame, "frame", viewer_->vp2() );
    viewer_->spinOnce();
    //
    display_dura = (ros::Time::now() - step_time).toSec() * 1000.0f;
    step_time = ros::Time::now();

    //
    total_dura = (start_time - step_time).toSec() * 1000.0f;
    // Print time
    cout << GREEN << "Processing total time: " << total_dura << endl;
    cout << "Time:"
         << " frame: " << frame_dura
         << ", tracking: " << track_dura
         << ", mapping: " << map_dura
         << ", display: " << display_dura
         << RESET << endl;

    last_frame = frame;
    last_frame_valid = true;
}

void KinectListener::processFrame( Frame &frame, const tf::Transform &odom_pose )
{
//    static gtsam::Pose3 estimated_pose;
//    static KinectFrame last_frame;
//    static gtsam::Pose3 last_odom_pose;
//    static gtsam::Pose3 visual_odometry_pose;

//    //
//    gtsam::Pose3 odom_pose3 = tfToPose3( odom_pose );

//    // if use downsample cloud
//    cloud_size_type_ = cloud_size_type_config_;
//    if( cloud_size_type_ == QVGA)
//    {
//        cout << GREEN << "QVGA" << RESET << endl;
//        downsampleOrganizedCloud( frame.cloud, camera_parameters_, frame.cloud_in, real_camera_parameters_, (int)QVGA);
//        downsampleImage( frame.visual_image, frame.visual, (int)QVGA );
//    }
//    else if( cloud_size_type_ == QQVGA)
//    {
//        cout << GREEN << "QQVGA" << RESET << endl;
//        downsampleOrganizedCloud( frame.cloud, camera_parameters_, frame.cloud_in, real_camera_parameters_, (int)QQVGA);
//        downsampleImage( frame.visual_image, frame.visual, (int)QQVGA );
//    }
//    else
//    {
//        cout << GREEN << "VGA" << RESET << endl;
//        frame.cloud_in = frame.cloud; // copy pointer
//        frame.visual = frame.visual_image;
//        real_camera_parameters_ = camera_parameters_;
//    }

//    double start_time = pcl::getTime();
//    pcl::console::TicToc time;
//    time.tic();
//    float segment_dura = 0;
//    float keypoint_dura = 0;
//    float solveRT_dura = 0;
//    float slam_dura = 0;
//    float display_dura = 0;
//    float total_dura = 0;

//    // Plane Segment
//    if( plane_segment_method_ == ORGANSIZED)
//    {
//        organizedPlaneSegment( frame.cloud_in, frame.segment_planes );
//        cout << GREEN << "Organized segmentation, planes = " << frame.segment_planes.size() << RESET << endl;
//    }
//    else if( plane_segment_method_ == LINE_BADED )
//    {
//        lineBasedPlaneSegment( frame.cloud_in, frame.segment_planes );
//        cout << GREEN << "Line based segmentation, planes = " << frame.segment_planes.size() << RESET << endl;
//    }
//    else
//    {
//        cout << RED << "[Error]: Invalid segmentation method error." << RESET << endl;
//        exit(0);
//    }
//    segment_dura = time.toc();
//    time.tic();

//    // Compute Keypoints
//    cout << GREEN << feature_detector_type_ << "-" << feature_extractor_type_ << RESET << endl;
//    computeORBKeypoint( frame.visual_image, frame.cloud, frame.feature_locations_2d,
//                     frame.feature_locations_3d, frame.feature_cloud, frame.feature_descriptors );
////    displayKeypoint( frame.visual_image, frame.feature_locations_2d );
//    keypoint_dura = time.toc();
//    time.tic();

//    //
//    std::vector<PlaneType> landmarks;
//    if( do_visual_odometry_ ) // Do visual odometry
//    {
//        cout << GREEN << "Estimate visual odometry." << RESET << endl;

//        if( !is_initialized )
//        {
//            // Initialize visual odometry pose
//            visual_odometry_pose = odom_pose3;
//            is_initialized = true;

//            // first pose
//            odometry_poses_.clear();
//            odometry_poses_.push_back( pose3ToGeometryPose(visual_odometry_pose) );
//            publishOdometryPath();

//            // first true pose
//            true_poses_.clear();
//            true_poses_.push_back( pose3ToGeometryPose(odom_pose3) );
//            publishTruePath();
//        }
//        else
//        {
//            // Estimate motion
//            RESULT_OF_MOTION motion;
//            std::vector<cv::DMatch> inlier;
//            time.tic();
//            motion.valid = solveRelativeTransform( last_frame, frame, motion, inlier );
//            solveRT_dura = time.toc();
//            time.tic();

//            //
//            if( !motion.valid )
//                return;

//            // New visual odometry pose
//            gtsam::Pose3 rel( gtsam::Rot3(motion.rotation), gtsam::Point3(motion.translation) );
//            gtsam::Pose3 o_pose = visual_odometry_pose * rel;
//            visual_odometry_pose = o_pose;

//            // Publish visual odometry pose
//            odometry_poses_.push_back( pose3ToGeometryPose(visual_odometry_pose) );
//            publishOdometryPath();

//            // Publish true path
//            true_poses_.push_back( tfToGeometryPose(odom_pose) );
//            publishTruePath();
//        }

//    }
//    else if( do_mapping_ )
//    {

//        if( !is_initialized ) // First frame, initialize slam system
//        {
//            if( plane_slam_->initialize( odom_pose3, frame ) )
//            {
//                is_initialized = true;
//                last_odom_pose = odom_pose3; // store odom pose
//                last_frame = frame; // store current frame
//                cout << GREEN << "Initialized Planar SLAM." << RESET << endl;

//                // True pose
//                true_poses_.clear();
//                true_poses_.push_back( pose3ToGeometryPose( odom_pose3 ) );
//                publishTruePath();

//                // Initialize visual odometry pose
//                visual_odometry_pose = odom_pose3;
//                odometry_poses_.clear();
//                odometry_poses_.push_back( pose3ToGeometryPose(visual_odometry_pose) );
//                publishOdometryPath();
//            }
//            else
//                return;

//        }
//        else
//        {
//            // Estimate motion
//            RESULT_OF_MOTION motion;
//            std::vector<cv::DMatch> inlier;
//            time.tic();
//            motion.valid = solveRelativeTransform( last_frame, frame, motion, inlier );
//            solveRT_dura = time.toc();
//            time.tic();

//            // Print motion info
//            if( motion.valid )
//            {
//                // print motion
//                gtsam::Rot3 rot3( motion.rotation );
//                cout << MAGENTA << " estimated motion, rmse = " << motion.rmse << endl;
//                cout << "  - R(rpy): " << rot3.roll()
//                     << ", " << rot3.pitch()
//                     << ", " << rot3.yaw() << endl;
//                cout << "  - T:      " << motion.translation[0]
//                     << ", " << motion.translation[1]
//                     << ", " << motion.translation[2] << RESET << endl;

//                gtsam::Pose3 real_r_pose = last_odom_pose.inverse() * odom_pose3;
//                cout << CYAN << " true motion: " << endl;
//                cout << "  - R(rpy): " << real_r_pose.rotation().roll()
//                     << ", " << real_r_pose.rotation().pitch()
//                     << ", " << real_r_pose.rotation().yaw() << endl;
//                cout << "  - T:      " << real_r_pose.translation().x()
//                     << ", " << real_r_pose.translation().y()
//                     << ", " << real_r_pose.translation().z() << RESET << endl;
//            }
//            else
//            {
//                cout << RED << "Failed to estimate relative motion, exit. " << RESET << endl;
//                return;
////                cout << RED << "Use Identity as transformation." << RESET << endl;
////                motion.setTransform4d( Eigen::Matrix4d::Identity() );
//            }



//            // Iteration
//            gtsam::Pose3 rel( gtsam::Rot3(motion.rotation), gtsam::Point3(motion.translation) );
//            gtsam::Pose3 pose3 = visual_odometry_pose * rel;
//            visual_odometry_pose = pose3;
//            if( use_keyframe_ )
//            {
//                if( rel.translation().norm() >= keyframe_linear_threshold_
//                        || acos( cos(rel.rotation().yaw()) * cos(rel.rotation().pitch()) * cos(rel.rotation().roll()) ) > keyframe_angular_threshold_ )
//                {
//                    estimated_pose = plane_slam_->planeMapping( pose3, frame );
//                    // visualize landmark
//                    landmarks = plane_slam_->getLandmarks();
//                }
//                else
//                {
//                    return;
//                }
//            }
//            else
//            {
//                if( motion.valid && frame.segment_planes.size() > 0)    // do slam
//                {
//                    estimated_pose = plane_slam_->planeMapping( pose3, frame );
//                    // visualize landmark
//                    landmarks = plane_slam_->getLandmarks();
//                }
//                else if( motion.valid && frame.segment_planes.size() == 0) // accumulate estimated pose
//                {
//                    estimated_pose = pose3;
//                }
//                else
//                {
//                    cout << RED << "[Error]: " << "Motion estimation failed, stop processing current frame." << RESET << endl;
//                    return;
//                }
//            }
//            // Publish pose and trajectory
//            publishPose( pose3 );

//            // Publish true path
//            true_poses_.push_back( pose3ToGeometryPose(odom_pose3) );
//            publishTruePath();

//            // Publish visual odometry path
//            odometry_poses_.push_back( pose3ToGeometryPose(visual_odometry_pose) );
//            publishOdometryPath();

//        }
//    }
//    else if( do_slam_ ) // Do slam
//    {

////        // convert ax + by + cz-d = 0
////        std::vector<PlaneType> &planes = frame.segment_planes;
////        for( int i = 0; i < planes.size(); i++)
////        {
////            planes[i].coefficients[3] = -planes[i].coefficients[3];
////        }


//        if( !is_initialized ) // First frame, initialize slam system
//        {
//            if( plane_slam_->initialize( odom_pose3, frame ) )
//            {
//                is_initialized = true;
//                estimated_pose = odom_pose3; // store pose as initial pose
//                last_odom_pose = odom_pose3; // store odom pose
//                last_frame = frame; // store current frame
//                cout << GREEN << "Initialized Planar SLAM." << RESET << endl;

//                // True pose
//                true_poses_.clear();
//                true_poses_.push_back( pose3ToGeometryPose( odom_pose3 ) );
//                publishTruePath();

//                // Initialize visual odometry pose
//                visual_odometry_pose = odom_pose3;
//                odometry_poses_.clear();
//                odometry_poses_.push_back( pose3ToGeometryPose(visual_odometry_pose) );
//                publishOdometryPath();
//            }
//            else
//                return;

//        }
//        else
//        {
//            // Estimate motion
//            RESULT_OF_MOTION motion;
//            std::vector<cv::DMatch> inlier;
//            time.tic();
//            motion.valid = solveRelativeTransform( last_frame, frame, motion, inlier );
//            solveRT_dura = time.toc();
//            time.tic();

//            // Print motion info
//            if( motion.valid )
//            {
//                // print motion
//                gtsam::Rot3 rot3( motion.rotation );
//                cout << MAGENTA << " estimated motion, rmse = " << motion.rmse << endl;
//                cout << "  - R(rpy): " << rot3.roll()
//                     << ", " << rot3.pitch()
//                     << ", " << rot3.yaw() << endl;
//                cout << "  - T:      " << motion.translation[0]
//                     << ", " << motion.translation[1]
//                     << ", " << motion.translation[2] << RESET << endl;

//                gtsam::Pose3 real_r_pose = last_odom_pose.inverse() * odom_pose3;
//                cout << CYAN << " true motion: " << endl;
//                cout << "  - R(rpy): " << real_r_pose.rotation().roll()
//                     << ", " << real_r_pose.rotation().pitch()
//                     << ", " << real_r_pose.rotation().yaw() << endl;
//                cout << "  - T:      " << real_r_pose.translation().x()
//                     << ", " << real_r_pose.translation().y()
//                     << ", " << real_r_pose.translation().z() << RESET << endl;
//            }
//            else
//            {
//                cout << RED << "Failed to estimate relative motion, exit. " << RESET << endl;
//                return;
////                cout << RED << "Use Identity as transformation." << RESET << endl;
////                motion.setTransform4d( Eigen::Matrix4d::Identity() );
//            }



//            // Slam iteration
//            gtsam::Pose3 rel( gtsam::Rot3(motion.rotation), gtsam::Point3(motion.translation) );
//            gtsam::Pose3 pose3 = estimated_pose * rel;
//            if( use_keyframe_ )
//            {
//                if( rel.translation().norm() >= keyframe_linear_threshold_
//                        || acos( cos(rel.rotation().yaw()) * cos(rel.rotation().pitch()) * cos(rel.rotation().roll()) ) > keyframe_angular_threshold_ )
//                {
//                    estimated_pose = plane_slam_->planeSlam( pose3, frame );
//                    // visualize landmark
//                    landmarks = plane_slam_->getLandmarks();
//                }
//                else
//                {
//                    return;
//                }
//            }
//            else
//            {
//                if( motion.valid && frame.segment_planes.size() > 0)    // do slam
//                {
//                    estimated_pose = plane_slam_->planeSlam( pose3, frame );
//                    // visualize landmark
//                    landmarks = plane_slam_->getLandmarks();
//                }
//                else if( motion.valid && frame.segment_planes.size() == 0) // accumulate estimated pose
//                {
//                    estimated_pose = pose3;
//                }
//                else
//                {
//                    cout << RED << "[Error]: " << "Motion estimation failed, stop processing current frame." << RESET << endl;
//                    return;
//                }
//            }
//            // Publish pose and trajectory
//            publishPose( estimated_pose );

//            // Publish true path
//            true_poses_.push_back( tfToGeometryPose(odom_pose) );
//            publishTruePath();

//            // Publish visual odometry path
//            gtsam::Pose3 o_pose = visual_odometry_pose * rel;
//            visual_odometry_pose = o_pose;
//            odometry_poses_.push_back( pose3ToGeometryPose(visual_odometry_pose) );
//            publishOdometryPath();

//            // Publish estimated path
//            if( display_path_ )
//                plane_slam_->publishEstimatedPath();
////            if( display_odom_path_ )
////                plane_slam_->publishOdomPath();


//        }

////        // convert ax + by + cz - d = 0
////        for( int i = 0; i < landmarks.size(); i++)
////        {
////            landmarks[i].coefficients[3] = -landmarks[i].coefficients[3];
////        }
////        for( int i = 0; i < planes.size(); i++)
////        {
////            planes[i].coefficients[3] = -planes[i].coefficients[3];
////        }
//    }

//    slam_dura = time.toc();
//    time.tic();


//    // Display map
//    if(display_landmarks_ && landmarks.size() > 0)
//    {
//        // Clear map display
//        map_viewer_->removeAllPointClouds();
//        map_viewer_->removeAllShapes();
//        displayLandmarks( landmarks, "landmark");
//        publishPlanarMap( landmarks );
//        map_viewer_->spinOnce(1);
//    }

//    // Clear Display
//    pcl_viewer_->removeAllPointClouds();
//    pcl_viewer_->removeAllShapes();

//    // display
//    if( display_input_cloud_ )
//    {
////        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> rgba_color(frame_current.point_cloud, 255, 255, 255);
//        pcl_viewer_->addPointCloud( frame.cloud_in, "rgba_cloud", viewer_v1_ );
//        pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "rgba_cloud", viewer_v1_);
//    }
//    displayPlanes( last_frame.cloud_in, last_frame.segment_planes, "last_planes", viewer_v2_ );
//    displayPlanes( frame.cloud_in, frame.segment_planes, "planes", viewer_v4_ );
//    pcl_viewer_->addText( "Last Frame", 200, 20, "viewer_v2_name", viewer_v2_);
//    pcl_viewer_->addText( "Current Frame", 200, 20, "viewer_v4_name", viewer_v4_);
//    //
////    display3DKeypoint( frame.feature_locations_3d, "current", viewer_v1_ );
////    display3DKeypoint( last_frame.feature_locations_3d, "last", viewer_v3_ );
////    displayMatched3DKeypoint( last_frame.feature_locations_3d, frame.feature_locations_3d, good_matches );
//    pcl_viewer_->addText( "Last 3D Features", 200, 20, "viewer_v1_name", viewer_v1_);
//    pcl_viewer_->addText( "Current 3D Features", 200, 20, "viewer_v3_name", viewer_v3_);
//    pcl_viewer_->spinOnce(1);


//    display_dura = time.toc();
//    total_dura = (pcl::getTime() - start_time) * 1000;

//    cout << GREEN << "Total time: " << total_dura << ", segment: " << segment_dura
//         << ", keypoints: " << keypoint_dura << ", solveRT: " << solveRT_dura
//         << ", slam: " << slam_dura << ", display: " << display_dura << RESET << endl;
//    cout << "----------------------------------- END -------------------------------------" << endl;

//    last_frame = frame; // store frame
//    last_odom_pose = odom_pose3; // store odom pose
}

void KinectListener::planeSlamReconfigCallback(plane_slam::PlaneSlamConfig &config, uint32_t level)
{
    do_visual_odometry_ = config.do_visual_odometry;
    do_mapping_ = config.do_mapping;
    do_slam_ = config.do_slam;
    map_frame_ = config.map_frame;
    base_frame_ = config.base_frame;
    odom_frame_ = config.odom_frame;
    skip_message_ = config.skip_message;
    //
    cout << GREEN <<"PlaneSlam Config." << RESET << endl;
}

void KinectListener::publishPose( gtsam::Pose3 &pose)
{
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "/world";
    msg.header.stamp = ros::Time::now();
    msg.pose.position.x = pose.x();
    msg.pose.position.y = pose.y();
    msg.pose.position.z = pose.z();
    gtsam::Vector3 rpy = pose.rotation().rpy();
    msg.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw( rpy[0], rpy[1], rpy[2] );

    pose_publisher_.publish( msg );
}

void KinectListener::publishPlanarMap( const std::vector<PlaneType> &landmarks)
{
    PointCloudTypePtr cloud ( new PointCloudType );

    for( int i = 0; i < landmarks.size(); i++)
    {
        const PlaneType &lm = landmarks[i];
        if( !lm.valid )
            continue;
       *cloud += *lm.cloud;
    }

    sensor_msgs::PointCloud2 cloud2;
    pcl::toROSMsg( *cloud, cloud2);
    cloud2.header.frame_id = "/world";
    cloud2.header.stamp = ros::Time::now();
    cloud2.is_dense = false;

    planar_map_publisher_.publish( cloud2 );
}



void KinectListener::getCameraParameter( const sensor_msgs::CameraInfoConstPtr &cam_info_msg,
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


//    // TUM3
//    camera.cx = 320.1;
//    camera.cy = 247.6;
//    camera.fx = 535.4;
//    camera.fy = 539.2;
//    //
//    camera.scale = 1.0;
//    camera.width = 640;
//    camera.height = 480;

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
        return false;
    }
    odom_pose.setOrigin( trans.getOrigin() );
    odom_pose.setRotation( trans.getRotation() );

    return true;
}

void KinectListener::publishOdometryPath()
{
    nav_msgs::Path path;
    path.header.frame_id = "/world";
    path.header.stamp = ros::Time::now();
    path.poses = odometry_poses_;
    odometry_path_publisher_.publish( path );
    cout << GREEN << "Publish odometry path, p = " << odometry_poses_.size() << RESET << endl;
}


void KinectListener::publishTruePath()
{
    nav_msgs::Path path;
    path.header.frame_id = "/world";
    path.header.stamp = ros::Time::now();
    path.poses = true_poses_;
    true_path_publisher_.publish( path );
    cout << GREEN << "Publish true path, p = " << true_poses_.size() << RESET << endl;
}

} // end of namespace plane_slam
