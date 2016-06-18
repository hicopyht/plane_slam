#include "kinect_listener.h"


const std::string KeypointWindow = "Keypoint";
const std::string MatchesWindow = "MatchesFeatures";

KinectListener::KinectListener() :
    private_nh_("~")
  , plane_slam_config_server_( ros::NodeHandle( "PlaneSlam" ) )
  , plane_segment_config_server_( ros::NodeHandle( "PlaneSegment" ) )
  , organized_segment_config_server_( ros::NodeHandle( "OrganizedSegment" ) )
  , line_based_segment_config_server_( ros::NodeHandle( "LineBasedSegment" ) )
  , tf_listener_( nh_, ros::Duration(10.0) )
  , pcl_viewer_( new pcl::visualization::PCLVisualizer("3D Viewer"))
  , map_viewer_( new pcl::visualization::PCLVisualizer("Map Viewer"))
  , viewer_v1_(1)
  , viewer_v2_(2)
  , viewer_v3_(3)
  , viewer_v4_(4)
  , rng(12345)
  , prttcp_ (new pcl::DefaultPointRepresentation<PointType>)
  , camera_parameters_()
  , organized_plane_segment_()
  , is_initialized( false )
  , plane_slam_( new PlaneSlam() )
{
    nh_.setCallbackQueue(&my_callback_queue_);

    // reconfigure
    bool use_reconfigure;
    private_nh_.param<bool>("use_reconfigure", use_reconfigure, true);
    if(use_reconfigure)
    {
        plane_slam_config_callback_ = boost::bind(&KinectListener::planeSlamReconfigCallback, this, _1, _2);
        plane_slam_config_server_.setCallback(plane_slam_config_callback_);
        plane_segment_config_callback_ = boost::bind(&KinectListener::planeSegmentReconfigCallback, this, _1, _2);
        plane_segment_config_server_.setCallback(plane_segment_config_callback_);
        organized_segment_config_callback_ = boost::bind(&KinectListener::organizedSegmentReconfigCallback, this, _1, _2);
        organized_segment_config_server_.setCallback(organized_segment_config_callback_);
        line_based_segment_config_callback_ = boost::bind(&KinectListener::lineBasedSegmentReconfigCallback, this, _1, _2);
        line_based_segment_config_server_.setCallback(line_based_segment_config_callback_);
    }

//    private_nh_.param<int>("subscriber_queue_size", subscriber_queue_size_, 4);
//    private_nh_.param<string>("topic_image_visual", topic_image_visual_, "");
//    private_nh_.param<string>("topic_image_depth", topic_image_depth_, "/camera/depth_registered/image");
//    private_nh_.param<string>("topic_camera_info", topic_camera_info_, "/camera/depth_registered/camera_info");
//    private_nh_.param<string>("topic_point_cloud", topic_point_cloud_, "");

    private_nh_.param<int>("subscriber_queue_size", subscriber_queue_size_, 4);
    private_nh_.param<string>("topic_image_visual", topic_image_visual_, "/camera/rgb/image_color");
    private_nh_.param<string>("topic_image_depth", topic_image_depth_, "/camera/depth/image");
    private_nh_.param<string>("topic_camera_info", topic_camera_info_, "/camera/depth/camera_info");
    private_nh_.param<string>("topic_point_cloud", topic_point_cloud_, "");


    pcl_viewer_->createViewPort(0, 0.5, 0.5, 1.0, viewer_v1_);
    pcl_viewer_->addText("RansacPlanes", 100, 3, "v3_text", viewer_v1_);
    pcl_viewer_->createViewPort(0.5, 0.5, 1.0, 1.0, viewer_v2_);
    pcl_viewer_->addText("OrganizedPlanes", 100, 3, "v4_text", viewer_v2_);
    pcl_viewer_->createViewPort(0, 0, 0.5, 0.5, viewer_v3_);
    pcl_viewer_->addText("LinesAndNormals", 100, 3, "v1_text", viewer_v3_);
    pcl_viewer_->createViewPort(0.5, 0, 1.0, 0.5, viewer_v4_);
    pcl_viewer_->addText("LineBasedPlanes", 100, 3, "v2_text", viewer_v4_);
    pcl_viewer_->addCoordinateSystem(0.000001);
    pcl_viewer_->initCameraParameters();
    pcl_viewer_->setCameraPosition(0.0, 0.0, -0.4, 0, 0, 0.6, 0, -1, 0);
    pcl_viewer_->setShowFPS(true);

    map_viewer_->addCoordinateSystem(0.5);
    map_viewer_->initCameraParameters();
    map_viewer_->setCameraPosition( 0, 3.0, 3.0, -3.0, 0, 0, -1, -1, 0 );
    map_viewer_->setShowFPS(true);

    //
    cv::namedWindow( KeypointWindow );
    cv::namedWindow( MatchesWindow );

    // feature detector
    detector_ = createDetector( feature_detector_type_ );
    extractor_ = createDescriptorExtractor( feature_extractor_type_ );

    true_path_publisher_ = nh_.advertise<nav_msgs::Path>("true_path", 10);
    pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("estimate_pose", 10);
    planar_map_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("planar_map", 10);
    auto_spin_map_viewer_ss_ = nh_.advertiseService("auto_spin_map_viewer", &KinectListener::autoSpinMapViewerCallback, this);

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
    else if( !topic_image_depth_.empty() && !topic_camera_info_.empty() )
    {
        //Depth, depth image, camera_info
        depth_sub_ = new image_sub_type (nh_, topic_image_depth_, subscriber_queue_size_);
        cinfo_sub_ = new cinfo_sub_type(nh_, topic_camera_info_, subscriber_queue_size_);
        depth_sync_ = new DepthSynchronizer(*depth_sub_, *cinfo_sub_, subscriber_queue_size_),
        depth_sync_->registerCallback(boost::bind(&KinectListener::depthCallback, this, _1, _2));
        ROS_INFO_STREAM("Listening to " << topic_image_depth_ << " and " << topic_camera_info_ << ".");

    }
    else
    {
        ROS_ERROR("Can not decide subscriber type");
        exit(1);
    }

    async_spinner_ =  new ros::AsyncSpinner(4, &my_callback_queue_);
    async_spinner_->start();

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

    printf("no cloud msg: %d\n", depth_img_msg->header.seq);

    // Get odom pose
    tf::Transform odom_pose;
    if( !getOdomPose( odom_pose, depth_img_msg->header.frame_id, ros::Time(0) ) )
        return;

    // Get camera parameter
    getCameraParameter( cam_info_msg, camera_parameters_);

    // Current Frame
    KinectFrame current_frame;

    // Get Mat Image
    current_frame.visual_image = cv_bridge::toCvCopy(visual_img_msg)->image; // to cv image
    current_frame.depth_image = cv_bridge::toCvCopy(depth_img_msg)->image; // to cv image
    depthToCV8UC1( current_frame.depth_image, current_frame.depth_mono );
    // Get PointCloud
    current_frame.cloud = image2PointCloud( current_frame.visual_image, current_frame.depth_image, camera_parameters_);


    // Process data
    skip = (skip + 1) % 5;
    if(!skip)
        processFrame( current_frame, odom_pose );

    //
//    if(!loop_one_message_)
//        processCloud( current_frame, odom_pose );
//    else
//        while(loop_one_message_ && ros::ok())
//        {
//            ros::Duration(0.2).sleep();
//            processCloud( current_frame, odom_pose );
//        }
}

void KinectListener::cloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                                const sensor_msgs::PointCloud2ConstPtr& point_cloud,
                                const sensor_msgs::CameraInfoConstPtr& cam_info_msg)
{
    printf("cloud msg: %d\n", point_cloud->header.seq);

//    // Get odom pose
//    tf::Transform odom_pose;
//    if( !getOdomPose( odom_pose, point_cloud->header.frame_id, ros::Time(0) ) )
//        return;

//    // Get camera parameter
//    getCameraParameter( cam_info_msg, camera_parameters_);

//    // Current Frame
//    KinectFrame current_frame;

//    // Get Mat Image
//    current_frame.visual_image = cv_bridge::toCvCopy(visual_img_msg)->image; // to cv image
//    // To pcl pointcloud
//    PointCloudTypePtr cloud_in ( new PointCloudType );
//    pcl::fromROSMsg( *point_cloud, *cloud_in);


//    //
//    if(!loop_one_message_)
//        processCloud( cloud_in, visual_image, odom_pose );
//    else
//        while(loop_one_message_ && ros::ok())
//        {
//            ros::Duration(0.2).sleep();
//            processCloud( cloud_in, visual_image, odom_pose );
//        }
}

void KinectListener::depthCallback ( const sensor_msgs::ImageConstPtr& depth_img_msg,
                                    const sensor_msgs::CameraInfoConstPtr& cam_info_msg)
{
    static int skip = 0;

    printf("depth msg: %d\n", depth_img_msg->header.seq);

    // Get odom pose
    tf::Transform odom_pose;
    if( !getOdomPose( odom_pose, depth_img_msg->header.frame_id, ros::Time(0) ) )
    {
        odom_pose.setIdentity();
        ROS_WARN("Odom_pose set to identity.");
    }

    // Get camera parameter
    getCameraParameter( cam_info_msg, camera_parameters_);

    // Current Frame
    KinectFrame current_frame;

    // Get Mat Image
    current_frame.depth_image = cv_bridge::toCvCopy(depth_img_msg)->image; // to cv image
    current_frame.visual_image = cv::Mat::ones( current_frame.depth_image.rows, current_frame.depth_image.cols, CV_8UC3 );
    depthToCV8UC1( current_frame.depth_image, current_frame.depth_mono );
    // Get PointCloud
    current_frame.cloud = image2PointCloud( current_frame.visual_image, current_frame.depth_image, camera_parameters_);

    // Process data
    processCloud( current_frame, odom_pose );
//    skip = (skip + 1) % 5;
//    if(!skip)
//        processCloud( current_frame, odom_pose );

}

void KinectListener::processCloud( KinectFrame &frame, const tf::Transform &odom_pose )
{
    static gtsam::Pose3 estimated_pose;
    static KinectFrame last_frame;
//    bool use_odom = true;
//    if( odom_pose == tf::Transform::getIdentity() )
//        use_odom = false;

    geometry_msgs::PoseStamped pstamped;
    pstamped.pose.position.x = odom_pose.getOrigin().x();
    pstamped.pose.position.y = odom_pose.getOrigin().y();
    pstamped.pose.position.z = odom_pose.getOrigin().z();
    tf::quaternionTFToMsg( odom_pose.getRotation(), pstamped.pose.orientation );
    true_poses_.push_back( pstamped );
    publishTruePath();

    // if use downsample cloud
    cloud_size_type_ = cloud_size_type_config_;
    if( cloud_size_type_ == QVGA)
    {
        cout << GREEN << "QVGA" << RESET << endl;
        downsampleOrganizedCloud( frame.cloud, frame.cloud_in, camera_parameters_, (int)QVGA);
    }
    else if( cloud_size_type_ == QQVGA)
    {
        cout << GREEN << "QQVGA" << RESET << endl;
        downsampleOrganizedCloud( frame.cloud, frame.cloud_in, camera_parameters_, (int)QQVGA);
    }
    else
    {
        cout << GREEN << "VGA" << RESET << endl;
        frame.cloud_in = frame.cloud; // copy pointer
    }

    double start_time = pcl::getTime();
    pcl::console::TicToc time;
    time.tic();
    float segment_dura = 0;
    float hull_dura = 0;
    float keypoint_dura = 0;
    float slam_dura = 0;
    float display_dura = 0;
    float total_dura = 0;

    // Plane Segment
    if( plane_segment_method_ == ORGANSIZED)
    {
        cout << GREEN << "Organized segmentation." << RESET << endl;
        organizedPlaneSegment( frame.cloud_in, frame.segment_planes );
        segment_dura = time.toc();
        time.tic();
    }
    else if( plane_segment_method_ == LINE_BADED )
    {
        cout << GREEN << "Line based segmentation." << RESET << endl;
        lineBasedPlaneSegment( frame.cloud_in, frame.segment_planes );
        segment_dura = time.toc();
        time.tic();
    }
    else
    {
        cout << RED << "[Error]: Invalid segmentation method error." << RESET << endl;
        exit(0);
    }

    cout << GREEN << " segment planes: " << frame.segment_planes.size() << RESET << endl;

    // extract Hull
//    plane_slam_->extractPlaneHulls( cloud_in, segment_planes );
    hull_dura = time.toc();
    time.tic();


    // display
    pcl_viewer_->removeAllPointClouds();
    pcl_viewer_->removeAllShapes();
    map_viewer_->removeAllPointClouds();
    map_viewer_->removeAllShapes();

    // Do slam
    std::vector<PlaneType> landmarks;
    if( do_slam_ )
    {
        if(!is_initialized)
        {
//            gtsam::Pose3 init_pose;
//            plane_slam_->tfToPose3( odom_pose, init_pose);
//            if( plane_slam_->initialize( init_pose, frame ) )
//            {
//                is_initialized = true;
//                estimated_pose = init_pose;
//                last_frame = frame; // store frame
//            }
//            else
//                return;

            gtsam::Pose3 init_pose;
            plane_slam_->tfToPose3( odom_pose, init_pose );
            estimated_pose = init_pose;
            is_initialized = true;

        }
        else
        {
            gtsam::Pose3 o_pose;
            plane_slam_->tfToPose3( odom_pose, o_pose );
            gtsam::Pose3 r_pose = estimated_pose.inverse() * o_pose;
            RESULT_OF_PNP motion;
            bool res = solveRelativeTransformPlanes( frame, last_frame, motion );
            if( res )
            {
                // print motion
                gtsam::Rot3 rot3( motion.rotation );
                cout << MAGENTA << " estimated motion, deviation = " << motion.deviation << endl;
                cout << "  - R(rpy): " << rot3.roll() << ", " << rot3.pitch() << ", " << rot3.yaw() << endl;
                cout << "  - T:      " << motion.translation[0]
                     << ", " << motion.translation[1]
                     << ", " << motion.translation[2] << RESET << endl;

                cout << CYAN << " true motion: " << endl;
                cout << "  - R(rpy): " << r_pose.rotation().roll() << ", " << r_pose.rotation().pitch() << ", " << r_pose.rotation().yaw() << endl;
                cout << "  - T:      " << r_pose.translation().x()
                     << ", " << r_pose.translation().y()
                     << ", " << r_pose.translation().z() << RESET << endl;
            }
            if( !res )
            {
                cout << RED << " failed to estimate relative motion. " << RESET << endl;
            }
            //
////            RESULT_OF_PNP motion = estimateMotion( frame, last_frame, camera_parameters_ );
//            gtsam::Pose3 rel( gtsam::Rot3(motion.rotation), gtsam::Point3(motion.translation) );
//            gtsam::Pose3 pose3 = estimated_pose * rel;
//            estimated_pose = plane_slam_->planeSlam( pose3, frame );
//            publishPose( estimated_pose );
//            if(display_path_)
//                plane_slam_->publishEstimatedPath();
//            if(display_odom_path_)
//                plane_slam_->publishOdomPath();

//            // visualize landmark
//            landmarks = plane_slam_->getLandmarks();
//            // project and recalculate contour

//            // store frame
//            last_frame = frame;

            // store frame
            estimated_pose = o_pose;
//            last_frame = frame;
        }
    }

    slam_dura = time.toc();
    time.tic();

    // display landmarks
    if(display_landmarks_ && landmarks.size() > 0)
    {
        displayLandmarks( landmarks, "landmark");
        publishPlanarMap( landmarks );
    }

    // display
    if(display_input_cloud_)
    {
//        pcl_viewer_->addPointCloud( frame.cloud_in, "rgba_cloud", viewer_v1_ );
//        pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "rgba_cloud", viewer_v1_);
    }
    displayPlanes( frame.cloud_in, frame.segment_planes, "planes", viewer_v2_ );
    displayPlanes( frame.cloud_in, last_frame.segment_planes, "last_planes", viewer_v4_ );

    pcl_viewer_->addText( "Current Frame", 200, 20, "viewer_v2_name", viewer_v2_);
    pcl_viewer_->addText( "Last Frame", 200, 20, "viewer_v4_name", viewer_v4_);
    pcl_viewer_->spinOnce(1);
    map_viewer_->spinOnce(1);

    display_dura = time.toc();
    total_dura = (pcl::getTime() - start_time) * 1000;

    cout << GREEN << "Total time: " << total_dura << ", segment: " << segment_dura << ", keypoints: "
         << keypoint_dura << ", slam: "<< slam_dura << ", display: " << display_dura << RESET << endl;
    cout << "----------------------------------- END -------------------------------------" << endl;

    // For next calculation
    last_frame = frame; // store frame
}

void KinectListener::processFrame( KinectFrame &frame, const tf::Transform &odom_pose )
{
    static gtsam::Pose3 estimated_pose;
    static KinectFrame last_frame;
//    bool use_odom = true;
//    if( odom_pose == tf::Transform::getIdentity() )
//        use_odom = false;

    geometry_msgs::PoseStamped pstamped;
    pstamped.pose.position.x = odom_pose.getOrigin().x();
    pstamped.pose.position.y = odom_pose.getOrigin().y();
    pstamped.pose.position.z = odom_pose.getOrigin().z();
    tf::quaternionTFToMsg( odom_pose.getRotation(), pstamped.pose.orientation );
    true_poses_.push_back( pstamped );
    publishTruePath();

    // if use downsample cloud
    cloud_size_type_ = cloud_size_type_config_;
    if( cloud_size_type_ == QVGA)
    {
        cout << GREEN << "QVGA" << RESET << endl;
        downsampleOrganizedCloud( frame.cloud, frame.cloud_in, camera_parameters_, (int)QVGA);
        downsampleImage( frame.visual_image, frame.visual, (int)QVGA );
    }
    else if( cloud_size_type_ == QQVGA)
    {
        cout << GREEN << "QQVGA" << RESET << endl;
        downsampleOrganizedCloud( frame.cloud, frame.cloud_in, camera_parameters_, (int)QQVGA);
        downsampleImage( frame.visual_image, frame.visual, (int)QQVGA );
    }
    else
    {
        cout << GREEN << "VGA" << RESET << endl;
        frame.cloud_in = frame.cloud; // copy pointer
        frame.visual = frame.visual_image;
    }

    double start_time = pcl::getTime();
    pcl::console::TicToc time;
    time.tic();
    float segment_dura = 0;
    float hull_dura = 0;
    float keypoint_dura = 0;
    float slam_dura = 0;
    float display_dura = 0;
    float total_dura = 0;

    // Plane Segment
    if( plane_segment_method_ == ORGANSIZED)
    {
        cout << GREEN << "Organized segmentation." << RESET << endl;
        organizedPlaneSegment( frame.cloud_in, frame.segment_planes );
        segment_dura = time.toc();
        time.tic();
    }
    else if( plane_segment_method_ == LINE_BADED )
    {
        cout << GREEN << "Line based segmentation." << RESET << endl;
        lineBasedPlaneSegment( frame.cloud_in, frame.segment_planes );
        segment_dura = time.toc();
        time.tic();
    }
    else
    {
        cout << RED << "[Error]: Invalid segmentation method error." << RESET << endl;
        exit(0);
    }

    cout << GREEN << " segment planes: " << frame.segment_planes.size() << RESET << endl;

    // extract Hull
//    plane_slam_->extractPlaneHulls( cloud_in, segment_planes );
    hull_dura = time.toc();
    time.tic();

    // Compute Keypoints
    computeKeypoint( frame.visual_image, frame.cloud, frame.feature_locations_2d,
                     frame.feature_locations_3d, frame.feature_descriptors, frame.depth_mono );
    displayKeypoint( frame.visual_image, frame.feature_locations_2d );
    keypoint_dura = time.toc();
    time.tic();

    // Match features
    vector<cv::DMatch> good_matches;
    if( is_initialized )
    {
        matchImageFeatures( last_frame, frame, good_matches, feature_good_match_threshold_);
        cout << GREEN << "Match features th = " << feature_good_match_threshold_
             << ", good matches = " << good_matches.size() << RESET << endl;
        cv::Mat image_matches;
        cv::drawMatches( last_frame.visual_image, last_frame.feature_locations_2d,
                         frame.visual_image, frame.feature_locations_2d,
                         good_matches, image_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                         vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        cv::imshow( MatchesWindow, image_matches );
        cv::waitKey( 1 );
    }

    // display
    pcl_viewer_->removeAllPointClouds();
    pcl_viewer_->removeAllShapes();
    map_viewer_->removeAllPointClouds();
    map_viewer_->removeAllShapes();


    // Do slam
    std::vector<PlaneType> landmarks;
    if( do_slam_ )
    {
        if(!is_initialized)
        {
//            gtsam::Pose3 init_pose;
//            plane_slam_->tfToPose3( odom_pose, init_pose);
//            if( plane_slam_->initialize( init_pose, frame ) )
//            {
//                is_initialized = true;
//                estimated_pose = init_pose;
//                last_frame = frame; // store frame
//            }
//            else
//                return;

            gtsam::Pose3 init_pose;
            plane_slam_->tfToPose3( odom_pose, init_pose );
            estimated_pose = init_pose;
            is_initialized = true;

        }
        else
        {
            gtsam::Pose3 o_pose;
            plane_slam_->tfToPose3( odom_pose, o_pose );
            gtsam::Pose3 r_pose = estimated_pose.inverse() * o_pose;
            RESULT_OF_PNP motion;
            bool res = solveRelativeTransformPlanes( frame, last_frame, motion );
            if( res )
            {
                // print motion
                gtsam::Rot3 rot3( motion.rotation );
                cout << MAGENTA << " estimated motion, deviation = " << motion.deviation << endl;
                cout << "  - R(rpy): " << rot3.roll() << ", " << rot3.pitch() << ", " << rot3.yaw() << endl;
                cout << "  - T:      " << motion.translation[0]
                     << ", " << motion.translation[1]
                     << ", " << motion.translation[2] << RESET << endl;

                cout << CYAN << " true motion: " << endl;
                cout << "  - R(rpy): " << r_pose.rotation().roll() << ", " << r_pose.rotation().pitch() << ", " << r_pose.rotation().yaw() << endl;
                cout << "  - T:      " << r_pose.translation().x()
                     << ", " << r_pose.translation().y()
                     << ", " << r_pose.translation().z() << RESET << endl;
            }
            if( !res )
            {
                cout << RED << " failed to estimate relative motion. " << RESET << endl;
            }
            //
////            RESULT_OF_PNP motion = estimateMotion( frame, last_frame, camera_parameters_ );
//            gtsam::Pose3 rel( gtsam::Rot3(motion.rotation), gtsam::Point3(motion.translation) );
//            gtsam::Pose3 pose3 = estimated_pose * rel;
//            estimated_pose = plane_slam_->planeSlam( pose3, frame );
//            publishPose( estimated_pose );
//            if(display_path_)
//                plane_slam_->publishEstimatedPath();
//            if(display_odom_path_)
//                plane_slam_->publishOdomPath();

//            // visualize landmark
//            landmarks = plane_slam_->getLandmarks();
//            // project and recalculate contour

//            // store frame
//            last_frame = frame;

            // store frame
            estimated_pose = o_pose;
//            last_frame = frame;
        }
    }

    slam_dura = time.toc();
    time.tic();

    // display landmarks
    if(display_landmarks_ && landmarks.size() > 0)
    {
        displayLandmarks( landmarks, "landmark");
        publishPlanarMap( landmarks );
    }

    // display
    if(display_input_cloud_)
    {
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> rgba_color(frame_current.point_cloud, 255, 255, 255);
        pcl_viewer_->addPointCloud( frame.cloud_in, "rgba_cloud", viewer_v1_ );
        pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "rgba_cloud", viewer_v1_);
    }
    displayPlanes( last_frame.cloud_in, last_frame.segment_planes, "last_planes", viewer_v2_ );
    displayPlanes( frame.cloud_in, frame.segment_planes, "planes", viewer_v4_ );
    pcl_viewer_->addText( "Last Frame", 200, 20, "viewer_v2_name", viewer_v2_);
    pcl_viewer_->addText( "Current Frame", 200, 20, "viewer_v4_name", viewer_v4_);
    //
//    display3DKeypoint( frame.feature_locations_3d, "current", viewer_v1_ );
//    display3DKeypoint( last_frame.feature_locations_3d, "last", viewer_v3_ );
    displayMatched3DKeypoint( last_frame.feature_locations_3d, frame.feature_locations_3d, good_matches );
    pcl_viewer_->addText( "Last 3D Features", 200, 20, "viewer_v1_name", viewer_v1_);
    pcl_viewer_->addText( "Current 3D Features", 200, 20, "viewer_v3_name", viewer_v3_);
    pcl_viewer_->spinOnce(1);
    map_viewer_->spinOnce(1);

    display_dura = time.toc();
    total_dura = (pcl::getTime() - start_time) * 1000;

    cout << GREEN << "Total time: " << total_dura << ", segment: " << segment_dura << ", keypoints: "
         << keypoint_dura << ", slam: "<< slam_dura << ", display: " << display_dura << RESET << endl;
    cout << "----------------------------------- END -------------------------------------" << endl;

    last_frame = frame; // store frame
}

void KinectListener::solveRT( const std::vector<PlaneCoefficients> &before,
                              const std::vector<PlaneCoefficients> &after,
                              const std::vector<Eigen::Vector3d>& from_points,
                              const std::vector<Eigen::Vector3d>& to_points,
                              RESULT_OF_PNP &result)
{
    const int num_points = from_points.size();
    const int num_planes = before.size();

    ROS_ASSERT( before.size() == after.size() );
    ROS_ASSERT( from_points.size() == to_points.size() );
    ROS_ASSERT( num_planes + num_points == 3 );

    // Rotation
    Eigen::MatrixXd froms(3, num_points), tos(3, num_points);
    Eigen::MatrixXd src(3, num_planes), dst(3, num_planes);
    for( int i = 0; i < num_points; i++)
    {
        froms.col(i) = from_points[i];
        tos.col(i) = to_points[i];
    }
    for( int i = 0; i < num_planes; i++)
    {
        src.col(i) = before[i].head<3>();
        dst.col(i) = after[i].head<3>();
    }
    const double wi = 1.0;// / num_planes;
    const double one_over_n = num_points > 0 ? 1.0 / num_points : 0;
    /// 1: For point
    // mean
    const Eigen::VectorXd froms_mean = froms.rowwise().sum() * one_over_n;
    const Eigen::VectorXd tos_mean = tos.rowwise().sum() * one_over_n;
    // demeaning
    const Eigen::MatrixXd froms_demean = froms.colwise() - froms_mean;
    const Eigen::MatrixXd tos_demean = tos.colwise() - tos_mean;
    // Eq. (38)
    const Eigen::MatrixXd sigma_points = one_over_n * tos_demean * froms_demean.transpose();

    /// 2: For plane
    const Eigen::MatrixXd sigma_planes = wi * dst * src.transpose();

    /// 3: sigma
    const Eigen::MatrixXd sigma = sigma_points + sigma_planes;

    /// 4: Umeyama solution
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3,3);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(sigma, ComputeFullU | ComputeFullV);
    // Eq. (39)
    Eigen::VectorXd S = Eigen::VectorXd::Ones( 3 );
    cout << "   det(sigma) = " << sigma.determinant() << endl;
    if( sigma.determinant() < 0 )
        S( 2 ) = -1;
    // Eq. (40) and (43)
    const Eigen::VectorXd& vs = svd.singularValues();
    int rank = 0;
    for (int i=0; i<3; ++i)
        if (!Eigen::internal::isMuchSmallerThan(vs.coeff(i),vs.coeff(0)))
            ++rank;
//    cout << "   D: " << endl;
//    cout << vs << endl;
    cout << "   rank(sigma) = " << rank << endl;
    if ( rank == 2 )
    {
        cout << "   det(U)*det(V) = " << svd.matrixU().determinant() * svd.matrixV().determinant() << endl;
        if ( svd.matrixU().determinant() * svd.matrixV().determinant() > 0 )
        {
            R = svd.matrixU()*svd.matrixV().transpose();
        }
        else
        {
            const double s = S(2);
            S(2) = -1;
            R = svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();
            S(2) = s;
        }
    }
    else
    {
        R = svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();
    }
    // Translation
    Eigen::VectorXd T = Eigen::Vector3d::Zero(3);
    /// 1: For points
    Eigen::MatrixXd I3 = Eigen::Matrix3d::Identity();
    Eigen::MatrixXd A1 = num_points * I3;
    Eigen::VectorXd b1 = num_points * ( tos_mean - R * froms_mean );
    /// 2: For planes
    Eigen::MatrixXd A2( num_planes, 3 );
    Eigen::VectorXd b2( num_planes );
    for( int i = 0; i < num_planes; i++)
    {
        A2.row(i) = wi * after[i].head<3>().transpose();
        b2(i) = before[i](3) - after[i](3);
    }
    /// 3:  ( A1 A2 )^T * t = ( b1 b2)^T
    Eigen::MatrixXd AA;( 3 + num_planes, 3 );
    Eigen::VectorXd bb;( 3 + num_planes );
    if( num_points != 0 )
    {
        AA = Eigen::MatrixXd( 3 + num_planes, 3);
        bb = Eigen::VectorXd( 3 + num_planes );
        AA << A1, A2;
        bb << b1, b2;
    }
    else
    {
        AA = Eigen::MatrixXd( num_planes, 3);
        bb = Eigen::VectorXd( num_planes );
        AA << A2;
        bb << b2;
    }


    /// 4: t = (A.transpose()*A).inverse()*A.transpose()*b;
    Eigen::MatrixXd AAT = AA.transpose();
    T = (AAT*AA).inverse()*AAT*bb;

    result.rotation = R;
    result.translation = T;
}

void KinectListener::solveRT(const std::vector<Eigen::Vector3d>& from_points,
                             const std::vector<Eigen::Vector3d>& to_points,
                             RESULT_OF_PNP &result)
{
    ROS_ASSERT( from_points.size() >= 3 && to_points.size() >=3 );

    Eigen::MatrixXd src(3,3), dst(3,3);
    for( int i = 0; i < 3; i++)
    {
        src.col(i) = from_points[i];
        dst.col(i) = to_points[i];
    }
    Eigen::Matrix4d transform = Eigen::umeyama(src, dst, false);
    result.rotation = transform.topLeftCorner(3,3);
    result.translation = transform.col(3).head<3>();
}

void KinectListener::solveRT(const std::vector<PlaneCoefficients> &before,
                             const std::vector<PlaneCoefficients> &after,
                             RESULT_OF_PNP &result)
{
    ROS_ASSERT( after.size() >= 3 && before.size() >=3 );

    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3,3);
    Eigen::VectorXd T = Eigen::Vector3d::Zero(3);
    // algorithm: "Least-squares estimation of transformation parameters between two point patterns", Shinji Umeyama, PAMI 1991, DOI: 10.1109/34.88573
    Eigen::MatrixXd A(3, 3), B(3, 3); // A = RB, A === dst, B === src
    Eigen::VectorXd distance( 3 );
    for(int i = 0; i < 3; i++ )
    {
        const Eigen::Vector3d from = before[i].head<3>();
        const Eigen::Vector3d to = after[i].head<3>();
        const double d1 = before[i](3);
        const double d2 = after[i](3);
        B.col(i) = from;
        A.col(i) = to;
        distance(i) = d1 - d2;   // d_src - d_dst
    }
    // Rotation
    // n_dst = R * n_src
    const Eigen::MatrixXd sigma = A * B.transpose();    // Eq. ABt
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(sigma, ComputeFullU | ComputeFullV);

    // Eq. (39)
    Eigen::VectorXd S = Eigen::VectorXd::Ones( 3 );
    cout << "   det(sigma) = " << sigma.determinant() << endl;
    if( sigma.determinant() < 0 )
        S( 2 ) = -1;

    // Eq. (40) and (43)
    const Eigen::VectorXd& vs = svd.singularValues();
    int rank = 0;
    for (int i=0; i<3; ++i)
        if (!Eigen::internal::isMuchSmallerThan(vs.coeff(i),vs.coeff(0)))
            ++rank;
//    cout << "   D: " << endl;
//    cout << vs << endl;
    cout << "   rank(sigma) = " << rank << endl;
    if ( rank == 2 )
    {
        cout << "   det(U)*det(V) = " << svd.matrixU().determinant() * svd.matrixV().determinant() << endl;
        if ( svd.matrixU().determinant() * svd.matrixV().determinant() > 0 )
        {
            R = svd.matrixU()*svd.matrixV().transpose();
        }
        else
        {
            const double s = S(2);
            S(2) = -1;
            R = svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();
            S(2) = s;
        }
    }
    else
    {
        R = svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();
    }
    // Translation
    // n_dst^T * t = d_dst - d_src
    Eigen::JacobiSVD<MatrixXd> svdA(A.transpose(), ComputeFullU | ComputeFullV);
    T = svdA.solve(distance);

    result.rotation = R;
    result.translation = T;
}

bool KinectListener::solveMotionPlanes( const std::vector<PlaneCoefficients> &last_planes,
                                        const std::vector<PlaneCoefficients> &planes,
                                        RESULT_OF_PNP &result)
{
    result.rotation = Eigen::Matrix3d::Identity();
    result.translation = Eigen::Vector3d::Zero();

    /// 1: Check number of planes
    if( planes.size() != 3 || last_planes.size() != 3)
        return false;

    /// 2: Check co-planar
    double dis1, dis2, dis3;
    double dir1, dir2, dir3;
    const double dir_thresh = 15.0 * DEG_TO_RAD;
    // check co-planar
    ITree::euclidianDistance( planes[0], planes[1], dir1, dis1 );
    ITree::euclidianDistance( planes[0], planes[2], dir2, dis2 );
    ITree::euclidianDistance( planes[1], planes[2], dir3, dis3 );
//    cout << BOLDBLUE << " dir " << dir_thresh << " : " << dir1 << " " << dir2 << " " << dir3 << RESET << endl;
    if( dir1 < dir_thresh || dir2 < dir_thresh || dir3 < dir_thresh )
        return false;
    // check co-planar
    ITree::euclidianDistance( last_planes[0], last_planes[1], dir1, dis1 );
    ITree::euclidianDistance( last_planes[0], last_planes[2], dir2, dis2 );
    ITree::euclidianDistance( last_planes[1], last_planes[2], dir3, dis3 );
//    cout << BOLDBLUE << " dir " << dir_thresh << " : " << dir1 << " " << dir2 << " " << dir3 << RESET << endl;
    if( dir1 < dir_thresh || dir2 < dir_thresh || dir3 < dir_thresh )
        return false;

    /// 3: compute Rt
    solveRT( last_planes, planes, result );

    return true;
}

bool KinectListener::solveRelativeTransformPlanes( KinectFrame& current_frame,
                                                   KinectFrame& last_frame,
                                                   RESULT_OF_PNP &result,
                                                   const Eigen::Matrix4d &estimated_transform)
{
    std::vector<PlaneType> &planes = current_frame.segment_planes;
    std::vector<PlaneType> &last_planes = last_frame.segment_planes;

    if( planes.size() < 3 || last_planes.size() < 3)
        return false;

    /// 1: Find correspondences
    std::vector<PlanePair> pairs;
    ITree::euclidianPlaneCorrespondences( planes, last_planes, pairs, estimated_transform );
    int pairs_num = pairs.size();
    if( pairs_num < 3 )
        return false;
    cout << GREEN << " pairs size: " << pairs_num << RESET << endl;

    /// 2: Estimate motion
    RESULT_OF_PNP best_transform;
    best_transform.deviation = 1e6; // no inlier
    for( int x1 = 0; x1 < pairs_num-2; x1++)
    {
        PlanePair &p1 = pairs[x1];
        for( int x2 = x1+1; x2 < pairs_num-1; x2++ )
        {
            PlanePair &p2 = pairs[x2];
            for( int x3 = x2+1; x3 < pairs_num; x3++)
            {
                PlanePair &p3 = pairs[x3];
                std::vector<PlaneCoefficients> currents;
                std::vector<PlaneCoefficients> lasts;
                currents.push_back( planes[p1.iobs].coefficients );
                currents.push_back( planes[p2.iobs].coefficients );
                currents.push_back( planes[p3.iobs].coefficients );
                lasts.push_back( last_planes[p1.ilm].coefficients );
                lasts.push_back( last_planes[p2.ilm].coefficients );
                lasts.push_back( last_planes[p3.ilm].coefficients );
                //
//                cout << YELLOW << " solve motion: (" << x1 << "/" << x2 << "/" << x3 << ")" << RESET << endl;
                RESULT_OF_PNP motion;
                bool res = solveMotionPlanes( currents, lasts, motion);

                if( res )
                {
                    // check if better
                    motion.deviation = computeEuclidianDistance( last_planes, planes, pairs, motion );
                    if( motion.deviation < best_transform.deviation )
                        best_transform = motion;

                    // print motion
                    gtsam::Rot3 rot3( motion.rotation );
                    cout << YELLOW << " relative motion: (" << p1.iobs << "/" << p2.iobs << "/" << p3.iobs << "), ("
                         << p1.ilm << "/" << p2.ilm << "/" << p3.ilm << ")"
                         << ", deviation = " << motion.deviation << endl;
                    cout << "  - R(rpy): " << rot3.roll() << ", " << rot3.pitch() << ", " << rot3.yaw() << endl;
                    cout << "  - T:      " << motion.translation[0]
                         << ", " << motion.translation[1]
                         << ", " << motion.translation[2] << RESET << endl;
                }
            }
        }
    }

//    Eigen::umeyama
    result = best_transform;
    return true;
}

bool KinectListener::estimateRelativeTransform( KinectFrame& current_frame, KinectFrame& last_frame, RESULT_OF_PNP &result )
{
    /// 1: Find matches
    vector< cv::DMatch > matches;
    cv::FlannBasedMatcher matcher;
    matcher.match( last_frame.feature_descriptors, current_frame.feature_descriptors, matches );

    cout << "find total " << matches.size() << " matches." <<endl;

    /// 2: Find good matches, sort
    vector< cv::DMatch > goodMatches;
    double minDis = 9999;
    double good_match_threshold = 4.0;
    for ( size_t i=0; i<matches.size(); i++ )
    {
        if ( matches[i].distance < minDis )
            minDis = matches[i].distance;
    }
    for ( size_t i=0; i<matches.size(); i++ )
    {
        cv::DMatch &m = matches[i];
        if ( m.distance < good_match_threshold*minDis )
        {
            if(!isnan(last_frame.feature_locations_3d[m.queryIdx](2))
               && !isnan(current_frame.feature_locations_3d[m.trainIdx](2)))
                goodMatches.push_back( m );
        }
    }

    std::sort(goodMatches.begin(), goodMatches.end()); //sort by distance, which is the nn_ratio
    cout << "good matches: " << goodMatches.size() << endl;

    /// 3: RANSAC
    // initialize result values of all iterations
    matches.clear();
    Eigen::Matrix4f resulting_transformation = Eigen::Matrix4f::Identity();
    float rmse = 1e6;
    unsigned int valid_iterations = 0;//, best_inlier_cnt = 0;
    const unsigned int sample_size = 3;// chose this many randomly from the correspondences:
    bool valid_tf = false; // valid is false iff the sampled points clearly aren't inliers themself

    /// 4: Registeration using points & planes

}

double KinectListener::computeEuclidianDistance( std::vector<PlaneType>& last_planes,
                                                 std::vector<PlaneType>& planes,
                                                 std::vector<PlanePair>& pairs,
                                                 RESULT_OF_PNP &relative )
{
    double deviation = 0;
    Eigen::Matrix4d transform = relative.transform();
    for( int i = 0; i < pairs.size(); i++)
    {
        PlaneType &plane = planes[ pairs[i].iobs ];
        PlaneType &last_plane = last_planes[ pairs[i].ilm ];
        PlaneType transformed_plane;
        transformPlane( last_plane.coefficients, transform, transformed_plane.coefficients );
//        cout << GREEN << " - tr p: " << pairs[i].iobs << "/" << pairs[i].ilm << ": " << endl;
//        cout << " current: " << plane.coefficients[0] << ", " << plane.coefficients[1]
//             << ", " << plane.coefficients[2] << ", " << plane.coefficients[3] << endl;
//        cout << " transformed: " << transformed_plane.coefficients[0] << ", " << transformed_plane.coefficients[1]
//             << ", " << transformed_plane.coefficients[2] << ", " << transformed_plane.coefficients[3]
//             << RESET << endl;
        double direction, distance;
        ITree::euclidianDistance( plane, transformed_plane, direction, distance );
        deviation = direction + distance;
    }

    return deviation / static_cast<int>( pairs.size() );
}

// estimateMotion 计算两个帧之间的运动
// 输入：帧1和帧2
// 输出：rvec 和 tvec
RESULT_OF_PNP KinectListener::estimateMotion( KinectFrame& current_frame, KinectFrame& last_frame, PlaneFromLineSegment::CAMERA_PARAMETERS& camera )
{
    vector< cv::DMatch > matches;
    cv::FlannBasedMatcher matcher;
    matcher.match( last_frame.feature_descriptors, current_frame.feature_descriptors, matches );

    cout << "find total " << matches.size() << " matches." <<endl;
    vector< cv::DMatch > goodMatches;
    double minDis = 9999;
    double good_match_threshold = 4.0;
    for ( size_t i=0; i<matches.size(); i++ )
    {
        if ( matches[i].distance < minDis )
            minDis = matches[i].distance;
    }

    for ( size_t i=0; i<matches.size(); i++ )
    {
        if (matches[i].distance < good_match_threshold*minDis)
            goodMatches.push_back( matches[i] );
    }

    cout << "good matches: " << goodMatches.size() << endl;

    // 3d points
    vector<cv::Point3f> pts_obj;
    // 2d points
    vector< cv::Point2f > pts_img;

    //
    for (size_t i=0; i<goodMatches.size(); i++)
    {
        int query_index = goodMatches[i].queryIdx;
        int train_index = goodMatches[i].trainIdx;

        // query
        cv::Point2f p = last_frame.feature_locations_2d[query_index].pt;
        PointType pc = last_frame.cloud->at((int) p.x,(int) p.y);
        if( !isValidPoint(pc) )
            continue;

        // 3d point
        cv::Point3f pd( pc.x, pc.y, pc.z );
        pts_obj.push_back( pd );

        // 2d point
        pts_img.push_back( cv::Point2f( current_frame.feature_locations_2d[train_index].pt ) );
    }

    // Construct camera intrinsic matrix
    double camera_matrix_data[3][3] = {
        {camera.fx, 0, camera.cx},
        {0, camera.fy, camera.cy},
        {0, 0, 1}
    };
    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
    cv::Mat rvec, tvec, inliers;
    // Solve pnp
    cout << "solving pnp" << endl;
    cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 50, 1.0, 50, inliers );

    RESULT_OF_PNP result;
    //
    result.inliers = inliers.rows;
    result.translation = Eigen::Vector3d( tvec.at<double>(0,0), tvec.at<double>(0,1), tvec.at<double>(0,2) );
    cv::Mat R;
    cv::Rodrigues( rvec, R );
//    cv::cv2eigen( R, result.rotation );
    cvToEigen( R, result.rotation );

    cout << " Trans: " << endl << result.translation << endl;
    cout << " Rot: " << endl << result.rotation << endl;

    return result;
}

void KinectListener::computeKeypoint( const cv::Mat &visual,
                                      const PointCloudTypePtr &cloud_in,
                                      std::vector<cv::KeyPoint> &keypoints,
                                      std_vector_of_eigen_vector4f &locations_3d,
                                      cv::Mat &feature_descriptors,
                                      const cv::Mat& mask )
{
    // Get gray image
    cv::Mat gray_img;
    if(visual.type() == CV_8UC3)
        cv::cvtColor( visual, gray_img, CV_RGB2GRAY );
    else
        gray_img = visual;

    // Compute eeypoints
    keypoints.clear();
    feature_descriptors= cv::Mat();
    detector_->detect( gray_img, keypoints, mask );// fill 2d locations

    // Project to 3D
    projectTo3D( cloud_in, keypoints, locations_3d);

    // Compute descriptors
    extractor_->compute( gray_img, keypoints, feature_descriptors ); //fill feature_descriptors_ with information

//    // Detect feature for all planes
//    for( int i = 0; i < planes.size(); i++)
//    {
//        PlaneType &plane = planes[i];
//        detector_->detect( gray_img, plane.feature_locations_2d, plane.mask );// fill 2d locations
//        extractor_->compute( gray_img, plane.feature_locations_2d, plane.feature_descriptors ); //fill feature_descriptors_ with information
//        projectTo3D( cloud, plane.feature_locations_2d, plane.feature_locations_3d ); // project to 3d
//    }

}

void KinectListener::projectTo3D( const PointCloudTypePtr &cloud,
                                  std::vector<cv::KeyPoint> &locations_2d,
                                  std_vector_of_eigen_vector4f &locations_3d)
{
    // Clear
    if(locations_3d.size())
        locations_3d.clear();

    for(int i = 0; i < locations_2d.size(); )
    {
        cv::Point2f p2d = locations_2d[i].pt;

        PointType p3d = cloud->at((int) p2d.x,(int) p2d.y);

        // Check for invalid measurements
        if ( isnan(p3d.x) || isnan(p3d.y) || isnan(p3d.z))
        {
            locations_2d.erase( locations_2d.begin()+i );
            continue;
        }

        locations_3d.push_back(Eigen::Vector4f(p3d.x, p3d.y, p3d.z, 1.0));
        i++; //Only increment if no element is removed from vector
//        if( locations_3d.size() > max_keyp )
//            break;
    }
}


void KinectListener::matchImageFeatures( KinectFrame& last_frame,
                                         KinectFrame& current_frame,
                                         vector< cv::DMatch > &goodMatches,
                                         double good_match_threshold)
{
    vector< cv::DMatch > matches;
    cv::FlannBasedMatcher matcher;
//    cout << MAGENTA << " features: " << last_frame.feature_locations_2d.size() << ", "
//         << current_frame.feature_locations_2d.size() << RESET << endl;
//    cout << MAGENTA << " descriptors: " << last_frame.feature_descriptors.rows << ", "
//         << current_frame.feature_descriptors.rows << RESET << endl;
    matcher.match( last_frame.feature_descriptors, current_frame.feature_descriptors, matches );

    const int query_num = last_frame.feature_descriptors.rows;
    const int train_num = current_frame.feature_descriptors.rows;

//    cout << "find total " << matches.size() << " matches." <<endl;
    double minDis = 9999;
    for ( size_t i=0; i<matches.size(); i++ )
    {
        if ( matches[i].distance < minDis )
            minDis = matches[i].distance;
    }

    for ( size_t i=0; i<matches.size(); i++ )
    {
        if (matches[i].distance < good_match_threshold*minDis
                && matches[i].queryIdx < query_num
                && matches[i].trainIdx < train_num )
            goodMatches.push_back( matches[i] );
    }

//    cout << "good matches: " << goodMatches.size() << endl;
}

void KinectListener::setlineBasedPlaneSegmentParameters()
{
    //
    plane_from_line_segment_.setUseHorizontalLines( use_horizontal_line_ );
    plane_from_line_segment_.setUseVerticleLines( use_verticle_line_ );
    plane_from_line_segment_.setYskip( y_skip_ );
    plane_from_line_segment_.setXSkip( x_skip_ );
    plane_from_line_segment_.setLinePointMinDistance( line_point_min_distance_ );
    plane_from_line_segment_.setUseDepthNoiseModel( use_depth_noise_model_ );
    plane_from_line_segment_.setRhoConstantError( scan_rho_constant_error_ );
    plane_from_line_segment_.setRhoDistanceError( scan_rho_distance_error_ );
    plane_from_line_segment_.setRhoQuadraticError( scan_rho_quadratic_error_ );
    plane_from_line_segment_.setSlideWindowSize( slide_window_size_ );
    plane_from_line_segment_.setLineMinInliers( line_min_inliers_ );
    plane_from_line_segment_.setLineFittingThreshold( line_fitting_threshold_ );
    //
    plane_from_line_segment_.setNormalsPerLine( normals_per_line_ );
    plane_from_line_segment_.setNormalUseDepthSmoothing( normal_use_depth_dependent_smoothing_ );
    plane_from_line_segment_.setNormalDepthChangeFactor( normal_max_depth_change_factor_ );
    plane_from_line_segment_.setNormalSmoothingSize( normal_smoothing_size_ );
    plane_from_line_segment_.setNormalMinInliersPercentage( normal_min_inliers_percentage_ );
    plane_from_line_segment_.setNormalMaximumCurvature( normal_maximum_curvature_ );
    //
    plane_from_line_segment_.setRemoveDuplicateCandidate( remove_duplicate_candidate_ );
    plane_from_line_segment_.setDuplicateCandidateThreshold( duplicate_candidate_normal_thresh_,
                                                             duplicate_candidate_distance_thresh_ );
    //
    plane_from_line_segment_.setPlaneSegmentCriterion( plane_segment_criterion_ );
    plane_from_line_segment_.setCriterionBothParameters( k_curvature_, k_inlier_ );
    plane_from_line_segment_.setMinInliers( min_inliers_ );
    plane_from_line_segment_.setDistanceThreshold( distance_threshold_ );
    plane_from_line_segment_.setNeighborThreshold( neighbor_threshold_ );
    plane_from_line_segment_.setOptimizeCoefficients( optimize_coefficients_ );
    plane_from_line_segment_.setProjectPoints( project_points_ );
    plane_from_line_segment_.setExtractBoundary( extract_boundary_ );
}

void KinectListener::lineBasedPlaneSegment(PointCloudTypePtr &input,
                                         std::vector<PlaneType> &planes)
{
    static int last_size_type = VGA;

    PointCloudTypePtr cloud_in (new PointCloudType);
    pcl::copyPointCloud( *input, *cloud_in);

    //
    if( is_update_line_based_parameters_ )
    {
        setlineBasedPlaneSegmentParameters();
        is_update_line_based_parameters_ = false;
    }

    if (!plane_from_line_segment_.isInitialized() || cloud_size_type_ != last_size_type)
    {
        last_size_type = cloud_size_type_;
        plane_from_line_segment_.setCameraParameters( camera_parameters_ );
        cout << "Initialize line base segment." << endl;
    }

    //
    std::vector<PlaneFromLineSegment::NormalType> line_based_planes;
    plane_from_line_segment_.setInputCloud( cloud_in );
    plane_from_line_segment_.segment( line_based_planes );

    // convert format
    for( int i = 0; i < line_based_planes.size(); i++)
    {
        PlaneFromLineSegment::NormalType &normal = line_based_planes[i];
        PlaneType plane;
        plane.mask = normal.mask;
        plane.centroid = normal.centroid;
        plane.coefficients[0] = normal.coefficients[0];
        plane.coefficients[1] = normal.coefficients[1];
        plane.coefficients[2] = normal.coefficients[2];
        plane.coefficients[3] = normal.coefficients[3];
        plane.sigmas[0] = fabs(normal.coefficients[0]*0.1);
        plane.sigmas[1] = fabs(normal.coefficients[1]*0.1);
        plane.sigmas[2] = fabs(normal.coefficients[3]*0.1);
        plane.inlier = normal.inliers;
        plane.boundary_inlier = normal.boundary_inlier;
        plane.hull_inlier = normal.hull_inlier;
        projectPoints( *input, plane.inlier, plane.coefficients, *(plane.cloud) );
        getPointCloudFromIndices( input, plane.boundary_inlier, plane.cloud_boundary );
        getPointCloudFromIndices( input, plane.hull_inlier, plane.cloud_hull );
        //
        planes.push_back( plane );
    }
}

void KinectListener::organizedPlaneSegment(PointCloudTypePtr &input, std::vector<PlaneType> &planes)
{
    // copy input cloud
    PointCloudTypePtr cloud_in ( new PointCloudType );
    pcl::copyPointCloud( *input, *cloud_in );
    // segment planes
    OrganizedPlaneSegmentResult result;
    organized_plane_segment_.segment( cloud_in, result);
    // store planes
    for(int i = 0; i < result.model_coeffs.size(); i++)
    {
        pcl::ModelCoefficients &coef = result.model_coeffs[i];
        pcl::PointIndices &indices = result.inlier_indices[i];
        pcl::PlanarRegion<PointType> &pr = result.regions[i];
        PlaneType plane;
        Eigen::Vector3f centroid = pr.getCentroid();
        plane.centroid.x = centroid[0];
        plane.centroid.y = centroid[1];
        plane.centroid.z = centroid[2];
        plane.coefficients[0] = coef.values[0];
        plane.coefficients[1] = coef.values[1];
        plane.coefficients[2] = coef.values[2];
        plane.coefficients[3] = coef.values[3];
        plane.sigmas[0] = fabs(plane.coefficients[0]*0.1);
        plane.sigmas[1] = fabs(plane.coefficients[1]*0.1);
        plane.sigmas[2] = fabs(plane.coefficients[3]*0.1);
        //
        plane.inlier = indices.indices;
        plane.boundary_inlier = result.boundary_indices[i].indices;
        plane.hull_inlier = result.boundary_indices[i].indices;
        //
        projectPoints( *input, plane.inlier, plane.coefficients, *(plane.cloud) );
        plane.cloud_boundary->points = pr.getContour();
        plane.cloud_boundary->height = 1;
        plane.cloud_boundary->width = plane.cloud_boundary->points.size();
        plane.cloud_boundary->is_dense = false;
        *plane.cloud_hull = *plane.cloud_boundary;
//        getPointCloudFromIndices( input, plane.boundary_inlier, plane.cloud_boundary );
//        getPointCloudFromIndices( input, plane.hull_inlier, plane.cloud_hull );
        //
        planes.push_back( plane );
    }
}

void KinectListener::planeSlamReconfigCallback(plane_slam::PlaneSlamConfig &config, uint32_t level)
{
    do_slam_ = config.do_slam;
    map_frame_ = config.map_frame;
    base_frame_ = config.base_frame;
    odom_frame_ = config.odom_frame;
    plane_slam_->setPlaneMatchThreshold( config.plane_match_direction_threshold * M_PI / 180.0, config.plane_match_distance_threshold);
    plane_slam_->setPlaneMatchCheckOverlap( config.plane_match_check_overlap );
    plane_slam_->setPlaneInlierLeafSize( config.plane_inlier_leaf_size );
    plane_slam_->setPlaneHullAlpha( config.plane_hull_alpha );
    plane_slam_->setRefinePlanarMap( config.refine_planar_map );
    plane_slam_->setPlanarMergeThreshold( config.planar_merge_direction_threshold, config.planar_merge_distance_threshold );
    display_path_ = config.display_path;
    display_odom_path_ = config.display_odom_path;
    display_landmarks_ = config.display_landmarks;
    display_landmark_inlier_ = config.display_landmark_inlier;
    display_landmark_arrow_ = config.display_landmark_arrow;
    display_landmark_boundary_ = config.display_landmark_boundary;
    display_landmark_hull_ = config.display_landmark_hull;

    cout << GREEN <<"Common Slam Config." << RESET << endl;
}

void KinectListener::planeSegmentReconfigCallback(plane_slam::PlaneSegmentConfig &config, uint32_t level)
{
    cloud_size_type_config_ = config.cloud_size_type;
    plane_segment_method_ = config.segment_method;
    feature_detector_type_ = config.feature_detector_type;
    feature_extractor_type_ = config.feature_extractor_type;
    feature_good_match_threshold_ = config.feature_good_match_threshold;
    display_input_cloud_ = config.display_input_cloud;
    display_line_cloud_ = config.display_line_cloud;
    display_normal_ = config.display_normal;
    display_normal_arrow_ = config.display_normal_arrow;
    display_plane_ = config.display_plane;
    display_plane_number_ = config.display_plane_number;
    display_plane_arrow_ = config.display_plane_arrow;
    display_plane_inlier_ = config.display_plane_inlier;
    display_plane_projected_inlier_ = config.display_plane_projected_inlier;
    display_plane_boundary_ = config.display_plane_boundary;
    display_plane_hull_ = config.display_plane_hull;
    loop_one_message_ = config.loop_one_message;

    cout << GREEN <<"Common Segment Config." << RESET << endl;
}

void KinectListener::organizedSegmentReconfigCallback(plane_slam::OrganizedSegmentConfig &config, uint32_t level)
{
    //
    organized_plane_segment_.ne_method_ = config.organized_ne_method;
    organized_plane_segment_.ne_max_depth_change_factor_ = config.organized_ne_max_depth_change_factor;
    organized_plane_segment_.ne_normal_smoothing_size_ = config.organized_ne_normal_smoothing_size;
    //
    organized_plane_segment_.angular_threshold_ = config.organized_angular_threshold;
    organized_plane_segment_.distance_threshold_ = config.organized_distance_threshold;
    organized_plane_segment_.min_inliers_ = config.organized_min_inliers;
    organized_plane_segment_.project_bounding_points_ = config.organized_project_bounding_points;

    cout << GREEN <<"Organized Segment Config." << RESET << endl;
}

void KinectListener::lineBasedSegmentReconfigCallback( plane_slam::LineBasedSegmentConfig &config, uint32_t level)
{
    //
    use_horizontal_line_ = config.use_horizontal_line;
    use_verticle_line_ = config.use_verticle_line;
    y_skip_ = config.y_skip;
    x_skip_ = config.x_skip;
    line_point_min_distance_ = config.line_point_min_distance;
    use_depth_noise_model_ = config.use_depth_noise_model;
    scan_rho_constant_error_ = config.scan_rho_constant_error;
    scan_rho_distance_error_ = config.scan_rho_distance_error;
    scan_rho_quadratic_error_ = config.scan_rho_quadratic_error;
    slide_window_size_ = config.slide_window_size;
    line_min_inliers_ = config.line_min_inliers;
    line_fitting_threshold_ = config.line_fitting_threshold;
    //
    normals_per_line_ = config.normals_per_line;
    normal_use_depth_dependent_smoothing_ = config.normal_use_depth_dependent_smoothing;
    normal_max_depth_change_factor_ = config.normal_max_depth_change_factor;
    normal_smoothing_size_ = config.normal_smoothing_size;
    normal_min_inliers_percentage_ = config.normal_min_inliers_percentage;
    normal_maximum_curvature_ = config.normal_maximum_curvature;
    //
    remove_duplicate_candidate_ = config.remove_duplicate_candidate;
    duplicate_candidate_normal_thresh_ = config.duplicate_candidate_normal_thresh;
    duplicate_candidate_distance_thresh_ = config.duplicate_candidate_distance_thresh;
    //
    plane_segment_criterion_ = config.plane_segment_criterion;
    k_curvature_ = config.k_curvature;
    k_inlier_ = config.k_inlier;
    min_inliers_ = config.min_inliers;
    distance_threshold_ = config.distance_threshold;
    neighbor_threshold_ = config.neighbor_threshold;
    optimize_coefficients_ = config.optimize_coefficients;
    project_points_ = config.project_points;
    extract_boundary_ = config.extract_boundary;
    //

    cout << GREEN <<"Line Based Segment Config." << RESET << endl;

    is_update_line_based_parameters_ = true;
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

void KinectListener::displayMatched3DKeypoint( std_vector_of_eigen_vector4f &query,
                                               std_vector_of_eigen_vector4f &train,
                                               std::vector<cv::DMatch> &matches,
                                               const std::string &id)
{
    PointCloudXYZPtr query_cloud( new PointCloudXYZ ), train_cloud( new PointCloudXYZ );
    query_cloud->is_dense = false;
    query_cloud->points.resize( matches.size() );
    query_cloud->height = 1;
    query_cloud->width = matches.size();
    train_cloud->is_dense = false;
    train_cloud->points.resize( matches.size() );
    train_cloud->height = 1;
    train_cloud->width = matches.size();
    //
    PointCloudXYZ::iterator query_it = query_cloud->begin(), train_it = train_cloud->begin();
    for(int i = 0; i < matches.size(); i++, query_it++, train_it++)
    {
        if( query_it == query_cloud->end() )
            break;
        if( train_it == train_cloud->end() )
            break;

        pcl::PointXYZ &qp = *query_it;
        pcl::PointXYZ &tp = *train_it;
        const int qIdx = matches[i].queryIdx;
        const int tIdx = matches[i].trainIdx;
        qp.x = query[qIdx](0);
        qp.y = query[qIdx](1);
        qp.z = query[qIdx](2);
        tp.x = train[tIdx](0);
        tp.y = train[tIdx](1);
        tp.z = train[tIdx](2);
    }
    //
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> query_color( query_cloud, 0, 255, 0);
    pcl_viewer_->addPointCloud( query_cloud, query_color, id+"_query", viewer_v1_ );
    pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id+"_query", viewer_v1_ );
    //
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> train_color( train_cloud, 255, 255, 0);
    pcl_viewer_->addPointCloud( train_cloud, train_color, id+"_train", viewer_v3_ );
    pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id+"_train", viewer_v3_ );

}

void KinectListener::display3DKeypoint( std_vector_of_eigen_vector4f &feature_location_3d, const std::string &id, int viewport )
{
    PointCloudXYZPtr cloud( new PointCloudXYZ );
    cloud->is_dense = false;
    cloud->points.resize( feature_location_3d.size() );
    cloud->height = 1;
    cloud->width = feature_location_3d.size();

    PointCloudXYZ::iterator it = cloud->begin();
    for(int i = 0; i < feature_location_3d.size(); i++, it++)
    {
        if( it == cloud->end() )
            break;
        pcl::PointXYZ &pt = *it;
        pt.x = feature_location_3d[i](0);
        pt.y = feature_location_3d[i](1);
        pt.z = feature_location_3d[i](2);
    }

    //
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color( cloud, 255, 255, 0);
    pcl_viewer_->addPointCloud( cloud, color, id+"_3Dfeatures", viewport );
    pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id+"_3Dfeatures", viewport );

}

void KinectListener::displayKeypoint( const cv::Mat &visual, std::vector<cv::KeyPoint> &keypoints )
{
    cv::Mat image;
    cv::drawKeypoints( visual, keypoints, image, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
    cv::imshow( KeypointWindow, image );
//    cv::waitKey(1);
}

void KinectListener::displayLandmarks( const std::vector<PlaneType> &landmarks, const std::string &prefix)
{

    if(display_landmarks_)
    {
        int invalid_count = 0;
        for(int i = 0; i < landmarks.size(); i++)
        {
            const PlaneType & plane = landmarks[i];
            if( !plane.valid )
            {
                invalid_count ++;
                continue;
            }

            pcl::ModelCoefficients coeff;
            coeff.values.resize( 4 );
            coeff.values[0] = plane.coefficients[0];
            coeff.values[1] = plane.coefficients[1];
            coeff.values[2] = plane.coefficients[2];
            coeff.values[3] = plane.coefficients[3];
            //
            stringstream ss;
            ss << prefix << "_" << i;
//            map_viewer_->addPlane( coeff, 1.0, 1.0, 1.0, ss.str());
            pclViewerLandmark( plane, ss.str() );
        }

        cout << GREEN << " Display landmarks: " << landmarks.size() << ", invalid = " << invalid_count << RESET << endl;
    }
}

void KinectListener::displayPlanes( const PointCloudTypePtr &input, std::vector<PlaneType> &planes, const std::string &prefix, int viewport)
{
    if(display_plane_)
    {
        for(int j = 0; j < planes.size(); j++)
        {
            stringstream ss;
            ss << "_" << j;
            pclViewerPlane( input, planes[j], prefix + ss.str(), viewport, j);
        }
    }
}

void KinectListener::displayLinesAndNormals( const PointCloudTypePtr &input,
                                            std::vector<PlaneFromLineSegment::LineType> &lines,
                                            std::vector<PlaneFromLineSegment::NormalType> &normals,
                                            int viewport)
{
    if(display_input_cloud_)
    {
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> rgba_color(frame_current.point_cloud, 255, 255, 255);
        pcl_viewer_->addPointCloud( input, "rgba_cloud" );
        pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "rgba_cloud");
    }

    if(display_line_cloud_)
    {
        for(int j = 0; j < lines.size(); j++)
        {
            stringstream ss;
            ss << "line_" << j;
            pclViewerLineRegion( input, lines[j], ss.str(), viewport );
        }
    }

    if(display_normal_)
    {
        for(int j = 0; j < normals.size(); j++)
        {
            stringstream ss;
            ss << "normal_" << j;
            pclViewerNormal( input, normals[j], ss.str(), viewport );
        }
    }
}

void KinectListener::pclViewerLandmark( const PlaneType &plane, const std::string &id )
{
    // inlier
    if( display_landmark_inlier_ )
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color( plane.cloud, plane.color.Red, plane.color.Green, plane.color.Blue);
        map_viewer_->addPointCloud( plane.cloud, color, id+"_inlier" );
        map_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id+"_inlier" );

        if( display_landmark_arrow_ )
        {
            // add a line
            PointType p1, p2;
            p1 = plane.centroid;
            // check centroid
            if( p1.z == 0)
            {
                Eigen::Vector4f cen;
                pcl::compute3DCentroid( *plane.cloud, cen );
                p1.x = cen[0];
                p1.y = cen[1];
                p1.z = cen[2];
            }

            p2.x = p1.x + plane.coefficients[0]*0.4;
            p2.y = p1.y + plane.coefficients[1]*0.4;
            p2.z = p1.z + plane.coefficients[2]*0.4;
            map_viewer_->addArrow(p2, p1, plane.color.Red/255.0, plane.color.Green/255.0, plane.color.Blue/255.0, false, id+"_arrow" );
            // add a sphere
        //    pcl_viewer_->addSphere( p1, 0.05, 255.0, 255.0, 0.0, id+"_point" );
        }
    }

    // boundary
    if( display_landmark_boundary_ )
    {
        double r = rng.uniform(0.0, 255.0);
        double g = rng.uniform(0.0, 255.0);
        double b = rng.uniform(0.0, 255.0);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color_boundary( plane.cloud_boundary, r, g, b);
        map_viewer_->addPointCloud( plane.cloud_boundary, color_boundary, id+"_boundary" );
        map_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id+"_boundary" );
    }

    // hull
    if( display_landmark_hull_ )
    {
        double r = rng.uniform(0.0, 1.0);
        double g = rng.uniform(0.0, 1.0);
        double b = rng.uniform(0.0, 1.0);

        const int num = plane.cloud_hull->size();
        if( num >= 3)
        {
            for(int i = 1; i < num; i++)
            {
                stringstream ss;
                ss << id << "_hull_line_" << i;
                map_viewer_->addLine(plane.cloud_hull->points[i-1], plane.cloud_hull->points[i], r, g, b, ss.str() );
            }
            stringstream ss;
            ss << id << "_hull_line_0";
            map_viewer_->addLine(plane.cloud_hull->points[0], plane.cloud_hull->points[num-1], r, g, b, ss.str() );
        }
    }

}

void KinectListener::pclViewerLineRegion( const PointCloudTypePtr &input, PlaneFromLineSegment::LineType &line, const std::string &id, int viewpoint)
{
    PointCloudTypePtr cloud = getPointCloudFromIndices( input, line.inliers );

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color(cloud, rng.uniform(0.0, 255.0), rng.uniform(0.0, 255.0), rng.uniform(0.0, 255.0));
    pcl_viewer_->addPointCloud(cloud, color, id, viewpoint);
    pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, id, viewpoint);

}

//void KinectListener::pclViewerNormal( const PointCloudTypePtr &input, PlaneFromLineSegment::NormalType &normal, const std::string &id, int viewpoint)
//{
//    PlaneType plane;
//    plane.centroid = normal.centroid;
//    plane.coefficients[0] = normal.coefficients[0];
//    plane.coefficients[1] = normal.coefficients[1];
//    plane.coefficients[2] = normal.coefficients[2];
//    plane.coefficients[3] = normal.coefficients[3];
//    plane.inlier = normal.inliers;

//    pclViewerPlane( input, plane, id, viewpoint);
//}

void KinectListener::pclViewerNormal( const PointCloudTypePtr &input, PlaneFromLineSegment::NormalType &normal, const std::string &id, int viewpoint)
{
    double r = rng.uniform(0.0, 255.0);
    double g = rng.uniform(0.0, 255.0);
    double b = rng.uniform(0.0, 255.0);

    if( display_normal_arrow_ )
    {
        // add a line
        PointType p1, p2;
        p1 = normal.centroid;
        // check centroid
        if( p1.z == 0)
        {
            Eigen::Vector4f cen;
            pcl::compute3DCentroid( *input, normal.inliers, cen);
            p1.x = cen[0];
            p1.y = cen[1];
            p1.z = cen[2];
        }

        p2.x = p1.x + normal.coefficients[0]*0.2;
        p2.y = p1.y + normal.coefficients[1]*0.2;
        p2.z = p1.z + normal.coefficients[2]*0.2;
        pcl_viewer_->addArrow(p2, p1, r/255.0, g/255.0, b/255.0, false, id+"_arrow", viewpoint);
    // add a sphere
//    pcl_viewer_->addSphere( p1, 0.05, 255.0, 255.0, 0.0, id+"_point", viewpoint);
    }
    // add inlier
    PointCloudTypePtr cloud (new PointCloudType );

    for(int i = 0; i < normal.inliers.size(); i++)
    {
        cloud->points.push_back( input->points[normal.inliers[i]] );
    }
    cloud->is_dense = false;
    cloud->height = 1;
    cloud->width = cloud->points.size();

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color(cloud, r, g, b);
    pcl_viewer_->addPointCloud(cloud, color, id+"_inlier", viewpoint);
    pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id+"_inlier", viewpoint);

}

void KinectListener::pclViewerPlane( const PointCloudTypePtr &input, PlaneType &plane, const std::string &id, int viewport, int number)
{
    double r = rng.uniform(0.0, 255.0);
    double g = rng.uniform(0.0, 255.0);
    double b = rng.uniform(0.0, 255.0);

    // inlier
    if( display_plane_inlier_ && display_plane_projected_inlier_ )
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color( plane.cloud, r, g, b);
        pcl_viewer_->addPointCloud( plane.cloud, color, id+"_inlier", viewport);
        pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id+"_inlier", viewport);

        if( display_plane_arrow_ )
        {
            // add a line
            PointType p1, p2;
            p1 = plane.centroid;
            // check centroid
            if( p1.z == 0)
            {
                Eigen::Vector4f cen;
                pcl::compute3DCentroid( *plane.cloud, cen );
                p1.x = cen[0];
                p1.y = cen[1];
                p1.z = cen[2];
            }

            p2.x = p1.x + plane.coefficients[0]*0.2;
            p2.y = p1.y + plane.coefficients[1]*0.2;
            p2.z = p1.z + plane.coefficients[2]*0.2;
            //
            pcl_viewer_->addArrow(p2, p1, r, g, b, false, id+"_arrow", viewport);
            // add a sphere
        //    pcl_viewer_->addSphere( p1, 0.05, 255.0, 255.0, 0.0, id+"_point", viewport);
        }

        if(display_plane_number_)
        {
            // add a plane number
            PointType p1, p2;
            p1 = plane.centroid;
            //
            p2.x = p1.x + plane.coefficients[0]*0.05;
            p2.y = p1.y + plane.coefficients[1]*0.05;
            p2.z = p1.z + plane.coefficients[2]*0.05;
            // add plane number
            stringstream ss;
            ss << number;
            pcl_viewer_->addText3D( ss.str(), p2, 0.05, 1.0, 1.0, 1.0, id+"_number", viewport+1);
        }
    }
    else if( display_plane_inlier_ )
    {
        PointCloudTypePtr cloud = getPointCloudFromIndices( input, plane.inlier );

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color(cloud, r, g, b);
        pcl_viewer_->addPointCloud(cloud, color, id+"_inlier", viewport);
        pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id+"_inlier", viewport);

        if( display_plane_arrow_ )
        {
            // add a line
            PointType p1, p2;
            p1 = plane.centroid;
            // check centroid
            if( p1.z == 0)
            {
                Eigen::Vector4f cen;
                pcl::compute3DCentroid( *cloud, cen);
                p1.x = cen[0];
                p1.y = cen[1];
                p1.z = cen[2];
            }

            p2.x = p1.x + plane.coefficients[0]*0.2;
            p2.y = p1.y + plane.coefficients[1]*0.2;
            p2.z = p1.z + plane.coefficients[2]*0.2;
            //
            pcl_viewer_->addArrow(p2, p1, r, g, b, false, id+"_arrow", viewport);
            // add a sphere
        //    pcl_viewer_->addSphere( p1, 0.05, 255.0, 255.0, 0.0, id+"_point", viewport);
        }

        if(display_plane_number_)
        {
            // add a plane number
            PointType p1, p2;
            p1 = plane.centroid;
            //
            p2.x = p1.x + plane.coefficients[0]*0.05;
            p2.y = p1.y + plane.coefficients[1]*0.05;
            p2.z = p1.z + plane.coefficients[2]*0.05;
            // add plane number
            stringstream ss;
            ss << number;
            pcl_viewer_->addText3D( ss.str(), p2, 0.05, 1.0, 1.0, 1.0, id+"_number", viewport+1);
        }
    }

    // boundary
    if( display_plane_boundary_ && display_plane_projected_inlier_ )
    {
        r = rng.uniform(0.0, 255.0);
        g = rng.uniform(0.0, 255.0);
        b = rng.uniform(0.0, 255.0);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color_boundary( plane.cloud_boundary, r, g, b);
        pcl_viewer_->addPointCloud( plane.cloud_boundary, color_boundary, id+"_boundary", viewport );
        pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id+"_boundary", viewport);
    }
    else if( display_plane_boundary_ )
    {
        PointCloudTypePtr cloud_boundary = getPointCloudFromIndices( input, plane.boundary_inlier );
        r = rng.uniform(0.0, 255.0);
        g = rng.uniform(0.0, 255.0);
        b = rng.uniform(0.0, 255.0);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color_boundary( cloud_boundary, r, g, b);
        pcl_viewer_->addPointCloud( cloud_boundary, color_boundary, id+"_boundary", viewport );
        pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id+"_boundary", viewport);

    }

    // hull
    if( display_plane_hull_ && display_plane_projected_inlier_ )
    {
        r = rng.uniform(0.0, 1.0);
        g = rng.uniform(0.0, 1.0);
        b = rng.uniform(0.0, 1.0);

        const int num = plane.cloud_hull->size();
        if( num >= 3)
        {
            for(int i = 1; i < num; i++)
            {
                stringstream ss;
                ss << id << "_hull_line_" << i;
                pcl_viewer_->addLine(plane.cloud_hull->points[i-1], plane.cloud_hull->points[i], r, g, b, ss.str(), viewport );
            }
            stringstream ss;
            ss << id << "_hull_line_0";
            pcl_viewer_->addLine(plane.cloud_hull->points[0], plane.cloud_hull->points[num-1], r, g, b, ss.str(), viewport );
        }

//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color_hull( plane.cloud_hull, r, g, b);
//        pcl_viewer_->addPointCloud( plane.cloud_hull, color_hull, id+"_hull", viewpoint );
//        pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id+"_hull", viewport );

    }
    else if( display_plane_hull_ )
    {
        PointCloudTypePtr cloud_hull = getPointCloudFromIndices( input, plane.hull_inlier );
        r = rng.uniform(0.0, 1.0);
        g = rng.uniform(0.0, 1.0);
        b = rng.uniform(0.0, 1.0);

        const int num = cloud_hull->size();
        if( num >= 3)
        {
            for(int i = 1; i < num; i++)
            {
                stringstream ss;
                ss << id << "_hull_line_" << i;
                pcl_viewer_->addLine(cloud_hull->points[i-1], cloud_hull->points[i], r, g, b, ss.str(), viewport );
            }
            stringstream ss;
            ss << id << "_hull_line_0";
            pcl_viewer_->addLine(cloud_hull->points[0], cloud_hull->points[num-1], r, g, b, ss.str(), viewport );
        }
    }
}

void KinectListener::getPointCloudFromIndices( const PointCloudTypePtr &input,
                                               pcl::PointIndices &indices,
                                               PointCloudTypePtr &output)
{
    output->clear();
    for(int i = 0; i < indices.indices.size(); i++)
    {
        output->points.push_back( input->points[ indices.indices[i] ]);
    }
    output->is_dense = false;
    output->height = 1;
    output->width = output->points.size();
}

void KinectListener::getPointCloudFromIndices( const PointCloudTypePtr &input,
                                               std::vector<int> &indices,
                                               PointCloudTypePtr &output)
{
    output->clear();
    for(int i = 0; i < indices.size(); i++)
    {
        output->points.push_back( input->points[ indices[i] ]);
    }
    output->is_dense = false;
    output->height = 1;
    output->width = output->points.size();
}

PointCloudTypePtr KinectListener::getPointCloudFromIndices( const PointCloudTypePtr &input,
                                             pcl::PointIndices &indices)
{
    PointCloudTypePtr output (new PointCloudType );
    getPointCloudFromIndices( input, indices, output);
    return output;
}

PointCloudTypePtr KinectListener::getPointCloudFromIndices( const PointCloudTypePtr &input,
                                             std::vector<int> &indices)
{
    PointCloudTypePtr output (new PointCloudType );
    getPointCloudFromIndices( input, indices, output);
    return output;
}

void KinectListener::downsampleOrganizedCloud(const PointCloudTypePtr &input, PointCloudTypePtr &output,
                                            PlaneFromLineSegment::CAMERA_PARAMETERS &out_camera, int size_type)
{
    int skip = pow(2, size_type);
    int width = input->width / skip;
    int height = input->height / skip;
    output->width = width;
    output->height = height;
    output->is_dense = false;
    output->points.resize( width * height);
    for( size_t i = 0, y = 0; i < height; i++, y+=skip)
    {
        for( size_t j = 0, x = 0; j < width; j++, x+=skip)
        {
            output->points[i*width + j] = input->points[input->width*y + x];
        }
    }

    out_camera.width = width;
    out_camera.height = height;
    out_camera.cx = width / 2 - 0.5;
    out_camera.cy = height / 2 - 0.5;
    out_camera.fx = 525.0 / skip;
    out_camera.fy = 525.0 / skip;
    out_camera.scale = 1.0;
}

void KinectListener::downsampleImage(const cv::Mat &input, cv::Mat &output, int size_type)
{
    int skip = pow(2, size_type);
    int width = input.cols / skip;
    int height = input.rows / skip;

    cout << GREEN << " - downsample image size: " << width << "x" << height << RESET << endl;

    cv::pyrDown( input, output, cv::Size( width, height ) );
}

void KinectListener::getCameraParameter(const sensor_msgs::CameraInfoConstPtr &cam_info_msg,
                                      PlaneFromLineSegment::CAMERA_PARAMETERS &camera)
{
    /* Intrinsic camera matrix for the raw (distorted) images.
         [fx  0 cx]
     K = [ 0 fy cy]
         [ 0  0  1] */
    camera.cx = cam_info_msg->K[2];
    camera.cy = cam_info_msg->K[5];
    camera.fx = cam_info_msg->K[0];
    camera.fy = cam_info_msg->K[4];
    camera.scale = 1.0;
    // Additionally, organized cloud width and height.
    camera.width = cam_info_msg->width;
    camera.height = cam_info_msg->height;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr KinectListener::image2PointCloud( const cv::Mat &rgb_img, const cv::Mat &depth_img,
                                                                        const PlaneFromLineSegment::CAMERA_PARAMETERS& camera )
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud ( new pcl::PointCloud<pcl::PointXYZRGBA> );
    cloud->is_dense = false;
    cloud->width = depth_img.cols;
    cloud->height = depth_img.rows;
    cloud->points.resize(cloud->width * cloud->height);

    const double fx = 1.0 / camera.fx;
    const double fy = 1.0 / camera.fy;
//    const double min_depth = range_min_depth_;
    const double min_depth = 0.1;
    pcl::PointCloud<pcl::PointXYZRGBA>::iterator pt_iter = cloud->begin();
    int depth_idx = 0;
    int color_idx = 0;
    int color_skip_idx = 3;
    for (int v = 0; v < depth_img.rows; v++)
    {
        for (int u = 0; u < depth_img.cols; u++)
        {
            if(pt_iter == cloud->end())
            {
                break;
            }
            pcl::PointXYZRGBA &pt = *pt_iter;
            float Z = depth_img.at<float>(depth_idx);
            // Check for invalid measurements
            if (Z <= min_depth) //Should also be trigger on NaN//std::isnan (Z))
            {
                pt.x = (u - camera.cx) * 1.0 * fx; //FIXME: better solution as to act as at 1meter?
                pt.y = (v - camera.cy) * 1.0 * fy;
                pt.z = std::numeric_limits<float>::quiet_NaN();
            }
            else // Fill in XYZ
            {
                pt.x = (u - camera.cx) * Z * fx;
                pt.y = (v - camera.cy) * Z * fy;
                pt.z = Z;
            }

            RGBValue color;

            color.Blue = rgb_img.at<uint8_t>(color_idx);
            color.Green = rgb_img.at<uint8_t>(color_idx+1);
            color.Red = rgb_img.at<uint8_t>(color_idx+2);
            color.Alpha = 1.0;
            pt.rgb = color.float_value;
            //
            pt_iter ++;
            depth_idx ++;
            color_idx += color_skip_idx;
        }
    }

    return cloud;
}

bool KinectListener::getOdomPose( tf::Transform &odom_pose, const std::string &camera_frame, const ros::Time &time)
{
    // get transform
    tf::StampedTransform trans;
    try{
        tf_listener_.lookupTransform(odom_frame_, camera_frame, time, trans);
    }catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        return false;
    }
    odom_pose.setOrigin( trans.getOrigin() );
    odom_pose.setRotation( trans.getRotation() );

//    Eigen::Matrix3d m33 = matrixTF2Eigen( trans.getBasis() );
//    gtsam::Rot3 rot3(m33);
//    gtsam::Point3 point3;
//    point3[0] = trans.getOrigin()[0];
//    point3[1] = trans.getOrigin()[1];
//    point3[2] = trans.getOrigin()[2];
//    odom_pose = gtsam::Pose3( rot3, gtsam::Point3(trans.getOrigin()) );

    return true;
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

bool KinectListener::autoSpinMapViewerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    auto_spin_map_viewer_ = true;
    ROS_INFO("Auto spin map viewer for 30 seconds.");
    ros::Time time = ros::Time::now() + ros::Duration(30.0);
    ros::Rate loop_rate( 50 );
    while( auto_spin_map_viewer_ && nh_.ok())
    {
        if( !nh_.ok() )
        {
            res.message = "Node shutdown.";
            return true;
        }

        map_viewer_->spinOnce( 20 );
        loop_rate.sleep();

        if( ros::Time::now() > time )
            auto_spin_map_viewer_ = false;
    }
    res.success = true;
    return true;
}

///Analog to opencv example file and modified to use adjusters
cv::FeatureDetector* KinectListener::createDetector( const std::string& detectorType )
{
    cv::FeatureDetector* fd = 0;
    if( !detectorType.compare( "FAST" ) )
    {
//        fd = new cv::FastFeatureDetector( 20/*threshold*/, true/*nonmax_suppression*/ );
        fd = new cv::FastFeatureDetector();
    }
    else if( !detectorType.compare( "SURF" ) )
    {
        fd = new cv::SurfFeatureDetector(200.0, 6, 5);
//        fd = new cv::SurfFeatureDetector();
    }
    else if( !detectorType.compare( "ORB" ) )
    {
        fd = new cv::OrbFeatureDetector();
    }
    else if( !detectorType.compare( "SIFT" ) )
    {
        fd = new cv::SiftFeatureDetector();
    }
    else
    {
        ROS_WARN("No valid detector-type given: %s. Using SURF.", detectorType.c_str());
        fd = createDetector("SURF"); //recursive call with correct parameter
    }
    ROS_ERROR_COND(fd == 0, "No detector could be created");
    return fd;
}

cv::DescriptorExtractor* KinectListener::createDescriptorExtractor( const string& descriptorType )
{
    cv::DescriptorExtractor* extractor = 0;
    if( !descriptorType.compare( "SURF" ) )
    {
        extractor = new cv::SurfDescriptorExtractor();/*( int nOctaves=4, int nOctaveLayers=2, bool extended=false )*/
    }
    else if( !descriptorType.compare( "ORB" ) )
    {
        extractor = new cv::OrbDescriptorExtractor();
    }
    else if( !descriptorType.compare( "SIFT" ) )
    {
        extractor = new cv::SiftDescriptorExtractor();/*( double magnification=SIFT::DescriptorParams::GET_DEFAULT_MAGNIFICATION(), bool isNormalize=true, bool recalculateAngles=true, int nOctaves=SIFT::CommonParams::DEFAULT_NOCTAVES, int nOctaveLayers=SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS, int firstOctave=SIFT::CommonParams::DEFAULT_FIRST_OCTAVE, int angleMode=SIFT::CommonParams::FIRST_ANGLE )*/
    }
    else
    {
        ROS_ERROR("No valid descriptor-matcher-type given: %s. Using SURF", descriptorType.c_str());
        extractor = createDescriptorExtractor("SURF");
    }
    ROS_ERROR_COND(extractor == 0, "No extractor could be created");
    return extractor;
}
