#include "kinect_listener.h"


const std::string KeypointWindow = "Keypoint";
const std::string MatchesWindow = "MatchesFeatures";

KinectListener::KinectListener() :
    private_nh_("~")
  , plane_slam_config_server_( ros::NodeHandle( "PlaneSlam" ) )
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
  , real_camera_parameters_()
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
    map_viewer_->setCameraPosition(0.0, 0.0, -2.4, 0, 0, 0.6, 0, -1, 0);
//    map_viewer_->setCameraPosition( 0, 3.0, 3.0, -3.0, 0, 0, -1, -1, 0 );
    map_viewer_->setShowFPS(true);

    //
    cv::namedWindow( KeypointWindow );
    cv::namedWindow( MatchesWindow );

    // feature detector
    detector_ = createDetector( feature_detector_type_ );
    extractor_ = createDescriptorExtractor( feature_extractor_type_ );
    cout << GREEN << "Create feature detector: " << feature_detector_type_
         << ", descriptor: " << feature_extractor_type_ << RESET << endl;

    //
    int nFeatures = 1000;
    float fScaleFactor = 1.2;
    int nLevels = 8;
    int fIniThFAST = 20;
    int fMinThFAST = 7;
    orb_extractor_ = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

    true_path_publisher_ = nh_.advertise<nav_msgs::Path>("true_path", 10);
    odometry_path_publisher_ = nh_.advertise<nav_msgs::Path>("odometry_path", 10);
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

    printf("no cloud msg: %d\n", depth_img_msg->header.seq);

    skip = (skip + 1) % skip_message_;
    if( skip )
        return;

    // Get odom pose
    tf::Transform odom_pose;
    if( !getOdomPose( odom_pose, depth_img_msg->header.frame_id, ros::Time(0) ) )
    {
        odom_pose.setIdentity();
        return;
    }

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
    processFrame( current_frame, odom_pose );

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

    skip = (skip + 1) % skip_message_;
    if( skip )
        return;

    // Get odom pose
    tf::Transform odom_pose;
    if( !getOdomPose( odom_pose, depth_img_msg->header.frame_id, ros::Time(0) ) )
    {
        odom_pose.setIdentity();
        ROS_WARN("Set odom pose to identity.");
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
    processCloud( current_frame );
}

void KinectListener::trackDepthImage( const sensor_msgs::ImageConstPtr &depth_img_msg,
                                      PlaneFromLineSegment::CAMERA_PARAMETERS &camera_parameters )
{
    static int skip = 0;

    printf("depth msg: %d\n", depth_img_msg->header.seq);

    skip = (skip + 1) % skip_message_;
    if( skip )
        return;

    // Current Frame
    KinectFrame current_frame;
    camera_parameters_ = camera_parameters;

    // Get Mat Image
    current_frame.depth_image = cv_bridge::toCvCopy(depth_img_msg)->image; // to cv image
    current_frame.visual_image = cv::Mat::ones( current_frame.depth_image.rows, current_frame.depth_image.cols, CV_8UC3 );
    depthToCV8UC1( current_frame.depth_image, current_frame.depth_mono );
    // Get PointCloud
    current_frame.cloud = image2PointCloud( current_frame.visual_image, current_frame.depth_image, camera_parameters_);

    // Process data
    processCloud( current_frame );

}

void KinectListener::trackDepthRgbImage( const sensor_msgs::ImageConstPtr &depth_img_msg,
                                         const sensor_msgs::ImageConstPtr &visual_img_msg,
                                         const PlaneFromLineSegment::CAMERA_PARAMETERS & camera)
{
    static int skip = 0;

    printf("no cloud msg: %d\n", depth_img_msg->header.seq);

    skip = (skip + 1) % skip_message_;
    if( skip )
        return;

    // Current Frame
    KinectFrame current_frame;
    camera_parameters_ = camera;

    // Get Mat Image
    current_frame.visual_image = cv_bridge::toCvCopy(visual_img_msg)->image; // to cv image
    current_frame.depth_image = cv_bridge::toCvCopy(depth_img_msg)->image; // to cv image
    depthToCV8UC1( current_frame.depth_image, current_frame.depth_mono );
    // Get PointCloud
    current_frame.cloud = image2PointCloud( current_frame.visual_image, current_frame.depth_image, camera_parameters_);

    // Process data
    processFrame( current_frame );
}

void KinectListener::processCloud( KinectFrame &frame, const tf::Transform &odom_pose )
{
    static gtsam::Pose3 estimated_pose;
    static KinectFrame last_frame;

    static int frame_count = 0;
    static int good_frame_count = 0;
    frame_count ++;

    // if use downsample cloud
    cloud_size_type_ = cloud_size_type_config_;
    if( cloud_size_type_ == QVGA)
    {
        cout << GREEN << "QVGA" << RESET << endl;
        downsampleOrganizedCloud( frame.cloud, camera_parameters_, frame.cloud_in, real_camera_parameters_, (int)QVGA);
    }
    else if( cloud_size_type_ == QQVGA)
    {
        cout << GREEN << "QQVGA" << RESET << endl;
        downsampleOrganizedCloud( frame.cloud, camera_parameters_, frame.cloud_in, real_camera_parameters_, (int)QQVGA);
    }
    else
    {
        cout << GREEN << "VGA" << RESET << endl;
        frame.cloud_in = frame.cloud; // copy pointer
        real_camera_parameters_ = camera_parameters_;
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
        if(!is_initialized && frame.segment_planes.size() >= 3)
        {
            gtsam::Pose3 init_pose = gtsam::Pose3::identity();
            if( plane_slam_->initialize( init_pose, frame ) )
            {
                is_initialized = true;
                estimated_pose = init_pose;
                good_frame_count ++;

                // visualize landmark
                landmarks = plane_slam_->getLandmarks();
            }
            else
                return;
        }
        else
        {
            RESULT_OF_MOTION motion;
            std::vector<PlanePair> pairs;
            ITree::euclidianPlaneCorrespondences( frame.segment_planes, last_frame.segment_planes, pairs );
            motion.valid = solveRelativeTransformPlanes( last_frame, frame, pairs, motion );
            if( motion.valid )
            {
                // print motion
                gtsam::Rot3 rot3( motion.rotation );
                cout << MAGENTA << " estimated motion, rmse = " << motion.rmse << endl;
                cout << "  - R(rpy): " << rot3.roll() << ", " << rot3.pitch() << ", " << rot3.yaw() << endl;
                cout << "  - T:      " << motion.translation[0]
                     << ", " << motion.translation[1]
                     << ", " << motion.translation[2] << RESET << endl;

                good_frame_count ++;
            }
            if( !motion.valid )
            {
                cout << RED << " failed to estimate relative motion. " << RESET << endl;
            }

            if( motion.valid )
            {
                gtsam::Pose3 rel( gtsam::Rot3(motion.rotation), gtsam::Point3(motion.translation) );
                gtsam::Pose3 pose3 = estimated_pose * rel;
                gtsam::Pose3 estimated_pose_old = estimated_pose;
                estimated_pose = plane_slam_->planeSlam( pose3, frame );
                publishPose( estimated_pose );
                if(display_path_)
                    plane_slam_->publishEstimatedPath();
                if(display_odom_path_)
                    plane_slam_->publishOdomPath();

                // project and recalculate contour

                //
                gtsam::Pose3 gtsam_relpose = estimated_pose_old.inverse() * estimated_pose;
                cout << WHITE << " gtsam relative motion :" << endl;
                cout << "  - R(rpy): " << gtsam_relpose.rotation().roll()
                     << ", " << gtsam_relpose.rotation().pitch()
                     << ", " << gtsam_relpose.rotation().yaw() << endl;
                cout << "  - T:      " << gtsam_relpose.translation().x()
                     << ", " << gtsam_relpose.translation().y()
                     << ", " << gtsam_relpose.translation().z() << RESET << endl;
            }

            // visualize landmark
            landmarks = plane_slam_->getLandmarks();
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
    cout << GREEN << "Frame count = " << frame_count << ", good frame count = " << good_frame_count << RESET << endl;
    cout << "----------------------------------- END -------------------------------------" << endl;

    // For next calculation
    last_frame = frame; // store frame
}

void KinectListener::processFrame( KinectFrame &frame, const tf::Transform &odom_pose )
{
    static gtsam::Pose3 estimated_pose;
    static KinectFrame last_frame;
    static gtsam::Pose3 last_odom_pose;
    static gtsam::Pose3 visual_odometry_pose;

    //
    gtsam::Pose3 odom_pose3 = tfToPose3( odom_pose );

    // if use downsample cloud
    cloud_size_type_ = cloud_size_type_config_;
    if( cloud_size_type_ == QVGA)
    {
        cout << GREEN << "QVGA" << RESET << endl;
        downsampleOrganizedCloud( frame.cloud, camera_parameters_, frame.cloud_in, real_camera_parameters_, (int)QVGA);
        downsampleImage( frame.visual_image, frame.visual, (int)QVGA );
    }
    else if( cloud_size_type_ == QQVGA)
    {
        cout << GREEN << "QQVGA" << RESET << endl;
        downsampleOrganizedCloud( frame.cloud, camera_parameters_, frame.cloud_in, real_camera_parameters_, (int)QQVGA);
        downsampleImage( frame.visual_image, frame.visual, (int)QQVGA );
    }
    else
    {
        cout << GREEN << "VGA" << RESET << endl;
        frame.cloud_in = frame.cloud; // copy pointer
        frame.visual = frame.visual_image;
        real_camera_parameters_ = camera_parameters_;
    }

    double start_time = pcl::getTime();
    pcl::console::TicToc time;
    time.tic();
    float segment_dura = 0;
    float keypoint_dura = 0;
    float solveRT_dura = 0;
    float slam_dura = 0;
    float display_dura = 0;
    float total_dura = 0;

    // Plane Segment
    if( plane_segment_method_ == ORGANSIZED)
    {
        organizedPlaneSegment( frame.cloud_in, frame.segment_planes );
        cout << GREEN << "Organized segmentation, planes = " << frame.segment_planes.size() << RESET << endl;
    }
    else if( plane_segment_method_ == LINE_BADED )
    {
        lineBasedPlaneSegment( frame.cloud_in, frame.segment_planes );
        cout << GREEN << "Line based segmentation, planes = " << frame.segment_planes.size() << RESET << endl;
    }
    else
    {
        cout << RED << "[Error]: Invalid segmentation method error." << RESET << endl;
        exit(0);
    }
    segment_dura = time.toc();
    time.tic();

    // Compute Keypoints
    cout << GREEN << feature_detector_type_ << "-" << feature_extractor_type_ << RESET << endl;
    computeORBKeypoint( frame.visual_image, frame.cloud, frame.feature_locations_2d,
                     frame.feature_locations_3d, frame.feature_cloud, frame.feature_descriptors );
//    displayKeypoint( frame.visual_image, frame.feature_locations_2d );
    keypoint_dura = time.toc();
    time.tic();

    //
    std::vector<PlaneType> landmarks;
    if( do_visual_odometry_ ) // Do visual odometry
    {
        cout << GREEN << "Estimate visual odometry." << RESET << endl;

        if( !is_initialized )
        {
            // Initialize visual odometry pose
            visual_odometry_pose = odom_pose3;
            is_initialized = true;

            // first pose
            odometry_poses_.clear();
            odometry_poses_.push_back( pose3ToGeometryPose(visual_odometry_pose) );
            publishOdometryPath();

            // first true pose
            true_poses_.clear();
            true_poses_.push_back( pose3ToGeometryPose(odom_pose3) );
            publishTruePath();
        }
        else
        {
            // Estimate motion
            RESULT_OF_MOTION motion;
            std::vector<cv::DMatch> inlier;
            time.tic();
            motion.valid = solveRelativeTransform( last_frame, frame, motion, inlier );
            solveRT_dura = time.toc();
            time.tic();

            //
            if( !motion.valid )
                return;

            // New visual odometry pose
            gtsam::Pose3 rel( gtsam::Rot3(motion.rotation), gtsam::Point3(motion.translation) );
            gtsam::Pose3 o_pose = visual_odometry_pose * rel;
            visual_odometry_pose = o_pose;

            // Publish visual odometry pose
            odometry_poses_.push_back( pose3ToGeometryPose(visual_odometry_pose) );
            publishOdometryPath();

            // Publish true path
            true_poses_.push_back( tfToGeometryPose(odom_pose) );
            publishTruePath();
        }

    }
    else if( do_mapping_ )
    {

        if( !is_initialized ) // First frame, initialize slam system
        {
            if( plane_slam_->initialize( odom_pose3, frame ) )
            {
                is_initialized = true;
                last_odom_pose = odom_pose3; // store odom pose
                last_frame = frame; // store current frame
                cout << GREEN << "Initialized Planar SLAM." << RESET << endl;

                // True pose
                true_poses_.clear();
                true_poses_.push_back( pose3ToGeometryPose( odom_pose3 ) );
                publishTruePath();

                // Initialize visual odometry pose
                visual_odometry_pose = odom_pose3;
                odometry_poses_.clear();
                odometry_poses_.push_back( pose3ToGeometryPose(visual_odometry_pose) );
                publishOdometryPath();
            }
            else
                return;

        }
        else
        {
            // Estimate motion
            RESULT_OF_MOTION motion;
            std::vector<cv::DMatch> inlier;
            time.tic();
            motion.valid = solveRelativeTransform( last_frame, frame, motion, inlier );
            solveRT_dura = time.toc();
            time.tic();

            // Print motion info
            if( motion.valid )
            {
                // print motion
                gtsam::Rot3 rot3( motion.rotation );
                cout << MAGENTA << " estimated motion, rmse = " << motion.rmse << endl;
                cout << "  - R(rpy): " << rot3.roll()
                     << ", " << rot3.pitch()
                     << ", " << rot3.yaw() << endl;
                cout << "  - T:      " << motion.translation[0]
                     << ", " << motion.translation[1]
                     << ", " << motion.translation[2] << RESET << endl;

                gtsam::Pose3 real_r_pose = last_odom_pose.inverse() * odom_pose3;
                cout << CYAN << " true motion: " << endl;
                cout << "  - R(rpy): " << real_r_pose.rotation().roll()
                     << ", " << real_r_pose.rotation().pitch()
                     << ", " << real_r_pose.rotation().yaw() << endl;
                cout << "  - T:      " << real_r_pose.translation().x()
                     << ", " << real_r_pose.translation().y()
                     << ", " << real_r_pose.translation().z() << RESET << endl;
            }
            else
            {
                cout << RED << "Failed to estimate relative motion, exit. " << RESET << endl;
                return;
//                cout << RED << "Use Identity as transformation." << RESET << endl;
//                motion.setTransform4d( Eigen::Matrix4d::Identity() );
            }



            // Iteration
            gtsam::Pose3 rel( gtsam::Rot3(motion.rotation), gtsam::Point3(motion.translation) );
            gtsam::Pose3 pose3 = visual_odometry_pose * rel;
            visual_odometry_pose = pose3;
            if( use_keyframe_ )
            {
                if( rel.translation().norm() >= keyframe_linear_threshold_
                        || acos( cos(rel.rotation().yaw()) * cos(rel.rotation().pitch()) * cos(rel.rotation().roll()) ) > keyframe_angular_threshold_ )
                {
                    estimated_pose = plane_slam_->planeMapping( pose3, frame );
                    // visualize landmark
                    landmarks = plane_slam_->getLandmarks();
                }
                else
                {
                    return;
                }
            }
            else
            {
                if( motion.valid && frame.segment_planes.size() > 0)    // do slam
                {
                    estimated_pose = plane_slam_->planeMapping( pose3, frame );
                    // visualize landmark
                    landmarks = plane_slam_->getLandmarks();
                }
                else if( motion.valid && frame.segment_planes.size() == 0) // accumulate estimated pose
                {
                    estimated_pose = pose3;
                }
                else
                {
                    cout << RED << "[Error]: " << "Motion estimation failed, stop processing current frame." << RESET << endl;
                    return;
                }
            }
            // Publish pose and trajectory
            publishPose( pose3 );

            // Publish true path
            true_poses_.push_back( pose3ToGeometryPose(odom_pose3) );
            publishTruePath();

            // Publish visual odometry path
            odometry_poses_.push_back( pose3ToGeometryPose(visual_odometry_pose) );
            publishOdometryPath();

        }
    }
    else if( do_slam_ ) // Do slam
    {

//        // convert ax + by + cz-d = 0
//        std::vector<PlaneType> &planes = frame.segment_planes;
//        for( int i = 0; i < planes.size(); i++)
//        {
//            planes[i].coefficients[3] = -planes[i].coefficients[3];
//        }


        if( !is_initialized ) // First frame, initialize slam system
        {
            if( plane_slam_->initialize( odom_pose3, frame ) )
            {
                is_initialized = true;
                estimated_pose = odom_pose3; // store pose as initial pose
                last_odom_pose = odom_pose3; // store odom pose
                last_frame = frame; // store current frame
                cout << GREEN << "Initialized Planar SLAM." << RESET << endl;

                // True pose
                true_poses_.clear();
                true_poses_.push_back( pose3ToGeometryPose( odom_pose3 ) );
                publishTruePath();

                // Initialize visual odometry pose
                visual_odometry_pose = odom_pose3;
                odometry_poses_.clear();
                odometry_poses_.push_back( pose3ToGeometryPose(visual_odometry_pose) );
                publishOdometryPath();
            }
            else
                return;

        }
        else
        {
            // Estimate motion
            RESULT_OF_MOTION motion;
            std::vector<cv::DMatch> inlier;
            time.tic();
            motion.valid = solveRelativeTransform( last_frame, frame, motion, inlier );
            solveRT_dura = time.toc();
            time.tic();

            // Print motion info
            if( motion.valid )
            {
                // print motion
                gtsam::Rot3 rot3( motion.rotation );
                cout << MAGENTA << " estimated motion, rmse = " << motion.rmse << endl;
                cout << "  - R(rpy): " << rot3.roll()
                     << ", " << rot3.pitch()
                     << ", " << rot3.yaw() << endl;
                cout << "  - T:      " << motion.translation[0]
                     << ", " << motion.translation[1]
                     << ", " << motion.translation[2] << RESET << endl;

                gtsam::Pose3 real_r_pose = last_odom_pose.inverse() * odom_pose3;
                cout << CYAN << " true motion: " << endl;
                cout << "  - R(rpy): " << real_r_pose.rotation().roll()
                     << ", " << real_r_pose.rotation().pitch()
                     << ", " << real_r_pose.rotation().yaw() << endl;
                cout << "  - T:      " << real_r_pose.translation().x()
                     << ", " << real_r_pose.translation().y()
                     << ", " << real_r_pose.translation().z() << RESET << endl;
            }
            else
            {
                cout << RED << "Failed to estimate relative motion, exit. " << RESET << endl;
                return;
//                cout << RED << "Use Identity as transformation." << RESET << endl;
//                motion.setTransform4d( Eigen::Matrix4d::Identity() );
            }



            // Slam iteration
            gtsam::Pose3 rel( gtsam::Rot3(motion.rotation), gtsam::Point3(motion.translation) );
            gtsam::Pose3 pose3 = estimated_pose * rel;
            if( use_keyframe_ )
            {
                if( rel.translation().norm() >= keyframe_linear_threshold_
                        || acos( cos(rel.rotation().yaw()) * cos(rel.rotation().pitch()) * cos(rel.rotation().roll()) ) > keyframe_angular_threshold_ )
                {
                    estimated_pose = plane_slam_->planeSlam( pose3, frame );
                    // visualize landmark
                    landmarks = plane_slam_->getLandmarks();
                }
                else
                {
                    return;
                }
            }
            else
            {
                if( motion.valid && frame.segment_planes.size() > 0)    // do slam
                {
                    estimated_pose = plane_slam_->planeSlam( pose3, frame );
                    // visualize landmark
                    landmarks = plane_slam_->getLandmarks();
                }
                else if( motion.valid && frame.segment_planes.size() == 0) // accumulate estimated pose
                {
                    estimated_pose = pose3;
                }
                else
                {
                    cout << RED << "[Error]: " << "Motion estimation failed, stop processing current frame." << RESET << endl;
                    return;
                }
            }
            // Publish pose and trajectory
            publishPose( estimated_pose );

            // Publish true path
            true_poses_.push_back( tfToGeometryPose(odom_pose) );
            publishTruePath();

            // Publish visual odometry path
            gtsam::Pose3 o_pose = visual_odometry_pose * rel;
            visual_odometry_pose = o_pose;
            odometry_poses_.push_back( pose3ToGeometryPose(visual_odometry_pose) );
            publishOdometryPath();

            // Publish estimated path
            if( display_path_ )
                plane_slam_->publishEstimatedPath();
//            if( display_odom_path_ )
//                plane_slam_->publishOdomPath();


        }

//        // convert ax + by + cz - d = 0
//        for( int i = 0; i < landmarks.size(); i++)
//        {
//            landmarks[i].coefficients[3] = -landmarks[i].coefficients[3];
//        }
//        for( int i = 0; i < planes.size(); i++)
//        {
//            planes[i].coefficients[3] = -planes[i].coefficients[3];
//        }
    }

    slam_dura = time.toc();
    time.tic();


    // Display map
    if(display_landmarks_ && landmarks.size() > 0)
    {
        // Clear map display
        map_viewer_->removeAllPointClouds();
        map_viewer_->removeAllShapes();
        displayLandmarks( landmarks, "landmark");
        publishPlanarMap( landmarks );
        map_viewer_->spinOnce(1);
    }

    // Clear Display
    pcl_viewer_->removeAllPointClouds();
    pcl_viewer_->removeAllShapes();

    // display
    if( display_input_cloud_ )
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
//    displayMatched3DKeypoint( last_frame.feature_locations_3d, frame.feature_locations_3d, good_matches );
    pcl_viewer_->addText( "Last 3D Features", 200, 20, "viewer_v1_name", viewer_v1_);
    pcl_viewer_->addText( "Current 3D Features", 200, 20, "viewer_v3_name", viewer_v3_);
    pcl_viewer_->spinOnce(1);


    display_dura = time.toc();
    total_dura = (pcl::getTime() - start_time) * 1000;

    cout << GREEN << "Total time: " << total_dura << ", segment: " << segment_dura
         << ", keypoints: " << keypoint_dura << ", solveRT: " << solveRT_dura
         << ", slam: " << slam_dura << ", display: " << display_dura << RESET << endl;
    cout << "----------------------------------- END -------------------------------------" << endl;

    last_frame = frame; // store frame
    last_odom_pose = odom_pose3; // store odom pose
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
    use_keyframe_ = config.use_keyframe;
    keyframe_linear_threshold_ = config.keyframe_linear_threshold;
    keyframe_angular_threshold_ = config.keyframe_angular_threshold * DEG_TO_RAD;
    plane_slam_->setPlaneMatchThreshold( config.plane_match_direction_threshold * DEG_TO_RAD, config.plane_match_distance_threshold);
    plane_slam_->setPlaneMatchCheckOverlap( config.plane_match_check_overlap );
    plane_slam_->setPlaneMatchOverlapAlpha( config.plane_match_overlap_alpha );
    plane_slam_->setPlaneInlierLeafSize( config.plane_inlier_leaf_size );
    plane_slam_->setPlaneHullAlpha( config.plane_hull_alpha );
    plane_slam_->setRefinePlanarMap( config.refine_planar_map );
    plane_slam_->setPlanarMergeThreshold( config.planar_merge_direction_threshold * DEG_TO_RAD, config.planar_merge_distance_threshold );
    display_path_ = config.display_path;
    display_odom_path_ = config.display_odom_path;
    display_landmarks_ = config.display_landmarks;
    display_landmark_inlier_ = config.display_landmark_inlier;
    display_landmark_arrow_ = config.display_landmark_arrow;
    display_landmark_number_ = config.display_landmark_number;
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
    feature_min_good_match_size_ = config.feature_min_good_match_size;
    ransac_sample_size_ = config.ransac_sample_size;
    ransac_iterations_ = config.ransac_iterations;
    ransac_min_inlier_ = config.ransac_min_inlier;
    ransac_inlier_max_mahal_distance_ = config.ransac_inlier_max_mahal_distance;
    //
    icp_max_distance_ = config.icp_max_distance;
    icp_iterations_ = config.icp_iterations;
    icp_tf_epsilon_ = config.icp_tf_epsilon;
    icp_min_indices_ = config.icp_min_indices;
    icp_score_threshold_ = config.icp_score_threshold;
    //
    pnp_iterations_ = config.pnp_iterations;
    pnp_min_inlier_ = config.pnp_min_inlier;
    pnp_repreject_error_ = config.pnp_repreject_error;
    //
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
            pclViewerLandmark( plane, ss.str(), i);
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

void KinectListener::pclViewerLandmark( const PlaneType &plane, const std::string &id, const int number )
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

        if( display_landmark_number_ && number >= 0 )
        {
            // add a plane number
            PointType p1, p2;
            p1 = plane.centroid;
            //
            p2.x = p1.x + plane.coefficients[0]*0.43;
            p2.y = p1.y + plane.coefficients[1]*0.43;
            p2.z = p1.z + plane.coefficients[2]*0.43;
            // add plane number
            stringstream ss;
            ss << number;
            map_viewer_->addText3D( ss.str(), p2, 0.1, 1.0, 1.0, 1.0, id+"_number");
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

        if( display_plane_number_ && number >= 0 )
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

        if( display_plane_number_ && number >= 0 )
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

bool KinectListener::autoSpinMapViewerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    auto_spin_map_viewer_ = true;
    ROS_INFO("Auto spin map viewer for 30 seconds.");
    double dura = 30.0;
    int sec = 0;
    bool added = false;
    ros::Time time = ros::Time::now();
    ros::Time finish_time = time + ros::Duration(dura);
    ros::Rate loop_rate( 40 );
    while( auto_spin_map_viewer_ && ros::ok())
    {
        if( !ros::ok() || ros::isShuttingDown() )
        {
            res.message = "Node shutdown.";
            auto_spin_map_viewer_ = false;
            return true;
        }

        // info
        if( ros::Time::now() > (time + ros::Duration(1.0)) )
        {
            time += ros::Duration(1.0);
            sec ++;
            ROS_INFO("Spinning time %d of %d seconds...", sec, (int)dura);
            //
            stringstream ss;
            ss << "Spinning " << sec << "/" << ((int)dura) << " seconds";
            if( ! added )
            {
                map_viewer_->addText( ss.str(), 100, 20, "spinning_text" );
                added = true;
            }
            else
                map_viewer_->updateText( ss.str(), 100, 20, "spinning_text" );
        }

        map_viewer_->spinOnce( 20 );
        loop_rate.sleep();

        if( ros::Time::now() > finish_time )
            auto_spin_map_viewer_ = false;
    }

    map_viewer_->updateText( " ", 100, 20, "spinning_text" );
    map_viewer_->spinOnce( 20 );
    ROS_INFO("Stop spinning.");
    res.success = true;
    return true;
}

