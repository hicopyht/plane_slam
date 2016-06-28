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
//    private_nh_.param<string>("topic_image_visual", topic_image_visual_, "");
//    private_nh_.param<string>("topic_image_depth", topic_image_depth_, "/camera/depth_registered/image");
//    private_nh_.param<string>("topic_camera_info", topic_camera_info_, "/camera/depth_registered/camera_info");
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
    map_viewer_->setCameraPosition( 0, 3.0, 3.0, -3.0, 0, 0, -1, -1, 0 );
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

    float nan1 = std::numeric_limits<float>::quiet_NaN();
    cout << "Nan1 = " << nan1 << ", isnan = " << (std::isnan(nan1)?"true":"false") << endl;


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

    skip = (skip + 1) % 5;
    if( skip )
        return;

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

    skip = (skip + 1) % 5;
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

    skip = (skip + 1) % 5;
    if( skip )
        return;

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
    float hull_dura = 0;
    float keypoint_dura = 0;
    float match_dura = 0;
    float solveRT_dura = 0;
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
    cout << GREEN << feature_detector_type_ << "-" << feature_extractor_type_ << RESET << endl;
//    computeKeypoint( frame.visual_image, frame.cloud, frame.feature_locations_2d,
//                     frame.feature_locations_3d, frame.feature_cloud, frame.feature_descriptors, cv::Mat());
    computeORBKeypoint( frame.visual_image, frame.cloud, frame.feature_locations_2d,
                     frame.feature_locations_3d, frame.feature_cloud, frame.feature_descriptors );
//    displayKeypoint( frame.visual_image, frame.feature_locations_2d );
    keypoint_dura = time.toc();
    time.tic();

    // Match features
    vector<cv::DMatch> good_matches;
    if( is_initialized )
    {
        cout << GREEN << "Last features = " << last_frame.feature_locations_3d.size()
             << ", current features = " << frame.feature_locations_3d.size() << RESET << endl;
        matchImageFeatures( last_frame, frame, good_matches,
                            feature_good_match_threshold_, feature_min_good_match_size_ );
        cout << GREEN << "Match features th = " << feature_good_match_threshold_
             << ", good matches = " << good_matches.size() << RESET << endl;
        cv::Mat image_matches;
        cv::drawMatches( last_frame.visual_image, last_frame.feature_locations_2d,
                         frame.visual_image, frame.feature_locations_2d,
                         good_matches, image_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                         vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        cv::imshow( MatchesWindow, image_matches );
//        cv::waitKey( 1 );
    }
    cv::waitKey( 1 );
    match_dura = time.toc();
    time.tic();

    if( is_initialized )
    {
        RESULT_OF_MOTION motion;
        cout << YELLOW << "******************* Test estimate transformation *******************" << RESET << endl;
        std::vector<cv::DMatch> inlier;
        solveRelativeTransform( last_frame, frame, motion, inlier );
        cout << YELLOW << "********************************************************************" << RESET << endl;
    }

//    if( is_initialized && good_matches.size() > 40 )
//    {
//        std::vector<cv::DMatch> inlier;
//        inlier.assign( good_matches.begin(), good_matches.begin() + 40);
//        cout << BLUE << "Choose matches = " << inlier.size() << RESET << endl;
//        bool valid_tf;
//        Eigen::Matrix4f transform4f = getRelativeTransformPoints( last_frame.feature_locations_3d,
//                                                                  frame.feature_locations_3d,
//                                                                  inlier, valid_tf );
//        if( valid_tf )
//            printTransform( transform4f );
//        else
//            cout << RED << " Failed to solve transformation." << RESET << endl;
//    }

//    if( is_initialized )
//    {
//        RESULT_OF_MOTION motion;
//        std::vector<cv::DMatch> inlier;
//        bool res = solveRelativeTransformPointsRansac( last_frame, frame, good_matches, motion, inlier);
//        if( res )
//        {
//            // print motion
//            gtsam::Rot3 rot3( motion.rotation );
//            cout << YELLOW << " estimated motion RANSAC, inlier = " << inlier.size()
//                 << ", rmse = " << motion.rmse << endl;
//            cout << "  - R(rpy): " << rot3.roll() << ", " << rot3.pitch() << ", " << rot3.yaw() << endl;
//            cout << "  - T:      " << motion.translation[0]
//                 << ", " << motion.translation[1]
//                 << ", " << motion.translation[2] << RESET << endl;
//        }
//        if( !res )
//        {
//            cout << RED << " failed to estimate relative motion using RANSAC. " << RESET << endl;
//        }
//    }

//    if( is_initialized )
//    {
//        RESULT_OF_MOTION motion;
//        PointCloudXYZPtr cloud_icp( new PointCloudXYZ );
//        motion.valid = solveICP( frame.feature_cloud, last_frame.feature_cloud, cloud_icp, motion );

//        // print motion
//        gtsam::Rot3 rot3( motion.rotation );
//        cout << YELLOW << " estimated motion ICP, inlier = " << motion.inlier
//             << ", score = " << motion.score
//             << ", converged = " << (motion.valid?"true":"false") << endl;
//        cout << "  - R(rpy): " << rot3.roll() << ", " << rot3.pitch() << ", " << rot3.yaw() << endl;
//        cout << "  - T:      " << motion.translation[0]
//             << ", " << motion.translation[1]
//             << ", " << motion.translation[2] << RESET << endl;
//    }

//    if( is_initialized )
//    {
//        RESULT_OF_MOTION motion;
//        motion.valid = solveRelativeTransformPnP( last_frame, frame, good_matches, camera_parameters_, motion );

//        // print motion
//        gtsam::Rot3 rot3( motion.rotation );
//        cout << YELLOW << " estimated motion PnP, inlier = " << motion.inlier
//             << ", valid = " << (motion.valid?"true":"false") << endl;
//        cout << "  - R(rpy): " << rot3.roll() << ", " << rot3.pitch() << ", " << rot3.yaw() << endl;
//        cout << "  - T:      " << motion.translation[0]
//             << ", " << motion.translation[1]
//             << ", " << motion.translation[2] << RESET << endl;
//    }

    solveRT_dura = time.toc();
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
            RESULT_OF_MOTION motion;
            std::vector<PlanePair> pairs;
            ITree::euclidianPlaneCorrespondences( frame.segment_planes, last_frame.segment_planes, pairs );
            cout << GREEN << " plane pairs = " << pairs.size() << RESET << endl;
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

                cout << CYAN << " true motion: " << endl;
                cout << "  - R(rpy): " << r_pose.rotation().roll() << ", " << r_pose.rotation().pitch() << ", " << r_pose.rotation().yaw() << endl;
                cout << "  - T:      " << r_pose.translation().x()
                     << ", " << r_pose.translation().y()
                     << ", " << r_pose.translation().z() << RESET << endl;
            }
            if( !motion.valid )
            {
                cout << RED << " failed to estimate relative motion using planes. " << RESET << endl;
            }
            //
////            RESULT_OF_MOTION motion = estimateMotion( frame, last_frame, camera_parameters_ );
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

    cout << GREEN << "Total time: " << total_dura << ", segment: " << segment_dura
         << ", keypoints: " << keypoint_dura << ", match: "<< match_dura
         << ", solveRT: " << solveRT_dura << ", slam: " << slam_dura
         << ", display: " << display_dura << RESET << endl;
    cout << "----------------------------------- END -------------------------------------" << endl;

    last_frame = frame; // store frame
}


//bool KinectListener::solveRelativeTransform( KinectFrame &last_frame,
//                                             KinectFrame &current_frame,
//                                             RESULT_OF_MOTION &result,
//                                             Eigen::Matrix4d estimated_transform )
//{
//    ros::Time start_time = ros::Time::now();
//    double icp_dura, planes_dura, match_f_dura, points_dura, pnp_dura;

//    /// Use icp result as initial estimation
//    RESULT_OF_MOTION icp_result;
//    PointCloudXYZPtr cloud_icp( new PointCloudXYZ );
//    icp_result.valid = solveRtIcp( current_frame.feature_cloud, last_frame.feature_cloud, cloud_icp, icp_result );
//    if( icp_result.valid )
//    {
//        estimated_transform = icp_result.transform4d();
//    }
//    cout << GREEN << "ICP result as estimated transformation: valid = " << (icp_result.valid?"true":"false") << RESET << endl;
//    cout << GREEN << "  - score: " << icp_result.score << RESET << endl;
//    printTransform( estimated_transform );
//    //
//    icp_dura = (ros::Time::now() - start_time).toSec() * 1000;
//    start_time = ros::Time::now();


//    /// case 1: using planes for motion estimation
//    RESULT_OF_MOTION planes_result;
//    std::vector<PlanePair> pairs;
//    ITree::euclidianPlaneCorrespondences( current_frame.segment_planes, last_frame.segment_planes, pairs, estimated_transform);
//    planes_result.valid = solveRelativeTransformPlanes( last_frame, current_frame, pairs, planes_result );
//    //
//    cout << GREEN << "Transformation from plane correspondences: valid = " << (planes_result.valid?"true":"false") << RESET << endl;
//    cout << GREEN << "  - rmse: " << planes_result.rmse << RESET << endl;
//    printTransform( planes_result.transform4d() );
//    //
//    planes_dura = (ros::Time::now() - start_time).toSec() * 1000;
//    start_time = ros::Time::now();

//    /// matches keypoint features
//    std::vector<cv::DMatch> good_matches;
//    matchImageFeatures( last_frame, current_frame, good_matches,
//                        feature_good_match_threshold_, feature_min_good_match_size_ );
//    //

//    //
//    match_f_dura = (ros::Time::now() - start_time).toSec() * 1000;
//    start_time = ros::Time::now();

//    /// case 2: using planes & points



//    /// case 3: using points ransac
//    RESULT_OF_MOTION points_result;
//    std::vector<cv::DMatch> inlier;
//    points_result.valid = solveRelativeTransformPointsRansac( last_frame, current_frame, good_matches, points_result, inlier );
//    //
//    cout << GREEN << "Transformation from points correspondences by RANSAC: valid = " << (points_result.valid?"true":"false") << RESET << endl;
//    cout << GREEN << "  - inlier: " << points_result.inlier << ", rmse: " << points_result.rmse << RESET << endl;
//    printTransform( points_result.transform4d() );
//    //
//    points_dura = (ros::Time::now() - start_time).toSec() * 1000;
//    start_time = ros::Time::now();

//    /// case 4: using PnP
//    RESULT_OF_MOTION pnp_result;
//    pnp_result.valid = solveRelativeTransformPnP( last_frame, current_frame, good_matches, camera_parameters_, pnp_result );
//    //
//    cout << GREEN << "Transformation from PnP: valid = " << (pnp_result.valid?"true":"false") << RESET << endl;
//    cout << GREEN << "  - inlier: " << pnp_result.inlier << RESET << endl;
//    printTransform( pnp_result.transform4d() );
//    //
//    pnp_dura = (ros::Time::now() - start_time).toSec() * 1000;
//    start_time = ros::Time::now();

//    /// Print info
//    cout << GREEN << "Transformation total time: " << (icp_dura + planes_dura + match_f_dura + points_dura + pnp_dura)
//         << ", icp: " << icp_dura
//         << ", planes: " << planes_dura
//         << ", match_f: "<< match_f_dura
//         << ", points: " << points_dura
//         << ", pnp: " << pnp_dura << RESET << endl;

//    return result.valid;
//}



bool KinectListener::solveRtIcp( const PointCloudXYZPtr &source,
                               const PointCloudXYZPtr &target,
                               PointCloudXYZPtr &cloud_icp,
                               RESULT_OF_MOTION &result )
{
    Eigen::Matrix4d Tm = Eigen::Matrix4d::Identity();
    const double icp_score_threshold = icp_score_threshold_;
    double score = 1.0;
    bool is_converged = false;

    // icp
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr icp(new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>());
    icp->setMaxCorrespondenceDistance(icp_max_distance_);
    icp->setMaximumIterations (icp_iterations_);
    icp->setTransformationEpsilon( icp_tf_epsilon_ );
    icp->setInputSource( source );
    icp->setInputTarget( target );
    icp->align ( *cloud_icp );
    score = icp->getFitnessScore();
    is_converged = icp->hasConverged();

    // save result
    Tm = icp->getFinalTransformation().cast<double>();
    result.setTransform4d( Tm );
    result.inlier = cloud_icp->size();
    result.score = score;

    if(is_converged && score <= icp_score_threshold )
    {
        result.valid = true;
        return true;
    }
    else
    {
        result.valid = false;
        return false;
    }
}

Eigen::Matrix4f KinectListener::solveRtPcl( const std_vector_of_eigen_vector4f &query_points,
                                                   const std_vector_of_eigen_vector4f &train_points,
                                                   const std::vector<cv::DMatch> &matches,
                                                   bool &valid)
{
    pcl::TransformationFromCorrespondences tfc;
    valid = true;
    float weight = 1.0;

    BOOST_FOREACH(const cv::DMatch& m, matches)
    {
//        Eigen::Vector3f from = query_points[m.queryIdx].head<3>();
//        Eigen::Vector3f to = train_points[m.trainIdx].head<3>();
        Eigen::Vector3f to = query_points[m.queryIdx].head<3>();
        Eigen::Vector3f from = train_points[m.trainIdx].head<3>();
        if( std::isnan(from(2)) || std::isnan(to(2)) )
            continue;
        weight = 1.0/(from(2) * to(2));
        tfc.add(from, to, weight);// 1.0/(to(2)*to(2)));//the further, the less weight b/c of quadratic accuracy decay
    }

    // get relative movement from samples
    if( tfc.getNoOfSamples() < 3)
    {
        valid = false;
        return Eigen::Matrix4f();
    }
    else
        return tfc.getTransformation().matrix();
}

void KinectListener::solveRt( const std::vector<PlaneCoefficients> &before,
                              const std::vector<PlaneCoefficients> &after,
                              const std::vector<Eigen::Vector3d>& from_points,
                              const std::vector<Eigen::Vector3d>& to_points,
                              RESULT_OF_MOTION &result)
{
    const int num_points = from_points.size();
    const int num_planes = before.size();

//    cout << " solveRT, planes = " << num_planes << ", points = " << num_points << RESET << endl;

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
//    cout << "   det(sigma) = " << sigma.determinant() << endl;
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
//    cout << "   rank(sigma) = " << rank << endl;
    if ( rank == 2 )
    {
//        cout << "   det(U)*det(V) = " << svd.matrixU().determinant() * svd.matrixV().determinant() << endl;
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
    result.valid = true;
}

void KinectListener::solveRt(const std::vector<Eigen::Vector3d>& from_points,
                             const std::vector<Eigen::Vector3d>& to_points,
                             RESULT_OF_MOTION &result)
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

void KinectListener::solveRt(const std::vector<PlaneCoefficients> &before,
                             const std::vector<PlaneCoefficients> &after,
                             RESULT_OF_MOTION &result)
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

bool KinectListener::solveRtPlanes( const std::vector<PlaneCoefficients> &last_planes,
                                        const std::vector<PlaneCoefficients> &planes,
                                        RESULT_OF_MOTION &result)
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
    solveRt( last_planes, planes, result );

    return true;
}


Eigen::Matrix4f KinectListener::solveRtPlanesPoints( std::vector<PlaneType> &last_planes,
                                                     std::vector<PlaneType> &planes,
                                                     std::vector<PlanePair> &pairs,
                                                     std_vector_of_eigen_vector4f &last_feature_3d,
                                                     std_vector_of_eigen_vector4f &feature_3d,
                                                     std::vector<cv::DMatch> &matches,
                                                     bool &valid )
{
    if( !pairs.size() || ! matches.size() || (pairs.size() + matches.size()) != 3)
    {
        valid = false;
        ROS_ERROR("Invalid number of pairs and matches to solve RT, pairs = %d, matches = %d ",
                  pairs.size(), matches.size() );
        return Eigen::Matrix4f::Identity();
    }

    std::vector<PlaneCoefficients> before;
    std::vector<PlaneCoefficients> after;
    std::vector<Eigen::Vector3d> from_points;
    std::vector<Eigen::Vector3d> to_points;

    // solve RT
    for( int i = 0; i < pairs.size(); i++)
    {
        before.push_back( last_planes[pairs[i].ilm].coefficients );
        after.push_back( planes[pairs[i].iobs].coefficients );
    }
    for( int i = 0; i < matches.size(); i++)
    {
        from_points.push_back( last_feature_3d[matches[i].queryIdx].head<3>().cast<double>() );
        to_points.push_back( feature_3d[matches[i].trainIdx].head<3>().cast<double>() );
    }

    // check geometric constrains
    // 2 planes and 1 point
    // 1 plane and 2 points

    RESULT_OF_MOTION motion;
    solveRt( after, before, to_points, from_points, motion );
    valid = true;

    return motion.transform4f();
}

bool KinectListener::solveRelativeTransformPlanes( KinectFrame &last_frame,
                                                   KinectFrame &current_frame,
                                                   const std::vector<PlanePair> &pairs,
                                                   RESULT_OF_MOTION &result)
{
    std::vector<PlaneType> &planes = current_frame.segment_planes;
    std::vector<PlaneType> &last_planes = last_frame.segment_planes;

    if( planes.size() < 3 || last_planes.size() < 3 || pairs.size() < 3)
        return false;

    const unsigned int pairs_num = pairs.size();

    // Estimate transformation using all the plane correspondences
    RESULT_OF_MOTION best_transform;
    best_transform.rmse = 1e9; // no inlier
    for( int x1 = 0; x1 < pairs_num-2; x1++)
    {
        const PlanePair &p1 = pairs[x1];
        for( int x2 = x1+1; x2 < pairs_num-1; x2++ )
        {
            const PlanePair &p2 = pairs[x2];
            for( int x3 = x2+1; x3 < pairs_num; x3++)
            {
                const PlanePair &p3 = pairs[x3];
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
                RESULT_OF_MOTION motion;
                motion.valid = solveRtPlanes( currents, lasts, motion);

                if( motion.valid )
                {
                    // check if better
                    std::vector<PlanePair> pairs3;
                    pairs3.push_back( p1 );
                    pairs3.push_back( p2 );
                    pairs3.push_back( p3 );
                    motion.rmse = computeEuclidianDistance( last_planes, planes, pairs3, motion );
                    if( motion.rmse < best_transform.rmse )
                        best_transform = motion;

//                    // print motion
//                    gtsam::Rot3 rot3( motion.rotation );
//                    cout << YELLOW << " relative motion: (" << p1.iobs << "/" << p2.iobs << "/" << p3.iobs << "), ("
//                         << p1.ilm << "/" << p2.ilm << "/" << p3.ilm << ")"
//                         << ", rmse = " << motion.rmse << endl;
//                    cout << "  - R(rpy): " << rot3.roll() << ", " << rot3.pitch() << ", " << rot3.yaw() << endl;
//                    cout << "  - T:      " << motion.translation[0]
//                         << ", " << motion.translation[1]
//                         << ", " << motion.translation[2] << RESET << endl;
                }
            }
        }
    }

//    Eigen::umeyama
    result = best_transform;
    return true;
}


bool KinectListener::solveRelativeTransformPointsRansac(KinectFrame &last_frame,
                                             KinectFrame &frame,
                                             std::vector<cv::DMatch> &good_matches,
                                             RESULT_OF_MOTION &result,
                                             std::vector<cv::DMatch> &matches)
{
//    // match feature
//    std::vector<cv::DMatch> good_matches;
//    matchImageFeatures( last_frame, frame, good_matches, feature_good_match_threshold_, feature_min_good_match_size_);
//
//    // sort
//    std::sort(good_matches.begin(), good_matches.end()); //sort by distance, which is the nn_ratio

    int min_inlier_threshold = ransac_min_inlier_;
    if( min_inlier_threshold > 0.6*good_matches.size() )
        min_inlier_threshold = 0.6*good_matches.size();

    std::sort( good_matches.begin(), good_matches.end() );

    //
    Eigen::Matrix4f resulting_transformation;
    double rmse = 1e6;
    //
    matches.clear();
    const unsigned int sample_size = ransac_sample_size_;
    unsigned int valid_iterations = 0;
    const unsigned int max_iterations = ransac_iterations_;
    int real_iterations = 0;
    double inlier_error;
    double max_dist_m = ransac_inlier_max_mahal_distance_;
    bool valid_tf;
    for( int n = 0; n < max_iterations && good_matches.size() >= sample_size; n++)
    {
        double refined_error = 1e6;
        std::vector<cv::DMatch> refined_matches;
        std::vector<cv::DMatch> inlier = randomChooseMatchesPreferGood( sample_size, good_matches); //initialization with random samples
//        std::vector<cv::DMatch> inlier = randomChooseMatches( sample_size, good_matches); //initialization with random samples
        Eigen::Matrix4f refined_transformation = Eigen::Matrix4f::Identity();

        real_iterations++;
//        cout << "Iteration = " << real_iterations << endl;
        for( int refine = 0; refine < 20; refine ++)
        {
            Eigen::Matrix4f transformation = solveRtPcl( last_frame.feature_locations_3d,
                                                                         frame.feature_locations_3d,
                                                                         inlier, valid_tf );
            if( !valid_tf || transformation != transformation )
            {
//                cout << BLUE << "- valid = " << (valid_tf?"true":"false") << ", equal = " << (transformation == transformation) << RESET << endl;
                break;
            }

            computeCorrespondenceInliersAndError( good_matches, transformation, last_frame.feature_locations_3d, frame.feature_locations_3d,
                                                  min_inlier_threshold, inlier, inlier_error, max_dist_m );

            if( inlier.size() < min_inlier_threshold || inlier_error > max_dist_m)
                break;

//            cout << BOLDBLUE << " - refine = " << refine << ", relative transform: " << RESET << endl;
//            printTransform( transformation );
//            cout << BLUE << " - inlier = " << inlier.size() << ", error = " << inlier_error << RESET << endl;
//            cout << endl;

            if( inlier.size() > refined_matches.size() && inlier_error < refined_error )
            {
                unsigned int prev_num_inliers = refined_matches.size();
                assert( inlier_error>=0 );
                refined_transformation = transformation;
                refined_matches = inlier;
                refined_error = inlier_error;
                if( inlier.size() == prev_num_inliers )
                    break; //only error improved -> no change would happen next iteration
            }
            else
                break;
        }
        // Success
        if( refined_matches.size() > 0 )
        {
            valid_iterations++;

            //Acceptable && superior to previous iterations?
            if (refined_error <= rmse &&
                refined_matches.size() >= matches.size() &&
                refined_matches.size() >= min_inlier_threshold)
            {
                rmse = refined_error;
                resulting_transformation = refined_transformation;
                matches.assign(refined_matches.begin(), refined_matches.end());
                //Performance hacks:
                if ( refined_matches.size() > good_matches.size()*0.5 ) n+=10;///Iterations with more than half of the initial_matches inlying, count tenfold
                if ( refined_matches.size() > good_matches.size()*0.75 ) n+=10;///Iterations with more than 3/4 of the initial_matches inlying, count twentyfold
                if ( refined_matches.size() > good_matches.size()*0.8 ) break; ///Can this get better anyhow?
            }
        }
    }

    if( valid_iterations == 0 ) // maybe no depth. Try identity?
    {
        cout << BOLDRED << "No valid iteration, try identity." << RESET << endl;
        //ransac iteration with identity
        Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();//hypothesis
        std::vector<cv::DMatch> inlier; //result
        double refined_error = 1e6;
        std::vector<cv::DMatch> refined_matches;
        Eigen::Matrix4f refined_transformation = Eigen::Matrix4f::Identity();

        //test which samples are inliers
        computeCorrespondenceInliersAndError( good_matches, Eigen::Matrix4f::Identity(), last_frame.feature_locations_3d, frame.feature_locations_3d,
                                            min_inlier_threshold, inlier, inlier_error, max_dist_m );

        cout << BOLDRED << "inlier = " << inlier.size() << ", error = " << inlier_error << RESET << endl;
        if( inlier.size() > sample_size && inlier_error < refined_error )
        {
            refined_matches = inlier;
            refined_error = inlier_error;
        }

        // refine
        for( int refine = 0; refine < 20; refine ++)
        {
            if( inlier.size() < sample_size )
                break;

            Eigen::Matrix4f transformation = solveRtPcl( last_frame.feature_locations_3d,
                                                                         frame.feature_locations_3d,
                                                                         inlier, valid_tf );
            if( !valid_tf || transformation != transformation )
            {
//                cout << BLUE << "- valid = " << (valid_tf?"true":"false") << ", equal = " << (transformation == transformation) << RESET << endl;
                break;
            }

            computeCorrespondenceInliersAndError( good_matches, transformation, last_frame.feature_locations_3d, frame.feature_locations_3d,
                                                  min_inlier_threshold, inlier, inlier_error, max_dist_m );

            if( inlier.size() < min_inlier_threshold || inlier_error > max_dist_m)
                break;

            cout << BOLDBLUE << " - refine = " << refine << ", relative transform: " << RESET << endl;
            printTransform( transformation );
            cout << BLUE << " - inlier = " << inlier.size() << ", error = " << inlier_error << RESET << endl;
            cout << endl;

            if( inlier.size() > refined_matches.size() && inlier_error < refined_error )
            {
                unsigned int prev_num_inliers = refined_matches.size();
                assert( inlier_error>=0 );
                refined_transformation = transformation;
                refined_matches = inlier;
                refined_error = inlier_error;
                if( inlier.size() == prev_num_inliers )
                    break; //only error improved -> no change would happen next iteration
            }
            else
                break;
        }

        // Success
        if( refined_matches.size() > 0 )
        {
            if (refined_error <= rmse &&
                refined_matches.size() >= matches.size() &&
                refined_matches.size() >= min_inlier_threshold)
            {
                rmse = refined_error;
                resulting_transformation = refined_transformation;
                matches.assign(refined_matches.begin(), refined_matches.end());
            }
        }
    }

    result.setTransform4f( resulting_transformation );    // save result
    result.rmse = rmse;
    result.inlier = matches.size();
    result.valid = matches.size() >= min_inlier_threshold;
    return ( matches.size() >= min_inlier_threshold );
}


bool KinectListener::solveRelativeTransformPlanesPointsRansac( KinectFrame &last_frame,
                                                               KinectFrame &current_frame,
                                                               std::vector<PlanePair> &pairs,
                                                               std::vector<cv::DMatch> &good_matches,
                                                               RESULT_OF_MOTION &result,
                                                               std::vector<cv::DMatch> &matches )
{
    if( !pairs.size() )
        return false;

    const int pairs_num = pairs.size();
    std::vector<PlaneType> &planes = current_frame.segment_planes;
    std::vector<PlaneType> &last_planes = last_frame.segment_planes;

    // using all plane correspondences
    std::vector< std::vector<PlanePair> > sample_plane_pairs;
    for( int i = 0; i < pairs_num-1; i++)
    {
        for( int j = i+1; j < pairs_num; j++)
        {
            std::vector<PlanePair> ps;
            ps.push_back( pairs[i] );
            ps.push_back( pairs[j] );
            sample_plane_pairs.push_back( ps ); // 2 planes
        }
    }
    for( int i = 0; i < pairs_num; i++)
    {
        std::vector<PlanePair> ps;
        ps.push_back( pairs[i] );
        sample_plane_pairs.push_back( ps ); // 1 planes
    }

    // Will be the final result
    matches.clear();
    double resulting_error = 1e9;
    Eigen::Matrix4f resulting_transformation = Eigen::Matrix4f::Identity();
    //
    const unsigned int sample_size = 3;
    unsigned int valid_iterations = 0;
    int real_iterations = 0;
    double inlier_error;
    unsigned int min_inlier_threshold = good_matches.size() * 0.6;  // minimum point inlier
    double max_dist_m = ransac_inlier_max_mahal_distance_;
    bool valid_tf;
//    cout << BLUE << " min_inlier = " << min_inlier_threshold << ", iterations = " << ransac_iterations_ << RESET << endl;
    for( int i = 0; i < sample_plane_pairs.size(); i++)
    {
        //
        std::vector<PlanePair> &ps = sample_plane_pairs[i];
        const unsigned int sample_points_size = sample_size - ps.size(); // number of sample points
        unsigned int max_iterations = ransac_iterations_; // Size of sample plane/point size dependent or not?
        for( int n = 0; n < max_iterations; n++)
        {
            real_iterations ++;
            // Compute transformation
            std::vector<cv::DMatch> inlier = randomChooseMatchesPreferGood( sample_points_size, good_matches ); //initialization with random samples
            Eigen::Matrix4f transformation = solveRtPlanesPoints( last_planes, planes, ps, last_frame.feature_locations_3d,
                                                                  current_frame.feature_locations_3d, inlier, valid_tf );
//            cout << " - valid_tf = " << (valid_tf?"true":"false") << endl;
            if( !valid_tf || transformation != transformation )
                continue;
            if( valid_tf )  // valid transformation
            {
                computeCorrespondenceInliersAndError( good_matches, transformation, last_frame.feature_locations_3d, current_frame.feature_locations_3d,
                                                      min_inlier_threshold, inlier, inlier_error, max_dist_m );

//                cout << BOLDBLUE << " - refine = " << n << ", relative transform: " << RESET << endl;
//                printTransform( transformation );
//                cout << BLUE << " - inlier = " << inlier.size() << ", error = " << inlier_error << RESET << endl;

                //
                if( inlier.size() > min_inlier_threshold && inlier_error < resulting_error )
                {
                    valid_iterations ++;
                    //
                    matches = inlier;
                    resulting_error = inlier_error;
                    resulting_transformation = transformation;
                }
            }
        }
    }

    //
    cout << BLUE << "Real iterations = " << real_iterations
         << ", valid iterations = " << valid_iterations << RESET << endl;

    // check if success
    if( matches.size() > min_inlier_threshold && resulting_error < max_dist_m)
    {
        result.setTransform4f( resulting_transformation );
        result.inlier = matches.size();
        result.rmse = resulting_error;
        result.valid = true;
        return true;
    }
    else
        return false;
}


bool KinectListener::solveRelativeTransformIcp( KinectFrame &last_frame,
                                                KinectFrame &current_frame,
                                                RESULT_OF_MOTION &result)
{
    PointCloudXYZPtr cloud_icp( new PointCloudXYZ );
    result.valid = solveRtIcp( current_frame.feature_cloud, last_frame.feature_cloud, cloud_icp, result );
    return result.valid;
}

// estimateMotion 计算两个帧之间的运动
// 输入：帧1和帧2
// 输出：rvec 和 tvec
bool KinectListener::solveRelativeTransformPnP( KinectFrame& last_frame,
                                                KinectFrame& current_frame,
                                                std::vector<cv::DMatch> &good_matches,
                                                PlaneFromLineSegment::CAMERA_PARAMETERS& camera,
                                                RESULT_OF_MOTION &result)
{
    int min_inlier_threshold = pnp_min_inlier_;
    if( min_inlier_threshold > 0.75*good_matches.size() )
        min_inlier_threshold = 0.75*good_matches.size();

    // 3d points
    vector<cv::Point3f> pts_obj;
    // 2d points
    vector< cv::Point2f > pts_img;

    //
    for (size_t i=0; i<good_matches.size(); i++)
    {
        int query_index = good_matches[i].queryIdx;
        int train_index = good_matches[i].trainIdx;

        // train
        // 3d point
        Eigen::Vector4f pc = current_frame.feature_locations_3d[train_index];
        cv::Point3f pd( pc[0], pc[1], pc[2] );
        pts_obj.push_back( pd );

        // query
        // 2d point
        pts_img.push_back( cv::Point2f( last_frame.feature_locations_2d[query_index].pt ) );
    }

    // Construct camera intrinsic matrix
    double camera_matrix_data[3][3] = {
        {camera.fx, 0, camera.cx},
        {0, camera.fy, camera.cy},
        {0, 0, 1}
    };
    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
    cv::Mat rvec, tvec, inlier;
    // Solve pnp
//    cout << "solving pnp" << endl;
    cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false,
                        pnp_iterations_, pnp_repreject_error_, min_inlier_threshold, inlier );

    //
    result.inlier = inlier.rows;
    result.translation = Eigen::Vector3d( tvec.at<double>(0,0), tvec.at<double>(0,1), tvec.at<double>(0,2) );
    cv::Mat R;
    cv::Rodrigues( rvec, R );
    cvToEigen( R, result.rotation );

//    cout << " Trans: " << endl << result.translation << endl;
//    cout << " Rot: " << endl << result.rotation << endl;

    return ( result.inlier >= min_inlier_threshold );
}


bool KinectListener::solveRelativeTransform( KinectFrame &last_frame,
                                             KinectFrame &current_frame,
                                             RESULT_OF_MOTION &result,
                                             std::vector<cv::DMatch> &matches,
                                             Eigen::Matrix4d estimated_transform)
{
    result.valid = false;
    result.rmse = 1e9;
    result.inlier = 0;

    std::vector<PlaneType> &planes = current_frame.segment_planes;
    std::vector<PlaneType> &last_planes = last_frame.segment_planes;

    ros::Time start_time = ros::Time::now();
    double pairs_dura, planes_dura, match_f_dura, points_planes_dura,
            points_dura, icp_dura, pnp_dura;

    // Find plane correspondences
    std::vector<PlanePair> pairs;
    if( planes.size() > 0 && last_planes.size() > 0 )
    {
        ITree::euclidianPlaneCorrespondences( planes, last_planes, pairs, estimated_transform );
        std::sort( pairs.begin(), pairs.end() );
    }
    const int pairs_num = pairs.size();

    pairs_dura = (ros::Time::now() - start_time).toSec() * 1000;
    start_time = ros::Time::now();

    /// case 1: Estimate motion using plane correspondences
    RESULT_OF_MOTION best_transform;
    best_transform.rmse = 1e6; // no inlier
    best_transform.valid = false;   // no result
    if( pairs_num >= 3 )
    {
        best_transform.valid = solveRelativeTransformPlanes( last_frame, current_frame, pairs, best_transform );
        if( best_transform.valid )
        {
            result = best_transform;
//            return true;
        }
    }

    //
    planes_dura = (ros::Time::now() - start_time).toSec() * 1000;
    start_time = ros::Time::now();

    // print info
    cout << GREEN << "Transformation from plane correspondences: valid = " << (best_transform.valid?"true":"false") << RESET << endl;
    cout << GREEN << "  - rmse: " << best_transform.rmse << RESET << endl;
    printTransform( best_transform.transform4d() );

    best_transform.valid = false;   // for test
    // matches keypoint features
    std::vector<cv::DMatch> good_matches;
    matchImageFeatures( last_frame, current_frame, good_matches,
                        feature_good_match_threshold_, feature_min_good_match_size_ );

    cout << GREEN << "Matches features, good_matches = " << good_matches.size() << RESET << endl;

    //
    match_f_dura = (ros::Time::now() - start_time).toSec() * 1000;
    start_time = ros::Time::now();

    /// case 2: Estimate motion using plane and point correspondences
    if( !best_transform.valid && pairs_num > 0 && good_matches.size() >= 3)
    {
        best_transform.valid = solveRelativeTransformPlanesPointsRansac( last_frame, current_frame, pairs, good_matches, best_transform, matches );
    }
    if( best_transform.valid )
    {
        result = best_transform;
//        return true;
    }
    //
    points_planes_dura = (ros::Time::now() - start_time).toSec() * 1000;
    start_time = ros::Time::now();

    // print info
    cout << GREEN << "Transformation from plane/point correspondences: valid = " << (best_transform.valid?"true":"false") << RESET << endl;
    cout << GREEN << "  - rmse: " << best_transform.rmse << ", inlier = " << best_transform.inlier << RESET << endl;
    printTransform( best_transform.transform4d() );

    best_transform.valid = false; // for test
    /// case 3: Estimate motion using point correspondences
    if( !best_transform.valid && good_matches.size() >= 3 )
    {
        matches.clear();
        best_transform.valid = solveRelativeTransformPointsRansac( last_frame, current_frame, good_matches, best_transform, matches );
    }

    if( best_transform.valid )
    {
        result = best_transform;
//        return true;
    }

    points_dura = (ros::Time::now() - start_time).toSec() * 1000;
    start_time = ros::Time::now();

    // print info
    cout << GREEN << "Transformation from point correspondences: valid = " << (best_transform.valid?"true":"false") << RESET << endl;
    cout << GREEN << "  - rmse: " << best_transform.rmse << ", inlier = " << best_transform.inlier << RESET << endl;
    printTransform( best_transform.transform4d() );

    /// case 4: Using ICP
    if( !best_transform.valid && good_matches.size() >= 20 )
    {
        best_transform.valid = solveRelativeTransformIcp( last_frame, current_frame, best_transform );
    }

    if( best_transform.valid )
    {
        result = best_transform;
//        return true;
    }
    //
    icp_dura = (ros::Time::now() - start_time).toSec() * 1000;
    start_time = ros::Time::now();

    // print info
    cout << GREEN << "Transformation from ICP: valid = " << (best_transform.valid?"true":"false") << RESET << endl;
    cout << GREEN << "  - rmse: " << best_transform.rmse << RESET << endl;
    printTransform( best_transform.transform4d() );

    /// case 5: using PnP
    if( !best_transform.valid && good_matches.size() >= 20 )
    {
        best_transform.valid = solveRelativeTransformPnP( last_frame, current_frame, good_matches, camera_parameters_, best_transform );
    }

    if( best_transform.valid )
    {
        result = best_transform;
//        return true;
    }
    //
    pnp_dura = (ros::Time::now() - start_time).toSec() * 1000;
    start_time = ros::Time::now();

    // print info
    cout << GREEN << "Transformation from PnP: valid = " << (best_transform.valid?"true":"false") << RESET << endl;
    cout << GREEN << "  - rmse: " << best_transform.rmse << RESET << endl;
    printTransform( best_transform.transform4d() );

    /// Print info
    double total_time = pairs_dura + planes_dura + match_f_dura
            + points_planes_dura + points_dura + icp_dura + pnp_dura;
    cout << GREEN << "Transformation total time: " << total_time << endl;
    cout << "Time:"
         << " pairs: " << pairs_dura
         << ", planes: " << planes_dura
         << ", match_f: "<< match_f_dura
         << ", planes/points: " << points_planes_dura
         << ", points: " << points_dura
         << ", icp: " << icp_dura
         << ", pnp: " << pnp_dura
         << RESET << endl;

    return result.valid;
}

// from: https://github.com/felixendres/rgbdslam_v2/src/node.cpp
void KinectListener::computeCorrespondenceInliersAndError( const std::vector<cv::DMatch> & matches,
                                  const Eigen::Matrix4f& transform4f,
                                  const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& query_points,
                                  const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& train_points,
                                  unsigned int min_inlier,
                                  std::vector<cv::DMatch>& inlier, //pure output var
                                  double& return_mean_error,//pure output var: rms-mahalanobis-distance
                                  double squared_max_distance) const
{
    inlier.clear();
    assert(matches.size() > 0);
    inlier.reserve(matches.size());
    //errors.clear();
    const size_t all_matches_size = matches.size();
    double mean_error = 0.0;
    Eigen::Matrix4d transform4d = transform4f.cast<double>();

    //parallelization is detrimental here
    //#pragma omp parallel for reduction (+: mean_error)
    for(int i=0; i < all_matches_size; ++i)
    //BOOST_FOREACH(const cv::DMatch& m, all_matches)
    {
        const cv::DMatch& m = matches[i];
//        const Eigen::Vector4f& from = froms[m.queryIdx];
//        const Eigen::Vector4f& to = tos[m.trainIdx];
        const Eigen::Vector4f& to = query_points[m.queryIdx];
        const Eigen::Vector4f& from = train_points[m.trainIdx];

        if( std::isnan(from(2)) || std::isnan(to(2)) )
        { //does NOT trigger on NaN
            continue;
        }
        double mahal_dist = errorFunction2(from, to, transform4d);
//        double mahal_dist = errorFunction2(to, from, transform4d);
//        cout << " (" << from[0] << "," << from[1] << "," << from[2]
//             << ")->(" << to[0] << "," << to[1] << "," << to[2] << ") = "<< mahal_dist;
        if(mahal_dist > squared_max_distance)
        {
            continue; //ignore outliers
        }
        if(!(mahal_dist >= 0.0))
        {
            ROS_WARN_STREAM("Mahalanobis_ML_Error: "<<mahal_dist);
            ROS_WARN_STREAM("Transformation for error !>= 0:\n" << transform4d << "Matches: " << i);
            continue;
        }
        mean_error += mahal_dist;
        //#pragma omp critical
        inlier.push_back(m); //include inlier
    }


    if ( inlier.size()<3 )
    {
        //at least the samples should be inliers
        ROS_DEBUG("No inliers at all in %d matches!", (int)all_matches_size); // only warn if this checks for all initial matches
        return_mean_error = 1e9;
    }
    else
    {
        mean_error /= inlier.size();
        return_mean_error = sqrt(mean_error);
    }
}

double KinectListener::computeEuclidianDistance( const std::vector<PlaneType>& last_planes,
                                                 const std::vector<PlaneType>& planes,
                                                 const std::vector<PlanePair>& pairs,
                                                 RESULT_OF_MOTION &relative )
{
    double rmse = 0;
    Eigen::Matrix4d transform = relative.transform4d();
    double direction_squared = 0;
    double distance_squared = 0;
    for( int i = 0; i < pairs.size(); i++)
    {
        const PlaneType &plane = planes[ pairs[i].iobs ];
        const PlaneType &last_plane = last_planes[ pairs[i].ilm ];
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
//        deviation = direction + distance;
        direction_squared += direction * direction;
        distance_squared += distance * distance;
    }

    rmse = sqrt( direction_squared / pairs.size() ) + sqrt( distance_squared / pairs.size() );
    return rmse;
}


void KinectListener::computeORBKeypoint( const cv::Mat &visual,
                                      const PointCloudTypePtr &cloud_in,
                                      std::vector<cv::KeyPoint> &keypoints,
                                      std_vector_of_eigen_vector4f &locations_3d,
                                      cv::Mat &feature_descriptors )
{
    // Get gray image
    cv::Mat gray_img;
    if(visual.type() == CV_8UC3)
        cv::cvtColor( visual, gray_img, CV_RGB2GRAY );
    else
        gray_img = visual;

    //
    (*orb_extractor_)( gray_img, cv::Mat(), keypoints, feature_descriptors);

    // Project to 3D
    projectTo3D( cloud_in, keypoints, feature_descriptors, locations_3d);

}

void KinectListener::computeORBKeypoint( const cv::Mat &visual,
                                      const PointCloudTypePtr &cloud_in,
                                      std::vector<cv::KeyPoint> &keypoints,
                                      std_vector_of_eigen_vector4f &locations_3d,
                                      PointCloudXYZPtr &feature_cloud,
                                      cv::Mat &feature_descriptors )
{
    // Get gray image
    cv::Mat gray_img;
    if(visual.type() == CV_8UC3)
        cv::cvtColor( visual, gray_img, CV_RGB2GRAY );
    else
        gray_img = visual;

    //
    (*orb_extractor_)( gray_img, cv::Mat(), keypoints, feature_descriptors);

    // Project to 3D
    projectTo3D( cloud_in, keypoints, feature_descriptors, locations_3d, feature_cloud);

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

void KinectListener::computeKeypoint( const cv::Mat &visual,
                                      const PointCloudTypePtr &cloud_in,
                                      std::vector<cv::KeyPoint> &keypoints,
                                      std_vector_of_eigen_vector4f &locations_3d,
                                      PointCloudXYZPtr &feature_cloud,
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
    projectTo3D( cloud_in, keypoints, locations_3d, feature_cloud);

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
                                  cv::Mat &feature_descriptors,
                                  std_vector_of_eigen_vector4f &locations_3d )
{
    // Clear
    if(locations_3d.size())
        locations_3d.clear();

    for(int i = 0; i < locations_2d.size(); i++)
    {
        cv::Point2f p2d = locations_2d[i].pt;

        PointType p3d = cloud->at((int) p2d.x,(int) p2d.y);

//        // Check for invalid measurements
//        if ( isnan(p3d.x) || isnan(p3d.y) || isnan(p3d.z) )
//        {
//            locations_2d.erase( locations_2d.begin()+i );
//            // how to remove selected row in Mat, like erase( feature_descriptors.row(i) )
//            continue;
//            p3d.x = 0; p3d.y = 0; p3d.z = 0;
//        }

        locations_3d.push_back(Eigen::Vector4f(p3d.x, p3d.y, p3d.z, 1.0));
    }
}

void KinectListener::projectTo3D( const PointCloudTypePtr &cloud,
                                  std::vector<cv::KeyPoint> &locations_2d,
                                  cv::Mat &feature_descriptors,
                                  std_vector_of_eigen_vector4f &locations_3d,
                                  PointCloudXYZPtr &feature_cloud )
{
    // Clear
    if(locations_3d.size())
        locations_3d.clear();
    if(feature_cloud->size())
        feature_cloud->clear();

    for(int i = 0; i < locations_2d.size(); i++)
    {
        cv::Point2f p2d = locations_2d[i].pt;

        PointType p3d = cloud->at((int) p2d.x,(int) p2d.y);

//        // Check for invalid measurements
//        if ( isnan(p3d.x) || isnan(p3d.y) || isnan(p3d.z) )
//        {
//            locations_2d.erase( locations_2d.begin()+i );
//            // how to remove selected row in Mat, like erase( feature_descriptors.row(i) )
//            continue;
//        }

        locations_3d.push_back(Eigen::Vector4f(p3d.x, p3d.y, p3d.z, 1.0));
        if( !isnan(p3d.x) && !isnan(p3d.y) && !isnan(p3d.z) )
            feature_cloud->push_back( pcl::PointXYZ(p3d.x, p3d.y, p3d.z) );
    }

    feature_cloud->is_dense = false;
    feature_cloud->height = 1;
    feature_cloud->width = feature_cloud->points.size();
}

void KinectListener::projectTo3D( const PointCloudTypePtr &cloud,
                                  std::vector<cv::KeyPoint> &locations_2d,
                                  std_vector_of_eigen_vector4f &locations_3d )
{
    // Clear
    if(locations_3d.size())
        locations_3d.clear();


    for(int i = 0; i < locations_2d.size(); )
    {
        cv::Point2f p2d = locations_2d[i].pt;

        PointType p3d = cloud->at((int) p2d.x,(int) p2d.y);

        // Check for invalid measurements
        if ( isnan(p3d.x) || isnan(p3d.y) || isnan(p3d.z) )
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

void KinectListener::projectTo3D( const PointCloudTypePtr &cloud,
                                  std::vector<cv::KeyPoint> &locations_2d,
                                  std_vector_of_eigen_vector4f &locations_3d,
                                  PointCloudXYZPtr &feature_cloud )
{
    // Clear
    if(locations_3d.size())
        locations_3d.clear();
    if(feature_cloud->size())
        feature_cloud->clear();

    for(int i = 0; i < locations_2d.size(); )
    {
        cv::Point2f p2d = locations_2d[i].pt;

        PointType p3d = cloud->at((int) p2d.x,(int) p2d.y);

        // Check for invalid measurements
        if ( isnan(p3d.x) || isnan(p3d.y) || isnan(p3d.z) )
        {
            locations_2d.erase( locations_2d.begin()+i );
            continue;
        }

        locations_3d.push_back(Eigen::Vector4f(p3d.x, p3d.y, p3d.z, 1.0));
        feature_cloud->push_back( pcl::PointXYZ(p3d.x, p3d.y, p3d.z) );
        i++; //Only increment if no element is removed from vector
//        if( locations_3d.size() > max_keyp )
//            break;
    }

    feature_cloud->is_dense = true;
    feature_cloud->height = 1;
    feature_cloud->width = feature_cloud->points.size();
}

void KinectListener::matchImageFeatures( KinectFrame& last_frame,
                                         KinectFrame& current_frame,
                                         vector< cv::DMatch > &goodMatches,
                                         double good_match_threshold,
                                         int min_match_size)
{
    vector< cv::DMatch > matches;
    if( !feature_extractor_type_.compare(("ORB")))
    {
        uint64_t* query_value =  reinterpret_cast<uint64_t*>(last_frame.feature_descriptors.data);
        uint64_t* search_array = reinterpret_cast<uint64_t*>(current_frame.feature_descriptors.data);
        for(unsigned int i = 0; i < last_frame.feature_locations_2d.size(); ++i, query_value += 4)
        {   //ORB feature = 32*8bit = 4*64bit
            int result_index = -1;
            int hd = bruteForceSearchORB(query_value, search_array, current_frame.feature_locations_2d.size(), result_index);
            if(hd >= 128)
                continue;//not more than half of the bits matching: Random
            cv::DMatch match(i, result_index, hd /256.0 + (float)rand()/(1000.0*RAND_MAX));
            matches.push_back(match);
        }
    }
    else
    {
        cv::FlannBasedMatcher matcher;
    //    cout << MAGENTA << " features: " << last_frame.feature_locations_2d.size() << ", "
    //         << current_frame.feature_locations_2d.size() << RESET << endl;
    //    cout << MAGENTA << " descriptors: " << last_frame.feature_descriptors.rows << ", "
    //         << current_frame.feature_descriptors.rows << RESET << endl;
        matcher.match( last_frame.feature_descriptors, current_frame.feature_descriptors, matches );
    }

    // Sort
    std::sort( matches.begin(), matches.end() );

    // Get good matches, fixed size
    if( min_match_size != 0)
    {
        int add = 0;
        BOOST_FOREACH(const cv::DMatch& m, matches)
        {
            if( add >= min_match_size )
                break;

            if( last_frame.feature_locations_3d[m.queryIdx](2) != 0
                    && current_frame.feature_locations_3d[m.trainIdx](2) != 0)
            {
                goodMatches.push_back( m );
                add ++;
            }
        }
    }
    else
    {
        double minDis = -1.0;
//        for ( size_t i=0; i<matches.size(); i++ )
//        {
//            if ( matches[i].distance < minDis )
//                minDis = matches[i].distance;
//        }
        if( matches.size() >= ransac_sample_size_ )
            minDis = matches[0].distance;

        BOOST_FOREACH(const cv::DMatch& m, matches)
        {
            if( m.distance >= good_match_threshold * minDis )
                break;

            if( last_frame.feature_locations_3d[m.queryIdx](2) != 0
                    && current_frame.feature_locations_3d[m.trainIdx](2) != 0)
            {
                goodMatches.push_back( m );
            }
        }

    }

//    cout << "good matches: " << goodMatches.size() << endl;
}

std::vector<PlanePair> KinectListener::randomChoosePlanePairsPreferGood( const unsigned int sample_size,
                                         std::vector<PlanePair> &pairs )
{
    std::set<int> sampled_ids;
    int safety_net = 0;
    while( sampled_ids.size() < sample_size && pairs.size() >= sample_size )
    {
        int id1 = rand() % pairs.size();
        int id2 = rand() % pairs.size();
        if(id1 > id2) id1 = id2; //use smaller one => increases chance for lower id
            sampled_ids.insert(id1);
        if(++safety_net > 100)
        {
            ROS_ERROR("Infinite Plane Sampling");
            break;
        }
    }

    std::vector<PlanePair> sampled_pairs;
    sampled_pairs.reserve( sampled_ids.size() );
    BOOST_FOREACH(int id, sampled_ids)
    {
        sampled_pairs.push_back(pairs[id]);
    }
    return sampled_pairs;
}

std::vector<PlanePair> KinectListener::randomChoosePlanePairs( const unsigned int sample_size,
                                         std::vector<PlanePair> &pairs )
{
    std::set<int> sampled_ids;
    int safety_net = 0;
    while( sampled_ids.size() < sample_size && pairs.size() >= sample_size )
    {
        int id = rand() % pairs.size();
        sampled_ids.insert(id);
        if(++safety_net > 100)
        {
            ROS_ERROR("Infinite Plane Sampling");
            break;
        }
    }

    std::vector<PlanePair> sampled_pairs;
    sampled_pairs.reserve( sampled_ids.size() );
    BOOST_FOREACH(int id, sampled_ids)
    {
        sampled_pairs.push_back(pairs[id]);
    }
    return sampled_pairs;
}

std::vector<cv::DMatch> KinectListener::randomChooseMatchesPreferGood( const unsigned int sample_size,
                                         vector< cv::DMatch > &matches_with_depth )
{
    std::set<std::vector<cv::DMatch>::size_type> sampled_ids;
    int safety_net = 0;
    while(sampled_ids.size() < sample_size && matches_with_depth.size() >= sample_size)
    {
        int id1 = rand() % matches_with_depth.size();
        int id2 = rand() % matches_with_depth.size();
        if(id1 > id2) id1 = id2; //use smaller one => increases chance for lower id
            sampled_ids.insert(id1);
        if(++safety_net > 2000)
        {
            ROS_ERROR("Infinite Sampling");
            break;
        }
    }

    std::vector<cv::DMatch> sampled_matches;
    sampled_matches.reserve( sampled_ids.size() );
    BOOST_FOREACH(std::vector<cv::DMatch>::size_type id, sampled_ids)
    {
        sampled_matches.push_back(matches_with_depth[id]);
    }
    return sampled_matches;
}

std::vector<cv::DMatch> KinectListener::randomChooseMatches( const unsigned int sample_size,
                                         vector< cv::DMatch > &matches )
{
    std::set<std::vector<cv::DMatch>::size_type> sampled_ids;
    int safety_net = 0;
    while(sampled_ids.size() < sample_size && matches.size() >= sample_size)
    {
        int id = rand() % matches.size();
        sampled_ids.insert(id);
        if(++safety_net > 2000)
        {
            ROS_ERROR("Infinite Sampling");
            break;
        }
    }

    std::vector<cv::DMatch> sampled_matches;
    sampled_matches.reserve( sampled_ids.size() );
    BOOST_FOREACH(std::vector<cv::DMatch>::size_type id, sampled_ids)
    {
        sampled_matches.push_back(matches[id]);
    }
    return sampled_matches;
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
        plane_from_line_segment_.setCameraParameters( real_camera_parameters_ );
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
    plane_slam_->setPlaneMatchOverlapAlpha( config.plane_match_overlap_alpha );
    plane_slam_->setPlaneInlierLeafSize( config.plane_inlier_leaf_size );
    plane_slam_->setPlaneHullAlpha( config.plane_hull_alpha );
    plane_slam_->setRefinePlanarMap( config.refine_planar_map );
    plane_slam_->setPlanarMergeThreshold( config.planar_merge_direction_threshold, config.planar_merge_distance_threshold );
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

void KinectListener::downsampleOrganizedCloud(const PointCloudTypePtr &input, PlaneFromLineSegment::CAMERA_PARAMETERS &in_camera,
                                              PointCloudTypePtr &output, PlaneFromLineSegment::CAMERA_PARAMETERS &out_camera, int size_type)
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
    out_camera.cx = in_camera.cx / skip;
    out_camera.cy = in_camera.cy / skip;
    out_camera.fx = in_camera.fx / skip;
    out_camera.fy = in_camera.fy / skip;
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
//    camera.cx = cam_info_msg->K[2];
//    camera.cy = cam_info_msg->K[5];
//    camera.fx = cam_info_msg->K[0];
//    camera.fy = cam_info_msg->K[4];
//
//    //
//    camera.scale = 1.0;
//    // Additionally, organized cloud width and height.
//    camera.width = cam_info_msg->width;
//    camera.height = cam_info_msg->height;


    // TUM3
    camera.cx = 320.1;
    camera.cy = 247.6;
    camera.fx = 535.4;
    camera.fy = 539.2;
    //
    camera.scale = 1.0;
    camera.width = 640;
    camera.height = 480;
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
        ROS_WARN("%s",ex.what());
        return false;
    }
    odom_pose.setOrigin( trans.getOrigin() );
    odom_pose.setRotation( trans.getRotation() );

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
        fd = new cv::FastFeatureDetector( 20/*threshold*/, true/*nonmax_suppression*/ );
//        fd = new cv::FastFeatureDetector();
    }
    else if( !detectorType.compare( "SURF" ) )
    {
        fd = new cv::SurfFeatureDetector(300.0, 6, 5);
//        fd = new cv::SurfFeatureDetector();
    }
    else if( !detectorType.compare( "ORB" ) )
    {
//        detAdj = new DetectorAdjuster("AORB", 20);
        fd = new cv::OrbFeatureDetector( 10000, 1.2, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31 );
//        fd = new cv::OrbFeatureDetector();
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
        extractor = new cv::OrbDescriptorExtractor( 10000, 1.2, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31 );
//        extractor = new cv::OrbDescriptorExtractor();
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
