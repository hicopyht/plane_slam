#include "kinect_listener.h"


const std::string KeypointWindow = "Keypoint";


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

    private_nh_.param<int>("subscriber_queue_size", subscriber_queue_size_, 4);
    private_nh_.param<string>("topic_image_visual", topic_image_visual_, "/camera/rgb/image_color");
    private_nh_.param<string>("topic_image_depth", topic_image_depth_, "/camera/depth/image");
    private_nh_.param<string>("topic_camera_info", topic_camera_info_, "/camera/depth/camera_info");
    private_nh_.param<string>("topic_point_cloud", topic_point_cloud_, "");

    pcl_viewer_->createViewPort(0, 0, 0.5, 0.5, viewer_v1_);
    pcl_viewer_->addText("LinesAndNormals", 100, 3, "v1_text", viewer_v1_);
    pcl_viewer_->createViewPort(0.5, 0, 1.0, 0.5, viewer_v2_);
    pcl_viewer_->addText("LineBasedPlanes", 100, 3, "v2_text", viewer_v2_);
    pcl_viewer_->createViewPort(0, 0.5, 0.5, 1.0, viewer_v3_);
    pcl_viewer_->addText("RansacPlanes", 100, 3, "v3_text", viewer_v3_);
    pcl_viewer_->createViewPort(0.5, 0.5, 1.0, 1.0, viewer_v4_);
    pcl_viewer_->addText("OrganizedPlanes", 100, 3, "v4_text", viewer_v4_);
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

    // feature detector
    detector_ = createDetector( feature_detector_type_ );
    extractor_ = createDescriptorExtractor( feature_extractor_type_ );

    true_path_publisher_ = nh_.advertise<nav_msgs::Path>("true_path", 10);
    pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("estimate_pose", 10);
    planar_map_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("planar_map", 10);
    auto_spin_map_viewer_ss_ = nh_.advertiseService("auto_spin_map_viewer", &KinectListener::autoSpinMapViewerCallback, this);

    // config subscribers
    if(!topic_point_cloud_.empty()) // pointcloud2
    {
        // use visual image, depth image, pointcloud2
        visual_sub_ = new image_sub_type(nh_, topic_image_visual_, subscriber_queue_size_);
        cloud_sub_ = new pc_sub_type (nh_, topic_point_cloud_, subscriber_queue_size_);
        cinfo_sub_ = new cinfo_sub_type(nh_, topic_camera_info_, subscriber_queue_size_);
        cloud_sync_ = new message_filters::Synchronizer<CloudSyncPolicy>(CloudSyncPolicy(subscriber_queue_size_),  *visual_sub_, *cloud_sub_, *cinfo_sub_),
        cloud_sync_->registerCallback(boost::bind(&KinectListener::cloudCallback, this, _1, _2, _3));
        ROS_INFO_STREAM("Listening to " << topic_image_visual_ << ", " << topic_point_cloud_ << " and " << topic_camera_info_ << ".");
    }
    else if(!topic_camera_info_.empty())
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
    printf("no cloud msg: %d\n", depth_img_msg->header.seq);

    // Get odom pose
    tf::Transform odom_pose;
    if( !getOdomPose( odom_pose, depth_img_msg->header.frame_id, ros::Time(0) ) )
        return;

    // Get camera parameter
    getCameraParameter( cam_info_msg, camera_parameters_);
    PointCloudTypePtr cloud_in ( new PointCloudType );
    // Get Mat Image
    cv::Mat depth_image = cv_bridge::toCvCopy(depth_img_msg)->image; // to cv image
    cv::Mat visual_image = cv_bridge::toCvCopy(visual_img_msg)->image; // to cv image
    // Get PointCloud
    cloud_in = image2PointCloud( visual_image, depth_image, camera_parameters_);

    //
    if(!loop_one_message_)
        processCloud( cloud_in, visual_image, odom_pose );
    else
        while(loop_one_message_ && ros::ok())
        {
            ros::Duration(0.2).sleep();
            processCloud( cloud_in, visual_image, odom_pose );
        }
}

void KinectListener::cloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                                const sensor_msgs::PointCloud2ConstPtr& point_cloud,
                                const sensor_msgs::CameraInfoConstPtr& cam_info_msg)
{
    printf("cloud msg: %d\n", point_cloud->header.seq);

    // Get odom pose
    tf::Transform odom_pose;
    if( !getOdomPose( odom_pose, point_cloud->header.frame_id, ros::Time(0) ) )
        return;

    // Get camera parameter
    getCameraParameter( cam_info_msg, camera_parameters_);

    // Get Mat Image
    cv::Mat visual_image = cv_bridge::toCvCopy(visual_img_msg)->image; // to cv image
    // To pcl pointcloud
    PointCloudTypePtr cloud_in ( new PointCloudType );
    pcl::fromROSMsg( *point_cloud, *cloud_in);

    //
    if(!loop_one_message_)
        processCloud( cloud_in, visual_image, odom_pose );
    else
        while(loop_one_message_ && ros::ok())
        {
            ros::Duration(0.2).sleep();
            processCloud( cloud_in, visual_image, odom_pose );
        }
}

void KinectListener::processCloud( const PointCloudTypePtr &input, const cv::Mat visual_image, tf::Transform &odom_pose )
{
//    static tf::Transform last_pose = odom_pose;
    geometry_msgs::PoseStamped pstamped;
    pstamped.pose.position.x = odom_pose.getOrigin().x();
    pstamped.pose.position.y = odom_pose.getOrigin().y();
    pstamped.pose.position.z = odom_pose.getOrigin().z();
    tf::quaternionTFToMsg( odom_pose.getRotation(), pstamped.pose.orientation );
    true_poses_.push_back( pstamped );
    publishTruePath();

    std::vector<PlaneType> segment_planes;

    PointCloudTypePtr cloud_in( new PointCloudType );
    cv::Mat visual;
    // if use downsample cloud
    cloud_size_type_ = cloud_size_type_config_;
    if( cloud_size_type_ == QVGA)
    {
        cout << GREEN << "QVGA" << RESET << endl;
        downsampleOrganizedCloud( input, cloud_in, camera_parameters_, (int)QVGA);
        downsampleImage( visual_image, visual, (int)QVGA );
    }
    else if( cloud_size_type_ == QQVGA)
    {
        cout << GREEN << "QQVGA" << RESET << endl;
        downsampleOrganizedCloud( input, cloud_in, camera_parameters_, (int)QQVGA);
        downsampleImage( visual_image, visual, (int)QQVGA );
    }
    else
    {
        cout << GREEN << "VGA" << RESET << endl;
        cloud_in = input; // copy pointer
        visual = visual_image;
    }

    double start_time = pcl::getTime();
    pcl::console::TicToc time;
    time.tic();
    float segment_dura = 0;
    float hull_dura = 0;
    float slam_dura = 0;
    float display_dura = 0;
    float total_dura = 0;

    // Plane Segment
    if( plane_segment_method_ == ORGANSIZED)
    {
        cout << GREEN << "Organized segmentation." << RESET << endl;
        organizedPlaneSegment( cloud_in, segment_planes );
        segment_dura = time.toc();
        time.tic();
    }
    else if( plane_segment_method_ == LINE_BADED )
    {
        cout << GREEN << "Line based segmentation." << RESET << endl;
        lineBasedPlaneSegment( cloud_in, segment_planes );
        segment_dura = time.toc();
        time.tic();

        // Detect Keypoint
        computeKeypoint( cloud_in, visual, segment_planes );
        displayKeypoint( visual, segment_planes );
    }
    else
    {
        cout << RED << "[Error]: Invalid segmentation method error." << RESET << endl;
        exit(0);
    }

    cout << GREEN << " planes: " << segment_planes.size() << RESET << endl;

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
            gtsam::Pose3 init_pose;
            plane_slam_->tfToPose3( odom_pose, init_pose);
            if( plane_slam_->initialize( init_pose, segment_planes ) )
                is_initialized = true;
            else
                return;
        }
        else
        {
            gtsam::Pose3 pose3;
            plane_slam_->tfToPose3( odom_pose, pose3 );
            gtsam::Pose3 estmated_pose = plane_slam_->planeSlam( pose3, segment_planes );
            publishPose( estmated_pose );
            if(display_path_)
                plane_slam_->publishEstimatedPath();
            if(display_odom_path_)
                plane_slam_->publishOdomPath();

            // visualize landmark
            landmarks = plane_slam_->getLandmarks();
            // project and recalculate contour

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
        pcl_viewer_->addPointCloud( cloud_in, "rgba_cloud", viewer_v1_ );
        pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "rgba_cloud", viewer_v1_);
    }
    displayPlanes( cloud_in, segment_planes, "segment_plane", viewer_v2_ );
    pcl_viewer_->spinOnce(1);
    map_viewer_->spinOnce(1);

    display_dura = time.toc();
    total_dura = (pcl::getTime() - start_time) * 1000;

    cout << GREEN << "Total time: " << total_dura << ", segment: " << segment_dura << ", hull: "
         << hull_dura << ", slam: "<< slam_dura << ", display: " << display_dura << RESET << endl;
    cout << "----------------------------------- END -------------------------------------" << endl;
}

void KinectListener::computeKeypoint( const PointCloudTypePtr &cloud, const cv::Mat &visual_image, std::vector<PlaneType> &planes )
{
    // Get gray image
    cv::Mat gray_img;
    if(visual_image.type() == CV_8UC3)
        cv::cvtColor(visual_image, gray_img, CV_RGB2GRAY);
    else
        gray_img = visual_image;

    // Detect feature for all planes
    for( int i = 0; i < planes.size(); i++)
    {
        PlaneType &plane = planes[i];
        detector_->detect( gray_img, plane.feature_locations_2d, plane.mask );// fill 2d locations
        extractor_->compute( gray_img, plane.feature_locations_2d, plane.feature_descriptors ); //fill feature_descriptors_ with information
        projectTo3D( cloud, plane.feature_locations_2d, plane.feature_locations_3d ); // project to 3d
    }

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
        plane_slam_->projectPoints( *input, plane.inlier, plane.coefficients, *(plane.cloud) );
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
        plane_slam_->projectPoints( *input, plane.inlier, plane.coefficients, *(plane.cloud) );
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
    display_input_cloud_ = config.display_input_cloud;
    display_line_cloud_ = config.display_line_cloud;
    display_normal_ = config.display_normal;
    display_normal_arrow_ = config.display_normal_arrow;
    display_plane_ = config.display_plane;
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

void KinectListener::displayKeypoint( const cv::Mat visual_image, std::vector<PlaneType> &planes)
{
    // Keypoints
    std::vector<cv::KeyPoint> keypoints;
    if( planes.size() > 0)
        keypoints = planes[0].feature_locations_2d;
//    for( int i = 0; i < planes.size(); i++)
//    {
//        keypoints += planes[i].feature_locations_2d;
//    }

    cv::Mat image;
    cv::drawKeypoints( visual_image, keypoints, image, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    cv::imshow( KeypointWindow, image );
    cv::waitKey(1);
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
            pclViewerPlane( input, planes[j], prefix + ss.str(), viewport );
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

void KinectListener::pclViewerPlane( const PointCloudTypePtr &input, PlaneType &plane, const std::string &id, int viewport)
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
            pcl_viewer_->addArrow(p2, p1, r, g, b, false, id+"_arrow", viewport);
            // add a sphere
        //    pcl_viewer_->addSphere( p1, 0.05, 255.0, 255.0, 0.0, id+"_point", viewport);
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
            pcl_viewer_->addArrow(p2, p1, r, g, b, false, id+"_arrow", viewport);
            // add a sphere
        //    pcl_viewer_->addSphere( p1, 0.05, 255.0, 255.0, 0.0, id+"_point", viewport);
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
