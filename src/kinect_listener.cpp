#include "kinect_listener.h"


KinectListener::KinectListener() :
    private_nh_("~")
  , plane_slam_config_server_( ros::NodeHandle( "PlaneSlam" ) )
  , plane_segment_config_server_( ros::NodeHandle( "PlaneSegment" ) )
  , organized_segment_config_server_( ros::NodeHandle( "OrganizedSegment" ) )
  , tf_listener_( nh_, ros::Duration(10.0) )
  , pcl_viewer_( new pcl::visualization::PCLVisualizer("3D Viewer"))
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
    }

    private_nh_.param<int>("subscriber_queue_size", subscriber_queue_size_, 4);
    private_nh_.param<string>("topic_image_visual", topic_image_visual_, "/camera/rgb/image_color");
    private_nh_.param<string>("topic_image_depth", topic_image_depth_, "/camera/depth/image");
    private_nh_.param<string>("topic_camera_info", topic_camera_info_, "/camera/rgb/camera_info");
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


void KinectListener::noCloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                                const sensor_msgs::ImageConstPtr& depth_img_msg,
                                const sensor_msgs::CameraInfoConstPtr& cam_info_msg)
{
    printf("no cloud msg: %d\n", visual_img_msg->header.seq);

    // get transform
    tf::StampedTransform trans;
    try{
        tf_listener_.lookupTransform(visual_img_msg->header.frame_id, odom_frame_, ros::Time(0), trans);
    }catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        return;
    }
    gtsam::Rot3 rot3 = gtsam::Rot3::Quaternion( trans.getRotation().w(), trans.getRotation().x(),
                                                trans.getRotation().y(), trans.getRotation().z() );
    gtsam::Pose3 odom_pose(rot3, gtsam::Point3(trans.getOrigin()));

    //
    getCameraParameter( cam_info_msg, camera_parameters_);
    PointCloudTypePtr cloud_in ( new PointCloudType );
    // Get Mat Image
    cv::Mat depth_image = cv_bridge::toCvCopy(depth_img_msg)->image; // to cv image
    cv::Mat visual_image = cv_bridge::toCvCopy(visual_img_msg)->image; // to cv image
    // Get PointCloud
    cloud_in = image2PointCloud( visual_image, depth_image, camera_parameters_);

    //
    if(!loop_one_message_)
        processCloud( cloud_in, odom_pose );
    else
        while(loop_one_message_ && ros::ok())
        {
            ros::Duration(0.2).sleep();
            processCloud( cloud_in, odom_pose );
        }
}

void KinectListener::cloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                                const sensor_msgs::PointCloud2ConstPtr& point_cloud,
                                const sensor_msgs::CameraInfoConstPtr& cam_info_msg)
{
    printf("cloud msg: %d\n", visual_img_msg->header.seq);

    // get transform
    tf::StampedTransform trans;
    try{
        tf_listener_.lookupTransform(visual_img_msg->header.frame_id, odom_frame_, ros::Time(0), trans);
    }catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        return;
    }
    gtsam::Rot3 rot3 = gtsam::Rot3::Quaternion( trans.getRotation().w(), trans.getRotation().x(),
                                                trans.getRotation().y(), trans.getRotation().z() );
    gtsam::Pose3 odom_pose(rot3, gtsam::Point3(trans.getOrigin()));

    //
    getCameraParameter( cam_info_msg, camera_parameters_);

    //
    PointCloudTypePtr cloud_in ( new PointCloudType );
    pcl::fromROSMsg( *point_cloud, *cloud_in);

    //
    if(!loop_one_message_)
        processCloud( cloud_in, odom_pose );
    else
        while(loop_one_message_ && ros::ok())
        {
            ros::Duration(0.2).sleep();
            processCloud( cloud_in, odom_pose );
        }
}

void KinectListener::processCloud( const PointCloudTypePtr &input, gtsam::Pose3 &odom_pose )
{
    static gtsam::Pose3 last_pose = odom_pose;

    std::vector<PlaneType> organized_planes;

    PointCloudTypePtr cloud_in( new PointCloudType );
    // if use downsample cloud
    cloud_size_type_ = cloud_size_type_config_;
    if( cloud_size_type_ == QVGA)
    {
        cout << GREEN << "QVGA" << RESET << endl;
        downsampleOrganizedCloud( input, cloud_in, camera_parameters_, (int)QVGA);
    }
    else if( cloud_size_type_ == QQVGA)
    {
        cout << GREEN << "QQVGA" << RESET << endl;
        downsampleOrganizedCloud( input, cloud_in, camera_parameters_, (int)QQVGA);
    }
    else
    {
        cout << GREEN << "VGA" << RESET << endl;
        cloud_in = input; // copy pointer
    }

    double start_time = pcl::getTime();
    pcl::console::TicToc time;
    time.tic();
    float organized_dura = 0;
    float display_dura = 0;
    float total_dura = 0;

    // Plane Segment
    organizedPlaneSegment( cloud_in, organized_planes );
    organized_dura = time.toc();
    time.tic();

    // Do slam
    if(!is_initialized)
    {
        plane_slam_->initialize( odom_pose, organized_planes );
        is_initialized = true;
    }
    else
    {
        gtsam::Pose3 rel_pose = last_pose.inverse().compose( odom_pose );
        plane_slam_->planeSlam( rel_pose, organized_planes );
    }

    // display
    pcl_viewer_->removeAllPointClouds();
    pcl_viewer_->removeAllShapes();
    if(display_input_cloud_)
    {
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> rgba_color(frame_current.point_cloud, 255, 255, 255);
        pcl_viewer_->addPointCloud( cloud_in, "rgba_cloud" );
        pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "rgba_cloud");
    }
    displayPlanes( cloud_in, organized_planes, "organized_plane", viewer_v1_ );
    pcl_viewer_->spinOnce(1);

    display_dura = time.toc();
    total_dura = (pcl::getTime() - start_time) * 1000;

    ROS_INFO("Total time: %f, segment %f, display %f \n", total_dura, organized_dura, display_dura);
    cout << "----------------------------------- END -------------------------------------" << endl;
}

void KinectListener::organizedPlaneSegment(PointCloudTypePtr &input, std::vector<PlaneType> &planes)
{
    // copy input cloud
    PointCloudTypePtr cloud_in ( new PointCloudType );
    pcl::copyPointCloud( *input, *cloud_in);
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
        plane.inlier = indices.indices;
        plane.coefficients[0] = coef.values[0];
        plane.coefficients[1] = coef.values[1];
        plane.coefficients[2] = coef.values[2];
        plane.coefficients[3] = coef.values[3];
        plane.covariances = Eigen::Matrix4d::Zero();
        plane.covariances(0, 0) = pow(plane.coefficients[0]*1e-2, 2);
        plane.covariances(1, 1) = pow(plane.coefficients[1]*1e-2, 2);
        plane.covariances(2, 2) = pow(plane.coefficients[2]*1e-2, 2);
        plane.covariances(3, 3) = pow(plane.coefficients[3]*1e-2, 2);
        planes.push_back( plane );
    }
}

void KinectListener::planeSlamReconfigCallback(plane_slam::PlaneSlamConfig &config, uint32_t level)
{
    map_frame_ = config.map_frame;
    base_frame_ = config.base_frame;
    odom_frame_ = config.odom_frame;

    cout << GREEN <<"Common Slam Config." << RESET << endl;
}

void KinectListener::planeSegmentReconfigCallback(plane_slam::PlaneSegmentConfig &config, uint32_t level)
{
    cloud_size_type_config_ = config.cloud_size_type;
    plane_segment_method_ = config.segment_method;
    display_input_cloud_ = config.display_input_cloud;
    display_line_cloud_ = config.display_line_cloud;
    display_plane_ = config.display_plane;
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

void KinectListener::pclViewerLineRegion( const PointCloudTypePtr &input, PlaneFromLineSegment::LineType &line, const std::string &id, int viewpoint)
{
    PointCloudTypePtr cloud = getPointCloudFromIndices( input, line.inliers );

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color(cloud, rng.uniform(0.0, 255.0), rng.uniform(0.0, 255.0), rng.uniform(0.0, 255.0));
    pcl_viewer_->addPointCloud(cloud, color, id, viewpoint);
    pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, id, viewpoint);

}

void KinectListener::pclViewerNormal( const PointCloudTypePtr &input, PlaneFromLineSegment::NormalType &normal, const std::string &id, int viewpoint)
{
    PlaneType plane;
    plane.centroid = normal.centroid;
    plane.coefficients[0] = normal.coefficients[0];
    plane.coefficients[1] = normal.coefficients[1];
    plane.coefficients[2] = normal.coefficients[2];
    plane.coefficients[3] = normal.coefficients[3];
    plane.inlier = normal.inliers;

    pclViewerPlane( input, plane, id, viewpoint);
}

void KinectListener::pclViewerPlane( const PointCloudTypePtr &input, PlaneType &plane, const std::string &id, int viewpoint)
{
    double r = rng.uniform(0.0, 255.0);
    double g = rng.uniform(0.0, 255.0);
    double b = rng.uniform(0.0, 255.0);

    // add a line
    PointType p1, p2;
    p1 = plane.centroid;
    // check centroid
    if( p1.z == 0)
    {
        Eigen::Vector4f cen;
        pcl::compute3DCentroid( *input, plane.inlier, cen);
        p1.x = cen[0];
        p1.y = cen[1];
        p1.z = cen[2];
    }

    p2.x = p1.x + plane.coefficients[0]*0.2;
    p2.y = p1.y + plane.coefficients[1]*0.2;
    p2.z = p1.z + plane.coefficients[2]*0.2;
    pcl_viewer_->addArrow(p2, p1, r, g, b, false, id+"_arrow", viewpoint);
    // add a sphere
//    pcl_viewer_->addSphere( p1, 0.05, 255.0, 255.0, 0.0, id+"_point", viewpoint);
    // add inlier
    PointCloudTypePtr cloud = getPointCloudFromIndices( input, plane.inlier );

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color(cloud, r, g, b);
    pcl_viewer_->addPointCloud(cloud, color, id+"_inlier", viewpoint);
    pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id+"_inlier", viewpoint);
}

PointCloudTypePtr KinectListener::getPointCloudFromIndices( const PointCloudTypePtr &input,
                                             pcl::PointIndices &indices)
{
    PointCloudTypePtr output (new PointCloudType );
    for(int i = 0; i < indices.indices.size(); i++)
    {
        output->points.push_back( input->points[ indices.indices[i] ]);
    }
    output->is_dense = false;
    output->height = 1;
    output->width = output->points.size();
    return output;
}

PointCloudTypePtr KinectListener::getPointCloudFromIndices( const PointCloudTypePtr &input,
                                             std::vector<int> &indices)
{
    PointCloudTypePtr output (new PointCloudType );
    for(int i = 0; i < indices.size(); i++)
    {
        output->points.push_back( input->points[ indices[i] ]);
    }
    output->is_dense = false;
    output->height = 1;
    output->width = output->points.size();
    return output;
}

void KinectListener::downsampleOrganizedCloud(const PointCloudTypePtr &input, PointCloudTypePtr &output,
                                            CAMERA_INTRINSIC_PARAMETERS &out_camera, int size_type)
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

void KinectListener::getCameraParameter(const sensor_msgs::CameraInfoConstPtr &cam_info_msg,
                                      CAMERA_INTRINSIC_PARAMETERS &camera)
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
                                                                        const CAMERA_INTRINSIC_PARAMETERS& camera )
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
