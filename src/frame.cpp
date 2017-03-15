#include "frame.h"

namespace plane_slam
{

Frame::Frame()
    : valid_(false),
      key_frame_(false),
      keypoint_type_(""),
      pose_( tf::Quaternion(0, 0, 0, 1.0), tf::Vector3(0, 0, 0) ),
      odom_pose_( tf::Quaternion(0, 0, 0, 1.0), tf::Vector3(0, 0, 0) ),
      world_pose_( tf::Quaternion(0, 0, 0, 1.0), tf::Vector3(0, 0, 0) ),
      camera_params_(),
      cloud_downsampled_( new PointCloudType ),
      feature_cloud_( new PointCloudXYZ )
{

}

Frame::Frame( PointCloudTypePtr &input, CameraParameters &camera_params,
              LineBasedPlaneSegmentor* line_based_plane_segmentor)
    : valid_(false),
      key_frame_(false),
      keypoint_type_(""),
      pose_( tf::Quaternion(0, 0, 0, 1.0), tf::Vector3(0, 0, 0) ),
      odom_pose_( tf::Quaternion(0, 0, 0, 1.0), tf::Vector3(0, 0, 0) ),
      camera_params_(),
      cloud_downsampled_( new PointCloudType ),
      feature_cloud_( new PointCloudXYZ ),
      line_based_plane_segmentor_(line_based_plane_segmentor)
{
    // Observation
    cloud_ = input; // no copy
    camera_params_ = camera_params;

    // Downsample cloud
    downsampleOrganizedCloud( cloud_, camera_params_, cloud_downsampled_, camera_params_downsampled_, QQVGA );

    // Only segment planes
    lineBasedPlaneSegment();
}

Frame::Frame( cv::Mat &visual, PointCloudTypePtr &input, CameraParameters &camera_params,
              ORBextractor* orb_extractor, LineBasedPlaneSegmentor* line_based_plane_segmentor)
    : valid_(false),
      key_frame_(false),
      keypoint_type_( "ORB" ),
      pose_( tf::Quaternion(0, 0, 0, 1.0), tf::Vector3(0, 0, 0) ),
      odom_pose_( tf::Quaternion(0, 0, 0, 1.0), tf::Vector3(0, 0, 0) ),
      camera_params_(),
      cloud_downsampled_( new PointCloudType ),
      feature_cloud_( new PointCloudXYZ ),
      orb_extractor_(orb_extractor),
      line_based_plane_segmentor_(line_based_plane_segmentor)
{
    // Observation
    visual_image_ = visual; // no copy
    cloud_ = input; // no copy
    camera_params_ = camera_params;

    // Downsample cloud
    downsampleOrganizedCloud( cloud_, camera_params_, cloud_downsampled_, camera_params_downsampled_, QQVGA );

    // Spin 2 threads, one for plane segmentation, another for keypoint extraction.
    thread threadSegment( &Frame::lineBasedPlaneSegment, this );
    thread threadExtract( &Frame::extractORB, this );
    threadSegment.join();
    threadExtract.join();
}

Frame::Frame( cv::Mat &visual, cv::Mat &depth, CameraParameters &camera_params,
              cv::FeatureDetector* surf_detector, cv::DescriptorExtractor* surf_extractor,
              LineBasedPlaneSegmentor* line_based_plane_segmentor )
    : valid_(false),
      key_frame_(false),
      keypoint_type_( "SURF" ),
      pose_( tf::Quaternion(0, 0, 0, 1.0), tf::Vector3(0, 0, 0) ),
      odom_pose_( tf::Quaternion(0, 0, 0, 1.0), tf::Vector3(0, 0, 0) ),
      camera_params_(),
      cloud_downsampled_( new PointCloudType ),
      feature_cloud_( new PointCloudXYZ ),
      surf_detector_( surf_detector ),
      surf_extractor_( surf_extractor ),
      line_based_plane_segmentor_(line_based_plane_segmentor)
{
    // Construct organized pointcloud
    PointCloudTypePtr input = image2PointCloud( visual, depth, camera_params );

    // Observation
    visual_image_ = visual; // no copy
    depth_image_ = depth;   // no copy
    cloud_ = input; // no copy
    camera_params_ = camera_params;

    // Downsample cloud
    downsampleOrganizedCloud( cloud_, camera_params_, cloud_downsampled_, camera_params_downsampled_, QQVGA );

    // Spin 2 threads, one for plane segmentation, another for keypoint extraction.
//    extractORB();
//    segmentPlane();
    thread threadSegment( &Frame::lineBasedPlaneSegment, this );
    thread threadExtract( &Frame::extractSurf, this );
    threadSegment.join();
    threadExtract.join();
}

Frame::Frame( cv::Mat &visual, cv::Mat &depth, CameraParameters &camera_params,
              ORBextractor* orb_extractor, LineBasedPlaneSegmentor* line_based_plane_segmentor )
    : valid_(false),
      key_frame_(false),
      keypoint_type_( "ORB" ),
      pose_( tf::Quaternion(0, 0, 0, 1.0), tf::Vector3(0, 0, 0) ),
      odom_pose_( tf::Quaternion(0, 0, 0, 1.0), tf::Vector3(0, 0, 0) ),
      camera_params_(),
      cloud_downsampled_( new PointCloudType ),
      feature_cloud_( new PointCloudXYZ ),
      orb_extractor_( orb_extractor ),
      line_based_plane_segmentor_( line_based_plane_segmentor )
{
    ros::Time start_time = ros::Time::now();
    ros::Time dura_start = start_time;

    // Construct organized pointcloud
    PointCloudTypePtr input = image2PointCloud( visual, depth, camera_params );
    //
    pointcloud_cvt_duration_ = (ros::Time::now() - dura_start).toSec()*1000;
    dura_start = ros::Time::now();

    // Observation
    visual_image_ = visual; // no copy
    depth_image_ = depth;
    cloud_ = input; // no copy
    camera_params_ = camera_params;

    // Downsample cloud
    downsampleOrganizedCloud( cloud_, camera_params_, cloud_downsampled_, camera_params_downsampled_, QQVGA );
    //
    pointcloud_downsample_duration_ = (ros::Time::now() - dura_start).toSec()*1000;
    dura_start = ros::Time::now();

    // Spin 2 threads, one for plane segmentation, another for keypoint extraction.
//    extractORB();
//    segmentPlane();
    thread threadSegment( &Frame::lineBasedPlaneSegment, this );
    thread threadExtract( &Frame::extractORB, this );
    threadSegment.join();
    threadExtract.join();

    total_duration_ = (ros::Time::now() - start_time).toSec()*1000;
}


Frame::Frame( PointCloudTypePtr &input, CameraParameters &camera_params,
              OrganizedPlaneSegmentor* organized_plane_segmentor)
    : valid_(false),
      key_frame_(false),
      keypoint_type_(""),
      pose_( tf::Quaternion(0, 0, 0, 1.0), tf::Vector3(0, 0, 0) ),
      odom_pose_( tf::Quaternion(0, 0, 0, 1.0), tf::Vector3(0, 0, 0) ),
      camera_params_(),
      cloud_downsampled_( new PointCloudType ),
      feature_cloud_( new PointCloudXYZ ),
      organized_plane_segmentor_(organized_plane_segmentor)
{
    // Observation
    cloud_ = input; // no copy
    camera_params_ = camera_params;

    // Downsample cloud
    downsampleOrganizedCloud( cloud_, camera_params_, cloud_downsampled_, camera_params_downsampled_, QQVGA );

    // Only segment planes
    organizedPlaneSegment();
}

Frame::Frame( cv::Mat &visual, PointCloudTypePtr &input, CameraParameters &camera_params,
              ORBextractor* orb_extractor, OrganizedPlaneSegmentor* organized_plane_segmentor)
    : valid_(false),
      key_frame_(false),
      keypoint_type_( "ORB" ),
      pose_( tf::Quaternion(0, 0, 0, 1.0), tf::Vector3(0, 0, 0) ),
      odom_pose_( tf::Quaternion(0, 0, 0, 1.0), tf::Vector3(0, 0, 0) ),
      camera_params_(),
      cloud_downsampled_( new PointCloudType ),
      feature_cloud_( new PointCloudXYZ ),
      orb_extractor_(orb_extractor),
      organized_plane_segmentor_(organized_plane_segmentor)
{
    // Observation
    visual_image_ = visual; // no copy
    cloud_ = input; // no copy
    camera_params_ = camera_params;

    // Downsample cloud
    downsampleOrganizedCloud( cloud_, camera_params_, cloud_downsampled_, camera_params_downsampled_, QQVGA );

    // Spin 2 threads, one for plane segmentation, another for keypoint extraction.
    thread threadSegment( &Frame::organizedPlaneSegment, this );
    thread threadExtract( &Frame::extractORB, this );
    threadSegment.join();
    threadExtract.join();
}

Frame::Frame( cv::Mat &visual, cv::Mat &depth, CameraParameters &camera_params,
              cv::FeatureDetector* surf_detector, cv::DescriptorExtractor* surf_extractor,
              OrganizedPlaneSegmentor* organized_plane_segmentor )
    : valid_(false),
      key_frame_(false),
      keypoint_type_( "SURF" ),
      pose_( tf::Quaternion(0, 0, 0, 1.0), tf::Vector3(0, 0, 0) ),
      odom_pose_( tf::Quaternion(0, 0, 0, 1.0), tf::Vector3(0, 0, 0) ),
      camera_params_(),
      cloud_downsampled_( new PointCloudType ),
      feature_cloud_( new PointCloudXYZ ),
      surf_detector_( surf_detector ),
      surf_extractor_( surf_extractor ),
      organized_plane_segmentor_(organized_plane_segmentor)
{
    // Construct organized pointcloud
    PointCloudTypePtr input = image2PointCloud( visual, depth, camera_params );

    // Observation
    visual_image_ = visual; // no copy
    depth_image_ = depth;   // no copy
    cloud_ = input; // no copy
    camera_params_ = camera_params;

    // Downsample cloud
    downsampleOrganizedCloud( cloud_, camera_params_, cloud_downsampled_, camera_params_downsampled_, QQVGA );

    // Spin 2 threads, one for plane segmentation, another for keypoint extraction.
//    extractORB();
//    segmentPlane();
    thread threadSegment( &Frame::organizedPlaneSegment, this );
    thread threadExtract( &Frame::extractSurf, this );
    threadSegment.join();
    threadExtract.join();
}

Frame::Frame( cv::Mat &visual, cv::Mat &depth, CameraParameters &camera_params,
              ORBextractor* orb_extractor, OrganizedPlaneSegmentor* organized_plane_segmentor)
    : valid_(false),
      key_frame_(false),
      keypoint_type_( "ORB" ),
      pose_( tf::Quaternion(0, 0, 0, 1.0), tf::Vector3(0, 0, 0) ),
      odom_pose_( tf::Quaternion(0, 0, 0, 1.0), tf::Vector3(0, 0, 0) ),
      camera_params_(),
      cloud_downsampled_( new PointCloudType ),
      feature_cloud_( new PointCloudXYZ ),
      orb_extractor_( orb_extractor ),
      organized_plane_segmentor_( organized_plane_segmentor )
{
    ros::Time start_time = ros::Time::now();
    ros::Time dura_start = start_time;

    // Construct organized pointcloud
    PointCloudTypePtr input = image2PointCloud( visual, depth, camera_params );
    //
    pointcloud_cvt_duration_ = (ros::Time::now() - dura_start).toSec()*1000;
    dura_start = ros::Time::now();

    // Observation
    visual_image_ = visual; // no copy
    depth_image_ = depth;
    cloud_ = input; // no copy
    camera_params_ = camera_params;

    // Downsample cloud
    downsampleOrganizedCloud( cloud_, camera_params_, cloud_downsampled_, camera_params_downsampled_, QQVGA );
    //
    pointcloud_downsample_duration_ = (ros::Time::now() - dura_start).toSec()*1000;
    dura_start = ros::Time::now();


    // Spin 2 threads, one for plane segmentation, another for keypoint extraction.
//    extractORB();
//    segmentPlane();
    thread threadSegment( &Frame::organizedPlaneSegment, this );
    thread threadExtract( &Frame::extractORB, this );
    threadSegment.join();
    threadExtract.join();

    total_duration_ = (ros::Time::now() - start_time).toSec()*1000;
}

void Frame::throttleMemory()
{
    //
    visual_image_.release();
    gray_image_.release();
    depth_image_.release();
    depth_mono8_image_.release();
    visual_image_downsampled_.release();
    //
    cloud_->clear();
//    cloud_downsampled_->clear();
    feature_cloud_->clear();
}

// Feature extraction, using visual image and cloud in VGA resolution
void Frame::extractSurf()
{
    ros::Time start = ros::Time::now();

    // Get gray image
    if(visual_image_.type() == CV_8UC3)
        cv::cvtColor( visual_image_, gray_image_, CV_RGB2GRAY );
    else
        gray_image_ = visual_image_;

    // Build mask
    depthToCV8UC1(depth_image_, depth_mono8_image_);
    /// TODO:
    /// 1: detect
    /// 2: project
    /// 3: extract
    // Extract features
    surf_detector_->detect( gray_image_, feature_locations_2d_, depth_mono8_image_ ); // fill 2d locations
    surf_extractor_->compute( gray_image_, feature_locations_2d_, feature_descriptors_ ); //fill feature_descriptors_ with information

    // Project Keypoint to 3D
    projectKeypointTo3D( cloud_, feature_locations_2d_, feature_locations_3d_, feature_cloud_);

    //
    keypoint_extract_duration_ = (ros::Time::now() - start).toSec()*1000;
}

// Feature Extraction, using visual image and cloud in VGA resolution.
void Frame::extractORB()
{
    ros::Time start = ros::Time::now();
    // Get gray image
    if(visual_image_.type() == CV_8UC3)
        cv::cvtColor( visual_image_, gray_image_, CV_RGB2GRAY );
    else
        gray_image_ = visual_image_;
    // Extract features
    (*orb_extractor_)( gray_image_, cv::Mat(), feature_locations_2d_, feature_descriptors_);

    // Project Keypoint to 3D
    projectKeypointTo3D( cloud_, feature_locations_2d_, feature_locations_3d_, feature_cloud_);

    //
    keypoint_extract_duration_ = (ros::Time::now() - start).toSec()*1000;
}

// Plane segmentation, using downsampled pointcloud in QVGA resolution.
void Frame::lineBasedPlaneSegment()
{
    ros::Time start = ros::Time::now();
    (*line_based_plane_segmentor_)( cloud_downsampled_, segment_planes_, camera_params_downsampled_ );
    plane_segment_duration_ = (ros::Time::now() - start).toSec()*1000;
}

void Frame::organizedPlaneSegment()
{
    ros::Time start = ros::Time::now();
    (*organized_plane_segmentor_)( cloud_downsampled_, segment_planes_ );
    plane_segment_duration_ = (ros::Time::now() - start).toSec()*1000;
}

void Frame::downsampleOrganizedCloud( const PointCloudTypePtr &input, CameraParameters &in_camera,
                                      PointCloudTypePtr &output, CameraParameters &out_camera, int size_type)
{
//    ROS_ASSERT( input->width == 640 && input->header == 480);

    int skip = pow(2, size_type);
    int width = input->width / skip;
    int height = input->height / skip;
    output->width = width;
    output->height = height;
    output->is_dense = false;
    output->points.resize( width * height);
    for( int i = 0, y = 0; i < height; i++, y+=skip)
    {
        for( int j = 0, x = 0; j < width; j++, x+=skip)
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

void Frame::downsampleImage(const cv::Mat &input, cv::Mat &output, int size_type)
{
    int skip = pow(2, size_type);
    int width = input.cols / skip;
    int height = input.rows / skip;

//    cout << GREEN << " - downsample image size: " << width << "x" << height << RESET << endl;
    cv::pyrDown( input, output, cv::Size( width, height ) );
}

void Frame::projectKeypointTo3D( const PointCloudTypePtr &cloud,
                                std::vector<cv::KeyPoint> &locations_2d,
                                std_vector_of_eigen_vector4f &locations_3d,
                                PointCloudXYZPtr &feature_cloud )
{
    // Clear
    if(locations_3d.size())
    locations_3d.clear();
    if(feature_cloud->size())
    feature_cloud->clear();

    for(size_t i = 0; i < locations_2d.size(); i++)
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

PointCloudTypePtr Frame::image2PointCloud( const cv::Mat &rgb_img, const cv::Mat &depth_img,
                                           const CameraParameters& camera )
{
//    ROS_ASSERT( rgb_img.rows == depth_img.rows && rgb_img.cols == depth_img.cols );

    PointCloudTypePtr cloud ( new PointCloudType );
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
            color.Alpha = 255.0;
            pt.rgb = color.float_value;
            //
            pt_iter ++;
            depth_idx ++;
            color_idx += color_skip_idx;
        }
    }

    return cloud;
}

} // end of namespace plane_slam
