#include "frame.h"

namespace plane_slam
{

Frame::Frame( cv::Mat &visual, PointCloudTypePtr &input, CameraParameters &camera_params,
              ORBextractor* orb_extractor, LineBasedPlaneSegmentor* plane_segmentor)
    : orb_extractor_(orb_extractor),
      plane_segmentor_(plane_segmentor)
{
    // Observation
    visual_image_ = visual;
    cloud_ = input;
    camera_params_ = camera_params;

    // Downsample cloud
    downsampleOrganizedCloud( cloud_, camera_params_, cloud_downsampled_, camera_params_downsampled_, QVGA );

    // Spin 2 threads, one for plane segmentation, another for keypoint extraction.
    thread threadSegment( &Frame::segmentPlane, this );
    thread threadExtract( &Frame::extractORB, this );
    threadSegment.join();
    threadExtract.join();
}

Frame::Frame( cv::Mat &visual, cv::Mat &depth, CameraParameters &camera_params,
              ORBextractor* orb_extractor, LineBasedPlaneSegmentor* plane_segmentor)
    : orb_extractor_(orb_extractor),
      plane_segmentor_(plane_segmentor)
{
    // Construct organized pointcloud
    PointCloudTypePtr input = image2PointCloud( visual, depth, camera_params );

    // Observation
    visual_image_ = visual;
    cloud_ = input;
    camera_params_ = camera_params;

    // Downsample cloud
    downsampleOrganizedCloud( cloud_, camera_params_, cloud_downsampled_, camera_params_downsampled_, QVGA );

    // Spin 2 threads, one for plane segmentation, another for keypoint extraction.
    thread threadSegment( &Frame::segmentPlane, this );
    thread threadExtract( &Frame::extractORB, this );

}

// Feature Extraction, using visual image and cloud in VGA resolution.
void Frame::extractORB()
{
    // Get gray image
    if(visual_image_.type() == CV_8UC3)
        cv::cvtColor( visual_image_, gray_image_, CV_RGB2GRAY );
    else
        visual_image_ = gray_image_;
    // Extract features
    (*orb_extractor_)( gray_image_, cv::Mat(), feature_locations_2d_, feature_descriptors_);

    // Project Keypoint to 3D
    projectKeypointTo3D( cloud_, feature_locations_2d_, feature_locations_3d_, feature_cloud_);
}

// Plane segmentation, using downsampled pointcloud in QVGA resolution.
void Frame::segmentPlane()
{
    (*plane_segmentor_)( cloud_downsampled_, segment_planes_, camera_params_downsampled_ );
}

void Frame::downsampleOrganizedCloud( const PointCloudTypePtr &input, CameraParameters &in_camera,
                                      PointCloudTypePtr &output, CameraParameters &out_camera, int size_type)
{
    ROS_ASSERT( input->width == 640 && input->header == 480);

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

} // end of namespace plane_slam
