#ifndef FRAME_H
#define FRAME_H

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <opencv2/core/core.hpp>
#include <pcl/pcl_base.h>
#include <gtsam/geometry/Pose3.h>
#include <plane_from_line/plane_from_line_segment.h>
#include "orb_extractor.h"
#include "line_based_plane_segmentor.h"
#include "utils.h"

namespace plane_slam
{

class Frame
{

public:
    Frame();
    Frame( PointCloudTypePtr &input, CameraParameters &camera_params,
           LineBasedPlaneSegmentor* plane_segmentor);
    Frame( cv::Mat &visual, PointCloudTypePtr &input, CameraParameters &camera_params,
           ORBextractor *orb_extractor, LineBasedPlaneSegmentor *plane_segmentor );
    Frame( cv::Mat &visual, cv::Mat &depth, CameraParameters &camera_params,
           ORBextractor* orb_extractor, LineBasedPlaneSegmentor* plane_segmentor );
    Frame( cv::Mat &visual, cv::Mat &depth, CameraParameters &camera_params,
           cv::FeatureDetector* surf_detector, cv::DescriptorExtractor* surf_extractor,
           LineBasedPlaneSegmentor* plane_segmentor );

    void extractSurf();
    void extractORB();
    void segmentPlane();
    inline void setId( int id ) { id_ = id; }
    int &id() {return id_;}
    void throttleMemory();

    enum { VGA = 0, QVGA = 1, QQVGA = 2};
    //
    void downsampleOrganizedCloud( const PointCloudTypePtr &input, CameraParameters &in_camera,
                                   PointCloudTypePtr &output, CameraParameters &out_camera, int size_type);

    //
    void downsampleImage(const cv::Mat &input, cv::Mat &output, int size_type);

    //
    void projectKeypointTo3D( const PointCloudTypePtr &cloud,
                              std::vector<cv::KeyPoint> &locations_2d,
                              std_vector_of_eigen_vector4f &locations_3d,
                              PointCloudXYZPtr &feature_cloud );

    //
    PointCloudTypePtr image2PointCloud( const cv::Mat &rgb_img, const cv::Mat &depth_img,
                                        const CameraParameters& camera );

private:
    cv::FeatureDetector* surf_detector_;
    cv::DescriptorExtractor* surf_extractor_;
    ORBextractor *orb_extractor_;
    LineBasedPlaneSegmentor *plane_segmentor_;
    // Surf detector/extractor



public:
    // Key frame
    bool key_frame_;
    // Valid
    bool valid_;    // for first frame, valid is under the condition that the number of planes is not zero,
                    // for other frame, valid is under the condition that relative motion respect to previous frame is valid.
    std::string keypoint_type_;
    // Id
    int id_;
    // Time Stamp
    ros::Time stamp_;
    // Robot Pose
    tf::Transform pose_;
    // Sensor data
    cv::Mat visual_image_;  // visual image
    cv::Mat gray_image_;    // gray image
    cv::Mat depth_image_;   // depth image
    cv::Mat depth_mono8_image_; // depth mono8 image
    PointCloudTypePtr cloud_;   // organized point cloud
    // Camera parameters
    CameraParameters camera_params_;
    // Downsampling data
    PointCloudTypePtr cloud_downsampled_; //
    cv::Mat visual_image_downsampled_; //
    CameraParameters camera_params_downsampled_;
    // Keypoint
    std::vector<cv::KeyPoint> feature_locations_2d_;
    cv::Mat feature_descriptors_;
    std_vector_of_eigen_vector4f feature_locations_3d_;
    PointCloudXYZPtr feature_cloud_;
    // For mapping
    std::vector<cv::DMatch> good_matches_;
    std::vector<cv::DMatch> kp_inlier_;
    std::vector<int> good_features_;
    // Planes
    std::vector<PlaneType> segment_planes_;
};

} // end of namespace plane_slam

#endif // FRAME_H
