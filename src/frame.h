#ifndef FRAME_H
#define FRAME_H

#include <ros/ros.h>
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
    Frame( cv::Mat &visual, PointCloudTypePtr &input, CameraParameters &camera_params,
           ORBextractor *orb_extractor, LineBasedPlaneSegmentor *plane_segmentor );
    Frame( cv::Mat &visual, cv::Mat &depth, CameraParameters &camera_params,
           ORBextractor* orb_extractor, LineBasedPlaneSegmentor* plane_segmentor );

    void extractORB();
    void segmentPlane();

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


    ORBextractor *orb_extractor_;
    LineBasedPlaneSegmentor *plane_segmentor_;

public:
    // Robot Pose
    gtsam::Pose3 pose_;
    // Sensor data
    cv::Mat visual_image_;  // visual image
    cv::Mat gray_image_;    // gray image
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
    // Planes
    std::vector<PlaneType> segment_planes_;
};

} // end of namespace plane_slam

#endif // FRAME_H
