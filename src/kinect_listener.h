#ifndef KINECT_LISTENER_H
#define KINECT_LISTENER_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <dynamic_reconfigure/server.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
//
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/time.h>
#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
//
#include <plane_from_line/plane_from_line_segment.h>
#include <plane_slam/PlaneSegmentConfig.h>
#include <plane_slam/OrganizedSegmentConfig.h>
#include "organized_plane_segment.h"
#include "utils.h"
#include "plane_slam.h"

using namespace std;

typedef message_filters::Subscriber<sensor_msgs::Image> image_sub_type;
typedef message_filters::Subscriber<sensor_msgs::CameraInfo> cinfo_sub_type;
typedef message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_type;

//The policy merges kinect messages with approximately equal timestamp into one callback
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::PointCloud2,
                                                        sensor_msgs::CameraInfo> CloudSyncPolicy;

//The policy merges kinect messages with approximately equal timestamp into one callback
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image,
                                                        sensor_msgs::CameraInfo> NoCloudSyncPolicy;

class KinectListener
{
    enum { LINE_BADED = 0, RANSAC = 1, ORGANSIZED = 2, REGION_GROW = 3, ALL_METHOD = 4};
public:
    KinectListener();

    void processCloud( PointCloudTypePtr &input );

    void organizedPlaneSegment(PointCloudTypePtr &input, std::vector<PlaneType> &planes);



protected:
    void noCloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                         const sensor_msgs::ImageConstPtr& depth_img_msg,
                         const sensor_msgs::CameraInfoConstPtr& cam_info_msg);

    void cloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                        const sensor_msgs::PointCloud2ConstPtr& point_cloud,
                        const sensor_msgs::CameraInfoConstPtr& cam_info_msg);

    void planeSegmentReconfigCallback( plane_slam::PlaneSegmentConfig &config, uint32_t level);

    void organizedSegmentReconfigCallback( plane_slam::OrganizedSegmentConfig &config, uint32_t level);

    void getCameraParameter(const sensor_msgs::CameraInfoConstPtr &cam_info_msg,
                            CAMERA_INTRINSIC_PARAMETERS &camera);

    void displayPlanes( const PointCloudTypePtr &input, std::vector<PlaneType> &planes, const std::string &prefix, int viewport);

    void displayLinesAndNormals( const PointCloudTypePtr &input,
                                 std::vector<PlaneFromLineSegment::LineType> &lines,
                                 std::vector<PlaneFromLineSegment::NormalType> &normals,
                                 int viewport);

    void pclViewerLineRegion( const PointCloudTypePtr &input, PlaneFromLineSegment::LineType &line, const std::string &id, int viewpoint);

    void pclViewerNormal( const PointCloudTypePtr &input, PlaneFromLineSegment::NormalType &normal, const std::string &id, int viewpoint);

    void pclViewerPlane( const PointCloudTypePtr &input, PlaneType &plane, const std::string &id, int viewpoint);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr image2PointCloud( const cv::Mat &rgb_img, const cv::Mat &depth_img,
                                                            const CAMERA_INTRINSIC_PARAMETERS& camera );

    inline bool isValidPoint(const PointType &p)
    {
        return (prttcp_->isValid(p) && p.z > 0);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::CallbackQueue my_callback_queue_;
    ros::AsyncSpinner* async_spinner_;
    //
    dynamic_reconfigure::Server<plane_slam::PlaneSegmentConfig> plane_segment_config_server_;
    dynamic_reconfigure::Server<plane_slam::PlaneSegmentConfig>::CallbackType plane_segment_config_callback_;
    dynamic_reconfigure::Server<plane_slam::OrganizedSegmentConfig> organized_segment_config_server_;
    dynamic_reconfigure::Server<plane_slam::OrganizedSegmentConfig>::CallbackType organized_segment_config_callback_;

    // subscribers
    int subscriber_queue_size_;
    std::string topic_image_visual_;
    std::string topic_image_depth_;
    std::string topic_camera_info_;
    std::string topic_point_cloud_;
    //
    message_filters::Subscriber<sensor_msgs::Image> *visual_sub_;
    message_filters::Subscriber<sensor_msgs::Image> *depth_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> *cinfo_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_sub_;
    //
    message_filters::Synchronizer<CloudSyncPolicy>* cloud_sync_;
    message_filters::Synchronizer<NoCloudSyncPolicy>* no_cloud_sync_;
    //
    tf::TransformListener tf_listener_;
    //
    pcl::visualization::PCLVisualizer* pcl_viewer_;
    int viewer_v1_;
    int viewer_v2_;
    int viewer_v3_;
    int viewer_v4_;
    cv::RNG rng;
    PointRepresentationConstPtr prttcp_;

    //
    CAMERA_INTRINSIC_PARAMETERS camera_parameters_;

    // Plane segment
    int plane_segment_method_;
    bool display_input_cloud_;
    bool display_line_cloud_;
    bool display_normal_;
    bool display_plane_;
    bool loop_one_message_;

    // Organized Muit Plane segment parameters
    OrganizedPlaneSegment organized_plane_segment_;

    // RANSAC segment parameters
    int ransac_method_type_;
    double ransac_points_left_persentage_;
    double ransac_distance_threshold_;
    int ransac_max_iterations_;
    int ransac_min_points_size_;

    //
    bool is_initialized;
    PlaneSlam* plane_slam_;
};

#endif // KINECT_LISTENER_H
