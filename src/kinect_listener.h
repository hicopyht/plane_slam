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
#include <plane_slam/PlaneSlamConfig.h>
#include <plane_slam/PlaneSegmentConfig.h>
#include <plane_slam/OrganizedSegmentConfig.h>
#include <plane_slam/LineBasedSegmentConfig.h>
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
    enum { VGA = 0, QVGA = 1, QQVGA = 2};
    enum { LINE_BADED = 0, ORGANSIZED = 1};
public:
    KinectListener();

    void processCloud( const PointCloudTypePtr &input, tf::Transform &odom_pose );

    void lineBasedPlaneSegment(PointCloudTypePtr &input,
                               std::vector<PlaneType> &planes);

    void organizedPlaneSegment(PointCloudTypePtr &input, std::vector<PlaneType> &planes);



protected:
    void noCloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                         const sensor_msgs::ImageConstPtr& depth_img_msg,
                         const sensor_msgs::CameraInfoConstPtr& cam_info_msg);

    void cloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                        const sensor_msgs::PointCloud2ConstPtr& point_cloud,
                        const sensor_msgs::CameraInfoConstPtr& cam_info_msg);

    void setlineBasedPlaneSegmentParameters();

    void planeSlamReconfigCallback( plane_slam::PlaneSlamConfig &config, uint32_t level);

    void planeSegmentReconfigCallback( plane_slam::PlaneSegmentConfig &config, uint32_t level);

    void organizedSegmentReconfigCallback( plane_slam::OrganizedSegmentConfig &config, uint32_t level);

    void lineBasedSegmentReconfigCallback( plane_slam::LineBasedSegmentConfig &config, uint32_t level);

    void getPointCloudFromIndices( const PointCloudTypePtr &input,
                                   pcl::PointIndices &indices,
                                   PointCloudTypePtr &output);

    void getPointCloudFromIndices( const PointCloudTypePtr &input,
                                   std::vector<int> &indices,
                                   PointCloudTypePtr &output);

    PointCloudTypePtr getPointCloudFromIndices( const PointCloudTypePtr &input,
                                                 pcl::PointIndices &indices);

    PointCloudTypePtr getPointCloudFromIndices( const PointCloudTypePtr &input,
                                                 std::vector<int> &indices);

    void downsampleOrganizedCloud(const PointCloudTypePtr &input, PointCloudTypePtr &output,
                                  PlaneFromLineSegment::CAMERA_PARAMETERS &out_camera, int size_type);

    void getCameraParameter(const sensor_msgs::CameraInfoConstPtr &cam_info_msg,
                            PlaneFromLineSegment::CAMERA_PARAMETERS &camera);

    void publishPose( gtsam::Pose3 &pose);

    void displayLandmarks( const std::vector<PlaneType> &landmarks, const std::string &prefix = "landmark");

    void displayPlanes( const PointCloudTypePtr &input, std::vector<PlaneType> &planes, const std::string &prefix, int viewport);

    void displayLinesAndNormals( const PointCloudTypePtr &input,
                                 std::vector<PlaneFromLineSegment::LineType> &lines,
                                 std::vector<PlaneFromLineSegment::NormalType> &normals,
                                 int viewport);

    void pclViewerLandmark( const PlaneType &plane, const std::string &id );

    void pclViewerLineRegion( const PointCloudTypePtr &input, PlaneFromLineSegment::LineType &line, const std::string &id, int viewpoint);

    void pclViewerNormal( const PointCloudTypePtr &input, PlaneFromLineSegment::NormalType &normal, const std::string &id, int viewpoint);

    void pclViewerPlane( const PointCloudTypePtr &input, PlaneType &plane, const std::string &id, int viewport);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr image2PointCloud( const cv::Mat &rgb_img, const cv::Mat &depth_img,
                                                            const PlaneFromLineSegment::CAMERA_PARAMETERS& camera );

    bool getOdomPose( tf::Transform &odom_pose, const std::string &camera_frame );

    void publishTruePath();

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
    dynamic_reconfigure::Server<plane_slam::PlaneSlamConfig> plane_slam_config_server_;
    dynamic_reconfigure::Server<plane_slam::PlaneSlamConfig>::CallbackType plane_slam_config_callback_;
    dynamic_reconfigure::Server<plane_slam::PlaneSegmentConfig> plane_segment_config_server_;
    dynamic_reconfigure::Server<plane_slam::PlaneSegmentConfig>::CallbackType plane_segment_config_callback_;
    dynamic_reconfigure::Server<plane_slam::OrganizedSegmentConfig> organized_segment_config_server_;
    dynamic_reconfigure::Server<plane_slam::OrganizedSegmentConfig>::CallbackType organized_segment_config_callback_;
    dynamic_reconfigure::Server<plane_slam::LineBasedSegmentConfig> line_based_segment_config_server_;
    dynamic_reconfigure::Server<plane_slam::LineBasedSegmentConfig>::CallbackType line_based_segment_config_callback_;

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
    ros::Publisher true_path_publisher_;
    ros::Publisher pose_publisher_;
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
    pcl::visualization::PCLVisualizer* map_viewer_;

    // Plane slam
    string map_frame_;
    string base_frame_;
    string odom_frame_;
    bool display_path_;
    bool display_odom_path_;
    bool display_landmarks_;
    bool display_landmark_inlier_;
    bool display_landmark_arrow_;
    bool display_landmark_boundary_;
    bool display_landmark_hull_;

    // Plane segment
    int cloud_size_type_;
    int cloud_size_type_config_;
    int plane_segment_method_;

    bool display_input_cloud_;
    bool display_line_cloud_;
    bool display_normal_;
    bool display_normal_arrow_;
    bool display_plane_;
    bool display_plane_arrow_;
    bool display_plane_inlier_;
    bool display_plane_projected_inlier_;
    bool display_plane_boundary_;
    bool display_plane_hull_;
    bool loop_one_message_;

    // Organized Muit Plane segment parameters
    OrganizedPlaneSegment organized_plane_segment_;

    //
    // Plane segment based line segment
    PlaneFromLineSegment::CAMERA_PARAMETERS camera_parameters_;
    PlaneFromLineSegment plane_from_line_segment_;
    bool is_update_line_based_parameters_;
    // LineBased segment
    bool use_horizontal_line_;
    bool use_verticle_line_;
    unsigned y_skip_;
    unsigned x_skip_;
    float line_point_min_distance_;
    bool use_depth_noise_model_;
    float scan_rho_constant_error_;
    float scan_rho_distance_error_;
    float scan_rho_quadratic_error_;
    unsigned slide_window_size_;
    unsigned line_min_inliers_;
    float line_fitting_threshold_;
    int normals_per_line_;
    bool normal_use_depth_dependent_smoothing_;
    float normal_max_depth_change_factor_;
    unsigned normal_smoothing_size_;
    float normal_min_inliers_percentage_;
    float normal_maximum_curvature_;
    //
    bool remove_duplicate_candidate_;
    float duplicate_candidate_normal_thresh_;
    float duplicate_candidate_distance_thresh_;
    //
    int plane_segment_criterion_;
    float k_curvature_;
    float k_inlier_;
    unsigned min_inliers_;
    float distance_threshold_;
    float neighbor_threshold_;
    bool optimize_coefficients_;
    bool project_points_;
    bool extract_boundary_;

    //
    bool is_initialized;
    PlaneSlam* plane_slam_;
    std::vector<geometry_msgs::PoseStamped> true_poses_;
};

#endif // KINECT_LISTENER_H
