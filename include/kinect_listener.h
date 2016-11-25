#ifndef KINECT_LISTENER_H
#define KINECT_LISTENER_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/timer.h>
#include <dynamic_reconfigure/server.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <nav_msgs/Path.h>
#include <std_srvs/Trigger.h>
//
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/time.h>
#include <pcl/common/time.h>
//
#include <plane_slam/PlaneSlamConfig.h>
//#include <plane_slam/PlaneSegmentConfig.h>
#include "utils.h"
#include "frame.h"
#include "viewer.h"
#include "tracking.h"
#include "gtsam_mapping.h"
#include "feature_adjuster.h"
//
#include <yaml-cpp/yaml.h>

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
namespace plane_slam
{

class KinectListener
{

public:
    enum{ LineBased = 0, OMPS = 1}; // define segment method

public:
    KinectListener();

    ~KinectListener();

    void trackPointCloud( const sensor_msgs::PointCloud2ConstPtr &point_cloud,
                          CameraParameters& camera,
                          tf::Transform &odom_pose );

    void trackDepthRgbImage( const sensor_msgs::ImageConstPtr &visual_img_msg,
                             const sensor_msgs::ImageConstPtr &depth_img_msg,
                             CameraParameters &camera,
                             tf::Transform &odom_pose);

    void trackDepthRgbImage( const sensor_msgs::ImageConstPtr &visual_img_msg,
                             const sensor_msgs::ImageConstPtr &depth_img_msg,
                             CameraParameters &camera);

    Frame* pointCloudToFrame(const sensor_msgs::PointCloud2ConstPtr &point_cloud,
                             CameraParameters & camera);

    Frame* depthRgbToFrame(const sensor_msgs::ImageConstPtr &visual_img_msg,
                           const sensor_msgs::ImageConstPtr &depth_img_msg,
                           CameraParameters & camera);

    bool isBigOdomChange( tf::Transform rel_odom, double linear_threshold, double angular_threshold );

    void debugFrame( Frame* frame );

    bool trackFrameMotionOdom( Frame* last_frame, Frame* frame );

    bool trackFrameMotion( Frame* last_frame, Frame *frame );

    void recordVisualOdometry( Frame *last_frame, Frame *frame );

    void calculateOdomToMapTF( tf::Transform &map_tf, tf::Transform &odom_tf );

    void displayMappingResult( Frame* frame );

    void storeKeyFrame( Frame* &last_frame, Frame* &frame );

    void savePlaneLandmarks( const std::string &filename = "plane_slam_plane_landmarks.txt" );

    void saveKeypointLandmarks( const std::string &filename = "plane_slam_keypoint_landmarks.txt" );

    void savePathToFile( const std::vector<geometry_msgs::PoseStamped> &poses,
                         const std::string &filename = "plane_slam_pathes.txt");

    void saveRuntimes( const std::string &filename = "runtimes.txt" );

    void cvtCameraParameter( const sensor_msgs::CameraInfoConstPtr &cam_info_msg,
                             CameraParameters &camera);

    inline void setCameraParameters( const CameraParameters & camera ) { camera_parameters_ = camera; }

    inline void setInitPose( tf::Transform init_tf ) { init_pose_ = init_tf; set_init_pose_ = true; }

protected:
    void noCloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                         const sensor_msgs::ImageConstPtr& depth_img_msg,
                         const sensor_msgs::CameraInfoConstPtr& cam_info_msg);

    void cloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                        const sensor_msgs::PointCloud2ConstPtr& point_cloud,
                        const sensor_msgs::CameraInfoConstPtr& cam_info_msg);

    void planeSlamReconfigCallback( plane_slam::PlaneSlamConfig &config, uint32_t level);

    bool updateViewerOnceCallback( std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res );

    bool saveSlamResultSimpleCallback( std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res );

    bool saveSlamResultCallback( std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res );

    void publishTfTimerCallback( const ros::TimerEvent& event );

private:
    bool getOdomPose( tf::Transform &odom_pose, const std::string &camera_frame, const ros::Time &t = ros::Time(0) );

    void publishOdomPose();

    void publishOdomPath();

    void publishVisualOdometryPose();

    void publishVisualOdometryPath();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Timer tf_timer_;
    ros::CallbackQueue my_callback_queue_;
    ros::AsyncSpinner* async_spinner_;
    //
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;
    //
    dynamic_reconfigure::Server<plane_slam::PlaneSlamConfig> plane_slam_config_server_;
    dynamic_reconfigure::Server<plane_slam::PlaneSlamConfig>::CallbackType plane_slam_config_callback_;
    // subscribers
    bool verbose_;
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
    ros::Publisher odom_pose_publisher_;
    ros::Publisher odom_path_publisher_;
    ros::Publisher visual_odometry_pose_publisher_;
    ros::Publisher visual_odometry_path_publisher_;
    ros::ServiceServer update_viewer_once_ss_;
    ros::ServiceServer save_slam_result_simple_ss_;
    ros::ServiceServer save_slam_result_all_ss_;

    //
    std::string keypoint_type_;
    cv::FeatureDetector* surf_detector_;
    cv::DescriptorExtractor* surf_extractor_;
    ORBextractor* orb_extractor_;
    LineBasedPlaneSegmentor* line_based_plane_segmentor_;
    OrganizedPlaneSegmentor* organized_plane_segmentor_;
    Viewer *viewer_;
    Tracking *tracker_;
    GTMapping *gt_mapping_;

    // Plane slam common parameters
    int plane_segment_method_;
    bool do_visual_odometry_;
    bool do_mapping_;
    bool do_slam_;
    bool force_odom_;
    bool mapping_key_message_;
    bool use_odom_tracking_;
    bool mapping_keypoint_;
    string camera_frame_;
    string map_frame_;
    string base_frame_;
    string odom_frame_;
    bool set_init_pose_;
    int skip_message_;
    //
    bool save_map_full_pcd_;
    bool save_map_full_colored_pcd_;
    bool save_octomap_;
    bool save_structure_pcd_;
    bool save_input_cloud_pcd_;
    int save_message_pcd_;

    //
    tf::Transform init_pose_;
    // Camera Parameters
    CameraParameters camera_parameters_;
    // Path
    std::vector<geometry_msgs::PoseStamped> odom_poses_;
    std::vector<geometry_msgs::PoseStamped> visual_odometry_poses_;

    // Runtimes and frame count
    int frame_count_;   // full frame count
    std::vector<Runtime> runtimes_;

    //
    bool publish_map_tf_;
    double map_tf_freq_;
    boost::mutex map_tf_mutex_;
    tf::Transform odom_to_map_tf_;
};

} // end of namespace plane_slam
#endif // KINECT_LISTENER_H
