#ifndef KINECT_LISTENER_H
#define KINECT_LISTENER_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
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
#include <tf_conversions/tf_eigen.h>
#include <std_srvs/SetBool.h>
//
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
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
#include "itree.h"
#include "plane_slam.h"
#include "ORBextractor.h"

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

typedef message_filters::TimeSynchronizer<sensor_msgs::Image,
                                          sensor_msgs::CameraInfo> DepthSynchronizer;

class KinectListener
{
    enum { VGA = 0, QVGA = 1, QQVGA = 2};
    enum { LINE_BADED = 0, ORGANSIZED = 1};
    enum { SURF = 0, ORB = 1, SIFT = 2};
public:
    KinectListener();

    ~KinectListener();

    void trackDepthImage( const sensor_msgs::ImageConstPtr &depth_img_msg,
                          PlaneFromLineSegment::CAMERA_PARAMETERS &camera_parameters );

    void processCloud( KinectFrame &frame, const tf::Transform &odom_pose = tf::Transform::getIdentity() );

    void processFrame( KinectFrame &frame, const tf::Transform &odom_pose = tf::Transform::getIdentity() );

    void lineBasedPlaneSegment( PointCloudTypePtr &input,
                                std::vector<PlaneType> &planes);

    void organizedPlaneSegment( PointCloudTypePtr &input, std::vector<PlaneType> &planes);

//    bool solveRelativeTransform( KinectFrame &last_frame,
//                                 KinectFrame &current_frame,
//                                 RESULT_OF_MOTION &result,
//                                 Eigen::Matrix4d estimated_transform = Eigen::MatrixXd::Identity(4,4) );

    bool solveRtIcp( const PointCloudXYZPtr &source,
                     const PointCloudXYZPtr &target,
                     PointCloudXYZPtr &cloud_icp,
                     RESULT_OF_MOTION &result );

    Eigen::Matrix4f solveRtPcl( const std_vector_of_eigen_vector4f &query_points,
                                const std_vector_of_eigen_vector4f &train_points,
                                const std::vector<cv::DMatch> &matches,
                                bool &valid);

    void solveRt( const std::vector<PlaneCoefficients> &last_planes,
                  const std::vector<PlaneCoefficients> &planes,
                  const std::vector<Eigen::Vector3d>& from_points,
                  const std::vector<Eigen::Vector3d>& to_points,
                  RESULT_OF_MOTION &result);

    void solveRt( const std::vector<Eigen::Vector3d>& from_points,
                  const std::vector<Eigen::Vector3d>& to_points,
                  RESULT_OF_MOTION &result);

    void solveRt( const std::vector<PlaneCoefficients> &last_planes,
                  const std::vector<PlaneCoefficients> &planes,
                  RESULT_OF_MOTION &result);

    bool solveRtPlanes( const std::vector<PlaneCoefficients> &before,
                            const std::vector<PlaneCoefficients> &after,
                            RESULT_OF_MOTION &result);

    Eigen::Matrix4f solveRtPlanesPoints( std::vector<PlaneType> &last_planes,
                                         std::vector<PlaneType> &planes,
                                         std::vector<PlanePair> &pairs,
                                         std_vector_of_eigen_vector4f &last_feature_3d,
                                         std_vector_of_eigen_vector4f &feature_3d,
                                         std::vector<cv::DMatch> &matches,
                                         bool &valid );

    bool solveRelativeTransformPlanes( KinectFrame &last_frame,
                                       KinectFrame &current_frame,
                                       const std::vector<PlanePair> &pairs,
                                       RESULT_OF_MOTION &result);

    bool solveRelativeTransformPlanesPointsRansac( KinectFrame &last_frame,
                                                   KinectFrame &current_frame,
                                                   std::vector<PlanePair> &pairs,
                                                   std::vector<cv::DMatch> &good_matches,
                                                   RESULT_OF_MOTION &result,
                                                   std::vector<cv::DMatch> &matches );

    bool solveRelativeTransformPointsRansac( KinectFrame &last_frame,
                                             KinectFrame &frame,
                                             std::vector<cv::DMatch> &good_matches,
                                             RESULT_OF_MOTION &result,
                                             std::vector<cv::DMatch> &matches );

    bool solveRelativeTransformIcp( KinectFrame &last_frame,
                                    KinectFrame &current_frame,
                                    RESULT_OF_MOTION &result);

    bool solveRelativeTransformPnP( KinectFrame& last_frame,
                                    KinectFrame& current_frame,
                                    std::vector<cv::DMatch> &good_matches,
                                    PlaneFromLineSegment::CAMERA_PARAMETERS& camera,
                                    RESULT_OF_MOTION &result );

    bool solveRelativeTransform( KinectFrame &last_frame,
                                 KinectFrame &current_frame,
                                 RESULT_OF_MOTION &result,
                                 std::vector<cv::DMatch> &matches,
                                 Eigen::Matrix4d estimated_transform = Eigen::MatrixXd::Identity(4,4));



    void computeCorrespondenceInliersAndError( const std::vector<cv::DMatch> & matches,
                                               const Eigen::Matrix4f& transform4f,
                                               const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& query_points,
                                               const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& train_points,
                                               unsigned int min_inliers,
                                               std::vector<cv::DMatch>& inlier, //pure output var
                                               double& return_mean_error,//pure output var: rms-mahalanobis-distance
                                               double squared_max_distance) const;

    double computeEuclidianDistance( const std::vector<PlaneType>& last_planes,
                                     const std::vector<PlaneType>& planes,
                                     const std::vector<PlanePair>& pairs,
                                     RESULT_OF_MOTION &relative );

    void computeORBKeypoint( const cv::Mat &visual,
                             const PointCloudTypePtr &cloud_in,
                             std::vector<cv::KeyPoint> &keypoints,
                             std_vector_of_eigen_vector4f &locations_3d,
                             cv::Mat &feature_descriptors );

    void computeORBKeypoint( const cv::Mat &visual,
                             const PointCloudTypePtr &cloud_in,
                             std::vector<cv::KeyPoint> &keypoints,
                             std_vector_of_eigen_vector4f &locations_3d,
                             PointCloudXYZPtr &feature_cloud,
                             cv::Mat &feature_descriptors );

    void computeKeypoint( const cv::Mat &visual,
                          const PointCloudTypePtr &cloud_in,
                          std::vector<cv::KeyPoint> &keypoints,
                          std_vector_of_eigen_vector4f &locations_3d,
                          cv::Mat &feature_descriptors,
                          const cv::Mat& mask=cv::Mat());

    void computeKeypoint( const cv::Mat &visual,
                          const PointCloudTypePtr &cloud_in,
                          std::vector<cv::KeyPoint> &keypoints,
                          std_vector_of_eigen_vector4f &locations_3d,
                          PointCloudXYZPtr &feature_cloud,
                          cv::Mat &feature_descriptors,
                          const cv::Mat& mask );

    void projectTo3D( const PointCloudTypePtr &cloud,
                      std::vector<cv::KeyPoint> &locations_2d,
                      cv::Mat &feature_descriptors,
                      std_vector_of_eigen_vector4f &locations_3d );

    void projectTo3D( const PointCloudTypePtr &cloud,
                      std::vector<cv::KeyPoint> &locations_2d,
                      cv::Mat &feature_descriptors,
                      std_vector_of_eigen_vector4f &locations_3d,
                      PointCloudXYZPtr &feature_cloud );

    void projectTo3D(const PointCloudTypePtr &cloud,
                      std::vector<cv::KeyPoint> &locations_2d,
                      std_vector_of_eigen_vector4f &locations_3d);

    void projectTo3D(const PointCloudTypePtr &cloud,
                      std::vector<cv::KeyPoint> &locations_2d,
                      std_vector_of_eigen_vector4f &locations_3d,
                     PointCloudXYZPtr &feature_cloud);

    void matchImageFeatures( KinectFrame& last_frame,
                             KinectFrame& current_frame,
                             vector< cv::DMatch > &goodMatches,
                             double good_match_threshold = 4.0,
                             int min_match_size = 0);

    std::vector<PlanePair> randomChoosePlanePairsPreferGood( const unsigned int sample_size,
                                             std::vector<PlanePair> &pairs );

    std::vector<PlanePair> randomChoosePlanePairs( const unsigned int sample_size,
                                             std::vector<PlanePair> &pairs );

    std::vector<cv::DMatch> randomChooseMatchesPreferGood( const unsigned int sample_size,
                                             vector< cv::DMatch > &matches_with_depth );

    std::vector<cv::DMatch> randomChooseMatches( const unsigned int sample_size,
                                             vector< cv::DMatch > &matches );

protected:
    void noCloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                         const sensor_msgs::ImageConstPtr& depth_img_msg,
                         const sensor_msgs::CameraInfoConstPtr& cam_info_msg);

    void cloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                        const sensor_msgs::PointCloud2ConstPtr& point_cloud,
                        const sensor_msgs::CameraInfoConstPtr& cam_info_msg);

    void depthCallback (const sensor_msgs::ImageConstPtr& depth_img_msg,
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

    void downsampleOrganizedCloud(const PointCloudTypePtr &input, PlaneFromLineSegment::CAMERA_PARAMETERS &in_camera,
                                  PointCloudTypePtr &output, PlaneFromLineSegment::CAMERA_PARAMETERS &out_camera, int size_type);

    void downsampleImage(const cv::Mat &input, cv::Mat &output, int size_type);

    void getCameraParameter(const sensor_msgs::CameraInfoConstPtr &cam_info_msg,
                            PlaneFromLineSegment::CAMERA_PARAMETERS &camera);

    void publishPose( gtsam::Pose3 &pose);

    void publishPlanarMap( const std::vector<PlaneType> &landmarks);

    void displayMatched3DKeypoint( std_vector_of_eigen_vector4f &query,
                                   std_vector_of_eigen_vector4f &train,
                                   std::vector<cv::DMatch> &matches,
                                   const std::string &id = "matched_features");

    void display3DKeypoint( std_vector_of_eigen_vector4f &feature_location_3d, const std::string &id, int viewport );

    void displayKeypoint( const cv::Mat &visual, std::vector<cv::KeyPoint> &keypoints );

    void displayLandmarks( const std::vector<PlaneType> &landmarks, const std::string &prefix = "landmark");

    void displayPlanes( const PointCloudTypePtr &input, std::vector<PlaneType> &planes, const std::string &prefix, int viewport);

    void displayLinesAndNormals( const PointCloudTypePtr &input,
                                 std::vector<PlaneFromLineSegment::LineType> &lines,
                                 std::vector<PlaneFromLineSegment::NormalType> &normals,
                                 int viewport);

    void pclViewerLandmark( const PlaneType &plane, const std::string &id, const int number = -1);

    void pclViewerLineRegion( const PointCloudTypePtr &input, PlaneFromLineSegment::LineType &line, const std::string &id, int viewpoint);

    void pclViewerNormal( const PointCloudTypePtr &input, PlaneFromLineSegment::NormalType &normal, const std::string &id, int viewpoint);

    void pclViewerPlane( const PointCloudTypePtr &input, PlaneType &plane, const std::string &id, int viewport, int number = -1);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr image2PointCloud( const cv::Mat &rgb_img, const cv::Mat &depth_img,
                                                            const PlaneFromLineSegment::CAMERA_PARAMETERS& camera );

    bool getOdomPose( tf::Transform &odom_pose, const std::string &camera_frame, const ros::Time &time = ros::Time(0) );

    void publishTruePath();

    inline bool isValidPoint(const PointType &p)
    {
        return (prttcp_->isValid(p) && p.z > 0);
    }

    bool autoSpinMapViewerCallback( std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

    cv::FeatureDetector* createDetector( const std::string& detectorType );

    cv::DescriptorExtractor* createDescriptorExtractor( const string& descriptorType );

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
    DepthSynchronizer *depth_sync_;
    //
    ros::Publisher true_path_publisher_;
    ros::Publisher pose_publisher_;
    ros::Publisher planar_map_publisher_;
    ros::ServiceServer auto_spin_map_viewer_ss_;
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
    bool auto_spin_map_viewer_;

    //Variables
    cv::Ptr<cv::FeatureDetector> detector_;
    cv::Ptr<cv::DescriptorExtractor> extractor_;
    //
    ORBextractor* orb_extractor_;

    // Plane slam
    bool do_slam_;
    string map_frame_;
    string base_frame_;
    string odom_frame_;
    bool display_path_;
    bool display_odom_path_;
    bool display_landmarks_;
    bool display_landmark_inlier_;
    bool display_landmark_arrow_;
    bool display_landmark_number_;
    bool display_landmark_boundary_;
    bool display_landmark_hull_;

    // Plane segment
    int cloud_size_type_;
    int cloud_size_type_config_;
    int plane_segment_method_;
    std::string feature_detector_type_;
    std::string feature_extractor_type_;
    double feature_good_match_threshold_;
    int feature_min_good_match_size_;
    int ransac_sample_size_;
    int ransac_iterations_;
    int ransac_min_inlier_;
    double ransac_inlier_max_mahal_distance_;
    //
    int pnp_iterations_;
    int pnp_min_inlier_;
    double pnp_repreject_error_;

    bool display_input_cloud_;
    bool display_line_cloud_;
    bool display_normal_;
    bool display_normal_arrow_;
    bool display_plane_;
    bool display_plane_number_;
    bool display_plane_arrow_;
    bool display_plane_inlier_;
    bool display_plane_projected_inlier_;
    bool display_plane_boundary_;
    bool display_plane_hull_;
    bool loop_one_message_;

    // ICP parameters
    double icp_max_distance_;
    int icp_iterations_;
    double icp_tf_epsilon_;
    int icp_min_indices_;
    double icp_score_threshold_;

    // Organized Muit Plane segment parameters
    OrganizedPlaneSegment organized_plane_segment_;

    //
    // Plane segment based line segment
    PlaneFromLineSegment::CAMERA_PARAMETERS camera_parameters_;
    PlaneFromLineSegment::CAMERA_PARAMETERS real_camera_parameters_;
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
