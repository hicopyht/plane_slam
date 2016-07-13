#ifndef GTSAM_MAPPING_H
#define GTSAM_MAPPING_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <dynamic_reconfigure/server.h>
#include <plane_slam/GTMappingConfig.h>
#include <std_srvs/Trigger.h>
//
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/OrientedPlane3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/OrientedPlane3Factor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>

//
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_pointcloud_occupancy.h>
//
#include "utils.h"
#include "frame.h"
#include "viewer.h"

using namespace std;
using namespace gtsam;

namespace plane_slam
{

class GTMapping
{
public:
    GTMapping( ros::NodeHandle &nh, Viewer* viewer );

    bool mapping( const Frame &frame );

    void optimizeGraph( int n = 10 );

    void publishOptimizedPose();

    void publishOptimizedPath();

    void publishMapCloud();

    void updateMapViewer();

    // Save map to PCD file
    void saveMapPCD( const std::string &filename = "plane_slam_map.pcd");
    // Save graph
    inline void saveGraphDot( const std::string &filename = "plane_slam_graph.dot" ){
        isam2_->saveGraph( filename );
    }
    // Get optimized pose
    const tf::Transform &getOptimizedPoseTF() { return last_estimated_pose_tf_; }
    // Get optimized path
    std::vector<geometry_msgs::PoseStamped> getOptimizedPath();
    // Get landmarks
    const std::vector<PlaneType> &getLandmark() { return landmarks_; }
    // Set map frame
    inline void setMapFrame( const std::string &frame_id ) { map_frame_ = frame_id; }
    std::string getMapFrame() const { return map_frame_; }

protected:
    bool addFirstFrame( const Frame &frame );

    bool doMapping( const Frame &frame );

    bool isKeyFrame( const Frame &frame );

    std::vector<OrientedPlane3> getPredictedObservation( const std::vector<OrientedPlane3> &landmarks,
                                                         const Pose3 &pose );

    void matchPlanes( const std::vector<OrientedPlane3> &predicted_observations,
                      const std::vector<PlaneType> &landmarks,
                      const std::vector<OrientedPlane3> &observations,
                      const std::vector<PlaneType> &observed_planes,
                      const Pose3 pose,
                      std::vector<PlanePair> &pairs);

    bool checkOverlap( const PointCloudTypePtr &landmark_cloud,
                       const OrientedPlane3 &landmark,
                       const PointCloudTypePtr &observation,
                       const Pose3 &pose);

    bool checkLandmarksOverlap( const PlaneType &lm1, const PlaneType &lm2);

    void mergeLandmarkInlier( PlaneType &from, PlaneType &to);

    bool refinePlanarMap();

    bool removeBadInlier();

    void updateSlamResult( std::vector<Pose3> &poses, std::vector<OrientedPlane3> &planes );

    void updateLandmarks( std::vector<PlaneType> &landmarks,
                          const std::vector<PlaneType> &observations,
                          const std::vector<PlanePair> &pairs,
                          const Pose3 &estimated_pose,
                          const std::vector<OrientedPlane3> &estimated_planes);

    void voxelGridFilter(  const PointCloudTypePtr &cloud,
                           PointCloudTypePtr &cloud_filtered,
                           float leaf_size = 0.02f);

    void gtMappingReconfigCallback( plane_slam::GTMappingConfig &config, uint32_t level);

    bool optimizeGraphCallback( std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res );

    bool saveGraphCallback( std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res );

    bool saveMapPCDCallback( std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res );

    bool removeBadInlierCallback( std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res );

private:
    //
    Viewer *viewer_;
    //
    ros::NodeHandle nh_;
    ros::ServiceServer optimize_graph_service_server_;
    ros::ServiceServer save_graph_service_server_;
    ros::ServiceServer save_map_service_server_;
    ros::ServiceServer remove_bad_inlier_service_server_;
    dynamic_reconfigure::Server<plane_slam::GTMappingConfig> mapping_config_server_;
    dynamic_reconfigure::Server<plane_slam::GTMappingConfig>::CallbackType mapping_config_callback_;
    //
    ros::Publisher optimized_pose_publisher_;
    ros::Publisher optimized_path_publisher_;
    ros::Publisher map_cloud_publisher_;
    ros::Publisher marker_publisher_;

    // ISAM2
    ISAM2Params isam2_parameters_;
    ISAM2* isam2_;
    // Create a Factor Graph and Values to hold the new data
    NonlinearFactorGraph graph_; // factor graph
    Values initial_estimate_; // initial guess

    //
    std::string map_frame_;
    Pose3 last_estimated_pose_;
    tf::Transform last_estimated_pose_tf_;
    std::vector<Pose3> estimated_poses_;
    std::vector<OrientedPlane3> estimated_planes_;
    std::vector<PlaneType> landmarks_;
    int pose_count_;
    int landmark_max_count_;
    cv::RNG rng_;

    // Parameters
    bool use_keyframe_;
    double keyframe_linear_threshold_;
    double keyframe_angular_threshold_;
    // ISMA2
    double isam2_relinearize_threshold_;
    int isam2_relinearize_skip_;
    //
    double plane_match_direction_threshold_;
    double plane_match_distance_threshold_;
    bool plane_match_check_overlap_;
    double plane_match_overlap_alpha_;
    double plane_inlier_leaf_size_;
    double plane_hull_alpha_;
    //
    bool refine_planar_map_;
    double planar_merge_direction_threshold_;
    double planar_merge_distance_threshold_;
    double planar_bad_inlier_alpha_;
    //
    bool publish_optimized_path_;
};

} // end of namespace plane_slam

#endif // MAPPING_H
