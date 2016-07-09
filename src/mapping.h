#ifndef MAPPING_H
#define MAPPING_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <dynamic_reconfigure/server.h>
#include <plane_slam/MappingConfig.h>
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
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_pointcloud_occupancy.h>
//
#include "utils.h"
#include "frame.h"

using namespace std;
using namespace gtsam;

namespace plane_slam
{

class Mapping
{
public:
    Mapping( ros::NodeHandle &nh );

    bool mapping( const Frame &frame );

    void publishOptimizedPose();

    void publishOptimizedPath();

    void publishMapCloud();

    // Get optimized pose
    const gtsam::Pose3 &getOptimizedPose() { return last_estimated_pose_; }
    // Get landmarks
    const std::vector<PlaneType> &getLandmark() { return landmarks_; }
    // Set map frame
    inline void setMapFrame( const std::string &frame_id ) { map_frame_ = frame_id; }
    // Query map frame
    std::string getMapFrame() const { return map_frame_; }

protected:
    bool addFirstFrame( const Frame &frame );

    bool doMapping( const Frame &frame );

    void voxelGridFilter(  const PointCloudTypePtr &cloud,
                           PointCloudTypePtr &cloud_filtered,
                           float leaf_size = 0.02f);

    void mappingReconfigCallback( plane_slam::MappingConfig &config, uint32_t level);

private:
    //
    ros::NodeHandle nh_;
    dynamic_reconfigure::Server<plane_slam::MappingConfig> mapping_config_server_;
    dynamic_reconfigure::Server<plane_slam::MappingConfig>::CallbackType mapping_config_callback_;
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
    //
    bool publish_optimized_path_;
};

} // end of namespace plane_slam

#endif // MAPPING_H
