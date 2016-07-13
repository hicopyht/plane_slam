#ifndef G2O_MAPPING_H
#define G2O_MAPPING_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <dynamic_reconfigure/server.h>
#include <plane_slam/G2OMappingConfig.h>
#include <std_srvs/Trigger.h>
//
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/linear_solver.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"
#include "g2o/stuff/macros.h"
#include "g2o/stuff/command_args.h"
//
//
#include "utils.h"
#include "frame.h"
#include "viewer.h"

namespace plane_slam
{

using namespace g2o;

class G2OMapping
{
public:
    G2OMapping( ros::NodeHandle &nh, Viewer* viewer );

    bool mapping( const Frame &frame );

    void optimizeGraph( int n = 10 );

    void publishOptimizedPose();

    void publishOptimizedPath();

    void publishMapCloud();

    void updateMapViewer();

//    // Get optimized pose
//    const gtsam::Pose3 &getOptimizedPose() { return last_estimated_pose_; }
//    // Get optimized path
//    const std::vector<gtsam::Pose3> &getOptimizedPath() { return estimated_poses_; }
//    // Get landmarks
//    const std::vector<PlaneType> &getLandmark() { return landmarks_; }
//    // Set map frame
//    inline void setMapFrame( const std::string &frame_id ) { map_frame_ = frame_id; }
//    // Query map frame
//    std::string getMapFrame() const { return map_frame_; }

protected:
    bool addFirstFrame( const Frame &frame );

    bool doMapping( const Frame &frame );

    bool isKeyFrame( const Frame &frame );

    void g2oMappingReconfigCallback( plane_slam::G2OMappingConfig &config, uint32_t level);

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
    dynamic_reconfigure::Server<plane_slam::G2OMappingConfig> mapping_config_server_;
    dynamic_reconfigure::Server<plane_slam::G2OMappingConfig>::CallbackType mapping_config_callback_;
    //
    ros::Publisher optimized_pose_publisher_;
    ros::Publisher optimized_path_publisher_;
    ros::Publisher map_cloud_publisher_;
    ros::Publisher marker_publisher_;

    tf::Transform last_estimated_pose_tf_;


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
    double planar_bad_inlier_alpha_;
    //
    bool publish_optimized_path_;
};

} // end of namespace plane_slam

#endif // G2O_MAPPING_H
