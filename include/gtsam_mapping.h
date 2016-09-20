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
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
//
#include "utils.h"
#include "frame.h"
#include "viewer.h"
#include "tracking.h"

using namespace std;
using namespace gtsam;

namespace plane_slam
{

class GTMapping
{
public:
    GTMapping( ros::NodeHandle &nh, Viewer* viewer, Tracking* tracker = NULL);

    bool mappingMix( Frame *frame );

    bool mapping( Frame *frame );

    void optimizeGraph( int n = 10 );

    void publishOptimizedPose();

    void publishOptimizedPath();

    void publishMapCloud();

    void publishOctoMap();

    void updateMapViewer();

    void reset();

    octomap::OcTree * createOctoMap( double resolution = 0);

    // Save map to PCD file
    void saveMapPCD( const std::string &filename = "plane_slam_map.pcd");
    void saveMapFullPCD( const std::string &filename = "plane_slam_map_full.pcd");
    void saveMapFullColoredPCD( const std::string &filename = "plane_slam_map_full_colored.pcd");
    void saveStructurePCD( const std::string &filename = "plane_slam_structure.pcd" );
    // Save graph
    inline void saveGraphDot( const std::string &filename = "plane_slam_graph.dot" ){
        isam2_->saveGraph( filename );
    }
    // Get optimized pose
    const tf::Transform &getOptimizedPoseTF() { return last_estimated_pose_tf_; }
    // Get optimized path
    std::vector<geometry_msgs::PoseStamped> getOptimizedPath();
    // Get landmarks
    const std::map<int, PlaneType*> &getLandmark() { return landmarks_list_; }
    // Get map cloud
    PointCloudTypePtr getMapCloud( bool force = false);
    PointCloudTypePtr getMapFullCloud( bool colored = false);
    PointCloudTypePtr getStructureCloud();
    // Set map frame
    inline void setMapFrame( const std::string &frame_id ) { map_frame_ = frame_id; }
    std::string getMapFrame() const { return map_frame_; }

protected:
    bool addFirstFrameMix( Frame *frame );

    bool doMappingMix( Frame *frame );

    bool addFirstFrame( Frame *frame );

    bool doMapping( Frame *frame );

    bool isKeyFrame( Frame *frame );

    void semanticMapLabel();

    void labelPlane( PlaneType *plane );

    std::map<int, gtsam::OrientedPlane3> getPredictedObservation( const Pose3 &pose );

    void matchObservationWithPredicted( std::map<int, OrientedPlane3> &predicted_observations,
                                        const std::vector<OrientedPlane3> &observations,
                                        const std::vector<PlaneType> &observed_planes,
                                        const Pose3 pose,
                                        std::vector<PlanePair> &pairs);

    bool checkOverlap( const PointCloudTypePtr &landmark_cloud,
                       const OrientedPlane3 &landmark,
                       const PointCloudTypePtr &observation,
                       const Pose3 &pose);

    bool checkLandmarksOverlap( const PlaneType &lm1, const PlaneType &lm2);

    bool refinePlanarMap();

    void mergeCoplanarLandmarks( std::map<int, std::set<int> > merge_list );

    bool removeLandmarksBadInlier();

    void removePlaneBadInlier( PointCloudTypePtr &cloud_voxel, double radius = 0, int min_neighbors = 0 );

    void updateOptimizedResult();

    void updateLandmarksInlier();

    void updateOctoMap();

    void gtMappingReconfigCallback( plane_slam::GTMappingConfig &config, uint32_t level);

    bool optimizeGraphCallback( std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res );

    bool saveGraphCallback( std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res );

    bool saveMapPCDCallback( std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res );

    bool saveMapFullPCDCallback( std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res );

    bool removeBadInlierCallback( std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res );

private:
    //
    Viewer *viewer_;
    Tracking *tracker_;
    //
    ros::NodeHandle nh_;
    ros::ServiceServer optimize_graph_service_server_;
    ros::ServiceServer save_graph_service_server_;
    ros::ServiceServer save_map_service_server_;
    ros::ServiceServer save_map_full_service_server_;
    ros::ServiceServer remove_bad_inlier_service_server_;
    dynamic_reconfigure::Server<plane_slam::GTMappingConfig> mapping_config_server_;
    dynamic_reconfigure::Server<plane_slam::GTMappingConfig>::CallbackType mapping_config_callback_;
    //
    ros::Publisher optimized_pose_publisher_;
    ros::Publisher optimized_path_publisher_;
    ros::Publisher map_cloud_publisher_;
    ros::Publisher octomap_publisher_;
    ros::Publisher marker_publisher_;

    // ISAM2
    ISAM2Params isam2_parameters_;
    ISAM2* isam2_;
    // Create a Factor Graph and Values to hold the new data
    NonlinearFactorGraph factor_graph_; // factor graph
    Values initial_estimate_; // initial guess
    // Buffer
    NonlinearFactorGraph factor_graph_buffer_; // factor graph
    Values initial_estimate_buffer_; // initial guess

    //
    int next_plane_id_; // set identical id to plane
    int next_frame_id_; // set identical id to frame
    std::map<int, Frame*> frames_list_;      // frames list
    std::map<int, PlaneType*> landmarks_list_;  // landmarks list
    std::map<int, gtsam::Pose3> optimized_poses_list_;  // optimized pose list
    std::map<int, gtsam::OrientedPlane3> optimized_landmarks_list_;    // optimized landmarks list
    PointCloudTypePtr map_cloud_;
//    std::map<int, gtsam::OrientedPlane3> optimized_landmarks_list_last_; // last optimized
    octomap::OcTree *octree_map_;
    //
    std::set<int> landmark_ids_;  // ids of landmarks
    std::set<int> key_frame_ids_;  // ids of key frames
    std::map<int, std::set<int> > landmarks_related_frames_list_;// <lm, set<frames>>
    //


    //
    std::string map_frame_;
    Pose3 last_estimated_pose_;
    tf::Transform last_estimated_pose_tf_;
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
    bool remove_plane_bad_inlier_;
    double planar_bad_inlier_alpha_;
    //
    double map_full_leaf_size_;
    bool map_full_remove_bad_inlier_;
    int map_full_min_neighbor_;
    double map_full_search_radius_;
    double map_full_min_neighbor_alpha_;
    double construct_full_leaf_size_;
    double octomap_resolution_;
    double octomap_max_depth_range_;
    bool publish_map_cloud_;
    bool publish_octomap_;
    bool publish_optimized_path_;
};

} // end of namespace plane_slam

#endif // MAPPING_H
