#ifndef PLANE_SLAM_H
#define PLANE_SLAM_H
// texture 11431

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
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

#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
//
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_pointcloud_occupancy.h>
//
#include "utils.h"

using namespace std;
using namespace gtsam;

///*
// * \brief Plane parameters
//  N*P + d = 0
//  */
//struct PlaneType
//{
//    PointType centroid;
//    Eigen::Vector4d coefficients;
//    Eigen::Vector3d sigmas;
//    std::vector<int> inlier;
//    PointCloudType cloud;
//};

//struct PlanePair
//{
//    int iobs;
//    int ilm;

//    PlanePair() : iobs(-1), ilm(-1) {}
//    PlanePair(int _iobs, int _ilm) : iobs(_iobs), ilm(_ilm) {}
//};



class PlaneSlam
{
public:
    PlaneSlam();

    void initialize(Pose3 &init_pose, std::vector<PlaneType> &planes);

    Pose3 planeSlam(Pose3 &rel_pose, std::vector<PlaneType> &planes);

    void matchPlanes( const std::vector<OrientedPlane3> &predicted_observations,
                      const std::vector<PlaneType> &landmarks,
                      const std::vector<OrientedPlane3> &observations,
                      const std::vector<PlaneType> &observed_planes,
                      const Pose3 pose,
                      std::vector<PlanePair> &pairs);

    void getPredictedObservation( Pose3 &pose, std::vector<OrientedPlane3> &predicted_observations );

    void predictObservation( std::vector<OrientedPlane3> &landmarks, Pose3 &pose,
                             std::vector<OrientedPlane3> &predicted_observations);

    void updateLandmarks( std::vector<PlaneType> &landmarks,
                          const std::vector<PlaneType> &observations,
                          const std::vector<PlanePair> &pairs,
                          const Pose3 &estimated_pose,
                          const std::vector<OrientedPlane3> &estimated_planes);

    void publishEstimatedPath();

    void publishOdomPath();

    inline std::vector<PlaneType> &getLandmarks() { return landmarks_; }

    void voxelGridFilter( const PointCloudTypePtr &cloud,
                           PointCloudTypePtr &cloud_filtered,
                           float leaf_size = 0.02f);

    void extractPlaneHulls(const PointCloudTypePtr &input, std::vector<PlaneType> &planes);

    void projectPoints ( const PointCloudType &input,
                         const Eigen::Vector4d &model_coefficients,
                         PointCloudType &projected_points );

    void projectPoints ( const PointCloudType &input,
                         const Eigen::Vector4f &model_coefficients,
                         PointCloudType &projected_points );

    void projectPoints ( const PointCloudType &input, const std::vector<int> &inlier,
                         const Eigen::Vector4d &model_coefficients, PointCloudType &projected_points );

    void projectPoints ( const PointCloudType &input, const std::vector<int> &inlier,
                         const Eigen::Vector4f &model_coefficients, PointCloudType &projected_points );

//    void projectPoints( const PointCloudTypePtr &input, std::vector<int> &inliers,
//                        Eigen::Vector4d &coeffs, PointCloudTypePtr &output);

    void cloudHull( const PointCloudTypePtr &cloud, PointCloudTypePtr &cloud_hull);

    bool checkOverlap( const PointCloudTypePtr &landmark_cloud, const OrientedPlane3 &landmark,
                       const PointCloudTypePtr &observation, const Pose3 &pose);

    void tfToPose3( tf::Transform &trans, gtsam::Pose3 &pose );

    void pose3ToTF( gtsam::Pose3 &pose, tf::Transform &trans );

    inline void setPlaneMatchThreshold( double direction_thresh, double distance_thresh) {
        plane_match_direction_threshold_ = direction_thresh;
        plane_match_distance_threshold_ = distance_thresh; }
    inline double getPlaneMatchDirectionThreshold() const { return plane_match_direction_threshold_; }
    inline double getPlaneMatchDistanceThreshold() const { return plane_match_distance_threshold_; }

    inline void setPlaneMatchCheckOverlap( bool check ) { plane_match_check_overlap_ = check; }
    inline bool getPlaneMatchCheckOverlap() const { return plane_match_check_overlap_; }

    inline void setPlaneMatchOverlapAlpha( double alpha ) { plane_match_overlap_alpha_ = alpha; }
    inline double getPlaneMatchOverlapAlpha() const { return plane_match_overlap_alpha_; }

    inline void setPlaneInlierLeafSize( double leaf_size ) { plane_inlier_leaf_size_ = leaf_size; }
    inline double getPlaneInlierLeafSize() const { return plane_inlier_leaf_size_; }

    inline void setPlaneHullAlpha( double alpha ) { plane_hull_alpha_ = alpha; }
    inline double getPlaneHullAlpha() const { return plane_hull_alpha_; }


private:
    //
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher path_publisher_;
    ros::Publisher marker_publisher_;
    ros::Publisher odom_path_publisher_;
    //
    Pose3 odom_pose_;
    std::vector<Pose3> odom_poses_;
    //
    ISAM2Params isam2_parameters_;
    ISAM2* isam2_;
    Pose3 last_estimate_pose_;
    //
    // Create a Factor Graph and Values to hold the new data
    NonlinearFactorGraph graph_; // factor graph
    Values initial_estimate_; // initial guess
    //
    bool first_pose_;
    int pose_count_;
    int landmark_count_;
    cv::RNG rng;

    //
    std::vector<Pose3> estimated_poses_;
    std::vector<OrientedPlane3> estimated_planes_;
    std::vector<PlaneType> landmarks_;
    // Parameters
    double plane_match_direction_threshold_;
    double plane_match_distance_threshold_;
    bool plane_match_check_overlap_;
    double plane_match_overlap_alpha_;
    double plane_inlier_leaf_size_;
    double plane_hull_alpha_;
};

#endif // PLANE_SLAM_H
