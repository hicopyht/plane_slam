#ifndef PLANE_SLAM_H
#define PLANE_SLAM_H

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
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
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

    void matchPlanes( std::vector<OrientedPlane3> &predicted_observations,
                      std::vector<OrientedPlane3> &observations,
                      std::vector<PlanePair> &pairs);

    void getPredictedObservation( Pose3 &pose, std::vector<OrientedPlane3> &predicted_observations );

    void publishPath();

    void publishOdomPath();

    void getLandmarks( std::vector<PlaneType> &planes );

    void extractPlaneHulls(const PointCloudTypePtr &input, std::vector<PlaneType> &planes);

    void projectPoints ( const PointCloudTypePtr &input, const std::vector<int> &inliers,
                         const Eigen::Vector4f &model_coefficients, PointCloudType &projected_points );

//    void projectPoints( const PointCloudTypePtr &input, std::vector<int> &inliers,
//                        Eigen::Vector4d &coeffs, PointCloudTypePtr &output);

    void cloudHull( const PointCloudTypePtr &cloud, PointCloudTypePtr &cloud_hull);

    void tfToPose3( tf::Transform &trans, gtsam::Pose3 &pose );

    void pose3ToTF( gtsam::Pose3 &pose, tf::Transform &trans );

    inline void setPlaneMatchThreshold( double threshold) { plane_match_threshold_ = threshold; }
    inline double getPlaneMatchThreshold() const { return plane_match_threshold_; }

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
    Values poses_;
    int pose_count_;
    int landmark_count_;

    // Parameters
    double plane_match_threshold_;
};

#endif // PLANE_SLAM_H
