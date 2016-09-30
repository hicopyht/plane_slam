#ifndef TRACKING_H
#define TRACKING_H

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <boost/foreach.hpp>
#include <dynamic_reconfigure/server.h>
#include <plane_slam/TrackingConfig.h>
#include "frame.h"
#include "utils.h"
#include "itree.h"
#include "viewer.h"

namespace plane_slam
{

class Tracking
{
public:
    Tracking(ros::NodeHandle &nh, Viewer * viewer );

    bool trackPlanes(const Frame &source, const Frame &target, RESULT_OF_MOTION &motion,
                     const Eigen::Matrix4d estimated_transform = Eigen::MatrixXd::Identity(4,4) );

    bool track( const Frame &source, Frame &target, RESULT_OF_MOTION &motion,
                const Eigen::Matrix4d estimated_transform = Eigen::MatrixXd::Identity(4,4) );

public:
    bool solveRelativeTransformPlanes( const Frame &source,
                                       const Frame &target,
                                       const std::vector<PlanePair> &pairs,
                                       RESULT_OF_MOTION &result,
                                       std::vector<PlanePair> &return_inlier );

    bool solveRelativeTransformPlanesPointsRansac( const Frame &source,
                                                   const Frame &target,
                                                   const std::vector<PlanePair> &pairs,
                                                   const std::vector<cv::DMatch> &good_matches,
                                                   RESULT_OF_MOTION &result,
                                                   std::vector<cv::DMatch> &matches );

    bool solveRelativeTransformPointsRansac( const Frame &source,
                                             const Frame &target,
                                             const std::vector<cv::DMatch> &good_matches,
                                             RESULT_OF_MOTION &result,
                                             std::vector<cv::DMatch> &matches );

    bool solveRelativeTransformPointsRansac( const std_vector_of_eigen_vector4f &source_feature_3d,
                                             const std_vector_of_eigen_vector4f &target_feature_3d,
                                             const std::vector<cv::DMatch> &good_matches,
                                             RESULT_OF_MOTION &result,
                                             std::vector<cv::DMatch> &matches,
                                             int min_inlier = 3);

    bool solveRelativeTransformIcp( const Frame &source,
                                    const Frame &target,
                                    RESULT_OF_MOTION &result);

    bool solveRelativeTransformPnP( const Frame& source,
                                    const Frame& target,
                                    const std::vector<cv::DMatch> &good_matches,
                                    const CameraParameters& camera,
                                    RESULT_OF_MOTION &result );

    bool solveRelativeTransform( const Frame &source,
                                 const Frame &target,
                                 const std::vector<PlanePair> &pairs,
                                 const std::vector<cv::DMatch> &good_matches,
                                 RESULT_OF_MOTION &result,
                                 std::vector<PlanePair> &pl_inlier,
                                 std::vector<cv::DMatch> &kp_inlier );

    void matchSurfFeatures( const Frame& source,
                            const Frame& target,
                            vector< cv::DMatch > &good_matches,
                            double good_match_threshold,
                            int min_match_size );

    void matchImageFeatures( const Frame& source,
                             const Frame& target,
                             vector< cv::DMatch > &good_matches,
                             double good_match_threshold = 4.0,
                             int min_match_size = 0);

    void matchImageFeatures( const cv::Mat &feature_descriptor,
                             const std::vector<cv::DMatch> &kp_inlier,
                             const std::map<int, KeyPoint*> &keypoints_list,
                             const std::map<int, gtsam::Point3> &predicted_keypoints,
                             vector<cv::DMatch> &good_matches,
                             double good_match_threshold = 4.0,
                             int min_match_size = 0);

    void matchImageFeatures( const Frame &frame,
                             const std::map<int, KeyPoint*> &keypoints_list,
                             const std::map<int, gtsam::Point3> &predicted_keypoints,
                             vector<cv::DMatch> &good_matches,
                             double good_match_threshold = 4.0,
                             int min_match_size = 0);

    void computePairInliersAndError( const Eigen::Matrix4d &transform,
                                     const std::vector<PlanePair>& pairs,
                                     const std::vector<PlaneType>& last_planes,
                                     const std::vector<PlaneType>& planes,
                                     std::vector<PlanePair> &inlier,
                                     double &return_mean_error,
                                     const double max_direction_error,
                                     const double max_distance_error );

    void computeCorrespondenceInliersAndError( const std::vector<cv::DMatch> & matches,
                                               const Eigen::Matrix4f& transform4f,
                                               const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& query_points,
                                               const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& train_points,
                                               unsigned int min_inlier,
                                               std::vector<cv::DMatch>& inlier, //pure output var
                                               double& return_mean_error,//pure output var: rms-mahalanobis-distance
                                               double squared_max_distance) const;

public:
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

    Eigen::Matrix4f solveRt( const std_vector_of_eigen_vector4f &query_points,
                             const std_vector_of_eigen_vector4f &train_points,
                             const std::vector<cv::DMatch> &matches,
                             bool &valid );

    void solveRt( const std::vector<Eigen::Vector3d>& from_points,
                  const std::vector<Eigen::Vector3d>& to_points,
                  RESULT_OF_MOTION &result);

    void solveRt( const std::vector<PlaneCoefficients> &last_planes,
                  const std::vector<PlaneCoefficients> &planes,
                  RESULT_OF_MOTION &result);

    bool solveRtPlanes( const std::vector<PlaneCoefficients> &before,
                        const std::vector<PlaneCoefficients> &after,
                            RESULT_OF_MOTION &result);

    Eigen::Matrix4f solveRtPlanesPoints( const std::vector<PlaneType> &last_planes,
                                         const std::vector<PlaneType> &planes,
                                         const std::vector<PlanePair> &pairs,
                                         const std_vector_of_eigen_vector4f &last_feature_3d,
                                         const std_vector_of_eigen_vector4f &feature_3d,
                                         const std::vector<cv::DMatch> &matches,
                                         bool &valid );

    inline bool validRelativeTransform( const RESULT_OF_MOTION &motion )
    {
        if( !motion.valid )
            return false;

        if( motion.translation.norm() > 0.20 )
            return false;

        gtsam::Pose3 rel( gtsam::Rot3(motion.rotation), gtsam::Point3(motion.translation) );
        double angle = acos( cos(rel.rotation().yaw()) * cos(rel.rotation().pitch()) * cos(rel.rotation().roll()) );

        if( angle > 20.0*DEG_TO_RAD )
            return false;

        return true;

    }


private:
    // Random pick
    std::vector<PlanePair> randomChoosePlanePairsPreferGood( const std::vector< std::vector<PlanePair> > &sample_pairs );

    std::vector<PlanePair> randomChoosePlanePairs( const std::vector< std::vector<PlanePair> > &sample_pairs );

    std::vector<cv::DMatch> randomChooseMatchesPreferGood( const unsigned int sample_size,
                                                        const vector< cv::DMatch > &matches_with_depth );

    std::vector<cv::DMatch> randomChooseMatches( const unsigned int sample_size,
                                                const vector< cv::DMatch > &matches );

protected:

    void trackingReconfigCallback(plane_slam::TrackingConfig &config, uint32_t level);

private:
    //
    Viewer * viewer_;
    //
    ros::NodeHandle nh_;
    dynamic_reconfigure::Server<plane_slam::TrackingConfig> tracking_config_server_;
    dynamic_reconfigure::Server<plane_slam::TrackingConfig>::CallbackType tracking_config_callback_;
    // Feature match
    double feature_good_match_threshold_;
    int feature_min_good_match_size_;
    // Point ransac
    int ransac_sample_size_;
    int ransac_iterations_;
    int ransac_min_inlier_;
    double ransac_inlier_max_mahal_distance_;
    // ICP
    double icp_max_distance_;
    int icp_iterations_;
    double icp_tf_epsilon_;
    int icp_min_indices_;
    double icp_score_threshold_;
    // PnP
    int pnp_iterations_;
    int pnp_min_inlier_;
    double pnp_repreject_error_;
};

} // end of namespace plane_slam

#endif // TRACKING_H
