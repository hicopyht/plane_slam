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

namespace plane_slam
{

class Tracking
{
public:
    Tracking(ros::NodeHandle &nh);

    bool track( const Frame &source, const Frame &target, RESULT_OF_MOTION &motion );

public:
    bool solveRelativeTransformPlanes( const Frame &source,
                                       const Frame &target,
                                       const std::vector<PlanePair> &pairs,
                                       RESULT_OF_MOTION &result);

    bool solveRelativeTransformPlanesPointsRansac( Frame &last_frame,
                                                   Frame &current_frame,
                                                   std::vector<PlanePair> &pairs,
                                                   std::vector<cv::DMatch> &good_matches,
                                                   RESULT_OF_MOTION &result,
                                                   std::vector<cv::DMatch> &matches );

    bool solveRelativeTransformPointsRansac( const Frame &source,
                                             const Frame &target,
                                             const std::vector<cv::DMatch> &good_matches,
                                             RESULT_OF_MOTION &result,
                                             std::vector<cv::DMatch> &matches );

    bool solveRelativeTransformIcp( Frame &last_frame,
                                    Frame &current_frame,
                                    RESULT_OF_MOTION &result);

    bool solveRelativeTransformPnP( Frame& last_frame,
                                    Frame& current_frame,
                                    std::vector<cv::DMatch> &good_matches,
                                    PlaneFromLineSegment::CAMERA_PARAMETERS& camera,
                                    RESULT_OF_MOTION &result );

    bool solveRelativeTransform( Frame &last_frame,
                                 Frame &current_frame,
                                 RESULT_OF_MOTION &result,
                                 std::vector<cv::DMatch> &matches,
                                 Eigen::Matrix4d estimated_transform = Eigen::MatrixXd::Identity(4,4));

private:
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

    inline bool validRelativeTransform( const RESULT_OF_MOTION &motion )
    {
        if( !motion.valid )
            return false;

        if( motion.translation.norm() > 0.15 )
            return false;

        gtsam::Pose3 rel( gtsam::Rot3(motion.rotation), gtsam::Point3(motion.translation) );
        double angle = acos( cos(rel.rotation().yaw()) * cos(rel.rotation().pitch()) * cos(rel.rotation().roll()) );

        if( angle > 15.0*DEG_TO_RAD )
            return false;

        return true;

    }


    void matchImageFeatures( Frame& last_frame,
                             Frame& current_frame,
                             vector< cv::DMatch > &goodMatches,
                             double good_match_threshold,
                             int min_match_size);

    void computeCorrespondenceInliersAndError( const std::vector<cv::DMatch> & matches,
                                               const Eigen::Matrix4f& transform4f,
                                               const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& query_points,
                                               const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& train_points,
                                               unsigned int min_inlier,
                                               std::vector<cv::DMatch>& inlier, //pure output var
                                               double& return_mean_error,//pure output var: rms-mahalanobis-distance
                                               double squared_max_distance) const;

    void computeEuclidianDistance( const std::vector<PlaneType>& last_planes,
                                   const std::vector<PlaneType>& planes,
                                   const std::vector<PlanePair>& pairs,
                                   RESULT_OF_MOTION &relative );

    // Random pick
    std::vector<PlanePair> randomChoosePlanePairsPreferGood( std::vector< std::vector<PlanePair> > &sample_pairs );

    std::vector<PlanePair> randomChoosePlanePairs( std::vector< std::vector<PlanePair> > &sample_pairs );

    std::vector<cv::DMatch> randomChooseMatchesPreferGood( const unsigned int sample_size,
                                             vector< cv::DMatch > &matches_with_depth );

    std::vector<cv::DMatch> randomChooseMatches( const unsigned int sample_size,
                                             vector< cv::DMatch > &matches );

protected:

    void trackingReconfigCallback(plane_slam::TrackingConfig &config, uint32_t level);

private:
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
