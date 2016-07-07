#ifndef TRACKING_H
#define TRACKING_H

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <boost/foreach.hpp>
#include "frame.h"
#include "utils.h"

class Tracking
{
public:
    Tracking( Frame &source, Frame &target, RESULT_OF_MOTION &motion );

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


    void matchImageFeatures( KinectFrame& last_frame,
                             KinectFrame& current_frame,
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
};

#endif // TRACKING_H
