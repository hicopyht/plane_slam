#ifndef LINE_BASED_PLANE_SEGMENTOR_H
#define LINE_BASED_PLANE_SEGMENTOR_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <plane_from_line/plane_from_line_segment.h>
#include <plane_slam/LineBasedSegmentConfig.h>
#include "utils.h"

namespace plane_slam
{

class LineBasedPlaneSegmentor
{
public:
    LineBasedPlaneSegmentor( ros::NodeHandle &nh );
    void updateLineBasedPlaneSegmentParameters();
    void operator()(PointCloudTypePtr &input, std::vector<PlaneType> &planes,
                    CameraParameters &camera_parameters);

protected:
    void lineBasedSegmentReconfigCallback( plane_slam::LineBasedSegmentConfig &config, uint32_t level);

private:
    ros::NodeHandle private_nh_;
    dynamic_reconfigure::Server<plane_slam::LineBasedSegmentConfig> line_based_segment_config_server_;
    dynamic_reconfigure::Server<plane_slam::LineBasedSegmentConfig>::CallbackType line_based_segment_config_callback_;
    //
    CameraParameters camera_parameters_;
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
};

} // end of namespace plane_slam

#endif // LINE_BASED_PLANE_SEGMENTOR_H
