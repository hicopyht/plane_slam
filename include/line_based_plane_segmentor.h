#ifndef LINE_BASED_PLANE_SEGMENTOR_H
#define LINE_BASED_PLANE_SEGMENTOR_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <plane_slam/LineBasedSegmentConfig.h>
#include <line_based_plane_segmentation.h>
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
    line_based_plane_segment::LineBasedPlaneSegmentation plane_segmentor_;
    bool is_update_line_based_parameters_;
    //
    // LineBased segment
    bool use_horizontal_line_;
    bool use_verticle_line_;
    int y_interval_;
    int x_interval_;

    /** \brief Line extraction */
    float line_point_min_distance_;
    float line_fitting_angular_threshold_;
    int line_fitting_min_indices_;

    /** \brief Normals per line */
    int normals_per_line_;
    int normal_smoothing_size_;
    float normal_min_inliers_percentage_;
    float normal_maximum_curvature_;

    /** \brief Remove duplicate candidate if True */
    bool remove_reduplicate_candidate_;
    float reduplicate_candidate_normal_thresh_;
    float reduplicate_candidate_distance_thresh_;

    /** \brief Plane extraction */
    int min_inliers_;
    float max_curvature_;
    float distance_threshold_;
    float neighbor_threshold_;
    float angular_threshold_;

    /** \brief Refine Plane segmentation result. Note: Not Valid. */
    bool solve_over_segment_;
    bool refine_plane_;
    bool optimize_coefficients_;
    bool project_points_;
    bool extract_boundary_;

    /** \brief Normal cloud estimation */
    int normal_estimate_method_;
    float normal_estimate_depth_change_factor_;
    float normal_estimate_smoothing_size_;
};

} // end of namespace plane_slam

#endif // LINE_BASED_PLANE_SEGMENTOR_H
