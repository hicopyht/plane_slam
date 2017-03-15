#include "line_based_plane_segmentor.h"

namespace plane_slam
{

LineBasedPlaneSegmentor::LineBasedPlaneSegmentor( ros::NodeHandle &nh )
    : private_nh_(nh),
      line_based_segment_config_server_( ros::NodeHandle( private_nh_, "LineBasedSegment" ) ),
      plane_segmentor_("/home/lizhi/bags/rgbd/config/QQVGA.yaml"),
      is_update_line_based_parameters_( true )
{
    // reconfigure gui
    line_based_segment_config_callback_ = boost::bind(&LineBasedPlaneSegmentor::lineBasedSegmentReconfigCallback, this, _1, _2);
    line_based_segment_config_server_.setCallback(line_based_segment_config_callback_);

}

void LineBasedPlaneSegmentor::operator()(PointCloudTypePtr &input, std::vector<PlaneType> &planes, CameraParameters &camera_parameters)
{
    PointCloudTypePtr cloud_in (new PointCloudType);
//    pcl::copyPointCloud( *input, *cloud_in);
    cloud_in = input;   // only copy pointer

    // Update parameters
    if( is_update_line_based_parameters_ )
    {
        updateLineBasedPlaneSegmentParameters();
        is_update_line_based_parameters_ = false;
    }

    // Do segment
    std::vector<line_based_plane_segment::PlaneType> line_based_planes;
    plane_segmentor_.setInputCloud( cloud_in );
    plane_segmentor_.segment( line_based_planes );


    // convert format
    for( size_t i = 0; i < line_based_planes.size(); i++)
    {
        line_based_plane_segment::PlaneType &pl = line_based_planes[i];
        //
        PlaneType plane;
        plane.centroid = pl.centroid;
        plane.coefficients[0] = pl.coefficients[0];
        plane.coefficients[1] = pl.coefficients[1];
        plane.coefficients[2] = pl.coefficients[2];
        plane.coefficients[3] = pl.coefficients[3];
//        plane.sigmas[0] = 0.0004;
//        plane.sigmas[1] = 0.0004;
//        plane.sigmas[2] = 0.0004;
        plane.sigmas[0] = 0.008;
        plane.sigmas[1] = 0.008;
        plane.sigmas[2] = 0.008;
        plane.inlier = pl.indices;
        plane.boundary_inlier = pl.boundary_indices;
        plane.hull_inlier = pl.hull_indices;
        projectPoints( *input, plane.inlier, plane.coefficients, *(plane.cloud) );
//            getPointCloudFromIndices( input, plane.boundary_inlier, plane.cloud_boundary );
//            getPointCloudFromIndices( input, plane.hull_inlier, plane.cloud_hull );
        //
        planes.push_back( plane );
    }

}

// update parameters
void LineBasedPlaneSegmentor::updateLineBasedPlaneSegmentParameters()
{
    //
    plane_segmentor_.use_horizontal_line_ = use_horizontal_line_;
    plane_segmentor_.use_verticle_line_ = use_verticle_line_;
    plane_segmentor_.y_interval_ = y_interval_;
    plane_segmentor_.x_interval_ = x_interval_;
    //
    plane_segmentor_.line_point_min_distance_ = line_point_min_distance_;
    plane_segmentor_.line_fitting_angular_threshold_ = line_fitting_angular_threshold_;
    plane_segmentor_.line_fitting_min_indices_ = line_fitting_min_indices_;
    //
    plane_segmentor_.normals_per_line_ = normals_per_line_;
    plane_segmentor_.normal_smoothing_size_ = normal_smoothing_size_;
    plane_segmentor_.normal_min_inliers_percentage_ = normal_min_inliers_percentage_;
    plane_segmentor_.normal_maximum_curvature_ = normal_maximum_curvature_;
    //
    plane_segmentor_.remove_reduplicate_candidate_ = remove_reduplicate_candidate_;
    plane_segmentor_.reduplicate_candidate_normal_thresh_ = reduplicate_candidate_normal_thresh_;
    plane_segmentor_.reduplicate_candidate_distance_thresh_ = reduplicate_candidate_distance_thresh_;
    //
    plane_segmentor_.min_inliers_ = min_inliers_;
    plane_segmentor_.max_curvature_ = max_curvature_;
    plane_segmentor_.distance_threshold_ = distance_threshold_;
    plane_segmentor_.neighbor_threshold_ = neighbor_threshold_;
    plane_segmentor_.angular_threshold_ = angular_threshold_;
    //
    plane_segmentor_.optimize_coefficients_ = optimize_coefficients_;
    plane_segmentor_.solve_over_segment_ = solve_over_segment_;
    plane_segmentor_.refine_plane_ = refine_plane_;
    plane_segmentor_.project_points_ = project_points_;
    plane_segmentor_.extract_boundary_ = extract_boundary_;
    //
    plane_segmentor_.setNormalEstimateParams(normal_estimate_method_, normal_estimate_depth_change_factor_, normal_estimate_smoothing_size_);
}

void LineBasedPlaneSegmentor::lineBasedSegmentReconfigCallback( plane_slam::LineBasedSegmentConfig &config, uint32_t level)
{

    use_horizontal_line_ = config.use_horizontal_line;
    use_verticle_line_ = config.use_verticle_line;
    y_interval_ = config.y_interval;
    x_interval_ = config.x_interval;
    //
    line_point_min_distance_ = config.line_point_min_distance;
    line_fitting_angular_threshold_ = config.line_fitting_angular_threshold;
    line_fitting_min_indices_ = config.line_fitting_min_indices;
    //
    normals_per_line_ = config.normals_per_line;
    normal_smoothing_size_ = config.normal_smoothing_size;
    normal_min_inliers_percentage_ = config.normal_min_inliers_percentage;
    normal_maximum_curvature_ = config.normal_maximum_curvature;
    //
    remove_reduplicate_candidate_ = config.remove_reduplicate_candidate;
    reduplicate_candidate_normal_thresh_ = config.reduplicate_candidate_normal_thresh;
    reduplicate_candidate_distance_thresh_ = config.reduplicate_candidate_distance_thresh;
    //
    min_inliers_ = config.min_inliers;
    max_curvature_ = config.max_curvature;
    distance_threshold_ = config.distance_threshold;
    neighbor_threshold_ = config.neighbor_threshold;
    angular_threshold_ = config.angular_threshold;
    //
    optimize_coefficients_ = config.optimize_coefficients;
    solve_over_segment_ = config.solve_over_segment;
    refine_plane_ = config.refine_plane;
    project_points_ = config.project_points;
    extract_boundary_ = config.extract_boundary;

    //
    normal_estimate_method_ = config.normal_estimate_method;
    normal_estimate_depth_change_factor_ = config.normal_estimate_depth_change_factor;
    normal_estimate_smoothing_size_ = config.normal_estimate_smoothing_size;

    cout << GREEN <<" Line Based Segment Config." << RESET << endl;

    is_update_line_based_parameters_ = true;
}

} // end of namespace plane_slam
