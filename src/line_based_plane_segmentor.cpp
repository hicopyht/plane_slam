#include "line_based_plane_segmentor.h"

namespace plane_slam
{

LineBasedPlaneSegmentor::LineBasedPlaneSegmentor( ros::NodeHandle &nh )
    : private_nh_(nh),
      plane_from_line_segment_(),
      line_based_segment_config_server_( ros::NodeHandle( private_nh_, "LineBasedSegment" ) )
{
    // reconfigure gui
    line_based_segment_config_callback_ = boost::bind(&LineBasedPlaneSegmentor::lineBasedSegmentReconfigCallback, this, _1, _2);
    line_based_segment_config_server_.setCallback(line_based_segment_config_callback_);


}

void LineBasedPlaneSegmentor::operator()(PointCloudTypePtr &input, std::vector<PlaneType> &planes, CameraParameters &camera_parameters)
{
    PointCloudTypePtr cloud_in (new PointCloudType);
    pcl::copyPointCloud( *input, *cloud_in);
//    cloud_in = input;   // only copy pointer

    // Update parameters
    if( is_update_line_based_parameters_ )
    {
        updateLineBasedPlaneSegmentParameters();
        is_update_line_based_parameters_ = false;
    }

    // Initialize if not
    if (!plane_from_line_segment_.isInitialized() )
    {
        plane_from_line_segment_.setCameraParameters( camera_parameters );
        cout << "Initialize line base segment." << endl;
    }

    // Do segment
    std::vector<PlaneFromLineSegment::NormalType> line_based_planes;
    plane_from_line_segment_.setInputCloud( cloud_in );
    plane_from_line_segment_.segment( line_based_planes );

    // convert format
    for( int i = 0; i < line_based_planes.size(); i++)
    {
        PlaneFromLineSegment::NormalType &normal = line_based_planes[i];
        PlaneType plane;
        plane.mask = normal.mask;
        plane.centroid = normal.centroid;
        plane.coefficients[0] = normal.coefficients[0];
        plane.coefficients[1] = normal.coefficients[1];
        plane.coefficients[2] = normal.coefficients[2];
        plane.coefficients[3] = normal.coefficients[3];
        plane.sigmas[0] = 0.1;
        plane.sigmas[1] = 0.1;
        plane.sigmas[2] = 0.1;
        plane.inlier = normal.inliers;
        plane.boundary_inlier = normal.boundary_inlier;
        plane.hull_inlier = normal.hull_inlier;
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
    plane_from_line_segment_.setUseHorizontalLines( use_horizontal_line_ );
    plane_from_line_segment_.setUseVerticleLines( use_verticle_line_ );
    plane_from_line_segment_.setYskip( y_skip_ );
    plane_from_line_segment_.setXSkip( x_skip_ );
    plane_from_line_segment_.setLinePointMinDistance( line_point_min_distance_ );
    plane_from_line_segment_.setUseDepthNoiseModel( use_depth_noise_model_ );
    plane_from_line_segment_.setRhoConstantError( scan_rho_constant_error_ );
    plane_from_line_segment_.setRhoDistanceError( scan_rho_distance_error_ );
    plane_from_line_segment_.setRhoQuadraticError( scan_rho_quadratic_error_ );
    plane_from_line_segment_.setSlideWindowSize( slide_window_size_ );
    plane_from_line_segment_.setLineMinInliers( line_min_inliers_ );
    plane_from_line_segment_.setLineFittingThreshold( line_fitting_threshold_ );
    //
    plane_from_line_segment_.setNormalsPerLine( normals_per_line_ );
    plane_from_line_segment_.setNormalUseDepthSmoothing( normal_use_depth_dependent_smoothing_ );
    plane_from_line_segment_.setNormalDepthChangeFactor( normal_max_depth_change_factor_ );
    plane_from_line_segment_.setNormalSmoothingSize( normal_smoothing_size_ );
    plane_from_line_segment_.setNormalMinInliersPercentage( normal_min_inliers_percentage_ );
    plane_from_line_segment_.setNormalMaximumCurvature( normal_maximum_curvature_ );
    //
    plane_from_line_segment_.setRemoveDuplicateCandidate( remove_duplicate_candidate_ );
    plane_from_line_segment_.setDuplicateCandidateThreshold( duplicate_candidate_normal_thresh_,
                                                             duplicate_candidate_distance_thresh_ );
    //
    plane_from_line_segment_.setPlaneSegmentCriterion( plane_segment_criterion_ );
    plane_from_line_segment_.setCriterionBothParameters( k_curvature_, k_inlier_ );
    plane_from_line_segment_.setMinInliers( min_inliers_ );
    plane_from_line_segment_.setDistanceThreshold( distance_threshold_ );
    plane_from_line_segment_.setNeighborThreshold( neighbor_threshold_ );
    plane_from_line_segment_.setOptimizeCoefficients( optimize_coefficients_ );
    plane_from_line_segment_.setProjectPoints( project_points_ );
    plane_from_line_segment_.setExtractBoundary( extract_boundary_ );
}

void LineBasedPlaneSegmentor::lineBasedSegmentReconfigCallback( plane_slam::LineBasedSegmentConfig &config, uint32_t level)
{
    //
    use_horizontal_line_ = config.use_horizontal_line;
    use_verticle_line_ = config.use_verticle_line;
    y_skip_ = config.y_skip;
    x_skip_ = config.x_skip;
    line_point_min_distance_ = config.line_point_min_distance;
    use_depth_noise_model_ = config.use_depth_noise_model;
    scan_rho_constant_error_ = config.scan_rho_constant_error;
    scan_rho_distance_error_ = config.scan_rho_distance_error;
    scan_rho_quadratic_error_ = config.scan_rho_quadratic_error;
    slide_window_size_ = config.slide_window_size;
    line_min_inliers_ = config.line_min_inliers;
    line_fitting_threshold_ = config.line_fitting_threshold;
    //
    normals_per_line_ = config.normals_per_line;
    normal_use_depth_dependent_smoothing_ = config.normal_use_depth_dependent_smoothing;
    normal_max_depth_change_factor_ = config.normal_max_depth_change_factor;
    normal_smoothing_size_ = config.normal_smoothing_size;
    normal_min_inliers_percentage_ = config.normal_min_inliers_percentage;
    normal_maximum_curvature_ = config.normal_maximum_curvature;
    //
    remove_duplicate_candidate_ = config.remove_duplicate_candidate;
    duplicate_candidate_normal_thresh_ = config.duplicate_candidate_normal_thresh;
    duplicate_candidate_distance_thresh_ = config.duplicate_candidate_distance_thresh;
    //
    plane_segment_criterion_ = config.plane_segment_criterion;
    k_curvature_ = config.k_curvature;
    k_inlier_ = config.k_inlier;
    min_inliers_ = config.min_inliers;
    distance_threshold_ = config.distance_threshold;
    neighbor_threshold_ = config.neighbor_threshold;
    optimize_coefficients_ = config.optimize_coefficients;
    project_points_ = config.project_points;
    extract_boundary_ = config.extract_boundary;
    //

    cout << GREEN <<"Line Based Segment Config." << RESET << endl;

    is_update_line_based_parameters_ = true;
}

} // end of namespace plane_slam
