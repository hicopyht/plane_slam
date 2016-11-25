#include "organized_multi_plane_segmentor.h"


OrganizedPlaneSegmentor::OrganizedPlaneSegmentor( ros::NodeHandle &nh ):
    private_nh_(nh)
  , ne_()
  , mps_()
  , organized_segment_config_server_( ros::NodeHandle( private_nh_, "OrganizedSegment" ) )
  , is_update_organized_parameters_( true )
  , ne_method_(0)
  , ne_max_depth_change_factor_(0.02)
  , ne_normal_smoothing_size_(20.0)
  , min_inliers_(3600)
  , angular_threshold_(3.0)
  , distance_threshold_(0.02)
  , project_bounding_points_(true)
{
    updateOrganizedSegmentParameters();
}

void OrganizedPlaneSegmentor::updateOrganizedSegmentParameters()
{
    if( !is_update_organized_parameters_ )
        return;

//    cout << "/******** Organized Multi Plane extractor ********/" << endl;
//    cout << " ne method: " << ne_method_ << endl;
//    cout << " ne_max_depth_change_factor: " << ne_max_depth_change_factor_ << endl;
//    cout << " ne_normal_smoothing_size: " << ne_normal_smoothing_size_ << endl;
//    cout << " -------------------------------------------------" << endl;
//    cout << " min_inliers: " << min_inliers_ << endl;
//    cout << " angular_threshold: " << angular_threshold_ << endl;
//    cout << " distance_threshold: " << distance_threshold_ << endl;
//    cout << " project_bounding_points: " << project_bounding_points_ << endl;
//    cout << "/*************************************************/" << endl;

    switch(ne_method_)
    {
        case 0:
            ne_.setNormalEstimationMethod(ne_.COVARIANCE_MATRIX);
            break;
        case 1:
            ne_.setNormalEstimationMethod(ne_.AVERAGE_3D_GRADIENT);
            break;
        case 2:
            ne_.setNormalEstimationMethod(ne_.AVERAGE_DEPTH_CHANGE);
            break;
        case 3:
            ne_.setNormalEstimationMethod(ne_.SIMPLE_3D_GRADIENT);
            break;
        default:
            ne_.setNormalEstimationMethod(ne_.COVARIANCE_MATRIX);
    }
    ne_.setMaxDepthChangeFactor(ne_max_depth_change_factor_);
    ne_.setNormalSmoothingSize(ne_normal_smoothing_size_);
    //
    mps_.setMinInliers (min_inliers_);
    mps_.setAngularThreshold (0.017453 * angular_threshold_);
//    mps_.setAngularThreshold( DEG_TO_RAD * angular_threshold_ );
    mps_.setDistanceThreshold (distance_threshold_);
    mps_.setProjectPoints(project_bounding_points_);

    is_update_organized_parameters_ = false;
}


void OrganizedPlaneSegmentor::operator()( const PointCloudTypePtr &input, std::vector<PlaneType> &planes )
{
    OrganizedPlaneSegmentResult segment_result;
    segment( input, segment_result );

    // convert format
    for( int i = 0; i < segment_result.regions.size(); i++)
    {
        pcl::ModelCoefficients &coef = segment_result.model_coeffs[i];
        pcl::PointIndices &indices = segment_result.inlier_indices[i];
        pcl::PlanarRegion<PointType> &pr = segment_result.regions[i];
        pcl::PointIndices &boundary = segment_result.boundary_indices[i];
        //
        PlaneType plane;
        Eigen::Vector3f centroid = pr.getCentroid();
        plane.centroid.x = centroid[0];
        plane.centroid.y = centroid[1];
        plane.centroid.z = centroid[2];
        plane.coefficients[0] = coef.values[0];
        plane.coefficients[1] = coef.values[1];
        plane.coefficients[2] = coef.values[2];
        plane.coefficients[3] = coef.values[3];
        plane.sigmas[0] = 0.0004;
        plane.sigmas[1] = 0.0004;
        plane.sigmas[2] = 0.0004;
        plane.inlier = indices.indices;
        plane.boundary_inlier = boundary.indices;
        plane.hull_inlier = boundary.indices;
        projectPoints( *input, plane.inlier, plane.coefficients, *(plane.cloud) );
//        getPointCloudFromIndices( input, plane.inlier, plane.cloud );
//            getPointCloudFromIndices( input, plane.boundary_inlier, plane.cloud_boundary );
//            getPointCloudFromIndices( input, plane.hull_inlier, plane.cloud_hull );
        //
        planes.push_back( plane );
    }
}

void OrganizedPlaneSegmentor::segment(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &input, VectorPlanarRegion &regions)
{
    //
    updateOrganizedSegmentParameters();

    // Calculate Normals
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
    ne_.setInputCloud(input);
    ne_.compute(*normal_cloud);

    // Segment
    mps_.setInputNormals(normal_cloud);
    mps_.setInputCloud(input);
    mps_.segmentAndRefine(regions);
}

void OrganizedPlaneSegmentor::segment(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &input, OrganizedPlaneSegmentResult &result)
{
    //
    updateOrganizedSegmentParameters();

    // Calculate Normals
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
    ne_.setInputCloud(input);
    ne_.compute(*normal_cloud);

    // Segment
    mps_.setInputNormals(normal_cloud);
    mps_.setInputCloud(input);
    mps_.segmentAndRefine(result.regions, result.model_coeffs, result.inlier_indices, result.labels, result.label_indices, result.boundary_indices);
}

void OrganizedPlaneSegmentor::organizedSegmentReconfigCallback(plane_slam::OrganizedSegmentConfig &config, uint32_t level)
{
    //
    ne_method_ = config.ne_method;
    ne_max_depth_change_factor_ = config.ne_max_depth_change_factor;
    ne_normal_smoothing_size_ = config.ne_normal_smoothing_size;
    //
    angular_threshold_ = config.angular_threshold;
    distance_threshold_ = config.distance_threshold;
    min_inliers_ = config.min_inliers;
    project_bounding_points_ = config.project_bounding_points;

    cout << GREEN <<"Organized Segment Config." << RESET << endl;

    is_update_organized_parameters_ = true;
}
