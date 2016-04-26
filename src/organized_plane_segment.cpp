#include "organized_plane_segment.h"


OrganizedPlaneSegment::OrganizedPlaneSegment():
    ne_()
  , mps_()
  , ne_method_(0)
  , ne_max_depth_change_factor_(0.02)
  , ne_normal_smoothing_size_(20.0)
  , min_inliers_(4000)
  , angular_threshold_(2.0)
  , distance_threshold_(0.02)
  , project_bounding_points_(true)
{
    updateParameters();
}

void OrganizedPlaneSegment::updateParameters()
{
    cout << "/******** Organized Multi Plane extractor ********/" << endl;
    cout << " ne method: " << ne_method_ << endl;
    cout << " ne_max_depth_change_factor: " << ne_max_depth_change_factor_ << endl;
    cout << " ne_normal_smoothing_size: " << ne_normal_smoothing_size_ << endl;
    cout << " -------------------------------------------------" << endl;
    cout << " min_inliers: " << min_inliers_ << endl;
    cout << " angular_threshold: " << angular_threshold_ << endl;
    cout << " distance_threshold: " << distance_threshold_ << endl;
    cout << " project_bounding_points: " << project_bounding_points_ << endl;
    cout << "/*************************************************/" << endl;

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
    mps_.setDistanceThreshold (distance_threshold_);
    mps_.setProjectPoints(project_bounding_points_);
}

void OrganizedPlaneSegment::segment(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &input, VectorPlanarRegion &regions)
{
    // Calculate Normals
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
    ne_.setInputCloud(input);
    ne_.compute(*normal_cloud);

    // Segment
    mps_.setInputNormals(normal_cloud);
    mps_.setInputCloud(input);
    mps_.segmentAndRefine(regions);
}

void OrganizedPlaneSegment::segment(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &input, OrganizedPlaneSegmentResult &result)
{
    // Calculate Normals
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
    ne_.setInputCloud(input);
    ne_.compute(*normal_cloud);

    // Segment
    mps_.setInputNormals(normal_cloud);
    mps_.setInputCloud(input);
    mps_.segmentAndRefine(result.regions, result.model_coeffs, result.inlier_indices, result.labels, result.label_indices, result.boundary_indices);
}

