#ifndef UTILS_H
#define UTILS_H

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
//#include <opencv2/core/eigen.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_representation.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_pointcloud_occupancy.h>
//
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/OrientedPlane3.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <stdio.h>
#include <vector>
#include <stdlib.h>

using namespace std;
using namespace Eigen;

const double DEG_TO_RAD = ( M_PI / 180.0 );
const double RAD_TO_DEG = ( 180.0 / M_PI );

//
typedef pcl::PointCloud< pcl::PointXYZ > PointCloudXYZ;
typedef PointCloudXYZ::Ptr PointCloudXYZPtr;

typedef pcl::PointCloud< pcl::PointXYZRGBA > PointCloudXYZRGBA;
typedef PointCloudXYZRGBA::Ptr PointCloudXYZRGBAPtr;

typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointCloud< PointType > PointCloudType;
typedef PointCloudType::Ptr PointCloudTypePtr;
typedef PointCloudType::ConstPtr PointCloudTypeConstPtr;

typedef boost::shared_ptr<const pcl::PointRepresentation< PointType > > PointRepresentationConstPtr;

typedef std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > std_vector_of_eigen_vector4f;

typedef Eigen::Vector4d PlaneCoefficients;

// RGB Value
typedef union
{
    struct /*anonymous*/
    {
        unsigned char Blue;
        unsigned char Green;
        unsigned char Red;
        unsigned char Alpha;
    };
    float float_value;
    long long_value;
} RGBValue;

struct CAMERA_INTRINSIC_PARAMETERS
{
    int width, height;
    float cx, cy, fx, fy, scale;
    CAMERA_INTRINSIC_PARAMETERS() : width(640), height(480), cx(319.5), cy(239.5), fx(525.0), fy(525.0), scale(1.0) {}
};


/*
 * \brief Plane parameters
  N*P + d = 0
  */
struct PlaneType
{
    PointType centroid;
    Eigen::Vector4d coefficients;
    Eigen::Vector3d sigmas;
    //
    std::vector<int> inlier;
    std::vector<int> boundary_inlier;
    std::vector<int> hull_inlier;
    PointCloudTypePtr cloud;
    PointCloudTypePtr cloud_boundary;
    PointCloudTypePtr cloud_hull;
    RGBValue color;
    bool valid;
    //
    cv::Mat mask;
    std::vector<cv::KeyPoint> feature_locations_2d;
    std_vector_of_eigen_vector4f feature_locations_3d;
    cv::Mat feature_descriptors;

    PlaneType() : cloud( new PointCloudType)
      , cloud_boundary( new PointCloudType)
      , cloud_hull( new PointCloudType)
      , mask()
      , valid(true)
    {   color.Blue = 255; color.Green = 255; color.Red = 255; color.Alpha = 255;}

    PlaneType( bool is_valid ) : cloud( new PointCloudType)
      , cloud_boundary( new PointCloudType)
      , cloud_hull( new PointCloudType)
      , mask()
      , valid(is_valid)
    {   color.Blue = 255; color.Green = 255; color.Red = 255; color.Alpha = 255;}
};

struct PlanePair
{
    unsigned int iobs;
    unsigned int ilm;

    double distance;

    PlanePair() : iobs(-1), ilm(-1), distance(1e6) {}
    PlanePair(unsigned int _iobs, unsigned int _ilm) : iobs(_iobs), ilm(_ilm), distance(1e6) {}
    PlanePair(unsigned int _iobs, unsigned int _ilm, double _dis) : iobs(_iobs), ilm(_ilm), distance(_dis) {}

    // less is better
    bool operator<( const PlanePair &m ) const
    {
        return distance < m.distance;
    }
};


struct KinectFrame
{
    cv::Mat visual_image;
    cv::Mat depth_image;
    cv::Mat depth_mono;
    PointCloudTypePtr cloud;
    //
    PointCloudTypePtr cloud_in; //
    cv::Mat visual; //
    //
    std::vector<cv::KeyPoint> feature_locations_2d;
    std_vector_of_eigen_vector4f feature_locations_3d;
    cv::Mat feature_descriptors;
    PointCloudXYZPtr feature_cloud;
    //
    std::vector<PlaneType> segment_planes;

    KinectFrame() : cloud( new PointCloudType ), cloud_in( new PointCloudType )
    , feature_cloud( new PointCloudXYZ ){}
};

// PnP Result
struct RESULT_OF_MOTION
{
    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;
    bool valid;
    int inlier;
    double score;
    double rmse;

    RESULT_OF_MOTION() : valid(true),inlier(0), score(0), rmse(1e9)
    {
        rotation = Eigen::Matrix3d::Identity();
        translation = Eigen::Vector3d::Zero();
    }

    Eigen::Matrix4d transform4d()
    {
        Eigen::Matrix4d tr;
        tr.topLeftCorner(3,3) = rotation;
        tr.col(3).head<3>() = translation;
        return tr;
    }

    Eigen::Matrix4f transform4f()
    {
        Eigen::Matrix4f tr;
        tr.topLeftCorner(3,3) = rotation.cast<float>();
        tr.col(3).head<3>() = translation.cast<float>();
        return tr;
    }

    void setTransform4d( const Eigen::Matrix4d &tr )
    {
        rotation = tr.topLeftCorner(3,3);
        translation = tr.col(3).head<3>();
    }

    void setTransform4f( const Eigen::Matrix4f &tr )
    {
        rotation = tr.topLeftCorner(3,3).cast<double>();
        translation = tr.col(3).head<3>().cast<double>();
    }
};

void matrixTF2Eigen(const tf::Matrix3x3 &t, Eigen::Matrix3d &e);
Eigen::Matrix3d matrixTF2Eigen(const tf::Matrix3x3 &t);
void matrixEigen2TF( const Eigen::Matrix3d &e, tf::Matrix3x3 &t);
tf::Matrix3x3 matrixEigen2TF(const Eigen::Matrix3d &m33);

template <typename PointT>
PointT transformPoint (const PointT &point,
                     const Eigen::Matrix4d &transform);

template <typename PointT>
PointT transformPoint (const PointT &point,
                     const Eigen::Matrix4f &transform);

void transformPointCloud (const PointCloudType &cloud_in,
                          PointCloudType &cloud_out,
                          const Eigen::Matrix4d &transform);

void transformPointCloud (const PointCloudType &cloud_in,
                          PointCloudType &cloud_out,
                          const Eigen::Matrix4d &transform,
                          const RGBValue &color);

void transformPlane( const Eigen::Vector4d &input,
                     const Eigen::Matrix4d &transform,
                     Eigen::Vector4d &output);

void projectPoints ( const PointCloudType &input,
                     const Eigen::Vector4d &model_coefficients,
                     PointCloudType &projected_points );

void projectPoints ( const PointCloudType &input,
                     const Eigen::Vector4f &model_coefficients,
                     PointCloudType &projected_points );

void projectPoints ( const PointCloudType &input, const std::vector<int> &inlier,
                     const Eigen::Vector4d &model_coefficients, PointCloudType &projected_points );

void projectPoints ( const PointCloudType &input, const std::vector<int> &inlier,
                     const Eigen::Vector4f &model_coefficients, PointCloudType &projected_points );

void depthToCV8UC1(cv::Mat& depth_img, cv::Mat& mono8_img);

inline double depth_std_dev(double depth)
{
  // From Khoselham and Elberink?
  static double depth_std_dev = 0.01;
  // Previously used 0.006 from information on http://www.ros.org/wiki/openni_kinect/kinect_accuracy;
  // ...using 2sigma = 95%ile
  //static const double depth_std_dev  = 0.006;
  return depth_std_dev * depth * depth;
}
//Functions without dependencies
inline double depth_covariance(double depth)
{
  static double stddev = depth_std_dev(depth);
  static double cov = stddev * stddev;
  return cov;
}


void printTransform( const Eigen::Matrix4d &transform);
void printTransform( const Eigen::Matrix4f &transform);


double errorFunction2(const Eigen::Vector4f& x1,
                      const Eigen::Vector4f& x2,
                      const Matrix4d &tf_1_to_2);

void cvToEigen(const cv::Mat& src, Eigen::Matrix3d& dst );

void tfToPose3( const tf::Transform &trans, gtsam::Pose3 &pose );
gtsam::Pose3 tfToPose3( const tf::Transform &trans);
void pose3ToTF( const gtsam::Pose3 &pose, tf::Transform &trans );
tf::Transform pose3ToTF( const gtsam::Pose3 &pose );

geometry_msgs::PoseStamped pose3ToGeometryPose( const gtsam::Pose3 pose3 );
geometry_msgs::PoseStamped tfToGeometryPose( const tf::Transform &trans );
geometry_msgs::PoseStamped motionToGeometryPose( const RESULT_OF_MOTION &motion );

int bruteForceSearchORB(const uint64_t* v, const uint64_t* search_array, const unsigned int& size, int& result_index);


//the following are UBUNTU/LINUX ONLY terminal color
#define RESET "\033[0m"
#define BLACK "\033[30m" /* Black */
#define RED "\033[31m" /* Red */
#define GREEN "\033[32m" /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m" /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m" /* Cyan */
#define WHITE "\033[37m" /* White */
#define BOLDBLACK "\033[1m\033[30m" /* Bold Black */
#define BOLDRED "\033[1m\033[31m" /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m" /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m" /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m" /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m" /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m" /* Bold White */

#endif // UTILS_H
