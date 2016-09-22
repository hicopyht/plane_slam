#ifndef UTILS_H
#define UTILS_H

#include <ros/ros.h>
#include <ros/time.h>
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
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_pointcloud_occupancy.h>
//
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/OrientedPlane3.h>
#include <gtsam/sam/BearingRangeFactor.h>

#include <plane_from_line/plane_from_line_segment.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <stdio.h>
#include <vector>
#include <stdlib.h>
#include <thread>

using namespace std;
using namespace Eigen;

const double DEG_TO_RAD = ( M_PI / 180.0 );
const double RAD_TO_DEG = ( 180.0 / M_PI );

typedef PlaneFromLineSegment::CAMERA_PARAMETERS  CameraParameters;

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

bool isValidPoint(const PointType &p);


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


/*
 * \brief Plane parameters
  N*P + d = 0
  */
struct PlaneType
{
    int landmark_id;
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
    PointCloudTypePtr cloud_voxel;
    RGBValue color;
    bool valid;
    //
    cv::Mat mask;
    std::vector<cv::KeyPoint> feature_locations_2d;
    std_vector_of_eigen_vector4f feature_locations_3d;
    cv::Mat feature_descriptors;
    // semantic label
    std::string semantic_label; // NONE or "", FLOOR, WALL, DOOR, TABLE

    PlaneType() : cloud( new PointCloudType)
      , cloud_boundary( new PointCloudType)
      , cloud_hull( new PointCloudType)
      , cloud_voxel( new PointCloudType)
      , mask()
      , valid(true)
    {   color.Blue = 255; color.Green = 255; color.Red = 255; color.Alpha = 255;}

    PlaneType( bool is_valid ) : cloud( new PointCloudType)
      , cloud_boundary( new PointCloudType)
      , cloud_hull( new PointCloudType)
      , cloud_voxel( new PointCloudType)
      , mask()
      , valid(is_valid)
    {   color.Blue = 255; color.Green = 255; color.Red = 255; color.Alpha = 255;}

    void setId( int _id ) { landmark_id = _id; }
    int &id() { return landmark_id;}
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

//// Keypoint
struct KeyPoint
{
    int keypoint_id;
    uint64_t descriptor[4];
    gtsam::Point3 translation;
    //
    RGBValue color;
    bool valid;

    KeyPoint() : valid(true), translation(0, 0, 0) {
        descriptor[0] = random(); descriptor[1] = random(); descriptor[2] = random(); descriptor[3] = random();
        color.long_value = 0xFFFFFFFF;
    }
    void setId( int _id ) { keypoint_id = _id; }
    int &id() { return keypoint_id;}
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

void calAngleAndDistance(const Eigen::Isometry3d& t, double& rad, double& dist);
void calAngleAndDistance( const tf::Transform &trans, double& rad, double& dist );
//
void matrixTF2Eigen(const tf::Matrix3x3 &t, Eigen::Matrix3d &e);
Eigen::Matrix3d matrixTF2Eigen(const tf::Matrix3x3 &t);
void matrixEigen2TF( const Eigen::Matrix3d &e, tf::Matrix3x3 &t);
tf::Matrix3x3 matrixEigen2TF(const Eigen::Matrix3d &m33);

//
void transformTFToMatrix4d(const tf::Transform &t, Eigen::Matrix4d &e);
Eigen::Matrix4d transformTFToMatrix4d(const tf::Transform &t);
void transformTFToMatrix4f(const tf::Transform &t, Eigen::Matrix4f &e);
Eigen::Matrix4f transformTFToMatrix4f(const tf::Transform &t);
void transformMatrix4dToTF(const Eigen::Matrix4d &e, tf::Transform &t);
tf::Transform transformMatrix4dToTF(const Eigen::Matrix4d &e);

//
void setPointCloudColor( PointCloudType &cloud, RGBValue &color );

void voxelGridFilter( const PointCloudTypePtr &cloud,
                      PointCloudTypePtr &cloud_filtered,
                      float leaf_size = 0.02f);

void voxelGridFilter( const PointCloudTypePtr &cloud,
                      const std::vector<int> &inlier,
                      PointCloudTypePtr &cloud_filtered,
                      float leaf_size = 0.02f);

void radiusOutlierRemoval( const PointCloudTypePtr &cloud,
                           PointCloudTypePtr &cloud_filtered,
                           double radius,
                           int min_neighbors);

void radiusOutlierRemoval( const PointCloudTypePtr &cloud,
                           const std::vector<int> &inlier,
                           PointCloudTypePtr &cloud_filtered,
                           double radius,
                           int min_neighbors);

Eigen::Matrix3d kinectBearingRangeCov( const gtsam::Point3 &point );
Eigen::Matrix3d kinectPointCov( const Eigen::Vector4f &point );

// Transform a point to world
gtsam::Point3 transformPoint( const gtsam::Point3 &point, const Matrix4d &transform );

// Transform a point to world
Eigen::Vector4f transformPoint ( const Eigen::Vector4f &point, const Eigen::Matrix4f &transform );
Eigen::Vector4f transformPoint ( const Eigen::Vector4f &point, const Eigen::Matrix4d &transform );

// Transform a point to world
template <typename PointT>
PointT transformPoint (const PointT &point,
                     const Eigen::Matrix4d &transform);

// Transform a point to world
template <typename PointT>
PointT transformPoint (const PointT &point,
                     const Eigen::Matrix4f &transform);

// Transform a pointcloud to world
void transformPointCloud (const PointCloudType &cloud_in,
                          PointCloudType &cloud_out,
                          const Eigen::Matrix4d &transform);

// Transform a pointcloud to world
void transformPointCloud (const PointCloudType &cloud_in,
                          PointCloudType &cloud_out,
                          const Eigen::Matrix4d &transform,
                          const RGBValue &color);

// Transform a pointcloud to world
PointCloudType transformPointCloud(const PointCloudType &cloud_in,
                                   const Matrix4d &transform);

// Transform a pointcloud to world
PointCloudType transformPointCloud (const PointCloudType &cloud_in,
                          const Eigen::Matrix4d &transform,
                          const RGBValue &color);


// Transform a plane to pose coordinate
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

void getPointCloudFromIndices( const PointCloudTypePtr &input,
                               const pcl::PointIndices &indices,
                               PointCloudTypePtr &output);

void getPointCloudFromIndices( const PointCloudTypePtr &input,
                               const std::vector<int> &indices,
                               PointCloudTypePtr &output);

PointCloudTypePtr getPointCloudFromIndices( const PointCloudTypePtr &input,
                                            const pcl::PointIndices &indices);

PointCloudTypePtr getPointCloudFromIndices( const PointCloudTypePtr &input,
                                            const std::vector<int> &indices);


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
  double stddev = depth_std_dev(depth);
  double cov = stddev * stddev;
  return cov;
}


void printTransform( const Eigen::Matrix4d &transform, const std::string name = "", const std::string color = "\033[0m" );
void printTransform( const Eigen::Matrix4f &transform, const std::string name = "", const std::string color = "\033[0m" );

void printPose3( const gtsam::Pose3 &pose3, const std::string name = "Pose3", const std::string color = "\033[0m" );


double errorFunction2(const Eigen::Vector4f& x1,
                      const Eigen::Vector4f& x2,
                      const Matrix4d &tf_1_to_2);

void cvToEigen(const cv::Mat& src, Eigen::Matrix3d& dst );

void tfToPose3( const tf::Transform &trans, gtsam::Pose3 &pose );
//
gtsam::Pose3 tfToPose3( const tf::Transform &trans);
//
void pose3ToTF( const gtsam::Pose3 &pose, tf::Transform &trans );
//
tf::Transform pose3ToTF( const gtsam::Pose3 &pose );
//
geometry_msgs::PoseStamped pose3ToGeometryPose( const gtsam::Pose3 &pose3 );
//
gtsam::Pose3 geometryPoseToPose3( const geometry_msgs::PoseStamped &pose );
//
geometry_msgs::PoseStamped tfToGeometryPose( const tf::Transform &trans );
//
tf::Transform geometryPoseToTf( const geometry_msgs::PoseStamped &pose);
//
geometry_msgs::PoseStamped motionToGeometryPose( const RESULT_OF_MOTION &motion );
//
tf::Transform motionToTf( const RESULT_OF_MOTION &motion );
//
gtsam::Pose3 motionToPose3( RESULT_OF_MOTION &motion);


int bruteForceSearchORB(const uint64_t* v, const uint64_t* search_array, const unsigned int& size, int& result_index);
int bruteForceSearchORB(const uint64_t* v, const std::map<int, KeyPoint*> &keypoints_list,
                        const std::map<int, gtsam::Point3> &predicted_keypoints, int& result_index);

std::string timeToStr();

// Runtimes
struct Runtime{

    Runtime()
        : key_frame(false)
        , frame(0)
        , tracking(0)
        , mapping(0)
        , total(0) {}

    Runtime( bool _key_frame, double _frame, double _track, double _map, double _total )
        : key_frame(_key_frame)
        , frame(_frame)
        , tracking(_track)
        , mapping(_map)
        , total(_total) {}

    void operator += ( const Runtime &other )
    {
        this->frame += other.frame;
        this->tracking += other.tracking;
        this->mapping += other.mapping;
        this->total += other.total;
    }

    void operator -= ( const Runtime &other )
    {
        this->frame -= other.frame;
        this->tracking -= other.tracking;
        this->mapping -= other.mapping;
        this->total -= other.total;
    }

    void operator /= ( const double value )
    {
        this->frame /= value;
        this->tracking /= value;
        this->mapping /= value;
        this->total /= value;
    }

    void getMaximum( const Runtime &other )
    {
        this->frame = std::max( this->frame, other.frame);
        this->tracking = std::max( this->tracking, other.tracking);
        this->mapping = std::max( this->mapping, other.mapping);
        this->total = std::max( this->total, other.total);
    }

    void getMinimum( const Runtime &other )
    {
        this->frame = std::min( this->frame, other.frame);
        this->tracking = std::min( this->tracking, other.tracking);
        this->mapping = std::min( this->mapping, other.mapping);
        this->total = std::min( this->total, other.total);
    }

//    Runtime operator + ( const Runtime& left, const Runtime& right)
//    {
//        Runtime rt;
//        rt.frame = left.frame + right.frame;
//        rt.tracking = left.tracking + right.tracking;
//        rt.mapping = left.mapping + right.mapping;
//        rt.total = left.total + right.total;
//        return rt;
//    }

    bool key_frame;
    double frame;
    double tracking;
    double mapping;
    double total;
};

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
