#ifndef UTILS_H
#define UTILS_H

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_representation.h>

#include <Eigen/Core>
#include <stdio.h>
#include <vector>

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

struct Frame
{
    cv::Mat visual_image;
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
    std::vector<int> inlier;
    std::vector<int> boundary_inlier;
    std::vector<int> hull_inlier;
    PointCloudTypePtr cloud;
    PointCloudTypePtr cloud_boundary;
    PointCloudTypePtr cloud_hull;

    PlaneType() : cloud( new PointCloudType), cloud_boundary( new PointCloudType), cloud_hull( new PointCloudType) {}
};

struct PlanePair
{
    int iobs;
    int ilm;

    PlanePair() : iobs(-1), ilm(-1) {}
    PlanePair(int _iobs, int _ilm) : iobs(_iobs), ilm(_ilm) {}
};

void matrixTF2Eigen(const tf::Matrix3x3 &t, Eigen::Matrix3d &e);
Eigen::Matrix3d matrixTF2Eigen(const tf::Matrix3x3 &t);
void matrixEigen2TF( const Eigen::Matrix3d &e, tf::Matrix3x3 &t);
tf::Matrix3x3 matrixEigen2TF(const Eigen::Matrix3d &m33);

PointType transformPoint (const PointType &point,
                     const Eigen::Matrix4d &transform);

void transformPointCloud (const PointCloudType &cloud_in,
                          PointCloudType &cloud_out,
                          const Eigen::Matrix4d &transform);


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
