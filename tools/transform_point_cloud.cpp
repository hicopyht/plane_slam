#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <stdlib.h>

using namespace std;

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud< PointType > PointCloudType;
typedef PointCloudType::Ptr PointCloudTypePtr;
typedef PointCloudType::ConstPtr PointCloudTypeConstPtr;

void transformTFToMatrix4d(const tf::Transform &t, Eigen::Matrix4d &e)
{
    // Translation
    e(0,3) = t.getOrigin().x();
    e(1,3) = t.getOrigin().y();
    e(2,3) = t.getOrigin().z();
    // Rotation matrix
    e(0,0) = t.getBasis()[0][0];
    e(0,1) = t.getBasis()[0][1];
    e(0,2) = t.getBasis()[0][2];
    e(1,0) = t.getBasis()[1][0];
    e(1,1) = t.getBasis()[1][1];
    e(1,2) = t.getBasis()[1][2];
    e(2,0) = t.getBasis()[2][0];
    e(2,1) = t.getBasis()[2][1];
    e(2,2) = t.getBasis()[2][2];
    // Identity
    e(3,0) = 0;
    e(3,1) = 0;
    e(3,2) = 0;
    e(3,3) = 1;
}

void transformPointCloud (const PointCloudType &cloud_in,
                          PointCloudType &cloud_out,
                          const Eigen::Matrix4d &transform)
{
    if (&cloud_in != &cloud_out)
    {
        // Note: could be replaced by cloud_out = cloud_in
        cloud_out.header   = cloud_in.header;
        cloud_out.is_dense = cloud_in.is_dense;
        cloud_out.width    = cloud_in.width;
        cloud_out.height   = cloud_in.height;
        cloud_out.points.reserve (cloud_out.points.size ());
        cloud_out.points.assign (cloud_in.points.begin (), cloud_in.points.end ());
        cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
        cloud_out.sensor_origin_      = cloud_in.sensor_origin_;
    }

    if (cloud_in.is_dense)
    {
        // If the dataset is dense, simply transform it!
        for (size_t i = 0; i < cloud_out.points.size (); ++i)
        {
            //cloud_out.points[i].getVector3fMap () = transform * cloud_in.points[i].getVector3fMap ();
            Eigen::Matrix<double, 3, 1> pt (cloud_in[i].x, cloud_in[i].y, cloud_in[i].z);
            cloud_out[i].x = static_cast<float> (transform (0, 0) * pt.coeffRef (0) + transform (0, 1) * pt.coeffRef (1) + transform (0, 2) * pt.coeffRef (2) + transform (0, 3));
            cloud_out[i].y = static_cast<float> (transform (1, 0) * pt.coeffRef (0) + transform (1, 1) * pt.coeffRef (1) + transform (1, 2) * pt.coeffRef (2) + transform (1, 3));
            cloud_out[i].z = static_cast<float> (transform (2, 0) * pt.coeffRef (0) + transform (2, 1) * pt.coeffRef (1) + transform (2, 2) * pt.coeffRef (2) + transform (2, 3));
        }
    }
    else
    {
        // Dataset might contain NaNs and Infs, so check for them first,
        // otherwise we get errors during the multiplication (?)
        for (size_t i = 0; i < cloud_out.points.size (); ++i)
        {
            if (!pcl_isfinite (cloud_in.points[i].x) || !pcl_isfinite (cloud_in.points[i].y) ||
                !pcl_isfinite (cloud_in.points[i].z))
                continue;
            //cloud_out.points[i].getVector3fMap () = transform * cloud_in.points[i].getVector3fMap ();
            Eigen::Matrix<double, 3, 1> pt (cloud_in[i].x, cloud_in[i].y, cloud_in[i].z);
            cloud_out[i].x = static_cast<float> (transform (0, 0) * pt.coeffRef (0) + transform (0, 1) * pt.coeffRef (1) + transform (0, 2) * pt.coeffRef (2) + transform (0, 3));
            cloud_out[i].y = static_cast<float> (transform (1, 0) * pt.coeffRef (0) + transform (1, 1) * pt.coeffRef (1) + transform (1, 2) * pt.coeffRef (2) + transform (1, 3));
            cloud_out[i].z = static_cast<float> (transform (2, 0) * pt.coeffRef (0) + transform (2, 1) * pt.coeffRef (1) + transform (2, 2) * pt.coeffRef (2) + transform (2, 3));
        }
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "transform_point_cloud", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    if( argc < 9)
    {
        cout << "Usage: ./transform_point_cloud infile.pcd outfile.pcd x y z roll pitch yaw" << endl;
        ROS_ERROR_STREAM("Must define the pcd file.");
        exit(0);
    }
    std::string infile = argv[1];
    std::string outfile = argv[2];
    float x = atof(argv[3]);
    float y = atof(argv[4]);
    float z = atof(argv[5]);
    float roll = atof(argv[6]);
    float pitch = atof(argv[7]);
    float yaw = atof(argv[8]);


    // Read PCD file
    ROS_INFO("Read pcd file: %s ...", infile.c_str() );
    PointCloudTypePtr cloud( new PointCloudType );
    if( pcl::io::loadPCDFile<PointType>(infile, *cloud) == -1)
    {
        ROS_ERROR_STREAM("Failed to load pcd file, exit.");
        exit(1);
    }

    // Compute transform
    tf::Transform robot(tf::Quaternion(-0.5, 0.5, -0.5, 0.5),
                               tf::Vector3(-0.144, -0.009, 0.714));
    tf::Transform trans(tf::createQuaternionFromRPY(roll, pitch, yaw),
                        tf::Vector3(x, y, z));
    Eigen::Matrix4d m4;
    transformTFToMatrix4d(trans*robot, m4);

    // Do transform
    ROS_INFO("Transform point cloud, xyz = (%f, %f, %f), rpy = (%f, %f, %f)",
             x, y, z, roll, pitch, yaw);
    PointCloudTypePtr cloud_transformed( new PointCloudType );
    transformPointCloud(*cloud, *cloud_transformed, m4);

    // Save result
    pcl::io::savePCDFileASCII(outfile, *cloud_transformed);
    ROS_INFO("Save pcd file: %s ...", outfile.c_str() );
    ROS_INFO("Done.");

    ros::shutdown();
}

