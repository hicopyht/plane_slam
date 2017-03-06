#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <stdlib.h>

using namespace std;

typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointCloud< PointType > PointCloudType;
typedef PointCloudType::Ptr PointCloudTypePtr;
typedef PointCloudType::ConstPtr PointCloudTypeConstPtr;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_publisher", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    if( argc < 3)
    {
        cout << "Usage: ./pcd_publisher filename topicname <latch> <cycle>" << endl;
        ROS_ERROR_STREAM("Must define the pcd file.");
        exit(0);
    }
    std::string filename = argv[1];
    std::string topicname = argv[2];
    bool latched = true;
    if( argc >= 4 )
    {
        std::string bool_str = argv[3];
        latched = !bool_str.compare("true");  // latch mode ?
    }
    double cycle = 0;
    if( argc >= 5 )
        cycle = atof(argv[4]);

    // Read PCD file
    ROS_INFO("Reading pcd file: %s ...", filename.c_str() );
    PointCloudTypePtr cloud( new PointCloudType );
    if( pcl::io::loadPCDFile<PointType>(filename, *cloud) == -1)
    {
        ROS_ERROR_STREAM("Failed to load pcd file, exit.");
        exit(1);
    }

    // Publisher
    ros::Publisher cloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>(topicname, 4, latched);


//    // Only xyz channel
//    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    for(int i = 0; i < cloud->points.size(); i++)
//    {
//        PointType &pt = cloud->points[i];
//        pcl::PointXYZ xyz;
//        xyz.x = pt.x;
//        xyz.y = pt.y;
//        xyz.z = pt.z;
//        xyz_cloud->points.push_back(xyz);
//    }
//    xyz_cloud->height = 1;
//    xyz_cloud->width = xyz_cloud->points.size();
//    xyz_cloud->is_dense = cloud->is_dense;

    // Message conversion
    sensor_msgs::PointCloud2 cloud2;
    pcl::toROSMsg( *cloud, cloud2);
//    pcl::toROSMsg(*xyz_cloud, cloud2);
    cloud2.header.frame_id = "/map";
    cloud2.header.stamp = ros::Time::now();
    cloud2.is_dense = false;

    cloud_publisher_.publish( cloud2 );

    ROS_INFO_STREAM("Publish pcd file as " << cloud_publisher_.getTopic().c_str() << ", latch = "
                    << (latched?"true":"false") << ", cycle = " << cycle << "s." );

    // Publish in loop
    if( cycle > 0 )
    {
        ros::Rate loop_rate(1.0/cycle);
        while( ros::ok() )
        {
            cloud2.header.stamp = ros::Time::now();
            cloud_publisher_.publish( cloud2 );
            ROS_INFO_STREAM("Publish once...");
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    ros::spin();
}
