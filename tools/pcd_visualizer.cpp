#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <stdlib.h>

using namespace std;

typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointCloud< PointType > PointCloudType;
typedef PointCloudType::Ptr PointCloudTypePtr;
typedef PointCloudType::ConstPtr PointCloudTypeConstPtr;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_visualizer", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    if( argc < 2)
    {
        cout << "Must define the pcd file." << endl;
        exit(0);
    }
    std::string filename = argv[1];

    // Read PCD file
    ROS_INFO("Reading pcd file: %s ...", filename.c_str() );
    PointCloudTypePtr cloud( new PointCloudType );
    if( pcl::io::loadPCDFile<PointType>(filename, *cloud) == -1)
    {
        ROS_ERROR("Failed to load pcd file, exit.");
        exit(1);
    }

    // PCL Visualizer
    pcl::visualization::PCLVisualizer* pcl_viewer(new pcl::visualization::PCLVisualizer("PCD Viewer") );
    pcl_viewer->addCoordinateSystem(0.5);
    pcl_viewer->initCameraParameters();
//    pcl_viewer->setCameraPosition(0.0, 0.0, -2.4, 0, 0, 0.6, 0, -1, 0);
    pcl_viewer->setCameraPosition( 0, 3.0, 3.0, -3.0, 0, 0, -1, -1, 0 );
    pcl_viewer->setBackgroundColor( 1.0, 1.0, 1.0 );
    pcl_viewer->setShowFPS(true);

    // Add PointCloud
    ROS_INFO_STREAM("Visualize pcd file in pcl visualizer and spin viewer." );
    pcl_viewer->addPointCloud( cloud, "pcd_cloud" );
    pcl_viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "pcd_cloud" );

    pcl_viewer->spin();

//    // Spin in loop
//    ros::Rate loop_rate( 20 );
//    while( ros::ok() && !pcl_viewer->wasStopped() )
//    {
//        pcl_viewer->spinOnce();
//        loop_rate.sleep();
//    }

    ros::shutdown();
}
