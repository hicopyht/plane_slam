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
    ros::init(argc, argv, "map_visualizer", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    if( argc < 3)
    {
        cout <<"Usage: ./map_visualizer plane.pcd keypoint.pcd" << endl;
        ROS_ERROR_STREAM("Must define plane&point pcd files.");
        exit(0);
    }
    std::string plane_file = argv[1];
    std::string keypoint_file = argv[2];
    double plane_point_size = 1.0;
    double keypoint_point_size = 4.0;


    // Read PCD file
    ROS_INFO("Reading plane&keypoint pcd files: %s and %s ...", plane_file.c_str(), keypoint_file.c_str() );
    PointCloudTypePtr plane_cloud( new PointCloudType ), keypoint_cloud( new PointCloudType );
    if( pcl::io::loadPCDFile<PointType>(plane_file, *plane_cloud) == -1)
    {
        ROS_ERROR_STREAM("Failed to load plane pcd file: " << plane_file << ", exit.");
        exit(1);
    }
    if( pcl::io::loadPCDFile<PointType>(keypoint_file, *keypoint_cloud) == -1)
    {
        ROS_ERROR_STREAM("Failed to load keypoint pcd file: " << keypoint_file << ", exit.");
        exit(1);
    }

    // PCL Visualizer
    pcl::visualization::PCLVisualizer* pcl_viewer(new pcl::visualization::PCLVisualizer("Map Viewer") );
    pcl_viewer->addCoordinateSystem(0.0001);
    pcl_viewer->initCameraParameters();
//    pcl_viewer->setCameraPosition(0.0, 0.0, -2.4, 0, 0, 0.6, 0, -1, 0);
    // For stairs
    pcl_viewer->setCameraPosition( 1.5238,3.10527,8.24389,2.5285,-1.16693,1.38802,0.549193,-0.670762,0.498464 );
    // For TUM3 structure near
//    pcl_viewer->setCameraPosition( 3.08438,0.471339,2.87936,0.842887,0.475649,1.95646,-0.380643,0.0150451,0.9246);
    pcl_viewer->setBackgroundColor( 1.0, 1.0, 1.0 );
    pcl_viewer->setShowFPS(true);

    // Add PointCloud
    ROS_INFO_STREAM("Visualize plane&keypoin pcd files in pcl visualizer and spin viewer, point size = " << plane_point_size << " & " << keypoint_point_size );
    pcl_viewer->addPointCloud( plane_cloud, "plane_cloud" );
    pcl_viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, plane_point_size, "plane_cloud" );
    pcl_viewer->addPointCloud( keypoint_cloud, "keypoint_cloud" );
    pcl_viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, keypoint_point_size, "keypoint_cloud" );
//    pcl_viewer->spin();

    // Spin in loop
    ros::Rate loop_rate( 10 );
    while( ros::ok() && !pcl_viewer->wasStopped() )
    {
        pcl_viewer->spinOnce(10);
        ros::spinOnce();
        loop_rate.sleep();
    }
    pcl_viewer->close();

    ros::shutdown();
}

