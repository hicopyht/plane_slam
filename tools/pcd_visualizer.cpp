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
        cout <<"Usage: ./pcd_visualizer pcd_file <point_size>" << endl;
        ROS_ERROR_STREAM("Must define the pcd file.");
        exit(0);
    }
    std::string filename = argv[1];
    double point_size = 1.0;
    if( argc >= 3 )
    {
        point_size = atof(argv[2]);
    }


    // Read PCD file
    ROS_INFO("Reading pcd file: %s ...", filename.c_str() );
    PointCloudTypePtr cloud( new PointCloudType );
    if( pcl::io::loadPCDFile<PointType>(filename, *cloud) == -1)
    {
        ROS_ERROR_STREAM("Failed to load pcd file, exit.");
        exit(1);
    }

    // PCL Visualizer
    pcl::visualization::PCLVisualizer* pcl_viewer(new pcl::visualization::PCLVisualizer("PCD Viewer") );
    pcl_viewer->addCoordinateSystem(0.1);
    pcl_viewer->initCameraParameters();
//    pcl_viewer->setCameraPosition(0.0, 0.0, -2.4, 0, 0, 0.6, 0, -1, 0);
    // For stairs
//    pcl_viewer->setCameraPosition( -1.37337, 2.82294, 6.15235, 1.95741, -0.600154, 2.06345, 0.59628, -0.306218, 0.742079);
    // For TUM3 structure near
    pcl_viewer->setCameraPosition( 0.971344,0.475402,2.00935, 0.842887,0.475649,1.95646, -0.380643,0.0150451,0.9246);
    pcl_viewer->setBackgroundColor( 1.0, 1.0, 1.0 );
    pcl_viewer->setShowFPS(true);

    // Add PointCloud
    ROS_INFO_STREAM("Visualize pcd file in pcl visualizer and spin viewer, point size = " << point_size << "." );
    pcl_viewer->addPointCloud( cloud, "pcd_cloud" );
    pcl_viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "pcd_cloud" );

    pcl_viewer->spin();

//    // Spin in loop
//    ros::Rate loop_rate( 20 );
//    while( ros::ok() && !pcl_viewer->wasStopped() )
//    {
//        pcl_viewer->spinOnce(40);
//        ros::spinOnce();
//        loop_rate.sleep();
//    }
//    pcl_viewer->close();

    ros::shutdown();
}
