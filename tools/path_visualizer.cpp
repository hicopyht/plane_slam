#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>

#define LINESIZE 81920
using namespace std;


nav_msgs::Path readPathFile( const std::string &filename )
{
    ifstream fin(filename.c_str());
    if( !fin )
    {
        ROS_ERROR_STREAM( "Failed to read path file: " << filename << ", exit." );
        exit(1);
    }

    nav_msgs::Path path;
    // load the poses
    std::string line;
    while ( std::getline(fin, line) )
    {
        if( line.size() < 2 || line[0] == '#' )
            continue;

//        if( std::count(line.begin(), line.end(), ' ') < 6 );
//            continue;

        // Read
        stringstream ss(line);
        double x, y, z, qx, qy, qz, qw;
        ss >> x >> y >> z >> qx >> qy >> qz >> qw;
        // Push
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        pose.pose.orientation.x = qx;
        pose.pose.orientation.y = qy;
        pose.pose.orientation.z = qz;
        pose.pose.orientation.w = qw;
        path.poses.push_back( pose );
    }

    return path;
}

void displayPath( const std::vector<geometry_msgs::PoseStamped> &poses,
                  pcl::visualization::PCLVisualizer* visualizer,
                  const std::string &prefix, double r, double g, double b )
{
    for( int i = 1; i < poses.size(); i++)
    {
        const geometry_msgs::PoseStamped &pose1 = poses[i-1];
        const geometry_msgs::PoseStamped &pose2 = poses[i];
        // id
        stringstream ss;
        ss << prefix << "_line_" << i;
        // add line
        pcl::PointXYZ p1(pose1.pose.position.x, pose1.pose.position.y, pose1.pose.position.z),
                p2(pose2.pose.position.x, pose2.pose.position.y, pose2.pose.position.z);
        visualizer->addLine( p1, p2, r, g, b, ss.str() );
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_visualizer", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    if( argc < 2)
    {
        cout << "Usage: ./path_visualizer path_file1 <path_file2> ..." << endl;
        exit(0);
    }
    std::vector<std::string> pathfiles;
    std::vector<nav_msgs::Path> pathes;
    int idx = 1;
    while(idx<argc){
        pathfiles.push_back( std::string(argv[idx]));
        idx++;
    }

    // Load all pathes
    for( int i = 0; i < pathfiles.size(); i++){
        std::string &filename = pathfiles[i];
        // Read Path file
        cout << " Reading path file: " << filename;
        nav_msgs::Path path = readPathFile( filename );
        pathes.push_back( path );
        cout << ", poses = " << path.poses.size() << endl;
    }


    // PCL Visualizer
    pcl::visualization::PCLVisualizer* pcl_viewer(new pcl::visualization::PCLVisualizer("Map Viewer") );
    pcl_viewer->addCoordinateSystem(0.0001);
    pcl_viewer->initCameraParameters();
//    pcl_viewer->setCameraPosition(0.0, 0.0, -2.4, 0, 0, 0.6, 0, -1, 0);
    // For stairs
    //pcl_viewer->setCameraPosition( 1.5238,3.10527,8.24389,2.5285,-1.16693,1.38802,0.549193,-0.670762,0.498464 );
    pcl_viewer->setCameraPosition(-8.33769,-18.6937,27.1933,-8.40921,-12.0519,4.25967,-0.00491748,0.960514,0.278188);
    // For TUM3 structure near
//    pcl_viewer->setCameraPosition( 3.08438,0.471339,2.87936,0.842887,0.475649,1.95646,-0.380643,0.0150451,0.9246);
    pcl_viewer->setBackgroundColor( 1.0, 1.0, 1.0 );
    pcl_viewer->setShowFPS(true);


    // Add pathes
    ROS_INFO_STREAM("Visualize path files in pcl visualizer and spin viewer.");
    std::vector<geometry_msgs::Point> colors;
    colors.resize(3);
    colors[0].x = 0.0; colors[0].y = 1.0; colors[0].z = 0.0;
    colors[1].x = 0.0; colors[1].y = 0.0; colors[1].z = 1.0;
    colors[2].x = 1.0; colors[2].y = 0.0; colors[2].z = 0.0;
    for( int i = 0; i < pathes.size(); i++)
    {
        nav_msgs::Path &path = pathes[i];
        geometry_msgs::Point &color = colors[i%3];
        stringstream ss;
        ss << "path_" << i;
        displayPath( path.poses, pcl_viewer, ss.str(), color.x, color.y, color.z);
    }

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

