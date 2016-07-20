#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>

#define LINESIZE 81920
using namespace std;

nav_msgs::Path readPathFile( const std::string &filename )
{
    ifstream is(filename.c_str());
    if( !is )
    {
        ROS_ERROR_STREAM( "Failed to read path file: " << filename << ", exit." );
        exit(1);
    }

    nav_msgs::Path path;
    // load the poses
    while (!is.eof())
    {
        // Read
        double x, y, z, qx, qy, qz, qw;
        is >> x >> y >> z >> qx >> qy >> qz >> qw;
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
        is.ignore(LINESIZE, '\n');
    }

    return path;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_publisher", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    if( argc < 3)
    {
        cout << "Must define the topic name and path file." << endl;
        exit(0);
    }
    std::string topicname = argv[1];
    std::string filename = argv[2];
    bool latched = true;
    if( argc >= 4 )
    {
        std::string bool_str = argv[3];
        latched = !bool_str.compare("true");  // latch mode ?
    }
    double cycle = 0;
    if( argc >= 5 )
        cycle = atof(argv[4]);

    // Read Path file
    ROS_INFO("Reading path file: %s ...", filename.c_str() );
    nav_msgs::Path path = readPathFile( filename );
    ROS_INFO("Poses = %d.", path.poses.size() );

    // Publisher
    ros::Publisher path_publisher_ = nh.advertise<nav_msgs::Path>(topicname, 4, latched);
    path.header.frame_id = "/world";
    path.header.stamp = ros::Time::now();
    path_publisher_.publish( path );

    ROS_INFO_STREAM("Publish path as topic: " << topicname << ", latch = "
                    << (latched?"true":"false") << ", cycle = " << cycle << "s." );

    // Publish in loop
    if( cycle > 0 )
    {
        ros::Rate loop_rate(1.0/cycle);
        while( ros::ok() )
        {
            path.header.stamp = ros::Time::now();
            path_publisher_.publish( path );
            ROS_INFO_STREAM("Publish once...");
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    ros::spin();
}
