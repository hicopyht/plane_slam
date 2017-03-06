#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>

#include <Eigen/Eigen>

#define LINESIZE 81920
using namespace std;

void quaternionToRPY(const tf::Quaternion &q, double &roll, double &pitch, double &yaw)
{
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
}

void addPoseOffset(geometry_msgs::PoseStamped &pose)
{
    static tf::Transform offset(tf::Quaternion(-0.5,0.5,-0.5,0.5), tf::Vector3(-0.144,-0.009,0.714));
    tf::Transform tr(tf::Quaternion(pose.pose.orientation.x, pose.pose.orientation.y,
                                    pose.pose.orientation.z, pose.pose.orientation.w),
                     tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
    //
    double roll, pitch, yaw;
    quaternionToRPY(tr.getRotation(), roll, pitch, yaw);
    tr.setOrigin(tf::Vector3(pose.pose.position.x, 0, pose.pose.position.z));
    tr.setRotation(tf::createQuaternionFromRPY(0, pitch, 0));
    //
    tf::Transform trans = offset * tr;
    pose.pose.position.x = trans.getOrigin().x();
    pose.pose.position.y = trans.getOrigin().y();
    pose.pose.position.z = trans.getOrigin().z();
    tf::quaternionTFToMsg(trans.getRotation(), pose.pose.orientation);
}

nav_msgs::Path readPathFile( const std::string &filename)
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
        double seq;
        double x, y, z, qx, qy, qz, qw;
        ss >> seq >> x >> y >> z >> qx >> qy >> qz >> qw;
        // Push
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        pose.pose.orientation.x = qx;
        pose.pose.orientation.y = qy;
        pose.pose.orientation.z = qz;
        pose.pose.orientation.w = qw;
        //
//        addPoseOffset(pose);
        //
        path.poses.push_back( pose );
    }

    return path;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_publisher", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    if( argc < 3)
    {
        cout << "Usage: ./path_publisher pathfile topicname  <latch> <cycle>" << endl;
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

    // Read Path file
    ROS_INFO("Reading path file: %s ...", filename.c_str() );
    nav_msgs::Path path = readPathFile( filename );
    ROS_INFO("Poses = %d.", path.poses.size() );

    // Publisher
    ros::Publisher path_publisher_ = nh.advertise<nav_msgs::Path>(topicname, 4, latched);
    path.header.frame_id = "/map";
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
