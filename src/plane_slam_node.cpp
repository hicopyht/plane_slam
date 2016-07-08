#include "kinect_listener.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "plane_slam_node");
    plane_slam::KinectListener kl;
    ros::spin();
}
