#include "kinect_listener.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "plane_slam_node");
    KinectListener kl;
    ros::spin();
}
