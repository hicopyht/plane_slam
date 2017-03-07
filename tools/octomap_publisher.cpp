#include <ros/ros.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <stdio.h>

using namespace std;
using namespace octomap;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "octomap_publisher", ros::init_options::AnonymousName);
    ros::NodeHandle node_handle;

    if(argc < 2)
    {
        cout << endl << "Usage: ./octomap_publisher  file.bt or file.ot" << endl;
        exit(-1);
    }

    // Load octomap
    std::string filename = argv[1];
    AbstractOcTree* tree = OcTree::read(filename);
    OcTree* octree = dynamic_cast<OcTree*>(tree);

    cout << "Read octomap, size = " << octree->size() << endl;

    // Publish octomap
    ros::Publisher octomap_publisher = node_handle.advertise<octomap_msgs::Octomap>("octomap", 4, true);
    octomap_msgs::Octomap msg;
    octomap_msgs::fullMapToMsg( *octree, msg);
    msg.header.frame_id = "/map";
    msg.header.stamp = ros::Time::now();
    octomap_publisher.publish( msg );

    //
    cout << "Publisher octomap to topic: " << octomap_publisher.getTopic() << endl;

    ros::spin();
}
