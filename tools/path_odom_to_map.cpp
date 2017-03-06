#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <cstdio>
#include <vector>
#include <map>


#define LINESIZE 81920
using namespace std;

//the following are UBUNTU/LINUX ONLY terminal color
#define RESET "\033[0m"
#define BLACK "\033[30m" /* Black */
#define RED "\033[31m" /* Red */
#define GREEN "\033[32m" /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m" /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m" /* Cyan */
#define WHITE "\033[37m" /* White */
#define BOLDBLACK "\033[1m\033[30m" /* Bold Black */
#define BOLDRED "\033[1m\033[31m" /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m" /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m" /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m" /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m" /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m" /* Bold White */


bool readPathFile( const std::string &filename, std::map<int,tf::Transform> &poses )
{
    ifstream fin(filename.c_str());
    if( !fin )
    {
        ROS_ERROR_STREAM( "Failed to read path file: " << filename << ", exit." );
        exit(1);
    }

    // load the poses
    std::string line;
    while ( std::getline(fin, line) )
    {
        if( line.size() < 2 || line[0] == '#' )
            continue;

        // Read
        stringstream ss(line);
        int seq;
        double x, y, z, qx, qy, qz, qw;
        ss >> seq >> x >> y >> z >> qx >> qy >> qz >> qw;
        // Push
        tf::Transform pose(tf::Quaternion(qx, qy, qz, qw), tf::Vector3(x, y, z));
        poses[seq] = pose;
    }

    return true;
}

bool readPathFile2( const std::string &filename, std::map<int,tf::Transform> &poses )
{
    ifstream fin(filename.c_str());
    if( !fin )
    {
        ROS_ERROR_STREAM( "Failed to read path file: " << filename << ", exit." );
        exit(1);
    }

    // load the poses
    std::string line;
    while ( std::getline(fin, line) )
    {
        if( line.size() < 2 || line[0] == '#' )
            continue;

        // Read
        stringstream ss(line);
        int seq;
        double x, y, yaw;
        ss >> seq >> x >> y >> yaw;
        // Push
        tf::Transform pose(tf::createQuaternionFromYaw(yaw), tf::Vector3(x, y, 0));
        poses[seq] = pose;
    }

    return true;
}

bool addPoseDelta(std::map<int,tf::Transform> &odom_poses,
                  std::map<int,tf::Transform> &delta_poses,
                  std::map<int,tf::Transform> &map_poses)
{
    for(std::map<int,tf::Transform>::iterator it = odom_poses.begin(), end = odom_poses.end();
        it != end; it++)
    {
        int seq = it->first;
        tf::Transform pose = it->second;

        //
        if(delta_poses.find(seq) == delta_poses.end())
        {
            cout << RED << "[Error]: Cann't find key " << seq << " in delta poses." << RESET << endl;
            exit(1);
        }

        //
        tf::Transform delta = delta_poses[seq];
        tf::Transform pmap = delta * pose;
        map_poses[seq] = pmap;
    }
}

bool savePathFile(const std::string &filename, std::map<int,tf::Transform> &poses)
{
    FILE* yaml = std::fopen( filename.c_str(), "w" );
    fprintf( yaml, "# %s\n", filename.c_str() );
    fprintf( yaml, "# pose format: seq T(xyz) Q(xyzw)\n" );
    fprintf( yaml, "# size: %d\n", (int)(poses.size()) );
    // Save Data
    for(std::map<int,tf::Transform>::iterator it = poses.begin(), end = poses.end();
        it != end; it++)
    {
        int seq = it->first;
        tf::Transform pose = it->second;
        fprintf( yaml, "%d %f %f %f %f %f %f %f \n", seq, pose.getOrigin().x(), pose.getOrigin().y(),
                 pose.getOrigin().z(), pose.getRotation().x(), pose.getRotation().y(),
                 pose.getRotation().z(), pose.getRotation().w());
    }

    // close
    fclose(yaml);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_odom_to_map");
    std::string odom_file;
    std::string map_to_odom_file;
    std::string out_file;

    tf::Quaternion qt(-0.5, 0.5, -0.5, 0.5);
    tf::Matrix3x3 m(qt);
    double r, p, y;
    m.getRPY(r, p, y);
    cout << " RPY: " << r << " " << p << " " << y << endl;

    if(argc < 4)
    {
        cout << "Usage: ./path_odom_to_map odomPathFile mapToOdomFile outFile" << endl;
        exit(1);
    }

    odom_file = argv[1];
    map_to_odom_file = argv[2];
    out_file = argv[3];

    std::map<int,tf::Transform> odom_poses;
    std::map<int,tf::Transform> delta_poses;
    std::map<int,tf::Transform> map_poses;

    if(!readPathFile(odom_file, odom_poses) || !readPathFile2(map_to_odom_file, delta_poses))
    {
        exit(1);
    }

    addPoseDelta(odom_poses, delta_poses, map_poses);
    savePathFile(out_file, map_poses);
}


