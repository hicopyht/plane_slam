#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <cstdio>
#include <vector>
#include <map>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/features/feature.h>

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloudType;
typedef PointCloudType::Ptr PointCloudTypePtr;
typedef Eigen::Vector4f ModelCoefficients;


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

tf::Transform poseToTransform(geometry_msgs::Pose &pose)
{
    return tf::Transform(tf::Quaternion(pose.orientation.x, pose.orientation.y,
                                        pose.orientation.z, pose.orientation.w),
                         tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
}

geometry_msgs::Pose transformToPose(tf::Transform &trans)
{
    geometry_msgs::Pose pose;
    pose.position.x = trans.getOrigin().x();
    pose.position.y = trans.getOrigin().y();
    pose.position.z = trans.getOrigin().z();
    tf::quaternionTFToMsg(trans.getRotation(), pose.orientation);
    return pose;
}

bool readPathFile( const std::string &filename, std::map<int,geometry_msgs::PoseStamped> &poses )
{
    ifstream fin(filename.c_str());
    if( !fin )
    {
        ROS_ERROR_STREAM( "Failed to read path file: " << filename << ", exit." );
        exit(1);
    }

    int seq = 0;

    // load the poses
    std::string line;
    while ( std::getline(fin, line) )
    {
        if( line.size() < 2 || line[0] == '#' )
            continue;

        // Read
        stringstream ss(line);

        double stamp, x, y, z, qx, qy, qz, qw;
        ss >> stamp >> x >> y >> z >> qx >> qy >> qz >> qw;
        // Push
        geometry_msgs::PoseStamped ps;
        ps.pose.position.x = x;
        ps.pose.position.y = y;
        ps.pose.position.z = z;
        ps.pose.orientation.x = qx;
        ps.pose.orientation.y = qy;
        ps.pose.orientation.z = qz;
        ps.pose.orientation.w = qw;
        //
        ps.header.stamp.fromSec(stamp);
        ps.header.seq = seq;
        poses[seq] = ps;
        seq++;
    }

    return true;
}


bool readPathFileSeq( const std::string &filename, std::map<int,geometry_msgs::PoseStamped> &poses )
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
        geometry_msgs::PoseStamped ps;
        ps.pose.position.x = x;
        ps.pose.position.y = y;
        ps.pose.position.z = z;
        ps.pose.orientation.x = qx;
        ps.pose.orientation.y = qy;
        ps.pose.orientation.z = qz;
        ps.pose.orientation.w = qw;
        //
        ps.header.seq = seq;
        poses[seq] = ps;
    }

    return true;
}


void savePathFile(const std::string &filename, std::map<int,geometry_msgs::PoseStamped> &poses)
{
    FILE* yaml = std::fopen( filename.c_str(), "w" );
    fprintf( yaml, "# %s\n", filename.c_str() );
    fprintf( yaml, "# pose format: seq T(xyz) Q(xyzw)\n" );
    fprintf( yaml, "# size: %d\n", (int)(poses.size()) );
    // Save Data
    for(std::map<int,geometry_msgs::PoseStamped>::iterator it = poses.begin(), end = poses.end();
        it != end; it++)
    {
        geometry_msgs::PoseStamped ps = it->second;
        fprintf( yaml, "%d %f %f %f %f %f %f %f \n", ps.header.seq, ps.pose.position.x,
                 ps.pose.position.y, ps.pose.position.z, ps.pose.orientation.x,
                 ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w);
    }

    // close
    fclose(yaml);
}


PointCloudTypePtr createPointCloudFromPath(std::map<int,geometry_msgs::PoseStamped> &poses)
{
    PointCloudTypePtr cloud(new PointCloudType);
    PointType pt;
    for(std::map<int,geometry_msgs::PoseStamped>::iterator it = poses.begin(), end = poses.end(); it != end; it++)
    {
        geometry_msgs::PoseStamped pose = it->second;
        pt.x = pose.pose.position.x;
        pt.y = pose.pose.position.y;
        pt.z = pose.pose.position.z;
        cloud->points.push_back(pt);
    }
    cloud->height = 1;
    cloud->width = cloud->points.size();

    return cloud;
}

void computeModelCoefficients(PointCloudTypePtr &cloud, ModelCoefficients &coefficients, Eigen::Vector4f &centroid)
{
    float curvature;
    //
    EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
    Eigen::Vector4f xyz_centroid;
    pcl::computeMeanAndCovarianceMatrix (*cloud, covariance_matrix, centroid);

    // Get the plane normal and surface curvature
    pcl::solvePlaneParameters (covariance_matrix, centroid, coefficients, curvature);
    //
    if(coefficients[1] > 0)
    {
        coefficients[0] = -1.0 * coefficients[0];
        coefficients[1] = -1.0 * coefficients[1];
        coefficients[2] = -1.0 * coefficients[2];
    }
    coefficients[3] = 0;
    coefficients[3] = -1 * coefficients.dot (xyz_centroid);
}

void publishPath(ros::Publisher &publisher, std::map<int,geometry_msgs::PoseStamped> &poses)
{
    nav_msgs::Path msg;
    msg.header.frame_id = "/map";
    msg.header.stamp = ros::Time::now();

    for(std::map<int,geometry_msgs::PoseStamped>::iterator it = poses.begin(), end = poses.end(); it != end; it++)
    {
        msg.poses.push_back(it->second);
    }
    publisher.publish(msg);
}

void addDeltaPoses(std::map<int,geometry_msgs::PoseStamped> &inposes,
                   tf::Transform &delta,
                   std::map<int,geometry_msgs::PoseStamped> &outposes)
{
//    double roll, pitch, yaw;
//    tf::Matrix3x3 m33 (delta.getRotation());
//    m33.getRPY(roll, pitch, yaw);
//    cout << BOLDWHITE << " delta tf: " << CYAN << delta.getOrigin().x()
//         << " " << delta.getOrigin().y() << " " << delta.getOrigin().z()
//         << " " << roll << " " << pitch << " " << yaw << RESET << endl;

    for(std::map<int,geometry_msgs::PoseStamped>::iterator it = inposes.begin(), end = inposes.end();
        it != end; it++)
    {
        int seq = it->first;
        geometry_msgs::PoseStamped pose = it->second;
        //
        tf::Transform trans = poseToTransform(pose.pose);
        tf::Transform rt = delta*trans;
        geometry_msgs::PoseStamped ps;
        ps.pose = transformToPose(rt);
        ps.header = pose.header;

        //
        outposes.insert(std::pair<int,geometry_msgs::PoseStamped>(seq, ps));
    }
}

void printCentroidCoefficients(std::map<int,geometry_msgs::PoseStamped> &poses,
                               std::string &prefix)
{
    // Build point cloud
    PointCloudTypePtr cloud = createPointCloudFromPath(poses);
    cout << BOLDWHITE << "Build cloud from " << MAGENTA << prefix << RESET << endl;
    //
    ModelCoefficients coefficients;
    Eigen::Vector4f centroid;
    computeModelCoefficients(cloud, coefficients, centroid);
    cout << BOLDWHITE << " - Coefficients: " << BLUE << coefficients[0] << " " << coefficients[1]
         << " " << coefficients[2] << " " << coefficients[3] << RESET << endl;
    cout << BOLDWHITE << " - Centroid: " << BLUE << centroid[0]
         << " " << centroid[1] << " " << centroid[2] << RESET << endl;
}

void addPairs(std::map<int,geometry_msgs::PoseStamped> &poses1,
              std::map<int,geometry_msgs::PoseStamped> &poses2,
              ros::Publisher &marker_publisher)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "pair_lines";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.scale.x = 0.02;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    for(std::map<int,geometry_msgs::PoseStamped>::iterator it = poses1.begin(), end = poses1.end();
        it != end; it++)
    {
        geometry_msgs::PoseStamped pose1 = it->second;
        geometry_msgs::PoseStamped pose2 = poses2[it->first];
        //
        marker.points.push_back(pose1.pose.position);
        marker.points.push_back(pose2.pose.position);
    }

    // Publish
    marker_publisher.publish(marker);
}

double calculateRMSE(std::map<int,geometry_msgs::PoseStamped> &poses1,
                     std::map<int,geometry_msgs::PoseStamped> &poses2)
{
    double ddt = 0;
    double dda = 0;

    for(std::map<int,geometry_msgs::PoseStamped>::iterator it = poses1.begin(), end = poses1.end();
        it != end; it++)
    {
        geometry_msgs::PoseStamped pose1 = it->second;
        geometry_msgs::PoseStamped pose2 = poses2[it->first];

        //
        double dx = pose1.pose.position.x - pose2.pose.position.x;
        double dy = pose1.pose.position.y - pose2.pose.position.y;
        double dz = pose1.pose.position.z - pose2.pose.position.z;
        ddt += dx*dx + dy*dy + dz*dz;

        //
        tf::Transform tr1 = poseToTransform(pose1.pose);
        tf::Transform tr2 = poseToTransform(pose2.pose);
        tf::Transform dq = tr1.inverse() * tr2;
        double da = dq.getRotation().getAngle();
        dda += da*da;
    }

    double rmse = sqrt(ddt / poses1.size());
    double rot_rmse = sqrt(dda / poses1.size());
    cout << BOLDWHITE << "RMSE trans = " << CYAN << rmse
         << BOLDWHITE << ", rot = " << CYAN << (rot_rmse * 180.0 / M_PI) << RESET << endl;
    return rmse;
}


void assignPathOnce(std::map<int,geometry_msgs::PoseStamped> optimized_poses,
                    std::map<int,geometry_msgs::PoseStamped> reference_poses,
                    tf::Transform &delta,
                    ros::Publisher &optimized_path_publisher,
                    ros::Publisher &orb_path_publisher,
                    ros::Publisher &marker_publisher,
                    std::string &optimized_pathfile,
                    std::string &reference_pathfile,
                    std::string &save_pathfile)
{
    cout << "Delta: " << delta.getOrigin().x() << " " << delta.getOrigin().y()
         << " " << delta.getOrigin().z() << " " << delta.getRotation().x()
         << " " << delta.getRotation().y() << " " << delta.getRotation().z() << " " << delta.getRotation().w() << endl;

    // Add delta to poses
    std::map<int,geometry_msgs::PoseStamped> modified_poses;
    addDeltaPoses(reference_poses, delta, modified_poses);

    // Save poses
    if(!save_pathfile.empty())
        savePathFile(save_pathfile, modified_poses);

    // Add pairs' lines
//    cout << BOLDWHITE << "Add pairs = " << GREEN << modified_poses.size() << RESET << endl;
    addPairs(optimized_poses, modified_poses, marker_publisher);

    // Calculate error
    calculateRMSE(optimized_poses, modified_poses);

    // Publish
//    cout << BOLDWHITE << "Publish two pathes." << RESET << endl;
    publishPath(optimized_path_publisher, optimized_poses);
    publishPath(orb_path_publisher, modified_poses);

    // Compute model coefficients
    printCentroidCoefficients(optimized_poses, optimized_pathfile);
    printCentroidCoefficients(modified_poses, reference_pathfile);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "assign_path");
    ros::NodeHandle node_handle;

    ros::Publisher optimized_path_publisher = node_handle.advertise<nav_msgs::Path>("optimized_path", 4, true);
    ros::Publisher orb_path_publisher = node_handle.advertise<nav_msgs::Path>("orb_path", 4, true);
    ros::Publisher marker_publisher = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 4, true);

    std::string optimized_pathfile;
    std::string reference_pathfile;
    std::string save_pathfile = "";

    if(argc < 3)
    {
        cout << "Usage: ./assign_path optimized reference" << endl;
        exit(1);
    }

    if(argc >= 4)
        save_pathfile = argv[3];

    optimized_pathfile = argv[1];
    reference_pathfile = argv[2];

    std::map<int,geometry_msgs::PoseStamped> optimized_poses;
    std::map<int,geometry_msgs::PoseStamped> reference_poses;


    // Read path file
    if(!readPathFileSeq(optimized_pathfile, optimized_poses) || !readPathFileSeq(reference_pathfile, reference_poses))
        exit(1);
    cout << endl << BOLDWHITE << "Read " << optimized_pathfile << ", size = " << CYAN << optimized_poses.size() << RESET << endl;
    cout << endl << BOLDWHITE << "Read " << reference_pathfile << ", size = " << CYAN << reference_poses.size() << RESET << endl;

    // Publish pathes
    publishPath(optimized_path_publisher, optimized_poses);
    publishPath(orb_path_publisher, reference_poses);

    // Read parameters
    double dx, dy, dz, droll, dpitch, dyaw;
    node_handle.param<double>("dx", dx, 0.0);
    node_handle.param<double>("dy", dy, 0.0);
    node_handle.param<double>("dz", dz, 0.0);
    node_handle.param<double>("droll", droll, 0.0);
    node_handle.param<double>("dpitch", dpitch, 0.0);
    node_handle.param<double>("dyaw", dyaw, 0.0);

    // Set default parameters
    node_handle.setParam("dx", dx);
    node_handle.setParam("dy", dy);
    node_handle.setParam("dz", dz);
    node_handle.setParam("droll", droll);
    node_handle.setParam("dpitch", dpitch);
    node_handle.setParam("dyaw", dyaw);
    //
    //
    double ddx = dx, ddy = dy, ddz = dz, ddroll = droll, ddpitch = dpitch, ddyaw = dyaw;

    // Assign once
    {
        //
        cout << RED << endl << "/********************************************************/" << RESET << endl;
        cout << BOLDWHITE << "Assign once, delta = " << YELLOW << ddx << " " << ddy << " " << ddz
             << " " << ddroll << " " << ddpitch << " " << ddyaw << RESET << endl;
        //
        tf::Transform delta(tf::createQuaternionFromRPY(ddroll, ddpitch, ddyaw), tf::Vector3(ddx, ddy, ddz));
        assignPathOnce(optimized_poses, reference_poses, delta,
                       optimized_path_publisher, orb_path_publisher, marker_publisher,
                       optimized_pathfile, reference_pathfile, save_pathfile);
    }


    ros::Rate loop_rate(5.0);
    while(ros::ok())
    {
        //
        node_handle.getParam("dx", ddx);
        node_handle.getParam("dy", ddy);
        node_handle.getParam("dz", ddz);
        node_handle.getParam("droll", ddroll);
        node_handle.getParam("dpitch", ddpitch);
        node_handle.getParam("dyaw", ddyaw);

        if(dx != ddx || dy != ddy || dz != ddz || droll != ddroll || dpitch != ddpitch || dyaw != ddyaw)
        {
            //
            cout << RED << endl << "/********************************************************/" << RESET << endl;
            cout << BOLDWHITE << "Assign once, delta = " << YELLOW << ddx << " " << ddy << " " << ddz
                 << " " << ddroll << " " << ddpitch << " " << ddyaw << RESET << endl;
            //
            tf::Transform delta(tf::createQuaternionFromRPY(ddroll, ddpitch, ddyaw), tf::Vector3(ddx, ddy, ddz));
            assignPathOnce(optimized_poses, reference_poses, delta,
                           optimized_path_publisher, orb_path_publisher, marker_publisher,
                           optimized_pathfile, reference_pathfile, save_pathfile);

            // Update values
            dx = ddx;
            dy = ddy;
            dz = ddz;
            droll = ddroll;
            dpitch = ddpitch;
            dyaw = ddyaw;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::shutdown();
}




