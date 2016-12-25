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
        int seq;
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

void saveArray( const std::string &filename, std::vector<double> &array, int start = 0, int end = 1e6 )
{
    FILE* yaml = std::fopen( filename.c_str(), "w" );
    fprintf( yaml, "# %s\n", filename.c_str() );
    fprintf( yaml, "# size: %d\n", array.size() );
    for( int i = start; i < array.size() && i < end; i++)
    {
        fprintf(yaml, "%f\n", array[i]);
    }
    fclose(yaml);
}

double calculateMeanAndRMSE( const nav_msgs::Path &path0, const nav_msgs::Path &path1, int size )
{
    int pose_size = std::min( path0.poses.size(), path1.poses.size() );
    int idx0 = path0.poses.size() - pose_size;
    int idx1 = path1.poses.size() - pose_size;

    cout << " Calculate Mean & RMSE, path0 = " << path0.poses.size()
         << ", path1 = " << path1.poses.size()
         << ", pose size = " << pose_size
         << ", size = " << size << endl;

    pose_size = std::min(pose_size, size);

    std::vector<double> mean_values;
    std::vector<double> rmse_values;
    double sum = 0;
    double sum_square = 0;
    for( int i = 0; i < pose_size; i++, idx0++, idx1++)
    {
        if( idx0 >= path0.poses.size() || idx1 >= path1.poses.size() )
            break;

        const geometry_msgs::PoseStamped &pose1 = path0.poses[idx0];
        const geometry_msgs::PoseStamped &pose2 = path1.poses[idx1];

        //
        double dx, dy, dz;
        dx = pose1.pose.position.x - pose2.pose.position.x;
        dy = pose1.pose.position.y - pose2.pose.position.y;
        dz = pose1.pose.position.z - pose2.pose.position.z;

        //
        double dd = dx*dx + dy*dy + dz*dz;
        sum_square += dd;   //
        double tmp_rmse = sqrt(sum_square/(i+1));
        rmse_values.push_back(tmp_rmse);
        //
        sum += sqrt(dd);    //
        double tmp_mean = sqrt(sum/(i+1));
        mean_values.push_back(tmp_mean);

    }

//    // Print mean
//    cout << " Mean: size = " << mean_values.size() << endl;
//    for( int i = 0; i < mean_values.size(); i++)
//    {
//        cout << " " << mean_values[i];
//    }
//    cout << endl;

//    // Print RMSE
//    cout << " RMSE: size = " << rmse_values.size() << endl;
//    for( int i = 0; i < rmse_values.size(); i++)
//    {
//        cout << " " << rmse_values[i];
//    }
//    cout << endl;

    saveArray( "mean.txt", mean_values );
    saveArray( "rmse.txt", rmse_values );

    return sqrt(sum_square/pose_size);
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
    bool use_pair = false;
    nh.param<bool>("use_pair", use_pair, use_pair );
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
//    pcl_viewer->addCoordinateSystem(1.0);
    pcl_viewer->addCoordinateSystem(0.0001);
    pcl_viewer->initCameraParameters();
//    pcl_viewer->setCameraPosition(0.0, 0.0, -2.4, 0, 0, 0.6, 0, -1, 0);
    // For stairs
    //pcl_viewer->setCameraPosition( 1.5238,3.10527,8.24389,2.5285,-1.16693,1.38802,0.549193,-0.670762,0.498464 );
//    pcl_viewer->setCameraPosition(-8.33769,-18.6937,27.1933,-8.40921,-12.0519,4.25967,-0.00491748,0.960514,0.278188);
    // For TUM3 structure near
//    pcl_viewer->setCameraPosition( 3.08438,0.471339,2.87936,0.842887,0.475649,1.95646,-0.380643,0.0150451,0.9246);
//    pcl_viewer->setCameraPosition( 0.275419,0.199234,3.87505,-0.427779,0.297873,2.7054,-0.854668,0.0425966,0.517424);
    // For TUM3 structure notexture far
//    pcl_viewer->setCameraPosition( -0.117341,-0.946722,-2.4444,-1.59346,-0.128102,2.47,-0.953246,0.0596285,-0.296256);
    // For TUM3 structure texture far
//    pcl_viewer->setCameraPosition( 0.593963,-0.190738,3.65528,-0.0778136,-0.095626,2.46702,-0.86758,0.051606,0.494612);
    // For TUM3 structure notexture near
//    pcl_viewer->setCameraPosition( -1.16653,-0.408754,3.04816,-1.30347,-0.293638,2.43541,-0.97642,-0.0152819,0.21534 );
    // For TUM3 cabinet
    pcl_viewer->setCameraPosition( 0.508128,0.0660203,4.05525,-0.304722,0.181106,2.61745,-0.86758,0.051606,0.494612 );
    pcl_viewer->setBackgroundColor( 1.0, 1.0, 1.0 );
    pcl_viewer->setShowFPS(true);

    // Set camera position
    if( nh.hasParam("camera_position") )
    {
        std::vector<double> pos;
        // Get frames
        std::string pos_string;
        nh.param<string>("camera_position", pos_string, std::string());
        stringstream ss(pos_string);
        std::string element;
        cout << "Camera position:" << endl;
        while( std::getline( ss, element, ',') )
        {
            pos.push_back( atof(element.c_str()) );
            cout << MAGENTA << " " << pos.back();
        }
        cout << RESET << endl;

        if( pos.size() >= 9 ){
            pcl_viewer->setCameraPosition( pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], pos[6], pos[7], pos[8]);
        }
    }

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

    // Add pairs
    if( use_pair && pathes.size() == 2 )
    {
        geometry_msgs::Point &color = colors[2];

        nav_msgs::Path &path0 = pathes[0];
        nav_msgs::Path &path1 = pathes[1];
        int pose_size = std::min( path0.poses.size(), path1.poses.size() );
        int idx0 = path0.poses.size() - pose_size;
        int idx1 = path1.poses.size() - pose_size;

        cout << " Add pairs, path0 = " << path0.poses.size()
             << ", path1 = " << path1.poses.size()
             << ", size = " << pose_size << endl;

        for( int i = 0; i < pose_size; i++, idx0++, idx1++)
        {
            if( idx0 >= path0.poses.size() || idx1 >= path1.poses.size() )
                break;

            geometry_msgs::PoseStamped &pose1 = path0.poses[idx0];
            geometry_msgs::PoseStamped &pose2 = path1.poses[idx1];

            stringstream ss;
            ss << "pair_" << i;
            // add line
            pcl::PointXYZ p1(pose1.pose.position.x, pose1.pose.position.y, pose1.pose.position.z),
                    p2(pose2.pose.position.x, pose2.pose.position.y, pose2.pose.position.z);
            pcl_viewer->addLine( p1, p2, color.x, color.y, color.z, ss.str() );
        }

    }

    int rmse_size = 1e6;
    int last_rmse_size = -1;
    double rmse = 0;

    // Spin in loop
    ros::Rate loop_rate( 10 );
    while( ros::ok() && !pcl_viewer->wasStopped() )
    {
        pcl_viewer->spinOnce(10);

        if( pathes.size() == 2 )
        {
            nh.param<int>("rmse_size", rmse_size, rmse_size);
            if( last_rmse_size != rmse_size )
            {
                rmse = calculateMeanAndRMSE( pathes[0], pathes[1], rmse_size );
                ROS_INFO_STREAM(" RMSE = " << rmse );
                last_rmse_size = rmse_size;
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    pcl_viewer->close();

    ros::shutdown();
}

