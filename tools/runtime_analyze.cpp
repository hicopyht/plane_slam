#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>

#define LINESIZE 81920
using namespace std;

void readTimes( const std::string &filename, std::vector<double> &times, int index = 1 )
{
    ifstream fin(filename.c_str());
    if( !fin )
    {
        ROS_ERROR_STREAM( "Failed to read runtime file: " << filename << ", exit." );
        exit(1);
    }

    // load the runtimes
    index = std::min(3, index);
    std::string line;
    while ( std::getline(fin, line) )
    {
        if( line.size() < 2 || line[0] == '#' )
            continue;

        // Read
        stringstream ss(line);
        int id;
        std::vector<double> tt(4);
        ss >>id >> tt[0] >> tt[1] >> tt[2] >> tt[3];
        // Push
        times.push_back( tt[index]);
    }
}

void saveTimes( const std::string &filename, std::vector<double> &times )
{
    cout << " Save runtimes: " << filename << ", size = " << times.size() << endl;

    FILE* yaml = std::fopen( filename.c_str(), "w" );
    fprintf( yaml, "# times file: %s\n", filename.c_str() );
    fprintf( yaml, "# size: %d\n", times.size() );

    for( int i = 0; i < times.size(); i++)
    {
        fprintf( yaml, "%d %f\n", i, times[i]);
    }
    fclose(yaml);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "runtime_analyze", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    std::string filename = "runtimes.txt";

    if( argc >= 2 )
    {
        filename = argv[1];
    }

    std::vector<double> times;
    readTimes( filename, times, 1);
    saveTimes( "track_time.txt", times );

    ros::shutdown();
}


