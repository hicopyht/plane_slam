#include <iostream>
#include <fstream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

using namespace std;

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloudType;
typedef PointCloudType::Ptr PointCloudTypePtr;

PointCloudTypePtr readMapPoint(const std::string &filename)
{
    ifstream fin(filename.c_str());
    if( !fin )
    {
        cout <<"Failed to read map point: " << filename << ", exit." << endl;
        exit(1);
    }

    PointCloudTypePtr cloud( new PointCloudType);
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->header.frame_id = "/map";


    // load the poses
    std::string line;
    while ( std::getline(fin, line) )
    {
        if( line.size() < 3 || line[0] == '#' )
            continue;

        // Read
        stringstream ss(line);
        float x, y, z;
        ss >> x >> y >> z;
        PointType pt;
        pt.x = x; pt.y = y; pt.z = z;
        // Push
        cloud->points.push_back(pt);
    }
    cloud->width = cloud->points.size();
    //
    return cloud;
}

int main(int argc, char** argv)
{
    if(argc < 3)
    {
        cout << endl << "Usage: ./orb_map_cloud mapPointFile pcdFile" << endl;
        return 1;
    }

    std::string mapfile = argv[1];
    std::string pcdfile = argv[2];

    // Read map point, build pointcloud
    PointCloudTypePtr cloud = readMapPoint(mapfile);
    cout << endl << "Read map point: " << cloud->points.size() << endl;

    // Save pcd file
    pcl::io::savePCDFileASCII ( pcdfile, *cloud);
    cout << endl << "Save pcd file: " << pcdfile << endl;

    return 0;
}
