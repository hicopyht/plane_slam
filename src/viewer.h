#ifndef VIEWER_H
#define VIEWER_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <plane_slam/ViewerConfig.h>
#include <std_srvs/SetBool.h>
#include "utils.h"

namespace plane_slam
{

class Viewer
{
public:
    Viewer( ros::NodeHandle &nh );

    void displayMatched3DKeypoint( std_vector_of_eigen_vector4f &query,
                                   std_vector_of_eigen_vector4f &train,
                                   std::vector<cv::DMatch> &matches,
                                   const std::string &id = "matched_features");

    void displayKeypoint( const cv::Mat &visual, std::vector<cv::KeyPoint> &keypoints );

    void display3DKeypoint( std_vector_of_eigen_vector4f &feature_location_3d, const std::string &id, int viewport );

    void displayLandmarks( const std::vector<PlaneType> &landmarks, const std::string &prefix = "landmark");

    void displayPlanes( const PointCloudTypePtr &input, std::vector<PlaneType> &planes, const std::string &prefix, int viewport);

    void displayLinesAndNormals( const PointCloudTypePtr &input,
                                 std::vector<PlaneFromLineSegment::LineType> &lines,
                                 std::vector<PlaneFromLineSegment::NormalType> &normals,
                                 int viewport);

    void pclViewerLandmark( const PlaneType &plane, const std::string &id, const int number = -1);

    void pclViewerLineRegion( const PointCloudTypePtr &input, PlaneFromLineSegment::LineType &line, const std::string &id, int viewpoint);

    void pclViewerNormal( const PointCloudTypePtr &input, PlaneFromLineSegment::NormalType &normal, const std::string &id, int viewpoint);

    void pclViewerPlane( const PointCloudTypePtr &input, PlaneType &plane, const std::string &id, int viewport, int number = -1);


protected:
    void viewerReconfigCallback( plane_slam::ViewerConfig &config, uint32_t level);

    bool autoSpinMapViewerCallback( std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

private:
    ros::NodeHandle nh_;
    dynamic_reconfigure::Server<plane_slam::ViewerConfig> viewer_config_server_;
    dynamic_reconfigure::Server<plane_slam::ViewerConfig>::CallbackType viewer_config_callback_;
    // pcl_viewer
    pcl::visualization::PCLVisualizer* pcl_viewer_;
    int viewer_v1_;
    int viewer_v2_;
    int viewer_v3_;
    int viewer_v4_;
    cv::RNG rng;
    //
    pcl::visualization::PCLVisualizer* map_viewer_;
    ros::ServiceServer auto_spin_map_viewer_ss_;
    bool auto_spin_map_viewer_;

    // parameter frame
    bool display_input_cloud_;
    bool display_line_cloud_;
    bool display_normal_;
    bool display_normal_arrow_;
    bool display_plane_;
    bool display_plane_number_;
    bool display_plane_arrow_;
    bool display_plane_inlier_;
    bool display_plane_projected_inlier_;
    bool display_plane_boundary_;
    bool display_plane_hull_;
    // parameter for landmark
    bool display_landmarks_;
    bool display_landmark_inlier_;
    bool display_landmark_arrow_;
    bool display_landmark_number_;
    bool display_landmark_boundary_;
    bool display_landmark_hull_;

};

} // end of namespace plane_slam

#endif // VIEWER_H
