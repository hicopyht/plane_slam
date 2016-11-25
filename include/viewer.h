#ifndef VIEWER_H
#define VIEWER_H

#include <ros/ros.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <dynamic_reconfigure/server.h>
#include <plane_slam/ViewerConfig.h>
#include <std_srvs/Trigger.h>
#include "utils.h"
#include "frame.h"
#include <vtkTransform.h>

namespace plane_slam
{

class Viewer
{
public:
    Viewer( ros::NodeHandle &nh );

    // clear
    void removeFrames();
    void removeMap();
    void removeAll();

    // spin viewer
    void spinFramesOnce( int time = 1);
    void spinMapOnce( int time = 1);
    void spinOnce( int time = 1);


    void displayFrame(const Frame &frame, const std::string &prefix, int viewport);

    void displayInputCloud( const PointCloudTypePtr &cloud, const std::string &id = "rgbd_cloud", int viewport = 0 );

    void displayMatched3DKeypoint( const std_vector_of_eigen_vector4f &query,
                                   const std_vector_of_eigen_vector4f &train,
                                   const std::vector<cv::DMatch> &matches,
                                   int viewport_query,
                                   int viewport_train,
                                   const std::string &id = "matched_features" );

    void display3DKeypoint( const std_vector_of_eigen_vector4f &feature_location_3d, const std::string &id, int viewport );

    void displayKeypointMatches( const cv::Mat &img1, const std::vector<cv::KeyPoint> &keypoints1,
                                 const cv::Mat &img2, const std::vector<cv::KeyPoint> &keypoints2,
                                 const std::vector<cv::DMatch> &matches );

    void displayKeypoint( const cv::Mat &visual, const std::vector<cv::KeyPoint> &keypoints );

    void displayMapLandmarks( const PointCloudTypePtr &keypoints_cloud, const std::string &prefix = "point_landmark" );

    void displayMapLandmarks( std::map<int, PlaneType*> &landmarks, const std::string &prefix = "plane_landmark" );

    void displayMapLandmarks( const std::vector<PlaneType> &landmarks, const std::string &prefix = "landmark" );

    vtkSmartPointer<vtkPolyData> createCameraFOVPolygon( const gtsam::Pose3 &pose );

    void displayCameraFOV(  const gtsam::Pose3 pose );

    void displayPath( const std::vector<geometry_msgs::PoseStamped> &poses, const std::string &prefix = "path", double r = 255, double g = 0, double b = 0 );

    void displayPath( const std::map<int, gtsam::Pose3> &optimized_poses, const std::string &prefix = "optimized_path", double r = 255, double g = 0, double b = 0 );

    void displayPlanes( const PointCloudTypePtr &input, const std::vector<PlaneType> &planes, const std::string &prefix, int viewport);

    void displayLinesAndNormals( const PointCloudTypePtr &input,
                                 std::vector<PlaneFromLineSegment::LineType> &lines,
                                 std::vector<PlaneFromLineSegment::NormalType> &normals,
                                 int viewport);

    void pclViewerLandmark( const KeyPoint &keypoint, const std::string &id, const int number = -1 );

    void pclViewerLandmark( const PlaneType &plane, const std::string &id, const int number = -1);

    void pclViewerLineRegion( const PointCloudTypePtr &input, PlaneFromLineSegment::LineType &line, const std::string &id, int viewpoint);

    void pclViewerNormal( const PointCloudTypePtr &input, PlaneFromLineSegment::NormalType &normal, const std::string &id, int viewpoint);

    void pclViewerPlane( const PointCloudTypePtr &input, const PlaneType &plane, const std::string &id, int viewport, const int number = -1);

    void focusOnCamera( tf::Transform &pose );

    const int& vp1() const { return viewer_v1_; }
    const int& vp2() const { return viewer_v2_; }
    const int& vp3() const { return viewer_v3_; }
    const int& vp4() const { return viewer_v4_; }

    const bool& isDisplayKeypointLandmarks() const { return display_point_landmarks_; }
    const bool& isDisplayPlaneLandmarks() const { return display_plane_landmarks_; }

    template <typename DoubleVector>
    static void strToVector( const std::string &str_vector, DoubleVector &values, int size ){
        if( std::count(str_vector.begin(), str_vector.end(), ',') != (size -1) )
            return;

        int loc1 = 0;
        int loc2;
        std::string str;
        for( int i = 0; i < size; i++)
        {
            loc2 = str_vector.find(',', loc1);
            str = str_vector.substr(loc1, loc2-loc1);
            values[i] = atof(str.c_str());
            loc1 = loc2 + 1;
        }
    }

protected:
    void viewerReconfigCallback( plane_slam::ViewerConfig &config, uint32_t level);

    bool autoSpinMapViewerCallback( std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res );

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
    ros::ServiceServer spin_map_viewer_ss_;
    bool auto_spin_map_viewer_;

    // parameter frame
    bool focus_on_camera_;
    bool display_frame_;
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
    bool display_feature_cloud_;
    bool show_keypoint_;
    bool show_keypoint_matches_;
    bool display_3d_keypoint_matches_;
    // parameter for landmark
    bool display_plane_landmarks_;
    bool display_point_landmarks_;
    bool display_landmark_inlier_;
    bool display_landmark_arrow_;
    bool display_landmark_number_;
    bool display_landmark_boundary_;
    bool display_landmark_hull_;
    bool display_landmark_label_;
    //
    bool display_camera_fov_;
    bool display_pathes_;
    bool display_optimized_path_;
    bool display_odom_path_;
    bool display_visual_odom_path_;

};

} // end of namespace plane_slam

#endif // VIEWER_H
