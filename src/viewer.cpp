#include "viewer.h"

namespace plane_slam
{

const std::string KeypointWindow = "Keypoint";
const std::string MatchesWindow = "MatchesFeatures";

Viewer::Viewer( ros::NodeHandle &nh)
    : nh_(nh)
    , viewer_config_server_( ros::NodeHandle( nh_, "Viewer" ) )
    , pcl_viewer_( new pcl::visualization::PCLVisualizer("3D Viewer"))
    , map_viewer_( new pcl::visualization::PCLVisualizer("Map Viewer"))
    , viewer_v1_(1)
    , viewer_v2_(2)
    , viewer_v3_(3)
    , viewer_v4_(4)
    , rng(12345)
{
    viewer_config_callback_ = boost::bind(&Viewer::viewerReconfigCallback, this, _1, _2);
    viewer_config_server_.setCallback(viewer_config_callback_);
    spin_map_viewer_ss_ = nh_.advertiseService("spin_map_viewer", &Viewer::autoSpinMapViewerCallback, this);

    pcl_viewer_->createViewPort(0, 0.5, 0.5, 1.0, viewer_v1_);
    pcl_viewer_->addText("RansacPlanes", 100, 3, "v3_text", viewer_v1_);
    pcl_viewer_->createViewPort(0.5, 0.5, 1.0, 1.0, viewer_v2_);
    pcl_viewer_->addText("OrganizedPlanes", 100, 3, "v4_text", viewer_v2_);
    pcl_viewer_->createViewPort(0, 0, 0.5, 0.5, viewer_v3_);
    pcl_viewer_->addText("LinesAndNormals", 100, 3, "v1_text", viewer_v3_);
    pcl_viewer_->createViewPort(0.5, 0, 1.0, 0.5, viewer_v4_);
    pcl_viewer_->addText("LineBasedPlanes", 100, 3, "v2_text", viewer_v4_);
    pcl_viewer_->addCoordinateSystem(0.000001);
    pcl_viewer_->initCameraParameters();
    pcl_viewer_->setCameraPosition(0.0, 0.0, -0.4, 0, 0, 0.6, 0, -1, 0);
    pcl_viewer_->setBackgroundColor( 1.0, 1.0, 1.0);
    pcl_viewer_->setSize( 880, 700 );
    pcl_viewer_->setShowFPS(true);

    map_viewer_->addCoordinateSystem(1.0);
    map_viewer_->initCameraParameters();
//    map_viewer_->setCameraPosition(0.0, 0.0, -2.4, 0, 0, 0.6, 0, -1, 0);
//    map_viewer_->setCameraPosition( 0, 3.0, 3.0, -3.0, 0, 0, -1, -1, 0 );
//    7.83553,18.0737/7.98352,-1.33264,5.13725/-1.65863,-6.09147,13.7632/0.253037,0.699241,0.668606/0.8575/683,384/65,52
//    map_viewer_->setCameraPosition( -1.65863,-6.09147,13.7632, 7.98352,-1.33264,5.13725, 0.253037,0.699241,0.668606);
    // x,x/view(xyz)/pos(xyz)/up(xyz>)/x/x/x
    // Building A floor 5th
    // 18.4838,39.0089/-8.40921,-12.0519,4.25967/-8.33769,-18.6937,27.1933/-0.00491748,0.960514,0.278188/0.8575/683,384/64,79
    //map_viewer_->setCameraPosition(-8.33769,-18.6937,27.1933,-8.40921,-12.0519,4.25967,-0.00491748,0.960514,0.278188);
    //map_viewer_->setCameraPosition(-17.7387,-38.2842,26.9952,-11.0398,-15.4227,1.41248,0.285626,0.676216,0.679079);
    map_viewer_->setCameraPosition(-20.2139,-39.6031,24.9315,-11.0398,-15.4227,1.41248,0.271842,0.615997,0.739358);
    map_viewer_->setBackgroundColor( 1.0, 1.0, 1.0);
    map_viewer_->setShowFPS(true);
    map_viewer_->setSize( 880, 700 );
    map_viewer_->setRepresentationToSurfaceForAllActors();

    //
    cv::namedWindow( KeypointWindow );
    cv::namedWindow( MatchesWindow );

}

void Viewer::removeFrames()
{
    pcl_viewer_->removeAllPointClouds();
    pcl_viewer_->removeAllShapes();
}

void Viewer::removeMap()
{
    map_viewer_->removeAllPointClouds();
    map_viewer_->removeAllShapes();
}

void Viewer::removeAll()
{
    // Clear Display
    removeFrames();
    removeMap();
}

void Viewer::spinFramesOnce( int time )
{
    pcl_viewer_->spinOnce( time );
}

void Viewer::spinMapOnce( int time )
{
    map_viewer_->spinOnce( time );
}

void Viewer::spinOnce( int time )
{
    spinMapOnce( time );
    spinFramesOnce( time );
}

void Viewer::displayFrame(const Frame &frame, const std::string &prefix, int viewport )
{
    if( !display_frame_ )
        return;

    pcl_viewer_->addText(prefix, 100, 3, 0.0, 0.0, 0.0, prefix+"_text", viewport);

    // Input cloud
    if( display_input_cloud_ && frame.cloud_ && frame.cloud_->size() > 0 )
    {
        pcl_viewer_->addPointCloud( frame.cloud_, prefix+"_"+"rgba_cloud", viewport );
        pcl_viewer_->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, prefix+"_"+"rgba_cloud", viewport );
    }

    // 3d keypoint in viewer
    if( display_feature_cloud_ )
    {
        display3DKeypoint( frame.feature_locations_3d_, prefix+"_"+"3d_keypoint", viewport );
    }

    // Keypoints on image
//    displayKeypoint( frame.visual_image_, frame.feature_locations_2d_ );

    // planes
    displayPlanes( frame.cloud_downsampled_, frame.segment_planes_, prefix+"_"+"planes", viewport );
}

void Viewer::displayInputCloud( const PointCloudTypePtr &cloud, const std::string &id, int viewport )
{
    // Input cloud
    if( display_input_cloud_ && cloud && cloud->size() > 0 )
    {
        pcl_viewer_->addPointCloud( cloud, id, viewport );
//        pcl_viewer_->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id, viewport );
    }
}

void Viewer::displayMatched3DKeypoint( const std_vector_of_eigen_vector4f &query,
                                       const std_vector_of_eigen_vector4f &train,
                                       const std::vector<cv::DMatch> &matches,
                                       int viewport_query,
                                       int viewport_train,
                                       const std::string &id )
{
    if( !display_3d_keypoint_matches_ )
        return;

    PointCloudXYZPtr query_cloud( new PointCloudXYZ ), train_cloud( new PointCloudXYZ );
    query_cloud->is_dense = false;
    query_cloud->points.resize( matches.size() );
    query_cloud->height = 1;
    query_cloud->width = matches.size();
    train_cloud->is_dense = false;
    train_cloud->points.resize( matches.size() );
    train_cloud->height = 1;
    train_cloud->width = matches.size();
    //
    PointCloudXYZ::iterator query_it = query_cloud->begin(), train_it = train_cloud->begin();
    for(int i = 0; i < matches.size(); i++, query_it++, train_it++)
    {
        if( query_it == query_cloud->end() )
            break;
        if( train_it == train_cloud->end() )
            break;

        pcl::PointXYZ &qp = *query_it;
        pcl::PointXYZ &tp = *train_it;
        const int qIdx = matches[i].queryIdx;
        const int tIdx = matches[i].trainIdx;
        qp.x = query[qIdx](0);
        qp.y = query[qIdx](1);
        qp.z = query[qIdx](2);
        tp.x = train[tIdx](0);
        tp.y = train[tIdx](1);
        tp.z = train[tIdx](2);
    }
    //
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> query_color( query_cloud, 0, 255, 0);
    pcl_viewer_->addPointCloud( query_cloud, query_color, id+"_query", viewport_query );
    pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id+"_query", viewport_query );
    //
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> train_color( train_cloud, 255, 255, 0);
    pcl_viewer_->addPointCloud( train_cloud, train_color, id+"_train", viewport_train );
    pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id+"_train", viewport_query );

//    // spin
//    pcl_viewer_->spinOnce();
}

void Viewer::display3DKeypoint( const std_vector_of_eigen_vector4f &feature_location_3d, const std::string &id, int viewport )
{
    PointCloudXYZPtr cloud( new PointCloudXYZ );
    cloud->is_dense = false;
    cloud->points.resize( feature_location_3d.size() );
    cloud->height = 1;
    cloud->width = feature_location_3d.size();

    PointCloudXYZ::iterator it = cloud->begin();
    for(int i = 0; i < feature_location_3d.size(); i++, it++)
    {
        if( it == cloud->end() )
            break;
        pcl::PointXYZ &pt = *it;
        pt.x = feature_location_3d[i](0);
        pt.y = feature_location_3d[i](1);
        pt.z = feature_location_3d[i](2);
    }

    //
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color( cloud, 255, 255, 0);
    pcl_viewer_->addPointCloud( cloud, color, id+"_3Dfeatures", viewport );
    pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id+"_3Dfeatures", viewport );

}

void Viewer::displayKeypointMatches( const cv::Mat &img1, const std::vector<cv::KeyPoint> &keypoints1,
                                     const cv::Mat &img2, const std::vector<cv::KeyPoint> &keypoints2,
                                     const std::vector<cv::DMatch> &matches )
{
    if( !show_keypoint_matches_)
        return;

    cv::Mat image;
    cv::drawMatches( img1, keypoints1, img2, keypoints2,
               matches, image, cv::Scalar::all(-1), cv::Scalar::all(-1),
               std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    imshow( MatchesWindow, image );
    cv::waitKey(1);
}

void Viewer::displayKeypoint( const cv::Mat &visual, const std::vector<cv::KeyPoint> &keypoints )
{
    if( !show_keypoint_ )
        return;

    cv::Mat image;
    cv::drawKeypoints( visual, keypoints, image, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
    cv::imshow( KeypointWindow, image );
    cv::waitKey(1);
}

void Viewer::displayMapLandmarks( const PointCloudTypePtr &keypoints_cloud, const std::string &prefix )
{
    if( display_point_landmarks_ )
    {
        map_viewer_->addPointCloud( keypoints_cloud, prefix );
        map_viewer_->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, prefix );
        cout << GREEN << " Display keypoint landmarks: " << keypoints_cloud->size() << RESET << endl;
    }
}

void Viewer::displayMapLandmarks( std::map<int, PlaneType*> &landmarks, const std::string &prefix )
{
    if( display_plane_landmarks_ )
    {
        map_viewer_->addText(prefix, 300, 3, prefix+"_text");

        int invalid_count = 0;

        for( std::map<int, PlaneType*>::iterator it = landmarks.begin();
             it != landmarks.end(); it++)
        {
            const int id = it->first;
            PlaneType *lm = it->second;
            const PlaneType &plane = *lm;

            pcl::ModelCoefficients coeff;
            coeff.values.resize( 4 );
            coeff.values[0] = plane.coefficients[0];
            coeff.values[1] = plane.coefficients[1];
            coeff.values[2] = plane.coefficients[2];
            coeff.values[3] = plane.coefficients[3];
            //
            stringstream ss;
            ss << prefix << "_" << id;
//            map_viewer_->addPlane( coeff, 1.0, 1.0, 1.0, ss.str());
            pclViewerLandmark( plane, ss.str(), id);
        }

        cout << GREEN << " Display landmarks: " << landmarks.size() << ", invalid = " << invalid_count << RESET << endl;
    }
}

void Viewer::displayMapLandmarks( const std::vector<PlaneType> &landmarks, const std::string &prefix )
{
    if( display_plane_landmarks_ )
    {
        map_viewer_->addText(prefix, 300, 3, prefix+"_text");

        int invalid_count = 0;
        for(int i = 0; i < landmarks.size(); i++)
        {
            const PlaneType & plane = landmarks[i];
            if( !plane.valid )
            {
                invalid_count ++;
                continue;
            }

            pcl::ModelCoefficients coeff;
            coeff.values.resize( 4 );
            coeff.values[0] = plane.coefficients[0];
            coeff.values[1] = plane.coefficients[1];
            coeff.values[2] = plane.coefficients[2];
            coeff.values[3] = plane.coefficients[3];
            //
            stringstream ss;
            ss << prefix << "_" << i;
//            map_viewer_->addPlane( coeff, 1.0, 1.0, 1.0, ss.str());
            pclViewerLandmark( plane, ss.str(), i);
        }

        cout << GREEN << " Display landmarks: " << landmarks.size() << ", invalid = " << invalid_count << RESET << endl;
    }
}

vtkSmartPointer<vtkPolyData> Viewer::createCameraFOVPolygon( const gtsam::Pose3 &pose )
{
    double yg = pose.translation().z();
    double zg = yg / tan(DEG_TO_RAD*45.0/2.0);
    double xg = tan(DEG_TO_RAD*58.0/2.0) * zg;
    double z = 3.5; // meter
    double y = tan(DEG_TO_RAD*45.0/2.0) * z;
    double x = tan(DEG_TO_RAD*58.0/2.0) * z;

    /// Setup four points
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    points->InsertNextPoint(0.0, 0.0, 0.0);
    points->InsertNextPoint(-x, -y, z);
    points->InsertNextPoint(x, -y, z);
    points->InsertNextPoint(x, yg, z);
    points->InsertNextPoint(-x, yg, z);
    points->InsertNextPoint(xg, yg, zg);
    points->InsertNextPoint(-xg, yg, zg);

    // Create the polygon
    vtkSmartPointer<vtkPolygon> polygon1 = vtkSmartPointer<vtkPolygon>::New();
    polygon1->GetPointIds()->SetNumberOfIds(4);
    polygon1->GetPointIds()->SetId(0, 1);
    polygon1->GetPointIds()->SetId(1, 2);
    polygon1->GetPointIds()->SetId(2, 3);
    polygon1->GetPointIds()->SetId(3, 4);
    //
    vtkSmartPointer<vtkPolygon> polygon2 = vtkSmartPointer<vtkPolygon>::New();
    polygon2->GetPointIds()->SetNumberOfIds(3);
    polygon2->GetPointIds()->SetId(0, 0);
    polygon2->GetPointIds()->SetId(1, 1);
    polygon2->GetPointIds()->SetId(2, 2);

    //
    vtkSmartPointer<vtkPolygon> polygon3 = vtkSmartPointer<vtkPolygon>::New();
    polygon3->GetPointIds()->SetNumberOfIds(3);
    polygon3->GetPointIds()->SetId(0, 0);
    polygon3->GetPointIds()->SetId(1, 5);
    polygon3->GetPointIds()->SetId(2, 6);

    //
    vtkSmartPointer<vtkPolygon> polygon4 = vtkSmartPointer<vtkPolygon>::New();
    polygon4->GetPointIds()->SetNumberOfIds(4);
    polygon4->GetPointIds()->SetId(0, 3);
    polygon4->GetPointIds()->SetId(1, 4);
    polygon4->GetPointIds()->SetId(2, 6);
    polygon4->GetPointIds()->SetId(3, 5);

    // Add the polygon to a list of polygons
    vtkSmartPointer<vtkCellArray> polygons = vtkSmartPointer<vtkCellArray>::New();
    polygons->InsertNextCell(polygon1);
    polygons->InsertNextCell(polygon2);
    polygons->InsertNextCell(polygon3);
    polygons->InsertNextCell(polygon4);

    // Create a PolyData
    vtkSmartPointer<vtkPolyData> polygonPolyData = vtkSmartPointer<vtkPolyData>::New();
    polygonPolyData->SetPoints(points);
    polygonPolyData->SetPolys(polygons);

    return polygonPolyData;
}

vtkSmartPointer<vtkPolyData> Viewer::createCameraFOVPolygonSimple()
{
    double z = 0.4; // meter
    double y = tan(DEG_TO_RAD*45.0/2.0) * z;
    double x = tan(DEG_TO_RAD*58.0/2.0) * z;

    /// Setup four points
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    points->InsertNextPoint(0.0, 0.0, 0.0);
    points->InsertNextPoint(-x, -y, z);
    points->InsertNextPoint(x, -y, z);
    points->InsertNextPoint(x, y, z);
    points->InsertNextPoint(-x, y, z);

    // Create the polygon
    vtkSmartPointer<vtkPolygon> polygon1 = vtkSmartPointer<vtkPolygon>::New();
    polygon1->GetPointIds()->SetNumberOfIds(4);
    polygon1->GetPointIds()->SetId(0, 0);
    polygon1->GetPointIds()->SetId(1, 1);
    polygon1->GetPointIds()->SetId(2, 2);
    polygon1->GetPointIds()->SetId(3, 0);
    //
    vtkSmartPointer<vtkPolygon> polygon2 = vtkSmartPointer<vtkPolygon>::New();
    polygon2->GetPointIds()->SetNumberOfIds(4);
    polygon2->GetPointIds()->SetId(0, 0);
    polygon2->GetPointIds()->SetId(1, 3);
    polygon2->GetPointIds()->SetId(2, 4);
    polygon2->GetPointIds()->SetId(3, 0);
    //
    vtkSmartPointer<vtkPolygon> polygon3 = vtkSmartPointer<vtkPolygon>::New();
    polygon3->GetPointIds()->SetNumberOfIds(4);
    polygon3->GetPointIds()->SetId(0, 1);
    polygon3->GetPointIds()->SetId(1, 2);
    polygon3->GetPointIds()->SetId(2, 3);
    polygon3->GetPointIds()->SetId(3, 4);
    //

    // Add the polygon to a list of polygons
    vtkSmartPointer<vtkCellArray> polygons = vtkSmartPointer<vtkCellArray>::New();
    polygons->InsertNextCell(polygon1);
    polygons->InsertNextCell(polygon2);
    polygons->InsertNextCell(polygon3);

    // Create a PolyData
    vtkSmartPointer<vtkPolyData> polygonPolyData = vtkSmartPointer<vtkPolyData>::New();
    polygonPolyData->SetPoints(points);
    polygonPolyData->SetPolys(polygons);

    return polygonPolyData;
}

void Viewer::displayCameraFOV(  const gtsam::Pose3 pose )
{
//    static vtkSmartPointer<vtkPolyData> polygonPolyData = createCameraFOVPolygon( pose );
    static vtkSmartPointer<vtkPolyData> polygonPolyData = createCameraFOVPolygonSimple();

    if( display_camera_fov_ )
    {
        // Add camera pose
        gtsam::Point3 translation = pose.translation();
        gtsam::Vector3 xyz = pose.rotation().xyz();

        // Set up the transform filter
        vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
        transform->PostMultiply();
        transform->RotateX( xyz[0] * RAD_TO_DEG );
        transform->RotateY( xyz[1] * RAD_TO_DEG );
        transform->RotateZ( xyz[2] * RAD_TO_DEG );
        transform->Translate( translation.x(), translation.y(), translation.z());

        //
        map_viewer_->addModelFromPolyData( polygonPolyData, transform, "camera_fov");
    }
}

void Viewer::displayPath( const std::vector<geometry_msgs::PoseStamped> &poses, const std::string &prefix, double r, double g, double b )
{
    if( !display_pathes_ )
        return;

    if( prefix == "odom_path" && !display_odom_path_ )
        return;

    if( prefix == "visual_odom_path" && !display_visual_odom_path_ )
        return;

    if( prefix == "true_path" && !display_true_path_ )
        return;

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
        map_viewer_->addLine( p1, p2, r, g, b, ss.str() );
    }
}

void Viewer::displayPath( const std::map<int, gtsam::Pose3> &optimized_poses, const std::string &prefix, double r, double g, double b )
{
    if(!display_pathes_ || !display_optimized_path_)
        return;

    if( optimized_poses.size() == 0 )
        return;

    bool last_valid = false;
    pcl::PointXYZ p1, p2;
//    int last_index = 0;
    for( std::map<int, gtsam::Pose3>::const_iterator it = optimized_poses.begin();
            it != optimized_poses.end(); it++)
    {
        const gtsam::Pose3 &pose = it->second;
//        last_index = it->first;
        //
        if( !last_valid )
        {
            p1.x = pose.x();
            p1.y = pose.y();
            p1.z = pose.z();
            last_valid = true;
            continue;
        }

        // id
        stringstream ss;
        ss << prefix << "_line_" << it->first;
        // add line
        p2.x = pose.x();
        p2.y = pose.y();
        p2.z = pose.z();
        map_viewer_->addLine( p1, p2, r, g, b, ss.str() );
        //
        p1 = p2;
    }
}

void Viewer::displayPlanes( const PointCloudTypePtr &input, const std::vector<PlaneType> &planes, const std::string &prefix, int viewport)
{
    if(display_plane_)
    {
        for(int j = 0; j < planes.size(); j++)
        {
            stringstream ss;
            ss << "_" << j;
            pclViewerPlane( input, planes[j], prefix + ss.str(), viewport, j);
        }
    }
}

void Viewer::displayLinesAndNormals( const PointCloudTypePtr &input,
                                    std::vector<line_based_plane_segment::LineType> &lines,
                                    std::vector<line_based_plane_segment::NormalType> &normals,
                                    int viewport)
{
    if(display_input_cloud_)
    {
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> rgba_color(frame_current.point_cloud, 255, 255, 255);
        pcl_viewer_->addPointCloud( input, "rgba_cloud" );
        pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "rgba_cloud");
    }

    if(display_line_cloud_)
    {
        for(int j = 0; j < lines.size(); j++)
        {
            stringstream ss;
            ss << "line_" << j;
            pclViewerLineRegion( input, lines[j], ss.str(), viewport );
        }
    }

    if(display_normal_)
    {
        for(int j = 0; j < normals.size(); j++)
        {
            stringstream ss;
            ss << "normal_" << j;
            pclViewerNormal( input, normals[j], ss.str(), viewport );
        }
    }
}

void Viewer::pclViewerLandmark( const KeyPoint &keypoint, const std::string &id, const int number )
{
    if( !display_point_landmarks_ )
        return;

    PointType pt;
    pt.x = keypoint.translation.x();
    pt.y = keypoint.translation.y();
    pt.z = keypoint.translation.z();

    // add point
    map_viewer_->addSphere( pt, 0.02, keypoint.color.Red, keypoint.color.Green, keypoint.color.Blue, id+"_point");

    // add a plane number
    if( display_landmark_number_ && number >= 0 )
    {
        // add a plane number
        PointType pn;
        //
        pn.x = pt.x + 0.05;
        pn.y = pt.y + 0.05;
        pn.z = pt.z + 0.05;
        // add plane number
        stringstream ss;
        ss << number;
        map_viewer_->addText3D( ss.str(), pn, 0.05, 0.0, 0.0, 0.0, id+"_number");
    }
}

void Viewer::pclViewerLandmark( const PlaneType &plane, const std::string &id, const int number )
{
    // inlier
    if( display_landmark_inlier_ )
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color( plane.cloud_voxel, plane.color.Red, plane.color.Green, plane.color.Blue);
        map_viewer_->addPointCloud( plane.cloud_voxel, color, id+"_inlier" );
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color( plane.cloud, plane.color.Red, plane.color.Green, plane.color.Blue);
//        map_viewer_->addPointCloud( plane.cloud, color, id+"_inlier" );
        map_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, id+"_inlier" );

        if( display_landmark_arrow_ )
        {
            // add a line
            PointType p1, p2;
            p1 = plane.centroid;
            // check centroid
            if( p1.z == 0)
            {
                Eigen::Vector4f cen;
                pcl::compute3DCentroid( *plane.cloud_voxel, cen );
//                pcl::compute3DCentroid( *plane.cloud, cen );
                p1.x = cen[0];
                p1.y = cen[1];
                p1.z = cen[2];
            }

            p2.x = p1.x + plane.coefficients[0]*0.7;
            p2.y = p1.y + plane.coefficients[1]*0.7;
            p2.z = p1.z + plane.coefficients[2]*0.7;
            map_viewer_->addArrow(p2, p1, plane.color.Red/255.0, plane.color.Green/255.0, plane.color.Blue/255.0, false, id+"_arrow" );
            // add a sphere
        //    pcl_viewer_->addSphere( p1, 0.05, 255.0, 255.0, 0.0, id+"_point" );
        }

        if( display_landmark_label_ && !plane.semantic_label.empty() )
        {
            // add a plane label
            PointType p1, p2;
            p1 = plane.centroid;
            //
            p2.x = p1.x + plane.coefficients[0]*(0.45);
            p2.y = p1.y + plane.coefficients[1]*(0.25);
            p2.z = p1.z + plane.coefficients[2]*0.25;
            // add plane label
            map_viewer_->addText3D( plane.semantic_label, p2, 0.2, 0.0, 0.0, 0.0, id+"_label");
        }

        if( display_landmark_number_ && number >= 0 )
        {
            // add a plane number
            PointType p1, p2;
            p1 = plane.centroid;
            //
            p2.x = p1.x + plane.coefficients[0]*(0.25);
            p2.y = p1.y + plane.coefficients[1]*(0.25);
            p2.z = p1.z + plane.coefficients[2]*0.25;
            // add plane number
            stringstream ss;
            ss << number;
            map_viewer_->addText3D( ss.str(), p2, 0.2, 0.0, 0.0, 0.0, id+"_number");
        }
    }

    // boundary
    if( display_landmark_boundary_ )
    {
        double r = rng.uniform(0.0, 255.0);
        double g = rng.uniform(0.0, 255.0);
        double b = rng.uniform(0.0, 255.0);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color_boundary( plane.cloud_boundary, r, g, b);
        map_viewer_->addPointCloud( plane.cloud_boundary, color_boundary, id+"_boundary" );
        map_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id+"_boundary" );
    }

    // hull
    if( display_landmark_hull_ )
    {
        double r = rng.uniform(0.0, 1.0);
        double g = rng.uniform(0.0, 1.0);
        double b = rng.uniform(0.0, 1.0);

        const int num = plane.cloud_hull->size();
        if( num >= 3)
        {
            for(int i = 1; i < num; i++)
            {
                stringstream ss;
                ss << id << "_hull_line_" << i;
                map_viewer_->addLine(plane.cloud_hull->points[i-1], plane.cloud_hull->points[i], r, g, b, ss.str() );
            }
            stringstream ss;
            ss << id << "_hull_line_0";
            map_viewer_->addLine(plane.cloud_hull->points[0], plane.cloud_hull->points[num-1], r, g, b, ss.str() );
        }
    }

}

void Viewer::pclViewerLineRegion( const PointCloudTypePtr &input, line_based_plane_segment::LineType &line, const std::string &id, int viewpoint)
{
    PointCloudTypePtr cloud = getPointCloudFromIndices( input, line.indices );

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color(cloud, rng.uniform(0.0, 255.0), rng.uniform(0.0, 255.0), rng.uniform(0.0, 255.0));
    pcl_viewer_->addPointCloud(cloud, color, id, viewpoint);
    pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, id, viewpoint);

}

void Viewer::pclViewerNormal( const PointCloudTypePtr &input, line_based_plane_segment::NormalType &normal, const std::string &id, int viewpoint)
{
    double r = rng.uniform(0.0, 255.0);
    double g = rng.uniform(0.0, 255.0);
    double b = rng.uniform(0.0, 255.0);

    if( display_normal_arrow_ )
    {
        // add a line
        PointType p1, p2;
        p1 = normal.centroid;
        // check centroid
        if( p1.z == 0 && p1.x == 0 && p1.y == 0 )
        {
            Eigen::Vector4f cen;
            pcl::compute3DCentroid( *input, normal.indices, cen);
            p1.x = cen[0];
            p1.y = cen[1];
            p1.z = cen[2];
        }

        p2.x = p1.x + normal.coefficients[0]*0.2;
        p2.y = p1.y + normal.coefficients[1]*0.2;
        p2.z = p1.z + normal.coefficients[2]*0.2;
        pcl_viewer_->addArrow(p2, p1, r/255.0, g/255.0, b/255.0, false, id+"_arrow", viewpoint);
    // add a sphere
//    pcl_viewer_->addSphere( p1, 0.05, 255.0, 255.0, 0.0, id+"_point", viewpoint);
    }
    // add inlier
    PointCloudTypePtr cloud (new PointCloudType );

    for(int i = 0; i < normal.indices.size(); i++)
    {
        cloud->points.push_back( input->points[normal.indices[i]] );
    }
    cloud->is_dense = false;
    cloud->height = 1;
    cloud->width = cloud->points.size();

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color(cloud, r, g, b);
    pcl_viewer_->addPointCloud(cloud, color, id+"_inlier", viewpoint);
    pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id+"_inlier", viewpoint);

}

void Viewer::pclViewerPlane( const PointCloudTypePtr &input, const PlaneType &plane, const std::string &id, int viewport, int number)
{
    double r = rng.uniform(0.0, 255.0);
    double g = rng.uniform(0.0, 255.0);
    double b = rng.uniform(0.0, 255.0);

    // inlier
    if( display_plane_inlier_ && display_plane_projected_inlier_ )
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color( plane.cloud, r, g, b);
        pcl_viewer_->addPointCloud( plane.cloud, color, id+"_inlier", viewport);
        pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id+"_inlier", viewport);

        if( display_plane_arrow_ )
        {
            // add a line
            PointType p1, p2;
            p1 = plane.centroid;
            // check centroid
            if( p1.z == 0 && p1.x == 0 && p1.y == 0 )
            {
                Eigen::Vector4f cen;
                pcl::compute3DCentroid( *plane.cloud, cen );
                p1.x = cen[0];
                p1.y = cen[1];
                p1.z = cen[2];
            }

            p2.x = p1.x + plane.coefficients[0]*0.2;
            p2.y = p1.y + plane.coefficients[1]*0.2;
            p2.z = p1.z + plane.coefficients[2]*0.2;
            //
            pcl_viewer_->addArrow(p2, p1, r, g, b, false, id+"_arrow", viewport);
            // add a sphere
        //    pcl_viewer_->addSphere( p1, 0.05, 255.0, 255.0, 0.0, id+"_point", viewport);
        }

        if( display_plane_number_ && number >= 0 )
        {
            // add a plane number
            PointType p1, p2;
            p1 = plane.centroid;
            //
            p2.x = p1.x + plane.coefficients[0]*0.05;
            p2.y = p1.y + plane.coefficients[1]*0.05;
            p2.z = p1.z + plane.coefficients[2]*0.05;
            // add plane number
            stringstream ss;
            ss << number;
            pcl_viewer_->addText3D( ss.str(), p2, 0.05, 0.0, 0.0, 0.0, id+"_number", viewport+1);
        }
    }
    else if( display_plane_inlier_ && input && input->size() > 0 )
    {
        PointCloudTypePtr cloud = getPointCloudFromIndices( input, plane.inlier );

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color(cloud, r, g, b);
        pcl_viewer_->addPointCloud(cloud, color, id+"_inlier", viewport);
        pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id+"_inlier", viewport);

        if( display_plane_arrow_ )
        {
            // add a line
            PointType p1, p2;
            p1 = plane.centroid;
            // check centroid
            if( p1.z == 0 && p1.x == 0 && p1.y == 0 )
            {
                Eigen::Vector4f cen;
                pcl::compute3DCentroid( *cloud, cen);
                p1.x = cen[0];
                p1.y = cen[1];
                p1.z = cen[2];
            }

            p2.x = p1.x + plane.coefficients[0]*0.2;
            p2.y = p1.y + plane.coefficients[1]*0.2;
            p2.z = p1.z + plane.coefficients[2]*0.2;
            //
            pcl_viewer_->addArrow(p2, p1, r, g, b, false, id+"_arrow", viewport);
            // add a sphere
        //    pcl_viewer_->addSphere( p1, 0.05, 255.0, 255.0, 0.0, id+"_point", viewport);
        }

        if( display_plane_number_ && number >= 0 )
        {
            // add a plane number
            PointType p1, p2;
            p1 = plane.centroid;
            //
            p2.x = p1.x + plane.coefficients[0]*0.05;
            p2.y = p1.y + plane.coefficients[1]*0.05;
            p2.z = p1.z + plane.coefficients[2]*0.05;
            // add plane number
            stringstream ss;
            ss << number;
            pcl_viewer_->addText3D( ss.str(), p2, 0.05, 0.0, 0.0, 0.0, id+"_number", viewport+1);
        }
    }

    // boundary
    if( display_plane_boundary_ && display_plane_projected_inlier_ )
    {
        r = rng.uniform(0.0, 255.0);
        g = rng.uniform(0.0, 255.0);
        b = rng.uniform(0.0, 255.0);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color_boundary( plane.cloud_boundary, r, g, b);
        pcl_viewer_->addPointCloud( plane.cloud_boundary, color_boundary, id+"_boundary", viewport );
        pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id+"_boundary", viewport);
    }
    else if( display_plane_boundary_ && input && input->size() > 0 )
    {
        PointCloudTypePtr cloud_boundary = getPointCloudFromIndices( input, plane.boundary_inlier );
        r = rng.uniform(0.0, 255.0);
        g = rng.uniform(0.0, 255.0);
        b = rng.uniform(0.0, 255.0);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color_boundary( cloud_boundary, r, g, b);
        pcl_viewer_->addPointCloud( cloud_boundary, color_boundary, id+"_boundary", viewport );
        pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id+"_boundary", viewport);

    }

    // hull
    if( display_plane_hull_ && display_plane_projected_inlier_ )
    {
        r = rng.uniform(0.0, 1.0);
        g = rng.uniform(0.0, 1.0);
        b = rng.uniform(0.0, 1.0);

        const int num = plane.cloud_hull->size();
        if( num >= 3)
        {
            for(int i = 1; i < num; i++)
            {
                stringstream ss;
                ss << id << "_hull_line_" << i;
                pcl_viewer_->addLine(plane.cloud_hull->points[i-1], plane.cloud_hull->points[i], r, g, b, ss.str(), viewport );
            }
            stringstream ss;
            ss << id << "_hull_line_0";
            pcl_viewer_->addLine(plane.cloud_hull->points[0], plane.cloud_hull->points[num-1], r, g, b, ss.str(), viewport );
        }

//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color_hull( plane.cloud_hull, r, g, b);
//        pcl_viewer_->addPointCloud( plane.cloud_hull, color_hull, id+"_hull", viewpoint );
//        pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id+"_hull", viewport );

    }
    else if( display_plane_hull_ && input && input->size() > 0 )
    {
        PointCloudTypePtr cloud_hull = getPointCloudFromIndices( input, plane.hull_inlier );
        r = rng.uniform(0.0, 1.0);
        g = rng.uniform(0.0, 1.0);
        b = rng.uniform(0.0, 1.0);

        const int num = cloud_hull->size();
        if( num >= 3)
        {
            for(int i = 1; i < num; i++)
            {
                stringstream ss;
                ss << id << "_hull_line_" << i;
                pcl_viewer_->addLine(cloud_hull->points[i-1], cloud_hull->points[i], r, g, b, ss.str(), viewport );
            }
            stringstream ss;
            ss << id << "_hull_line_0";
            pcl_viewer_->addLine(cloud_hull->points[0], cloud_hull->points[num-1], r, g, b, ss.str(), viewport );
        }
    }
}

void Viewer::focusOnCamera( tf::Transform &pose )
{
    static tf::Transform rel = tf::Transform( tf::Quaternion(0, 0, 0, 1.0), tf::Vector3(0, 0, -0.4) );
    static tf::Transform rel2 = tf::Transform( tf::createQuaternionFromRPY(0, M_PI_4, 0), tf::Vector3(-3.0, 0, -1.0) );
    static tf::Transform rel3 = tf::Transform( tf::Quaternion(0, 0, 0, 1.0), tf::Vector3(0, 0, -3.0) );

    if( !focus_on_camera_ )
        return;

    tf::Transform cp = pose*rel2;
    tf::Transform vp = cp*rel3;
    map_viewer_->setCameraPosition(vp.getOrigin().x(), vp.getOrigin().y(), vp.getOrigin().z(),
                                   cp.getOrigin().x(), cp.getOrigin().y(), cp.getOrigin().z(),
                                   0.0, 0.0, 1.0);

//    tf::Transform vp = pose * rel;
//    map_viewer_->setCameraPosition(vp.getOrigin().x(), vp.getOrigin().y(), vp.getOrigin().z(),
//                                   pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z(),
//                                   0.0, 0.0, 1.0);
}

void Viewer::viewerReconfigCallback( plane_slam::ViewerConfig &config, uint32_t level)
{
    // parameter frame
    focus_on_camera_ = config.focus_on_camera;
    display_frame_ = config.display_frame;
    display_input_cloud_ = config.display_input_cloud;
    display_line_cloud_ = config.display_line_cloud;
    display_normal_ = config.display_normal;
    display_normal_arrow_ = config.display_normal_arrow;
    display_plane_ = config.display_plane;
    display_plane_number_ = config.display_plane_number;
    display_plane_arrow_ = config.display_plane_arrow;
    display_plane_inlier_ = config.display_plane_inlier;
    display_plane_projected_inlier_ = config.display_plane_projected_inlier;
    display_plane_boundary_ = config.display_plane_boundary;
    display_plane_hull_ = config.display_plane_hull;
    display_feature_cloud_ = config.display_feature_cloud;
    //
    show_keypoint_ = config.show_keypoint;
    show_keypoint_matches_ = config.show_keypoint_matches;
    display_3d_keypoint_matches_ = config.display_3d_keypoint_matches;
    // parameter for landmark
    display_plane_landmarks_ = config.display_plane_landmarks;
    display_point_landmarks_ = config.display_point_landmarks;
    display_landmark_inlier_ = config.display_landmark_inlier;
    display_landmark_arrow_ = config.display_landmark_arrow;
    display_landmark_number_ = config.display_landmark_number;
    display_landmark_boundary_ = config.display_landmark_boundary;
    display_landmark_hull_ = config.display_landmark_hull;
    display_landmark_label_ = config.display_landmark_label;
    //
    display_camera_fov_ = config.display_camera_fov;
    display_pathes_ = config.display_pathes;
    display_optimized_path_ = config.display_optimized_path;
    display_odom_path_ = config.display_odom_path;
    display_visual_odom_path_ = config.display_visual_odom_path;
    display_true_path_ = config.display_true_path;


    cout << GREEN <<" Viewer Config." << RESET << endl;
}

bool Viewer::autoSpinMapViewerCallback( std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res )
{
    bool added = false;
    auto_spin_map_viewer_ = true;
    ROS_INFO("Spin map viewer for 30 seconds.");
    double dura = 30.0;
    int sec = 0;
    ros::Time time = ros::Time::now();
    ros::Time finish_time = time + ros::Duration(dura);
    ros::Rate loop_rate( 40 );
    while( auto_spin_map_viewer_ && ros::ok())
    {
        if( !ros::ok() || ros::isShuttingDown() )
        {
            res.message = "Node shutdown.";
            auto_spin_map_viewer_ = false;
            return true;
        }

        // info
        if( ros::Time::now() > (time + ros::Duration(1.0)) )
        {
            time += ros::Duration(1.0);
            sec ++;
            cout << WHITE << "\rSpinning " << sec << " of " << (int)dura << " seconds...";
            flush(std::cout);
            //
            stringstream ss;
            ss << "Spinning " << sec << "/" << ((int)dura) << " seconds";
            if( ! added )
            {
                map_viewer_->addText( ss.str(), 100, 20, 0.0, 0.0, 0.0, "spinning_text" );
                added = true;
            }
            else
                map_viewer_->updateText( ss.str(), 100, 20, 0.0, 0.0, 0.0, "spinning_text" );
        }

        map_viewer_->spinOnce( 20 );
        loop_rate.sleep();

        if( ros::Time::now() > finish_time )
            auto_spin_map_viewer_ = false;
    }
    cout << endl;

    map_viewer_->updateText( " ", 100, 20, 0.0, 0.0, 0.0, "spinning_text" );
    map_viewer_->spinOnce( 20 );
    ROS_INFO("Stop spinning.");
    res.success = true;
    res.message = "Done spinning map viewer for 30 seconds.";
    return true;
}

} // end of namespace plane_slam
