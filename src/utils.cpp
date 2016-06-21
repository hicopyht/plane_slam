#include "utils.h"

void matrixTF2Eigen(const tf::Matrix3x3 &t, Eigen::Matrix3d &e)
{
    e.matrix()(0,0) = t[0][0];
    e.matrix()(0,1) = t[0][1];
    e.matrix()(0,2) = t[0][2];
    e.matrix()(1,0) = t[1][0];
    e.matrix()(1,1) = t[1][1];
    e.matrix()(1,2) = t[1][2];
    e.matrix()(2,0) = t[2][0];
    e.matrix()(2,1) = t[2][1];
    e.matrix()(2,2) = t[2][2];
}

Eigen::Matrix3d matrixTF2Eigen(const tf::Matrix3x3 &t)
{
    Eigen::Matrix3d m33;
    matrixTF2Eigen(t, m33);
    return m33;
}

void matrixEigen2TF( const Eigen::Matrix3d &e, tf::Matrix3x3 &t)
{
    t[0][0] = e.matrix()(0,0);
    t[0][1] = e.matrix()(0,1);
    t[0][2] = e.matrix()(0,2);
    t[1][0] = e.matrix()(1,0);
    t[1][1] = e.matrix()(1,1);
    t[1][2] = e.matrix()(1,2);
    t[2][0] = e.matrix()(2,0);
    t[2][1] = e.matrix()(2,1);
    t[2][2] = e.matrix()(2,2);
}

tf::Matrix3x3 matrixEigen2TF(const Eigen::Matrix3d &m33)
{
    tf::Matrix3x3 t;
    matrixEigen2TF( m33 , t );
    return t;
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
PointT transformPoint (const PointT &point,
                     const Eigen::Matrix4d &transform)
{
  PointT ret = point;
  //ret.getVector3fMap () = transform * point.getVector3fMap ();
  ret.x = static_cast<float> (transform (0, 0) * point.x + transform (0, 1) * point.y + transform (0, 2) * point.z + transform (0, 3));
  ret.y = static_cast<float> (transform (1, 0) * point.x + transform (1, 1) * point.y + transform (1, 2) * point.z + transform (1, 3));
  ret.z = static_cast<float> (transform (2, 0) * point.x + transform (2, 1) * point.y + transform (2, 2) * point.z + transform (2, 3));

  return (ret);
}

template <typename PointT>
PointT transformPoint (const PointT &point,
                     const Eigen::Matrix4f &transform)
{
  PointT ret = point;
  //ret.getVector3fMap () = transform * point.getVector3fMap ();
  ret.x = static_cast<float> (transform (0, 0) * point.x + transform (0, 1) * point.y + transform (0, 2) * point.z + transform (0, 3));
  ret.y = static_cast<float> (transform (1, 0) * point.x + transform (1, 1) * point.y + transform (1, 2) * point.z + transform (1, 3));
  ret.z = static_cast<float> (transform (2, 0) * point.x + transform (2, 1) * point.y + transform (2, 2) * point.z + transform (2, 3));

  return (ret);
}

void transformPointCloud (const PointCloudType &cloud_in,
                          PointCloudType &cloud_out,
                          const Eigen::Matrix4d &transform)
{
    if (&cloud_in != &cloud_out)
    {
        // Note: could be replaced by cloud_out = cloud_in
        cloud_out.header   = cloud_in.header;
        cloud_out.is_dense = cloud_in.is_dense;
        cloud_out.width    = cloud_in.width;
        cloud_out.height   = cloud_in.height;
        cloud_out.points.reserve (cloud_out.points.size ());
        cloud_out.points.assign (cloud_in.points.begin (), cloud_in.points.end ());
        cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
        cloud_out.sensor_origin_      = cloud_in.sensor_origin_;
    }

    if (cloud_in.is_dense)
    {
        // If the dataset is dense, simply transform it!
        for (size_t i = 0; i < cloud_out.points.size (); ++i)
        {
            //cloud_out.points[i].getVector3fMap () = transform * cloud_in.points[i].getVector3fMap ();
            Eigen::Matrix<double, 3, 1> pt (cloud_in[i].x, cloud_in[i].y, cloud_in[i].z);
            cloud_out[i].x = static_cast<float> (transform (0, 0) * pt.coeffRef (0) + transform (0, 1) * pt.coeffRef (1) + transform (0, 2) * pt.coeffRef (2) + transform (0, 3));
            cloud_out[i].y = static_cast<float> (transform (1, 0) * pt.coeffRef (0) + transform (1, 1) * pt.coeffRef (1) + transform (1, 2) * pt.coeffRef (2) + transform (1, 3));
            cloud_out[i].z = static_cast<float> (transform (2, 0) * pt.coeffRef (0) + transform (2, 1) * pt.coeffRef (1) + transform (2, 2) * pt.coeffRef (2) + transform (2, 3));
        }
    }
    else
    {
        // Dataset might contain NaNs and Infs, so check for them first,
        // otherwise we get errors during the multiplication (?)
        for (size_t i = 0; i < cloud_out.points.size (); ++i)
        {
            if (!pcl_isfinite (cloud_in.points[i].x) || !pcl_isfinite (cloud_in.points[i].y) ||
                !pcl_isfinite (cloud_in.points[i].z))
                continue;
            //cloud_out.points[i].getVector3fMap () = transform * cloud_in.points[i].getVector3fMap ();
            Eigen::Matrix<double, 3, 1> pt (cloud_in[i].x, cloud_in[i].y, cloud_in[i].z);
            cloud_out[i].x = static_cast<float> (transform (0, 0) * pt.coeffRef (0) + transform (0, 1) * pt.coeffRef (1) + transform (0, 2) * pt.coeffRef (2) + transform (0, 3));
            cloud_out[i].y = static_cast<float> (transform (1, 0) * pt.coeffRef (0) + transform (1, 1) * pt.coeffRef (1) + transform (1, 2) * pt.coeffRef (2) + transform (1, 3));
            cloud_out[i].z = static_cast<float> (transform (2, 0) * pt.coeffRef (0) + transform (2, 1) * pt.coeffRef (1) + transform (2, 2) * pt.coeffRef (2) + transform (2, 3));
        }
    }
}

void transformPointCloud (const PointCloudType &cloud_in,
                          PointCloudType &cloud_out,
                          const Eigen::Matrix4d &transform,
                          const RGBValue &color)
{
    if (&cloud_in != &cloud_out)
    {
        // Note: could be replaced by cloud_out = cloud_in
        cloud_out.header   = cloud_in.header;
        cloud_out.is_dense = cloud_in.is_dense;
        cloud_out.width    = cloud_in.width;
        cloud_out.height   = cloud_in.height;
        cloud_out.points.reserve (cloud_out.points.size ());
        cloud_out.points.assign (cloud_in.points.begin (), cloud_in.points.end ());
        cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
        cloud_out.sensor_origin_      = cloud_in.sensor_origin_;
    }

    if (cloud_in.is_dense)
    {
        // If the dataset is dense, simply transform it!
        for (size_t i = 0; i < cloud_out.points.size (); ++i)
        {
            //cloud_out.points[i].getVector3fMap () = transform * cloud_in.points[i].getVector3fMap ();
            Eigen::Matrix<double, 3, 1> pt (cloud_in[i].x, cloud_in[i].y, cloud_in[i].z);
            cloud_out[i].x = static_cast<float> (transform (0, 0) * pt.coeffRef (0) + transform (0, 1) * pt.coeffRef (1) + transform (0, 2) * pt.coeffRef (2) + transform (0, 3));
            cloud_out[i].y = static_cast<float> (transform (1, 0) * pt.coeffRef (0) + transform (1, 1) * pt.coeffRef (1) + transform (1, 2) * pt.coeffRef (2) + transform (1, 3));
            cloud_out[i].z = static_cast<float> (transform (2, 0) * pt.coeffRef (0) + transform (2, 1) * pt.coeffRef (1) + transform (2, 2) * pt.coeffRef (2) + transform (2, 3));
//            cloud_out[i].rgb = color.float_value;
            cloud_out[i].rgba = color.float_value;
        }
    }
    else
    {
        // Dataset might contain NaNs and Infs, so check for them first,
        // otherwise we get errors during the multiplication (?)
        for (size_t i = 0; i < cloud_out.points.size (); ++i)
        {
            if (!pcl_isfinite (cloud_in.points[i].x) || !pcl_isfinite (cloud_in.points[i].y) ||
                !pcl_isfinite (cloud_in.points[i].z))
                continue;
            //cloud_out.points[i].getVector3fMap () = transform * cloud_in.points[i].getVector3fMap ();
            Eigen::Matrix<double, 3, 1> pt (cloud_in[i].x, cloud_in[i].y, cloud_in[i].z);
            cloud_out[i].x = static_cast<float> (transform (0, 0) * pt.coeffRef (0) + transform (0, 1) * pt.coeffRef (1) + transform (0, 2) * pt.coeffRef (2) + transform (0, 3));
            cloud_out[i].y = static_cast<float> (transform (1, 0) * pt.coeffRef (0) + transform (1, 1) * pt.coeffRef (1) + transform (1, 2) * pt.coeffRef (2) + transform (1, 3));
            cloud_out[i].z = static_cast<float> (transform (2, 0) * pt.coeffRef (0) + transform (2, 1) * pt.coeffRef (1) + transform (2, 2) * pt.coeffRef (2) + transform (2, 3));
            cloud_out[i].rgb = color.float_value;
        }
    }
}

void transformPlane( const Eigen::Vector4d &input,
                     const Eigen::Matrix4d &transform,
                     Eigen::Vector4d &output)
{
    gtsam::OrientedPlane3 p1(input);
    gtsam::Pose3 pose(transform);
    output = p1.transform( pose ).planeCoefficients();
}

void projectPoints ( const PointCloudType &input,
                    const Eigen::Vector4d &model_coefficients,
                    PointCloudType &projected_points )
{
    Eigen::Vector4f coefficients;
    coefficients[0] = model_coefficients[0];
    coefficients[1] = model_coefficients[1];
    coefficients[2] = model_coefficients[2];
    coefficients[3] = model_coefficients[3];
    projectPoints( input, coefficients, projected_points );
}

void projectPoints ( const PointCloudType &input,
                    const Eigen::Vector4f &model_coefficients,
                    PointCloudType &projected_points )
{
    projected_points.header = input.header;
    projected_points.is_dense = input.is_dense;

    Eigen::Vector4f mc (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);

    // normalize the vector perpendicular to the plane...
    mc.normalize ();
    // ... and store the resulting normal as a local copy of the model coefficients
    Eigen::Vector4f tmp_mc = model_coefficients;
    tmp_mc[0] = mc[0];
    tmp_mc[1] = mc[1];
    tmp_mc[2] = mc[2];

    // Allocate enough space and copy the basics
    projected_points.points.resize (input.size ());
    projected_points.width    = static_cast<uint32_t> ( input.size() );
    projected_points.height   = 1;

    typedef typename pcl::traits::fieldList<PointType>::type FieldList;
    // Iterate over each point
    for (size_t i = 0; i < input.size (); ++i)
        // Iterate over each dimension
        pcl::for_each_type <FieldList> (pcl::NdConcatenateFunctor <PointType, PointType> (input.points[i], projected_points.points[i]));

    // Iterate through the 3d points and calculate the distances from them to the plane
    for (size_t i = 0; i < input.size (); ++i)
    {
        // Calculate the distance from the point to the plane
        Eigen::Vector4f p (input.points[i].x,
                            input.points[i].y,
                            input.points[i].z,
                            1);
        // use normalized coefficients to calculate the scalar projection
        float distance_to_plane = tmp_mc.dot (p);

        pcl::Vector4fMap pp = projected_points.points[i].getVector4fMap ();
        pp.matrix () = p - mc * distance_to_plane;        // mc[3] = 0, therefore the 3rd coordinate is safe
    }
}

void projectPoints ( const PointCloudType &input, const std::vector<int> &inlier,
                    const Eigen::Vector4d &model_coefficients, PointCloudType &projected_points )
{
    Eigen::Vector4f coefficients;
    coefficients[0] = model_coefficients[0];
    coefficients[1] = model_coefficients[1];
    coefficients[2] = model_coefficients[2];
    coefficients[3] = model_coefficients[3];
    projectPoints( input, inlier, coefficients, projected_points );
}

void projectPoints ( const PointCloudType &input, const std::vector<int> &inlier,
                    const Eigen::Vector4f &model_coefficients, PointCloudType &projected_points )
{
    projected_points.header = input.header;
    projected_points.is_dense = input.is_dense;

    Eigen::Vector4f mc (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);

    // normalize the vector perpendicular to the plane...
    mc.normalize ();
    // ... and store the resulting normal as a local copy of the model coefficients
    Eigen::Vector4f tmp_mc = model_coefficients;
    tmp_mc[0] = mc[0];
    tmp_mc[1] = mc[1];
    tmp_mc[2] = mc[2];

    // Allocate enough space and copy the basics
    projected_points.points.resize (inlier.size ());
    projected_points.width    = static_cast<uint32_t> (inlier.size ());
    projected_points.height   = 1;

    typedef typename pcl::traits::fieldList<PointType>::type FieldList;
    // Iterate over each point
    for (size_t i = 0; i < inlier.size (); ++i)
        // Iterate over each dimension
        pcl::for_each_type <FieldList> (pcl::NdConcatenateFunctor <PointType, PointType> (input.points[inlier[i]], projected_points.points[i]));

    // Iterate through the 3d points and calculate the distances from them to the plane
    for (size_t i = 0; i < inlier.size (); ++i)
    {
        // Calculate the distance from the point to the plane
        Eigen::Vector4f p (input.points[inlier[i]].x,
                            input.points[inlier[i]].y,
                            input.points[inlier[i]].z,
                            1);
        // use normalized coefficients to calculate the scalar projection
        float distance_to_plane = tmp_mc.dot (p);

        pcl::Vector4fMap pp = projected_points.points[i].getVector4fMap ();
        pp.matrix () = p - mc * distance_to_plane;        // mc[3] = 0, therefore the 3rd coordinate is safe
    }
}

void depthToCV8UC1(cv::Mat& depth_img, cv::Mat& mono8_img)
{
    //Process images
    if(depth_img.type() == CV_32FC1)
    {
        depth_img.convertTo(mono8_img, CV_8UC1, 100,0); //milimeter (scale of mono8_img does not matter)
    }
    else if(depth_img.type() == CV_16UC1)
    {
        mono8_img = cv::Mat(depth_img.size(), CV_8UC1);
        cv::Mat float_img;
        depth_img.convertTo(mono8_img, CV_8UC1, 0.05, -25); //scale to 2cm
        depth_img.convertTo(float_img, CV_32FC1, 0.001, 0);//From mm to m(scale of depth_img matters)
        depth_img = float_img;
    }
    else
    {
        ROS_ERROR_STREAM("Don't know how to handle depth image of type ");
    }
}

// https://github.com/felixendres/rgbdslam_v2
double errorFunction2(const Eigen::Vector4f& x1,
                      const Eigen::Vector4f& x2,
                      const Eigen::Matrix4d& transformation)
{
    //FIXME: Take from paramter_server or cam info
    static const double cam_angle_x = 58.0/180.0*M_PI;/*{{{*/
    static const double cam_angle_y = 45.0/180.0*M_PI;
    static const double cam_resol_x = 640;
    static const double cam_resol_y = 480;
    static const double raster_stddev_x = 3*tan(cam_angle_x/cam_resol_x);  //5pix stddev in x
    static const double raster_stddev_y = 3*tan(cam_angle_y/cam_resol_y);  //5pix stddev in y
    static const double raster_cov_x = raster_stddev_x * raster_stddev_x;
    static const double raster_cov_y = raster_stddev_y * raster_stddev_y;/*}}}*/
    static const bool use_error_shortcut = true;//ParameterServer::instance()->get<bool>("use_error_shortcut");

    bool nan1 = std::isnan(x1(2));
    bool nan2 = std::isnan(x2(2));
    if(nan1||nan2)
    {
        //TODO: Handle Features with NaN, by reporting the reprojection error
        return std::numeric_limits<double>::max();
    }
    Eigen::Vector4d x_1 = x1.cast<double>();
    Eigen::Vector4d x_2 = x2.cast<double>();

    Eigen::Matrix4d tf_12 = transformation;
    Eigen::Vector3d mu_1 = x_1.head<3>();
    Eigen::Vector3d mu_2 = x_2.head<3>();
    Eigen::Vector3d mu_1_in_frame_2 = (tf_12 * x_1).head<3>(); // μ₁⁽²⁾  = T₁₂ μ₁⁽¹⁾
    //New Shortcut to determine clear outliers
    if(use_error_shortcut)
    {
        double delta_sq_norm = (mu_1_in_frame_2 - mu_2).squaredNorm();
        double sigma_max_1 = std::max(raster_cov_x, depth_covariance(mu_1(2)));//Assuming raster_cov_x and _y to be approx. equal
        double sigma_max_2 = std::max(raster_cov_x, depth_covariance(mu_2(2)));//Assuming raster_cov_x and _y to be approx. equal
        if(delta_sq_norm > 2.0 * (sigma_max_1+sigma_max_2)) //FIXME: Factor 3 for mahal dist should be gotten from caller
        {
            return std::numeric_limits<double>::max();
        }
    }

    Eigen::Matrix3d rotation_mat = tf_12.block(0,0,3,3);

    //Point 1
    Eigen::Matrix3d cov1 = Eigen::Matrix3d::Zero();
    cov1(0,0) = raster_cov_x * mu_1(2); //how big is 1px std dev in meter, depends on depth
    cov1(1,1) = raster_cov_y * mu_1(2); //how big is 1px std dev in meter, depends on depth
    cov1(2,2) = depth_covariance(mu_1(2));

    //Point2
    Eigen::Matrix3d cov2 = Eigen::Matrix3d::Zero();
    cov2(0,0) = raster_cov_x* mu_2(2); //how big is 1px std dev in meter, depends on depth
    cov2(1,1) = raster_cov_y* mu_2(2); //how big is 1px std dev in meter, depends on depth
    cov2(2,2) = depth_covariance(mu_2(2));

    Eigen::Matrix3d cov1_in_frame_2 = rotation_mat.transpose() * cov1 * rotation_mat;//Works since the cov is diagonal => Eig-Vec-Matrix is Identity

    // Δμ⁽²⁾ =  μ₁⁽²⁾ - μ₂⁽²⁾
    Eigen::Vector3d delta_mu_in_frame_2 = mu_1_in_frame_2 - mu_2;

    if( std::isnan(delta_mu_in_frame_2(2) ))
    {
        ROS_ERROR("Unexpected NaN");
        return std::numeric_limits<double>::max();
    }
    // Σc = (Σ₁ + Σ₂)
    Eigen::Matrix3d cov_mat_sum_in_frame_2 = cov1_in_frame_2 + cov2;
    //ΔμT Σc⁻¹Δμ
    //double sqrd_mahalanobis_distance = delta_mu_in_frame_2.transpose() * cov_mat_sum_in_frame_2.inverse() * delta_mu_in_frame_2;
    double sqrd_mahalanobis_distance = delta_mu_in_frame_2.transpose() *cov_mat_sum_in_frame_2.llt().solve(delta_mu_in_frame_2);

    if(!(sqrd_mahalanobis_distance >= 0.0))
    {
        return std::numeric_limits<double>::max();
    }
    return sqrd_mahalanobis_distance;
}


void cvToEigen(const cv::Mat& src, Eigen::Matrix3d& dst )
{
    cv::Mat _dst( src.rows, src.cols, cv::DataType<double>::type,
                  dst.data(), (size_t)(dst.stride()*sizeof(double)));
    src.convertTo( _dst, _dst.type() );
}


//// test point RT
//// test solveRT points&planes
//void testRT()
//{
//    /// 1: Define transform
//    Eigen::Affine3d tr = Eigen::Translation3d(0.05, 0.06, 0.08)
//            * Eigen::AngleAxisd( 0.3, Eigen::Vector3d::UnitZ())
//            * Eigen::AngleAxisd( 0.2, Eigen::Vector3d::UnitY())
//            * Eigen::AngleAxisd( 0.1, Eigen::Vector3d::UnitX());;

//    /// 2: Construct points
//    PointType p1, p2, p3;
//    PointType tp1, tp2, tp3;
//    p1.x = 0.1;           p1.y = 0.6;             p1.z = 2.0;
//    p2.x = p1.x + 0.7;    p2.y = p1.y + 0.1,      p2.z = p1.z + 0.1;
//    p3.x = p1.x + 0.2;    p3.y = p1.y + 1.2,      p3.z = p1.z - 0.1;
//    tp1 = transformPoint( p1, tr.matrix() );
//    tp2 = transformPoint( p2, tr.matrix() );
//    tp3 = transformPoint( p3, tr.matrix() );
//    // Select 2 points
//    std::vector<Eigen::Vector3d> from_points;
//    std::vector<Eigen::Vector3d> to_points;
//    from_points.push_back( Eigen::Vector3d(p1.x, p1.y, p1.z) );
////        from_points.push_back( Eigen::Vector3d(p2.x, p2.y, p2.z) );
////        from_points.push_back( Eigen::Vector3d(p3.x, p3.y, p3.z) );
//    to_points.push_back( Eigen::Vector3d(tp1.x, tp1.y, tp1.z) );
////        to_points.push_back( Eigen::Vector3d(tp2.x, tp2.y, tp2.z) );
////        to_points.push_back( Eigen::Vector3d(tp3.x, tp3.y, tp3.z) );

//    /// 3: Construct planes
//    PlaneCoefficients plane1, plane2, plane3;
//    PlaneCoefficients tplane1, tplane2, tplane3;
//    plane1.head<3>() = gtsam::Unit3( 0.1, -0.2, -0.8 ).unitVector();
//    plane1(3) = 1.8;
//    plane2.head<3>() = gtsam::Unit3( 0.8, -0.4, -0.8 ).unitVector();
//    plane2(3) = 2.2;
//    plane3.head<3>() = gtsam::Unit3( 0.1, -0.8, -0.1 ).unitVector();
//    plane3(3) = 0.8;
//    transformPlane( plane1, tr.matrix(), tplane1 );
//    transformPlane( plane2, tr.matrix(), tplane2 );
//    transformPlane( plane3, tr.matrix(), tplane3 );
//    // Select 1 plane
//    std::vector<PlaneCoefficients> planes, tplanes;
////        planes.push_back( plane1 );
//    planes.push_back( plane2 );
//    planes.push_back( plane3 );
////        tplanes.push_back( tplane1 );
//    tplanes.push_back( tplane2 );
//    tplanes.push_back( tplane3 );

//    /// 4: solve RT
//    RESULT_OF_PNP pmotion;
//    solveRT( tplanes, planes, from_points, to_points, pmotion );
////        solveRT( tplanes, planes, pmotion );
////        solveRT( from_points, to_points, pmotion );

//    /// 5: print result
//    gtsam::Rot3 rot3( pmotion.rotation );
////        cout << YELLOW << " test relative motion 3 points: " << endl;
////        cout << YELLOW << " test relative motion 3 planes: " << endl;
////        cout << YELLOW << " test relative motion 2 points & 1 plane: " << endl;
//    cout << YELLOW << " test relative motion 1 point & 2 planes: " << endl;
//    cout << "  - R(rpy): " << rot3.roll() << ", " << rot3.pitch() << ", " << rot3.yaw() << endl;
//    cout << "  - T:      "
//         << pmotion.translation(0) << ", "
//         << pmotion.translation(1) << ", "
//         << pmotion.translation(2) << RESET << endl;
//    cout << CYAN << " true motion: " << endl;
//    cout << "  - R(rpy): " << 0.1 << ", " << 0.2 << ", " << 0.3 << endl;
//    cout << "  - T:      " << "0.05, 0.06, 0.08" << RESET << endl;
//}


