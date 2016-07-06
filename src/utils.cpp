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

void printTransform( const Eigen::Matrix4d &transform)
{
    gtsam::Pose3 pose3( transform );
    cout << CYAN;
    cout << "  - R(rpy): " << pose3.rotation().roll()
         << ", " << pose3.rotation().pitch()
         << ", " << pose3.rotation().yaw() << endl;
    cout << "  - T:      " << pose3.translation().x()
         << ", " << pose3.translation().y()
         << ", " << pose3.translation().z() << RESET << endl;
}

void printTransform( const Eigen::Matrix4f &transform)
{
    Eigen::Matrix4d tr = transform.cast<double>();
    printTransform(tr);
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
//    cout << RED << " cv to eigen: " << endl;
//    cout << src << RESET << endl;

    dst(0,0) = src.at<double>(0, 0);
    dst(0,1) = src.at<double>(0, 1);
    dst(0,2) = src.at<double>(0, 2);
    dst(1,0) = src.at<double>(1, 0);
    dst(1,1) = src.at<double>(1, 1);
    dst(1,2) = src.at<double>(1, 2);
    dst(2,0) = src.at<double>(2, 0);
    dst(2,1) = src.at<double>(2, 1);
    dst(2,2) = src.at<double>(2, 2);

// not correct
//    cv::Mat _dst( src.rows, src.cols, cv::DataType<double>::type,
//                  dst.data(), (size_t)(dst.stride()*sizeof(double)));
//    src.convertTo( _dst, _dst.type() );
//    cout<< BLUE << dst << RESET << endl;
}


void tfToPose3( const tf::Transform &trans, gtsam::Pose3 &pose )
{
    Eigen::Matrix3d m33 = matrixTF2Eigen( trans.getBasis() );
    gtsam::Rot3 rot3(m33);
    gtsam::Point3 point3;
    tf::Vector3 origin = trans.getOrigin();
    point3[0] = origin.getX();
    point3[1] = origin.getY();
    point3[2] = origin.getZ();
    pose = gtsam::Pose3( rot3, point3 );
}

gtsam::Pose3 tfToPose3( const tf::Transform &trans)
{
    gtsam::Pose3 pose3;
    tfToPose3( trans, pose3 );
    return pose3;
}

void pose3ToTF( const gtsam::Pose3 &pose, tf::Transform &trans )
{
    trans.setOrigin( tf::Vector3( pose.x(), pose.y(), pose.z()) );
    tf::Matrix3x3 m33;
    matrixEigen2TF( pose.rotation().matrix(), m33 );
    trans.setBasis( m33 );
}

tf::Transform pose3ToTF( const gtsam::Pose3 &pose )
{
    tf::Transform trans;
    pose3ToTF( pose, trans);
    return trans;
}

geometry_msgs::PoseStamped pose3ToGeometryPose( const gtsam::Pose3 pose3 )
{
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = pose3.translation()[0];
    pose.pose.position.y = pose3.translation()[1];
    pose.pose.position.z = pose3.translation()[2];
    tf::Transform trans;
    trans.setOrigin( tf::Vector3( pose3.translation()[0], pose3.translation()[1], pose3.translation()[2] ) );
    trans.setBasis( matrixEigen2TF( pose3.rotation().matrix() ) );
    tf::quaternionTFToMsg( trans.getRotation(), pose.pose.orientation );

    return pose;
}

geometry_msgs::PoseStamped tfToGeometryPose( const tf::Transform &trans )
{
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = trans.getOrigin().x();
    pose.pose.position.y = trans.getOrigin().y();
    pose.pose.position.z = trans.getOrigin().z();
    tf::quaternionTFToMsg( trans.getRotation(), pose.pose.orientation );

    return pose;
}

geometry_msgs::PoseStamped motionToGeometryPose( const RESULT_OF_MOTION &motion )
{
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = motion.translation.x();
    pose.pose.position.y = motion.translation.y();
    pose.pose.position.z = motion.translation.z();
    tf::Transform trans;
    trans.setOrigin( tf::Vector3( motion.translation.x(), motion.translation.y(), motion.translation.z() ) );
    trans.setBasis( matrixEigen2TF(motion.rotation) );
    tf::quaternionTFToMsg( trans.getRotation(), pose.pose.orientation );

    return pose;
}

static inline int hamming_distance_orb32x8_popcountll(const uint64_t* v1, const uint64_t* v2) {
  return (__builtin_popcountll(v1[0] ^ v2[0]) + __builtin_popcountll(v1[1] ^ v2[1])) +
         (__builtin_popcountll(v1[2] ^ v2[2]) + __builtin_popcountll(v1[3] ^ v2[3]));
}

int bruteForceSearchORB(const uint64_t* v, const uint64_t* search_array, const unsigned int& size, int& result_index)
{
    //constexpr unsigned int howmany64bitwords = 4;//32*8/64;
    const unsigned int howmany64bitwords = 4;//32*8/64;
    result_index = -1;//impossible
    int min_distance = 1 + 256;//More than maximum distance
    for(unsigned int i = 0; i < size-1; i+=1, search_array+=4)
    {
        int hamming_distance_i = hamming_distance_orb32x8_popcountll(v, search_array);
        if(hamming_distance_i < min_distance)
        {
            min_distance = hamming_distance_i;
            result_index = i;
        }
    }
    return min_distance;
}

//// Test solveRelativeTransform()
//    // Match features
//    vector<cv::DMatch> good_matches;
//    if( is_initialized )
//    {
//        cout << GREEN << "Last features = " << last_frame.feature_locations_3d.size()
//             << ", current features = " << frame.feature_locations_3d.size() << RESET << endl;
//        matchImageFeatures( last_frame, frame, good_matches,
//                            feature_good_match_threshold_, feature_min_good_match_size_ );
//        cout << GREEN << "Match features th = " << feature_good_match_threshold_
//             << ", good matches = " << good_matches.size() << RESET << endl;
//        cv::Mat image_matches;
//        cv::drawMatches( last_frame.visual_image, last_frame.feature_locations_2d,
//                         frame.visual_image, frame.feature_locations_2d,
//                         good_matches, image_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
//                         vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
//        cv::imshow( MatchesWindow, image_matches );
////        cv::waitKey( 1 );
//    }
//    cv::waitKey( 1 );
//    match_dura = time.toc();
//    time.tic();

//    if( is_initialized )
//    {
//        RESULT_OF_MOTION motion;
//        cout << YELLOW << "******************* Test estimate transformation *******************" << RESET << endl;
//        std::vector<cv::DMatch> inlier;
//        solveRelativeTransform( last_frame, frame, motion, inlier );
//        cout << MAGENTA << "relative transformation: " << RESET << endl;
//        printTransform( motion.transform4d() );
//        cout << YELLOW << "********************************************************************" << RESET << endl;
//    }


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

// test errorFunction2
//void testErrorFunction2()
//{
//    Eigen::Vector4f v1( 0.215883, 0.394139, 1.108, 1.0);
//    Eigen::Vector4f v2( 0.315883, 0.694139, 0.8, 1.0);
//    Eigen::Vector4f v3( -0.215883, 0.394139, 1.5, 1.0);
//    Eigen::Vector4f v4( 0.115883, -0.394139, 1.2, 1.0);
//    Eigen::Vector4f tv1, tv2, tv3, tv4;
//    tv1 += Eigen::Vector4f( 0.01, 0.01, 0.01, 0);
//    tv2 += Eigen::Vector4f( -0.01, 0.01, 0.004, 0);
//    tv3 += Eigen::Vector4f( 0.005, -0.005, 0.002, 0);
//    tv4 += Eigen::Vector4f( 0.0001, -0.0001, 0.001, 0);

//    /// 1: Define transform
//    Eigen::Affine3d tr = Eigen::Translation3d(0.05, 0.06, 0.08)
//            * Eigen::AngleAxisd( 0.3, Eigen::Vector3d::UnitZ())
//            * Eigen::AngleAxisd( 0.2, Eigen::Vector3d::UnitY())
//            * Eigen::AngleAxisd( 0.1, Eigen::Vector3d::UnitX());
//    Eigen::Matrix4f trf = tr.matrix().cast<float>();
//    tv1 = trf*v1;
//    tv2 = trf*v2;
//    tv3 = trf*v3;
//    tv4 = trf*v4;
//    double error = errorFunction2( v1, tv1, tr.matrix() );
//    cout << BLUE << "Test error = " << error << RESET << endl;
//    error = errorFunction2( v2, tv2, tr.matrix() );
//    cout << BLUE << "Test error = " << error << RESET << endl;
//    error = errorFunction2( v3, tv3, tr.matrix() );
//    cout << BLUE << "Test error = " << error << RESET << endl;
//    error = errorFunction2( v4, tv4, tr.matrix() );
//    cout << BLUE << "Test error = " << error << RESET << endl;

//    // get transform
//    pcl::TransformationFromCorrespondences tfc;
//    float weight = 1.0;
//    weight = 1.0/(v1(2) * tv1(2));
//    tfc.add( v1.head<3>(), tv1.head<3>(), weight );
//    weight = 1.0/(v2(2) * tv2(2));
//    tfc.add( v2.head<3>(), tv2.head<3>(), weight );
//    weight = 1.0/(v3(2) * tv3(2));
//    tfc.add( v3.head<3>(), tv3.head<3>(), weight );
//    weight = 1.0/(v4(2) * tv4(2));
//    tfc.add( v4.head<3>(), tv4.head<3>(), weight );

//    Eigen::Matrix4f motion = tfc.getTransformation().matrix();
//    printTransform( motion );

//}


//void testPnP()
//{
//if( is_initialized )
//{
//    RESULT_OF_PNP motion;
//    bool res = solveRelativeTransformPnP( last_frame, frame, good_matches, camera_parameters_, motion );
////        if( res )
//    {
//        // print motion
//        gtsam::Rot3 rot3( motion.rotation );
//        cout << YELLOW << " estimated motion PnP, inlier = " << motion.inliers << endl;
//        cout << "  - R(rpy): " << rot3.roll() << ", " << rot3.pitch() << ", " << rot3.yaw() << endl;
//        cout << "  - T:      " << motion.translation[0]
//             << ", " << motion.translation[1]
//             << ", " << motion.translation[2] << RESET << endl;
//    }
////        if( !res )
////        {
////            cout << RED << " failed to estimate relative motion using RANSAC. " << RESET << endl;
////        }
//}
//}


