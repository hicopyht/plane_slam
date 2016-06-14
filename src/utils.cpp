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
PointType transformPoint (const PointType &point,
                     const Eigen::Matrix4d &transform)
{
  PointType ret = point;
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

void cvToEigen(const cv::Mat& src, Eigen::Matrix3d& dst )
{
    cv::Mat _dst( src.rows, src.cols, cv::DataType<double>::type,
                  dst.data(), (size_t)(dst.stride()*sizeof(double)));
    src.convertTo( _dst, _dst.type() );
}

