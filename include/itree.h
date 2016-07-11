#ifndef ITREE_H
#define ITREE_H

#include "utils.h"

namespace plane_slam
{

class ITree
{
public:
    ITree();

    static bool euclidianPlaneCorrespondences(const vector<PlaneType> &planes,
                                           const vector<PlaneType> &last_planes,
                                           vector<PlanePair> &pairs,
                                           const Eigen::Matrix4d &estimated_transform = Eigen::MatrixXd::Identity(4,4),
                                           const double direction_threshold = 8.0,
                                           const double distance_threshold = 0.1);

    bool iTreeAssociate(const vector<PlaneType> &measurements, const vector<PlaneType> &landmarks, vector<PlanePair> &pairs);

    void nearestNeighborAssociate(const vector<PlaneType> &measurements, const vector<PlaneType> &landmarks, vector<PlanePair> &pairs);

    bool unaryMatch( const PlaneType &obs, const PlaneType &landmark);

    bool binaryMatch( const PlaneType &obs1, const PlaneType &obs2, const PlaneType &lm1, const PlaneType &lm2);

    static bool checkCoPlanar( const PlaneType &lm1, const PlaneType &lm2, const double angular_threshold = 15.0);

    static bool checkPlanesOverlap( const PlaneType &lm1, const PlaneType &lm2, const double &overlap = 0.5);

    static void euclidianDistance(const PlaneType &p1, const PlaneType &p2, double &direction, double &distance);

    static void euclidianDistance(const PlaneCoefficients &p1, const PlaneCoefficients &p2, double &direction, double &distance);

    static float distancePointToPlane( Eigen::Vector4f point, Eigen::Vector4d model_coefficients)
    {
        float distance = fabs ( model_coefficients[0] * point[0] + model_coefficients[1] * point[1]
                + model_coefficients[2] * point[2] + model_coefficients[3]);
        return distance;
    }

    /////////////////////////////////////////////////////////

    static bool checkPointCorrespondences( const std_vector_of_eigen_vector4f &froms,
                                    const std_vector_of_eigen_vector4f &tos,
                                    const double distance_threshold = 0.1);

    template <typename PointT>
    static bool checkPointCorrespondences( const std::vector<PointT> &froms,
                                    const std::vector<PointT> &tos,
                                    const double distance_threshold = 0.1);

    template <typename PointT>
    static float euclidianDistance( const PointT &p1, const PointT &p2)
    {
        return ( (p1.getVector4fMap() - p2.getVector4fMap()).norm() );
    }

    template <typename PointT>
    static float euclidianSquaredDistance( const PointT &p1, const PointT &p2 )
    {
        return ( (p1.getVector4fMap() - p2.getVector4fMap()).squaredNorm() );
    }

    template <typename PointT>
    static float euclidianSquaredDistance( const PointT &from, const PointT &to, const Eigen::Matrix4f &transform )
    {
        PointT pp = transformPoint( from, transform.inverse() );
        return ( (pp.getVector4fMap() - to.getVector4fMap()).squaredNorm() );
    }

    static float euclidianSquaredDistance( const Eigen::Vector4f &from, const Eigen::Vector4f &to, const Eigen::Matrix4f &trans_inv )
    {
        Eigen::Vector4f tto = trans_inv * from;
        return ( ( tto - to ).squaredNorm() );
    }

private:

};

} // end of namespace plane_slam

#endif // ITREE_H
