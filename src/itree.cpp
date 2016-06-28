#include "itree.h"

ITree::ITree()
{

}

bool ITree::euclidianPlaneCorrespondences( const vector<PlaneType> &planes,
                                       const vector<PlaneType> &last_planes,
                                       vector<PlanePair> &pairs,
                                       const Eigen::Matrix4d &estimated_transform,
                                       const double direction_threshold,
                                       const double distance_threshold )
{
    /// 1: Transform
    vector<PlaneType> predict_planes;
    const Eigen::Matrix4d transform = estimated_transform;
    for(int i = 0; i < last_planes.size(); i++)
    {
        const PlaneType &plane = last_planes[i];
        PlaneType p = plane;
        transformPlane( plane.coefficients, transform, p.coefficients);
        // TODO transform inlier point?
        predict_planes.push_back( p );
    }

    /// 2: Find correspondences
    const double direction_thresh = direction_threshold * DEG_TO_RAD;
    const double distance_thresh = distance_threshold;
    Eigen::VectorXd paired = Eigen::VectorXd::Zero( predict_planes.size() );
    for( int i = 0; i < planes.size(); i++)
    {
        const PlaneType &plane = planes[i];
        //
        for( int j = 0; j < predict_planes.size(); j++)
        {
            if( paired[j] ) // already paired
                continue;

            PlaneType &predicted = predict_planes[j];

            double dir_error, d_error;
            euclidianDistance( plane, predicted, dir_error, d_error);
            if( (fabs(dir_error) < direction_thresh )
                    && (d_error < distance_thresh) )
            {
                // check overlap
                bool overlap;
                if( plane.cloud->size() < predicted.cloud->size() )
                    overlap = checkPlanesOverlap( predicted, plane, 0.5 );
                else
                    overlap = checkPlanesOverlap( plane, predicted, 0.5 );
                if(overlap)
                {
                    paired[j] = 1;
                    pairs.push_back( PlanePair((unsigned int)i, (unsigned int)j, (dir_error+d_error) ));
                }
            }
        }
    }

    cout << MAGENTA << " plane pairs = " << pairs.size() << " (iobs, ilm, distance): " << endl;
    for( int i = 0; i < pairs.size(); i++)
    {
        cout << " - " << i << ": (" << pairs[i].iobs << ", " << pairs[i].ilm
             << ", " << pairs[i].distance << ")" << endl;
    }
    cout << RESET << endl;

//    std::sort( pairs.begin(), pairs.end() );
    return true;
}

void ITree::euclidianDistance(const PlaneType &p1, const PlaneType &p2, double &direction, double &distance)
{
    Eigen::Vector3d n1 = p1.coefficients.head<3>();
    Eigen::Vector3d n2 = p2.coefficients.head<3>();
    // normal angle
    const double ac = n1.dot(n2);
    if( ac == 1 )
        direction = 0;
    else if( ac == -1)
        direction = M_PI;
    else
        direction = std::acos( ac );
    // distance
    distance = fabs( p1.coefficients[3] - p2.coefficients[3] );
//    cout << CYAN << " n1.dot(n2): " << ac << RESET << endl;
//    cout << CYAN << " n1: " << n1(0) << ", " << n1(1) << ", " << n1(2) << RESET << endl;
//    cout << CYAN << " n2: " << n2(0) << ", " << n2(1) << ", " << n2(2) << RESET << endl;
}

void ITree::euclidianDistance(const PlaneCoefficients &p1, const PlaneCoefficients &p2, double &direction, double &distance)
{
    Eigen::Vector3d n1 = p1.head<3>();
    Eigen::Vector3d n2 = p2.head<3>();
    // normal angle
    const double ac = n1.dot(n2);
    if( ac == 1 )
        direction = 0;
    else if( ac == -1)
        direction = M_PI;
    else
        direction = std::acos( ac );
    // distance
    distance = fabs( p1[3] - p2[3] );
//    cout << CYAN << " n1.dot(n2): " << ac << RESET << endl;
//    cout << CYAN << " n1: " << n1(0) << ", " << n1(1) << ", " << n1(2) << RESET << endl;
//    cout << CYAN << " n2: " << n2(0) << ", " << n2(1) << ", " << n2(2) << RESET << endl;
}

// indices of lm1 must bigger than that of lm2
bool ITree::checkPlanesOverlap( const PlaneType &lm1, const PlaneType &lm2, const double &overlap)
{
    // project lm2 inlier to lm1 plane
    PointCloudTypePtr cloud_projected( new PointCloudType );
    projectPoints( *lm2.cloud, lm1.coefficients, *cloud_projected );

    // build octree from lm1
    const float resolution = 0.05;
    pcl::octree::OctreePointCloud<PointType> octreeD (resolution);
    octreeD.setInputCloud( lm1.cloud );
    octreeD.addPointsFromInputCloud();

    // check if occupied
    int collision = 0;
    const int threshold = cloud_projected->size() * overlap;
    PointCloudType::iterator it = cloud_projected->begin();
    for( ; it != cloud_projected->end(); it++)
    {
        PointType &pt = *it;
        if( octreeD.isVoxelOccupiedAtPoint( pt ) )
        {
            collision ++;
            if( collision > threshold )
                return true;
        }
    }

//    cout << GREEN << "  - collision: " << collision << "/" << cloud_projected->size() << RESET << endl;

//    if( (((float)collision) / (float)cloud_projected->size()) < overlap )
//        return false;
//    else
//        return true;

    return false;
}


/////////////////////////////////////////////////////////////////////////////////////

bool ITree::checkPointCorrespondences( const std_vector_of_eigen_vector4f &froms,
                                    const std_vector_of_eigen_vector4f &tos,
                                    const double distance_threshold)
{
    // check point-to-point distance constrains
    const double squared_distance = 0.2;    // 0.2m

    const double df1 = ( froms[0] - froms[1] ).norm();
    const double df2 = ( froms[0] - froms[2] ).norm();
    const double df3 = ( froms[1] - froms[2] ).norm();

    if( df1 < squared_distance || df2 < squared_distance || df3 < squared_distance )
        return false;

    const double dt1 = ( tos[0] - tos[1] ).norm();
    const double dt2 = ( tos[0] - tos[2] ).norm();
    const double dt3 = ( tos[1] - tos[2] ).norm();

    if( dt1 < squared_distance || dt2 < squared_distance || dt3 < squared_distance )
        return false;

    // check correspondence distance constrains
    const double correspondences_threshold = distance_threshold;
    if( fabs(df1 - dt1) > correspondences_threshold
            || fabs(df2 - dt2) > correspondences_threshold
            || fabs(df3 - dt3) > correspondences_threshold )
        return false;

    // pass check
    return true;
}

template <typename PointT>
bool ITree::checkPointCorrespondences( const std::vector<PointT> &froms,
                                       const std::vector<PointT> &tos,
                                       const double distance_threshold)
{
    // check point-to-point distance constrains
    const double squared_distance = 0.2;    // 0.2m

    const double df1 = euclidianDistance( froms[0] - froms[1] );
    const double df2 = euclidianDistance( froms[0] - froms[2] );
    const double df3 = euclidianDistance( froms[1] - froms[2] );

    if( df1 < squared_distance || df2 < squared_distance || df3 < squared_distance )
        return false;

    const double dt1 = euclidianDistance( tos[0] - tos[1] );
    const double dt2 = euclidianDistance( tos[0] - tos[2] );
    const double dt3 = euclidianDistance( tos[1] - tos[2] );

    if( dt1 < squared_distance || dt2 < squared_distance || dt3 < squared_distance )
        return false;

    // check correspondence distance constrains
    const double correspondences_threshold = distance_threshold;
    if( fabs(df1 - dt1) > correspondences_threshold
            || fabs(df2 - dt2) > correspondences_threshold
            || fabs(df3 - dt3) > correspondences_threshold )
        return false;

    // pass check
    return true;
}


