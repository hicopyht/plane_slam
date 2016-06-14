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
    const Eigen::Matrix4d transform = estimated_transform.inverse();
    for(int i = 0; i < last_planes.size(); i++)
    {
        const PlaneType &plane = last_planes[i];
        PlaneType p = plane;
        transformPlane( plane.coefficients, transform, p.coefficients);
        // TODO transform inlier point?
        predict_planes.push_back( p );
    }

    /// 2: Find correspondences
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
            if( (fabs(dir_error) < direction_threshold)
                    && (d_error < distance_threshold) )
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
                    pairs.push_back( PlanePair(i, j));
                }
            }
        }
    }

}

void ITree::euclidianDistance(const PlaneType &p1, const PlaneType &p2, double &direction, double &distance)
{
    // Transform landmark to local frame
    Eigen::Vector3d n1 = p1.coefficients.head<3>();
    Eigen::Vector3d n2 = p2.coefficients.head<3>();
    direction = acos( n1.dot(n2) );
    distance = fabs( p1.coefficients[3] - p2.coefficients[3] );
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
    PointCloudType::iterator it = cloud_projected->begin();
    for( ; it != cloud_projected->end(); it++)
    {
        PointType &pt = *it;
        if( octreeD.isVoxelOccupiedAtPoint( pt ) )
        {
            collision ++;
        }
    }

    cout << GREEN << "  - collision: " << collision << "/" << cloud_projected->size() << RESET << endl;

    if( (((float)collision) / (float)cloud_projected->size()) < overlap )
        return false;
    else
        return true;

    return false;
}

void ITree::mahalanobisDistance(const PlaneType &p1, const PlaneType &p2, double &direction, double &distance)
{

}
