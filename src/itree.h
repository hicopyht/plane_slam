#ifndef ITREE_H
#define ITREE_H

#include "utils.h"

class ITree
{
public:
    ITree();

    static bool euclidianPlaneCorrespondences(const vector<PlaneType> &planes,
                                           const vector<PlaneType> &last_planes,
                                           vector<PlanePair> &pairs,
                                           const Eigen::Matrix4d &estimated_transform = Eigen::MatrixXd::Identity(4,4),
                                           const double direction_threshold = 10.0,
                                           const double distance_threshold = 0.1);

    bool iTreeAssociate(const vector<PlaneType> &measurements, const vector<PlaneType> &landmarks, vector<PlanePair> &pairs);

    void nearestNeighborAssociate(const vector<PlaneType> &measurements, const vector<PlaneType> &landmarks, vector<PlanePair> &pairs);

    bool unaryMatch( const PlaneType &obs, const PlaneType &landmark);

    bool binaryMatch( const PlaneType &obs1, const PlaneType &obs2, const PlaneType &lm1, const PlaneType &lm2);

    static void euclidianDistance(const PlaneType &p1, const PlaneType &p2, double &direction, double &distance);

    static bool checkPlanesOverlap( const PlaneType &lm1, const PlaneType &lm2, const double &overlap = 0.5);

    static void mahalanobisDistance( const PlaneType &p1, const PlaneType &p2, double &direction, double &distance );

private:

};

#endif // ITREE_H
