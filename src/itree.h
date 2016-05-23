#ifndef ITREE_H
#define ITREE_H

#include "utils.h"

class ITree
{
public:
    ITree();

    bool iTreeAssociate(const vector<PlaneType> &measurements, const vector<PlaneType> &landmarks, vector<PlanePair> &pairs);

    void nearestNeighborAssociate(const vector<PlaneType> &measurements, const vector<PlaneType> &landmarks, vector<PlanePair> &pairs);

    bool unaryMatch( const PlaneType &obs, const PlaneType &landmark);

    bool binaryMatch( const PlaneType &obs1, const PlaneType &obs2, const PlaneType &lm1, const PlaneType &lm2);

    void euclidianDistance();

    void mahalanobisDistance();

private:

};

#endif // ITREE_H
