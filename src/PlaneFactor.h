#ifndef PLANEFACTOR_H
#define PLANEFACTOR_H

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/HessianFactor.h>

using namespace std;
using namespace gtsam;


class PlaneFactor:public NoiseModelFactor2<Pose3, Unit3>
{
    double distance_;   // normal n(nx, ny, nz), distance to origin

public:
    PlaneFactor(const SharedNoiseModel& noise_model, Key j1, Key j2, double d) :
        NoiseModelFactor2<Pose3, Unit3>(noise_model, j1, j2)
        , distance_(d)
    {}

    Vector evaluateError(const Pose3&, const Unit3&,
                         boost::optional<Matrix&> H1 = boost::none,
                         boost::optional<Matrix&> H2 = boost::none)
    {
        // error, H1, H2
    }

}


#endif // PLANEFACTOR_H
