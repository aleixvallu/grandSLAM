#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

using namespace gtsam;

class LandmarkFactor : public NoiseModelFactor2<Pose3, Point3> {
    Point3 m;

  public:

    LandmarkFactor(Key keyPose, Key keyLandmark, Point3 measured, const SharedNoiseModel &model)
        : NoiseModelFactor2<Pose3, Point3>(model, keyPose, keyLandmark), m(measured) {}

    Vector evaluateError(const Pose3 &pose, const Point3 &landmark, 
                         OptionalMatrixType H1,
                         OptionalMatrixType H2) 
                         const override {

        Point3 predicted = pose.transformTo(landmark, H1, H2);
        return predicted - m;
    }
};