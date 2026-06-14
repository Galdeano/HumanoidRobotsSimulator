#include <Eigen/Dense>
#include "rot2omega.h"
#include <cmath>

int rot2omega(const Eigen::Matrix3d & R, Eigen::Vector3d & w)
{
    double alpha = (R.trace() - 1.0) / 2.0;

    if (std::abs(alpha - 1.0) < 0.0000001)
    {
        w.setZero();
        return 0;
    }

    double th = std::acos(alpha);
    w(0) = R(2, 1) - R(1, 2);
    w(1) = R(0, 2) - R(2, 0);
    w(2) = R(1, 0) - R(0, 1);

    w *= 0.5 * th / std::sin(th);

    return 0;
}
