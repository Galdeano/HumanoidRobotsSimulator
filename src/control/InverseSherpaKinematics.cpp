#include <Eigen/Dense>
#include <cmath>
#include "InverseSherpaKinematics.h"

void InverseSherpaKinematics(Eigen::Ref<Eigen::VectorXd> q, const Eigen::Ref<const Eigen::Vector3d>& pos)
{
    double Lc=0.5073;
    double Lt=0.510;

    double x = pos(0);
    double y = pos(1);
    double z = pos(2);

    q(0) = std::atan2(y, -z);
    q(5) = -std::atan2(y, -z);

    double l = std::hypot(y, z);

    double val = (l*l + x*x - Lc*Lc - Lt*Lt) / (2.0 * Lc * Lt);
    // Clamp val to avoid nan with sqrt
    if (val > 1.0) val = 1.0;
    if (val < -1.0) val = -1.0;

    double q4 = std::atan2(std::sqrt(1.0 - val*val), val);
    q(3) = q4;

    double q2 = -std::atan2(x, l) - std::atan2(Lt * std::sin(q4), Lc + Lt * std::cos(q4));
    q(1) = q2;

    q(4) = -q4 - q2;
}

void InverseSherpaKinematics(double * q, const double * pos)
{
    Eigen::Map<Eigen::VectorXd> q_map(q, 6);
    Eigen::Map<const Eigen::Vector3d> pos_map(pos);
    InverseSherpaKinematics(q_map, pos_map);
}
