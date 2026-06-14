#include <Eigen/Dense>
#include "Rodrigues.h"
#include <cmath>

int Rodrigues(Eigen::Matrix3d & rot, const Eigen::Vector3d & w, double dt)
{
    double d = w.norm();
    if (d < 0.0000000001)
    {
        rot.setIdentity();
        return 0;
    }

    double th = d * dt;
    Eigen::Vector3d wn = w / d;
    rot = Eigen::AngleAxisd(th, wn).toRotationMatrix();
    return 0;
}
