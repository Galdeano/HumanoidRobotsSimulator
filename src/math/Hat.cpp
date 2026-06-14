#include <Eigen/Dense>
#include "Hat.h"

void Hat(Eigen::Matrix3d & hat, const Eigen::Vector3d & vec)
{
    hat <<      0.0, -vec(2),  vec(1),
            vec(2),      0.0, -vec(0),
           -vec(1),   vec(0),     0.0;
}
