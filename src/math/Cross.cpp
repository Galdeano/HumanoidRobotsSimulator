#include <Eigen/Dense>
#include "Cross.h"

void Cross(Eigen::Vector3d & a, Eigen::Vector3d & b, int i)
{
    Eigen::Vector3d c = a.cross(b);
    if (i == 0)
    {
        a = c;
    }
    else
    {
        b = c;
    }
}
