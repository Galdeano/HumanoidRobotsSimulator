#include <Eigen/Dense>
#include "vec2tab.h"

void vec2tab(double * tab, const Eigen::VectorXd & vec)
{
    for (int i = 0; i < vec.size(); i++)
    {
        tab[i] = vec(i);
    }
}
