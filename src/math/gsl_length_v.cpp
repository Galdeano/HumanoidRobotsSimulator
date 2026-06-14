#include <Eigen/Dense>
#include "gsl_length_v.h"

int gsl_length_v(const Eigen::VectorXd & vector)
{
    return vector.size();
}
