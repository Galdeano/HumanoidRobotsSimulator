#include <Eigen/Dense>
#include "tab2vec.h"

void tab2vec(Eigen::VectorXd & vec, const double * tab)
{
    for (int i = 0; i < vec.size(); i++)
    {
        vec(i) = tab[i];
    }
}
