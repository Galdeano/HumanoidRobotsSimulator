#include <Eigen/Dense>
#include "pinv.h"
#include <algorithm>

int pinv(Eigen::MatrixXd & invM, const Eigen::MatrixXd & M)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd s = svd.singularValues();
    Eigen::VectorXd sinv = Eigen::VectorXd::Zero(s.size());

    for (int i = 0; i < s.size(); ++i)
    {
        if (s(i) > 0.0000000001)
        {
            sinv(i) = 1.0 / s(i);
        }
    }

    invM = svd.matrixV() * sinv.asDiagonal() * svd.matrixU().transpose();
    return 0; // success (0)
}
