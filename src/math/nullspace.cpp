#include <Eigen/Dense>
#include "nullspace.h"

void nullspace(const Eigen::MatrixXd & A, Eigen::MatrixXd & Anull)
{
    int n_row = A.cols(); // n
    int n_col = A.rows(); // m
    
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(A.transpose());
    Eigen::MatrixXd Q = qr.householderQ();
    
    Anull.resize(n_row, n_row - n_col);
    for (int i = 0; i < (n_row - n_col); i++)
    {
        Anull.col(i) = Q.col(n_row - i - 1);
    }
}
