#include <stdio.h>
#include <Eigen/Dense>
#include "PrintGSLMatrix.h"

void PrintGSLMatrix(const Eigen::MatrixXd & M)
{
    int i, j;
    for (i = 0; i < M.rows(); i++)
    {
        for (j = 0; j < M.cols(); j++)
        {
            printf("%4.6f ", M(i, j));
        }
        printf("\n");
    }
    printf("\n");
}

void PrintGSLMatrixTranspose(const Eigen::MatrixXd & M)
{
    int i, j;
    for (i = 0; i < M.cols(); i++)
    {
        for (j = 0; j < M.rows(); j++)
        {
            printf("%4.6f ", M(j, i));
        }
        printf("\n");
    }
    printf("\n");
}

void PrintGSLVector(const Eigen::VectorXd & V)
{
    int i;
    for (i = 0; i < V.size(); i++)
    {
        printf("%4.6f ", V(i));
    }
    printf("\n");
    printf("\n");
}
