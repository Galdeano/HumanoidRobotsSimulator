#include <Eigen/Dense>
#include "uLink.h"
#include "Cross.h"
#include "TotalMass.h"
#include "gsl_length_v.h"
#include "CalcJacobianModif.h"

void CalcJacobianModif(SuLINK uLINK[], Eigen::MatrixXd & J, const Eigen::VectorXd & idx)
{
    int i, n, j, k;

    static Eigen::Vector3d a;
    static Eigen::Vector3d error;
    static int init_tmp = 1;
    if (init_tmp == 1)
    {
        a.setZero();
        error.setZero();
        init_tmp = 0;
    }

    int idx_len = idx.size();
    for (n = 1; n < (idx_len - 1); n++)
    {
        j = (int)idx(n);
        k = (int)idx(n + 1);

        if (j == uLINK[k].mother)
        {
            a = uLINK[j].R * uLINK[j].a;
        }
        else if (k == uLINK[j].mother)
        {
            a = uLINK[j].R * (-uLINK[j].a);
        }
        else if (k == j)
        {
            a = uLINK[j].R * uLINK[j].a;
        }
        else
        {
            a = uLINK[j].R * (-uLINK[j].a);
        }

        int target_idx = (int)idx(idx_len - 1);
        error = uLINK[target_idx].p - uLINK[j].p;
        Eigen::Vector3d cross_err = a.cross(error);

        for (i = 0; i < 3; i++)
        {
            J(i, j - 2) = cross_err(i);
            J(i + 3, j - 2) = a(i);
        }
    }
}
