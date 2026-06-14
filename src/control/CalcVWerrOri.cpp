#include <Eigen/Dense>
#include "uLink.h"
#include "CalcVWerrOri.h"
#include "rot2omega.h"

void CalcVWerrOri(SuLINK uLINK[], Eigen::VectorXd & err, const Eigen::Vector3d & p, const Eigen::Matrix3d & R, const Eigen::VectorXd & idx)
{
    int ori = idx(0);
    int now = idx(idx.size() - 1);

    Eigen::Vector3d perr = p - uLINK[now].p + uLINK[ori].p;
    Eigen::Matrix3d Rerr = uLINK[now].R.transpose() * uLINK[ori].R * R;

    Eigen::Vector3d werr_local;
    rot2omega(Rerr, werr_local);
    Eigen::Vector3d werr = uLINK[now].R * werr_local;

    err.head<3>() = perr;
    err.tail<3>() = werr;
}
