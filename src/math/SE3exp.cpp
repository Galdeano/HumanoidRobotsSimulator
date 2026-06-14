#include <Eigen/Dense>
#include "uLink.h"
#include "Cross.h"
#include "Hat.h"
#include "SE3exp.h"
#include "Setup.h"
#include <cmath>

void SE3exp(SuLINK uLINK[], int j)
{
    double norm_w = uLINK[j].w.norm();
    if (norm_w < std::pow(2.0, -53.0))
    {
        uLINK[j].p += Dtime * uLINK[j].vo;
    }
    else
    {
        double th = norm_w * Dtime;
        Eigen::Vector3d wn = uLINK[j].w / norm_w;
        Eigen::Vector3d vo = uLINK[j].vo / norm_w;

        Eigen::Matrix3d rot = Eigen::AngleAxisd(th, wn).toRotationMatrix();
        Eigen::Matrix3d drot = rot - Eigen::Matrix3d::Identity();

        uLINK[j].p = rot * uLINK[j].p - drot * wn.cross(vo) + wn * (wn.dot(vo)) * th;
        uLINK[j].R = rot * uLINK[j].R;
    }
}
