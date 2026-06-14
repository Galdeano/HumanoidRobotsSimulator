#include <Eigen/Dense>
#include "uLink.h"
#include "InverseSherpaKinematics.h"
#include "Spline.h"
#include "OneFootTrajectory.h"

void OneFootTrajectory(SuLINK uLINK[], State *Status, Eigen::VectorXd & u, double t)
{
    int n;
    double kd = 100.0;
    double kp = 100000.0;

    double t1 = 0.5;
    double t2 = 2.0;
    double t3 = 6.0;
    double t4 = 10.0;

    Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
    Eigen::Vector3d posd = Eigen::Vector3d::Zero();
    Eigen::Vector3d posg = Eigen::Vector3d::Zero();

    if (t < t1)
    {
        posd << 0.0, 0.0, -1.0172;
        posg << 0.0, 0.0, -1.0172;
    }
    else if (t >= t1 && t < t2)
    {
        posd << 0.0, 0.0, C2Spline(t, t1, (t1+t2)/2.0, t2, -1.0172, -1.000, -0.95, 0.0, 0.0);
        posg << 0.0, 0.0, C2Spline(t, t1, (t1+t2)/2.0, t2, -1.0172, -1.000, -0.95, 0.0, 0.0);
    }
    else if (t >= t2 && t < t3)
    {
        posd << Spline(t, t2, t3, 0.0, -0.09, 0.0, 0.0), Spline(t, t2, t3, 0.0, 0.29, 0.0, 0.0), -0.95;
        posg << Spline(t, t2, t3, 0.0, -0.09, 0.0, 0.0), Spline(t, t2, t3, 0.0, 0.29, 0.0, 0.0), -0.95;
    }
    else if (t >= t3 && t < t4)
    {
        posd << -0.09, 0.29, -0.95;
        posg << -0.09, 0.29, Spline(t, t3, t4, -0.95, -0.9, 0.0, 0.0);
    }
    else // t >= t4
    {
        posd << -0.09, 0.29, -0.95;
        posg << -0.09, 0.29, -0.9;
    }

    InverseSherpaKinematics(q, posd);
    for (n = 2; n < 8; n++)
    {
        uLINK[n].u_joint = kd * (0.0 - uLINK[n].dq) + kp * (q(n-2) - uLINK[n].q);
    }

    InverseSherpaKinematics(q, posg);
    for (n = 8; n < 14; n++)
    {
        uLINK[n].u_joint = kd * (0.0 - uLINK[n].dq) + kp * (q(n-8) - uLINK[n].q);
    }

    Status->desired_support = Status->support;

    for (n = 2; n < 14; n++)
    {
        u(n+4) = uLINK[n].u_joint;
    }
}
