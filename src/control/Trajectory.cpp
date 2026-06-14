#include <Eigen/Dense>
#include <cmath>
#include "uLink.h"
#include "InverseSherpaKinematics.h"
#include "CalcCoM.h"
#include "Spline.h"
#include "Trajectory.h"

void Trajectory(SuLINK uLINK[], Eigen::VectorXd & u, double t)
{
    int n;
    double kd = 100.0;
    double kp = 100000.0;

    // Marche static +splines
    if (true)
    {
        Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
        Eigen::Vector3d pos = Eigen::Vector3d::Zero();

        if (t < 0.3)
        {
            pos << 0.0, 0.0, -1.0172;
            InverseSherpaKinematics(q, pos);
            for (n = 2; n < 8; n++)
            {
                uLINK[n].u_joint = kd * (0.0 - uLINK[n].dq) + kp * (q(n-2) - uLINK[n].q);
            }

            pos << 0.0, 0.0, -1.0172;
            InverseSherpaKinematics(q, pos);
            for (n = 8; n < 14; n++)
            {
                uLINK[n].u_joint = kd * (0.0 - uLINK[n].dq) + kp * (q(n-8) - uLINK[n].q);
            }
        }
        else if (t >= 0.3 && t < 1.0)
        {
            pos << 0.0, 0.0, Spline(t, 0.3, 1.0, -1.0172, -0.95, 0.0, 0.0);
            InverseSherpaKinematics(q, pos);
            for (n = 2; n < 8; n++)
            {
                uLINK[n].u_joint = kd * (0.0 - uLINK[n].dq) + kp * (q(n-2) - uLINK[n].q);
            }

            pos << 0.0, 0.0, Spline(t, 0.3, 1.0, -1.0172, -0.95, 0.0, 0.0);
            InverseSherpaKinematics(q, pos);
            for (n = 8; n < 14; n++)
            {
                uLINK[n].u_joint = kd * (0.0 - uLINK[n].dq) + kp * (q(n-8) - uLINK[n].q);
            }
        }
        else if (t >= 1.0 && t < 3.0)
        {
            pos << Spline(t, 1.0, 3.0, 0.0, -0.09, 0.0, 0.0), Spline(t, 1.0, 3.0, 0.0, 0.32, 0.0, 0.0), -0.95;
            InverseSherpaKinematics(q, pos);
            for (n = 2; n < 8; n++)
            {
                uLINK[n].u_joint = kd * (0.0 - uLINK[n].dq) + kp * (q(n-2) - uLINK[n].q);
            }

            pos << Spline(t, 1.0, 3.0, 0.0, -0.09, 0.0, 0.0), Spline(t, 1.0, 3.0, 0.0, 0.32, 0.0, 0.0), -0.95;
            InverseSherpaKinematics(q, pos);
            for (n = 8; n < 14; n++)
            {
                uLINK[n].u_joint = kd * (0.0 - uLINK[n].dq) + kp * (q(n-8) - uLINK[n].q);
            }
        }
        else if (t >= 3.0 && t < 5.0)
        {
            pos << -0.09, 0.32, -0.95;
            InverseSherpaKinematics(q, pos);
            for (n = 2; n < 8; n++)
            {
                uLINK[n].u_joint = kd * (0.0 - uLINK[n].dq) + kp * (q(n-2) - uLINK[n].q);
            }

            pos << -0.09, 0.32, Spline(t, 3.0, 5.0, -0.95, -0.9, 0.0, 0.0);
            InverseSherpaKinematics(q, pos);
            for (n = 8; n < 14; n++)
            {
                uLINK[n].u_joint = kd * (0.0 - uLINK[n].dq) + kp * (q(n-8) - uLINK[n].q);
            }
        }
        else // t >= 5.0
        {
            pos << -0.09, 0.32, -0.95;
            InverseSherpaKinematics(q, pos);
            for (n = 2; n < 8; n++)
            {
                uLINK[n].u_joint = kd * (0.0 - uLINK[n].dq) + kp * (q(n-2) - uLINK[n].q);
            }

            pos << -0.09, 0.32, -0.9;
            InverseSherpaKinematics(q, pos);
            for (n = 8; n < 14; n++)
            {
                uLINK[n].u_joint = kd * (0.0 - uLINK[n].dq) + kp * (q(n-8) - uLINK[n].q);
            }
        }
    }

    for (n = 2; n < 14; n++)
    {
        u(n+4) = uLINK[n].u_joint;
    }
}
