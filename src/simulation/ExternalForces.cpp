#include <Eigen/Dense>
#include <cmath>
#include "uLink.h"
#include "Cross.h"
#include "CalcCoM.h"
#include "TotalMass.h"
#include "ExternalForces.h"
#include "Setup.h"

void ExternalForces(SuLINK uLINK[], State *Status, int j, Eigen::Vector3d & f, Eigen::Vector3d & t)
{
    f.setZero();
    t.setZero();

    if (j == 0)
    {
        return;
    }

    Eigen::Vector3d ftmp = Eigen::Vector3d::Zero();
    Eigen::Vector3d ttmp = Eigen::Vector3d::Zero();

    if (uLINK[j].isPolygon == 1)
    {
        int i, k;
        double val;
        
        Eigen::MatrixXd vert = uLINK[j].R * uLINK[j].vert;
        for (i = 0; i < 3; ++i)
        {
            for (k = 0; k < 8; ++k)
            {
                vert(i, k) += uLINK[j].p(i);
            }
        }

        if (j == Status->right_foot_ID)
        {
            Status->FootCenter_R(0) = (vert(0, 0) + vert(0, 2)) / 2.0;
            Status->FootCenter_R(1) = (vert(1, 0) + vert(1, 2)) / 2.0;
            Status->FootCenter_R(2) = (vert(2, 0) + vert(2, 2)) / 2.0;
            Status->forCoP_R.setZero();
            Status->posCoP_R.setZero();
            Status->right_scale = 0.0;
            Status->support &= (~0x01);
        }

        if (j == Status->left_foot_ID)
        {
            Status->FootCenter_L(0) = (vert(0, 0) + vert(0, 2)) / 2.0;
            Status->FootCenter_L(1) = (vert(1, 0) + vert(1, 2)) / 2.0;
            Status->FootCenter_L(2) = (vert(2, 0) + vert(2, 2)) / 2.0;
            Status->forCoP_L.setZero();
            Status->posCoP_L.setZero();
            Status->left_scale = 0.0;
            Status->support &= (~0x02);
        }

        uLINK[j].pcontact = uLINK[j].R * uLINK[j].contact;
        for (i = 0; i < 3; ++i)
        {
            for (k = 0; k < xContact*yContact; ++k)
            {
                uLINK[j].pcontact(i, k) += uLINK[j].p(i);
            }
        }

        for (i = 0; i < xContact*yContact; ++i)
        {
            if (uLINK[j].pcontact(2, i) < uLINK[j].supportHeight)
            {
                Eigen::Vector3d pos_i = uLINK[j].pcontact.col(i);
                Eigen::Vector3d v = uLINK[j].w.cross(pos_i) + uLINK[j].vo;

#if linearDamper
                double Kf = 500000.0;
                double Df = 700.0;
                ftmp(0) = Df * v(0);
                ftmp(1) = Df * v(1);
                ftmp(2) = Kf * (uLINK[j].pcontact(2, i) - uLINK[j].supportHeight) + Df * v(2);
#endif

#if nonLinearDamper
                double Kf = 5750.0;
                double Df = 60.0;
                double Df2 = 450.0;
                double pp = 1.2;
                double pd = 0.02;
                ftmp(0) = Df * v(0);
                ftmp(1) = Df * v(1);
                ftmp(2) = Kf * std::pow(uLINK[j].pcontact(2, i) - uLINK[j].supportHeight, pp) + Df2 * v(2) * std::pow(uLINK[j].pcontact(2, i) - uLINK[j].supportHeight, pd);
#endif

                if (ftmp(2) > 0.0)
                {
                    ftmp(2) = 0.0;
                }
                if (ftmp(2) < -300.0)
                {
                    ftmp(2) = -300.0;
                }

                f += ftmp;
                t += pos_i.cross(ftmp);

                uLINK[j].iscontact(i) = 1.0;
                uLINK[j].fcontact.col(i) = ftmp;

                if (j == Status->right_foot_ID)
                {
                    Eigen::Vector3d pos_scaled = pos_i * ftmp(2);
                    Status->right_scale += ftmp(2);
                    Status->forCoP_R += ftmp;
                    Status->posCoP_R += pos_scaled;
                    Status->support |= 0x01;
                }

                if (j == Status->left_foot_ID)
                {
                    Eigen::Vector3d pos_scaled = pos_i * ftmp(2);
                    Status->left_scale += ftmp(2);
                    Status->forCoP_L += ftmp;
                    Status->posCoP_L += pos_scaled;
                    Status->support |= 0x02;
                }
            }
            else
            {
                uLINK[j].iscontact(i) = 0.0;
            }
        }
    }

    Eigen::Vector3d fchild = Eigen::Vector3d::Zero();
    Eigen::Vector3d tchild = Eigen::Vector3d::Zero();
    ExternalForces(uLINK, Status, uLINK[j].child, fchild, tchild);

    f += fchild;
    t += tchild;

    if (j != 1)
    {
        uLINK[j].uef = uLINK[j].hv.dot(f) + uLINK[j].hw.dot(t);
    }

    Eigen::Vector3d fsister = Eigen::Vector3d::Zero();
    Eigen::Vector3d tsister = Eigen::Vector3d::Zero();
    ExternalForces(uLINK, Status, uLINK[j].sister, fsister, tsister);

    f += fsister;
    t += tsister;
}
