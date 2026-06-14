#include <Eigen/Dense>
#include "uLink.h"
#include "InverseSherpaKinematics.h"
#include "Spline.h"
#include "StaticTrajectory.h"

void PDStaticTrajectory(SuLINK uLINK[], State *Status, Eigen::VectorXd & qd, double d_time, double t)
{
    int n;
    double t1=2.0;
    double t2=5.0;
    double t3=15.0;
    double t4=25.0;
    double t5=35.0;
    double t6=45.0;
    double t7=55.0;
    double t8=65.0;

    double com_y = 0.29;
    double adjust_x = -0.03;

    Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
    Eigen::Vector3d posd = Eigen::Vector3d::Zero();
    Eigen::Vector3d posg = Eigen::Vector3d::Zero();

    if (t < t1)
    {
        Status->desired_support=3;
        posd << 0.0, 0.0, -1.0172;
        posg << 0.0, 0.0, -1.0172;
        Status->distribution_y=0.5;
    }
    else if (t >= t1 && t < t2)
    {
        Status->desired_support=3;
        posd << 0.0, 0.0, C2Spline(t,t1,(t1+t2)/2.0,t2,-1.0172,-1.000,-0.95,0.0,0.0);
        posg << 0.0, 0.0, C2Spline(t,t1,(t1+t2)/2.0,t2,-1.0172,-1.000,-0.95,0.0,0.0);
        Status->distribution_y=0.5;
    }
    else if (t >= t2 && t < t3)
    {
        Status->desired_support=3;
        posd << Spline(t,t2,t3,0.0,-0.05,0.0,0.0), Spline(t,t2,t3,0.0,com_y,0.0,0.0), -0.95;
        posg << Spline(t,t2,t3,0.0,-0.05,0.0,0.0), Spline(t,t2,t3,0.0,com_y,0.0,0.0), -0.95;
        Status->distribution_y=Spline(t,t2,t3,0.5,0.0,0.0,0.0);
    }
    else if (t >= t3 && t < t4)
    {
        Status->desired_support=1;
        posd << -0.05 + Spline(t,t3,t4,0.0,-adjust_x,0.0,0.0), com_y, -0.95;
        posg << Spline(t,t3,t4,-0.05,0.05,0.0,0.0) + Spline(t,t3,t4,0.0,-adjust_x,0.0,0.0), com_y, C2Spline(t,t3,(t3+t4)/2.0,t4,-0.95,-0.90,-0.95,0.0,0.0);
        Status->distribution_y=0.0;
    }
    else if (t >= t4 && t < t5)
    {
        Status->desired_support=3;
        posd << Spline(t,t4,t5,-0.05,-0.15,0.0,0.0) + Spline(t,t4,t5,-adjust_x,adjust_x,0.0,0.0), Spline(t,t4,t5,com_y,-com_y,0.0,0.0), -0.95;
        posg << Spline(t,t4,t5,0.05,-0.05,0.0,0.0) + Spline(t,t4,t5,-adjust_x,adjust_x,0.0,0.0), Spline(t,t4,t5,com_y,-com_y,0.0,0.0), -0.95;
        Status->distribution_y=Spline(t,t4,t5,0.0,1.0,0.0,0.0);
    }
    else if (t >= t5 && t < t6)
    {
        Status->desired_support=2;
        posd << Spline(t,t5,t6,-0.15,0.05,0.0,0.0) + Spline(t,t5,t6,adjust_x,-adjust_x,0.0,0.0), -com_y, C2Spline(t,t5,(t5+t6)/2.0,t6,-0.95,-0.90,-0.95,0.0,0.0);
        posg << -0.05 + Spline(t,t5,t6,adjust_x,-adjust_x,0.0,0.0), -com_y, -0.95;
        Status->distribution_y=1.0;
    }
    else if (t >= t6 && t < t7)
    {
        Status->desired_support=3;
        posd << Spline(t,t6,t7,0.05,-0.05,0.0,0.0) + Spline(t,t6,t7,-adjust_x,adjust_x,0.0,0.0), Spline(t,t6,t7,-com_y,com_y,0.0,0.0), -0.95;
        posg << Spline(t,t6,t7,-0.05,-0.15,0.0,0.0) + Spline(t,t6,t7,-adjust_x,adjust_x,0.0,0.0), Spline(t,t6,t7,-com_y,com_y,0.0,0.0), -0.95;
        Status->distribution_y=Spline(t,t6,t7,1.0,0.0,0.0,0.0);
    }
    else if (t >= t7 && t < t8)
    {
        Status->desired_support=1;
        posd << -0.05 + Spline(t,t7,t8,adjust_x,-adjust_x,0.0,0.0), com_y, -0.95;
        posg << Spline(t,t7,t8,-0.15,0.05,0.0,0.0) + Spline(t,t7,t8,adjust_x,-adjust_x,0.0,0.0), com_y, C2Spline(t,t7,(t7+t8)/2.0,t8,-0.95,-0.90,-0.95,0.0,0.0);
        Status->distribution_y=0.0;
    }
    else // t >= t8
    {
        Status->desired_support=3;
        posd << -0.05 - adjust_x, com_y, -0.95;
        posg << 0.05 - adjust_x, com_y, -0.95;
        Status->distribution_y=0.0;
    }

    InverseSherpaKinematics(q, posd);
    for (n=0; n<6; n++)
    {
        qd(n+6) = q(n);
    }

    InverseSherpaKinematics(q, posg);
    for (n=0; n<6; n++)
    {
        qd(n+12) = q(n);
    }
}

void PStaticTrajectory(SuLINK uLINK[], State *Status, Eigen::VectorXd & u, double t)
{
    int n;
    double kd = 100.0;
    double kp = 10000.0;

    double t1=0.5;
    double t2=2.0;
    double t3=5.0;
    double t4=8.0;
    double t5=11.0;
    double t6=14.0;
    double t7=17.0;
    double t8=20.0;

    double com_y = 0.28;

    while (t >= 20.0)
    {
        t -= (t8 - t4);
    }

    Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
    Eigen::Vector3d posd = Eigen::Vector3d::Zero();
    Eigen::Vector3d posg = Eigen::Vector3d::Zero();

    if (t < t1)
    {
        Status->desired_support=3;
        posd << 0.0, 0.0, -1.0172;
        posg << 0.0, 0.0, -1.0172;
    }
    else if (t >= t1 && t < t2)
    {
        Status->desired_support=3;
        posd << 0.0, 0.0, C2Spline(t,t1,(t1+t2)/2.0,t2,-1.0172,-1.000,-0.95,0.0,0.0);
        posg << 0.0, 0.0, C2Spline(t,t1,(t1+t2)/2.0,t2,-1.0172,-1.000,-0.95,0.0,0.0);
    }
    else if (t >= t2 && t < t3)
    {
        Status->desired_support=3;
        posd << Spline(t,t2,t3,0.0,-0.05,0.0,0.0), Spline(t,t2,t3,0.0,com_y,0.0,0.0), -0.95;
        posg << Spline(t,t2,t3,0.0,-0.05,0.0,0.0), Spline(t,t2,t3,0.0,com_y,0.0,0.0), -0.95;
    }
    else if (t >= t3 && t < t4)
    {
        Status->desired_support=1;
        posd << -0.05, com_y, -0.95;
        posg << Spline(t,t3,t4,-0.05,0.05,0.0,0.0), com_y, C2Spline(t,t3,(t3+t4)/2.0,t4,-0.95,-0.90,-0.95,0.0,0.0);
    }
    else if (t >= t4 && t < t5)
    {
        Status->desired_support=3;
        posd << Spline(t,t4,t5,-0.05,-0.15,0.0,0.0), Spline(t,t4,t5,com_y,-com_y,0.0,0.0), -0.95;
        posg << Spline(t,t4,t5,0.05,-0.05,0.0,0.0), Spline(t,t4,t5,com_y,-com_y,0.0,0.0), -0.95;
    }
    else if (t >= t5 && t < t6)
    {
        Status->desired_support=2;
        posd << Spline(t,t5,t6,-0.15,0.05,0.0,0.0), -com_y, C2Spline(t,t5,(t5+t6)/2.0,t6,-0.95,-0.90,-0.95,0.0,0.0);
        posg << -0.05, -com_y, -0.95;
    }
    else if (t >= t6 && t < t7)
    {
        Status->desired_support=3;
        posd << Spline(t,t6,t7,0.05,-0.05,0.0,0.0), Spline(t,t6,t7,-com_y,com_y,0.0,0.0), -0.95;
        posg << Spline(t,t6,t7,-0.05,-0.15,0.0,0.0), Spline(t,t6,t7,-com_y,com_y,0.0,0.0), -0.95;
    }
    else if (t >= t7 && t < t8)
    {
        Status->desired_support=1;
        posd << -0.05, com_y, -0.95;
        posg << Spline(t,t7,t8,-0.15,0.05,0.0,0.0), com_y, C2Spline(t,t7,(t7+t8)/2.0,t8,-0.95,-0.90,-0.95,0.0,0.0);
    }
    else // t >= t8
    {
        Status->desired_support=3;
        posd << -0.05, com_y, -0.95;
        posg << 0.05, com_y, -0.95;
    }

    InverseSherpaKinematics(q, posd);
    for (n=2; n<8; n++)
    {
        uLINK[n].u_joint = kd * (0.0 - uLINK[n].dq) + kp * (q(n-2) - uLINK[n].q);
    }

    InverseSherpaKinematics(q, posg);
    for (n=8; n<14; n++)
    {
        uLINK[n].u_joint = kd * (0.0 - uLINK[n].dq) + kp * (q(n-8) - uLINK[n].q);
    }

    for (n=2; n<14; n++)
    {
        u(n+4) = uLINK[n].u_joint;
    }
}
