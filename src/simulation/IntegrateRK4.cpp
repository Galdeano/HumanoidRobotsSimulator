#include <Eigen/Dense>
#include <vector>
#include <stdio.h>
#include "uLink.h"
#include "SE3exp.h"
#include "ForwardDynamics.h"
#include "IntegrateRK4.h"
#include "Setup.h"

struct LinkState {
    double q;
    double dq;
    Eigen::Vector3d p;
    Eigen::Matrix3d R;
    Eigen::Vector3d vo;
    Eigen::Vector3d w;
};

typedef std::vector<LinkState> RobotState;

static RobotState SaveRobotState(SuLINK uLINK[], int num_links)
{
    RobotState state(num_links);
    for (int j = 1; j < num_links; j++)
    {
        state[j].q = uLINK[j].q;
        state[j].dq = uLINK[j].dq;
        state[j].p = uLINK[j].p;
        state[j].R = uLINK[j].R;
        state[j].vo = uLINK[j].vo;
        state[j].w = uLINK[j].w;
    }
    return state;
}

static void RestoreRobotState(SuLINK uLINK[], const RobotState& state, int num_links)
{
    for (int j = 1; j < num_links; j++)
    {
        uLINK[j].q = state[j].q;
        uLINK[j].dq = state[j].dq;
        uLINK[j].p = state[j].p;
        uLINK[j].R = state[j].R;
        uLINK[j].vo = state[j].vo;
        uLINK[j].w = state[j].w;
    }
}

// Function to handle articular joint limits during intermediate updates
static void EnforceLimits(SuLINK uLINK[], int num_links)
{
    for (int j = 2; j < num_links; j++)
    {
        if (uLINK[j].q > uLINK[j].qmax)
        {
            uLINK[j].q = uLINK[j].qmax;
            uLINK[j].dq = -0.1 * uLINK[j].dq;
        }
        if (uLINK[j].q < uLINK[j].qmin)
        {
            uLINK[j].q = uLINK[j].qmin;
            uLINK[j].dq = -0.1 * uLINK[j].dq;
        }
    }
}

void IntegrateRK4(SuLINK uLINK[], State *Status, double t)
{
    int num_links = Status->dof - 6 + 2;
    double h = Dtime;

    // Save initial state y_n
    RobotState state_n = SaveRobotState(uLINK, num_links);

    // Derivatives for q, dq, vo, w
    std::vector<double> k1_q(num_links, 0.0), k1_dq(num_links, 0.0);
    std::vector<double> k2_q(num_links, 0.0), k2_dq(num_links, 0.0);
    std::vector<double> k3_q(num_links, 0.0), k3_dq(num_links, 0.0);
    std::vector<double> k4_q(num_links, 0.0), k4_dq(num_links, 0.0);

    Eigen::Vector3d k1_vo = Eigen::Vector3d::Zero(), k1_w = Eigen::Vector3d::Zero();
    Eigen::Vector3d k2_vo = Eigen::Vector3d::Zero(), k2_w = Eigen::Vector3d::Zero();
    Eigen::Vector3d k3_vo = Eigen::Vector3d::Zero(), k3_w = Eigen::Vector3d::Zero();
    Eigen::Vector3d k4_vo = Eigen::Vector3d::Zero(), k4_w = Eigen::Vector3d::Zero();

    // 1. Compute k1 (Current state is already in uLINK and ForwardDynamics was already run once in main.cpp, so current values are k1)
    for (int j = 1; j < num_links; j++)
    {
        k1_q[j] = uLINK[j].dq;
        k1_dq[j] = uLINK[j].ddq;
    }
    k1_vo = uLINK[1].dvo;
    k1_w = uLINK[1].dw;

    // Save intermediate velocities for SE3exp averaging
    Eigen::Vector3d vo1 = uLINK[1].vo;
    Eigen::Vector3d w1 = uLINK[1].w;

    // 2. Compute k2
    for (int j = 1; j < num_links; j++)
    {
        uLINK[j].q = state_n[j].q + (h / 2.0) * k1_q[j];
        uLINK[j].dq = state_n[j].dq + (h / 2.0) * k1_dq[j];
    }
    uLINK[1].vo = state_n[1].vo + (h / 2.0) * k1_vo;
    uLINK[1].w = state_n[1].w + (h / 2.0) * k1_w;

    uLINK[1].p = state_n[1].p;
    uLINK[1].R = state_n[1].R;
    double old_Dtime = Dtime;
    Dtime = h / 2.0;
    SE3exp(uLINK, 1);
    Dtime = old_Dtime;

    EnforceLimits(uLINK, num_links);

    // Run ForwardDynamics at t + h/2
    ForwardDynamics(uLINK, Status, (long int)(t + h/2.0));

    // Save k2
    for (int j = 1; j < num_links; j++)
    {
        k2_q[j] = uLINK[j].dq;
        k2_dq[j] = uLINK[j].ddq;
    }
    k2_vo = uLINK[1].dvo;
    k2_w = uLINK[1].dw;
    
    Eigen::Vector3d vo2 = uLINK[1].vo;
    Eigen::Vector3d w2 = uLINK[1].w;

    // 3. Compute k3
    for (int j = 1; j < num_links; j++)
    {
        uLINK[j].q = state_n[j].q + (h / 2.0) * k2_q[j];
        uLINK[j].dq = state_n[j].dq + (h / 2.0) * k2_dq[j];
    }
    uLINK[1].vo = state_n[1].vo + (h / 2.0) * k2_vo;
    uLINK[1].w = state_n[1].w + (h / 2.0) * k2_w;
    
    uLINK[1].p = state_n[1].p;
    uLINK[1].R = state_n[1].R;
    Dtime = h / 2.0;
    SE3exp(uLINK, 1);
    Dtime = old_Dtime;

    EnforceLimits(uLINK, num_links);

    ForwardDynamics(uLINK, Status, (long int)(t + h/2.0));

    // Save k3
    for (int j = 1; j < num_links; j++)
    {
        k3_q[j] = uLINK[j].dq;
        k3_dq[j] = uLINK[j].ddq;
    }
    k3_vo = uLINK[1].dvo;
    k3_w = uLINK[1].dw;

    Eigen::Vector3d vo3 = uLINK[1].vo;
    Eigen::Vector3d w3 = uLINK[1].w;

    // 4. Compute k4
    for (int j = 1; j < num_links; j++)
    {
        uLINK[j].q = state_n[j].q + h * k3_q[j];
        uLINK[j].dq = state_n[j].dq + h * k3_dq[j];
    }
    uLINK[1].vo = state_n[1].vo + h * k3_vo;
    uLINK[1].w = state_n[1].w + h * k3_w;
    
    uLINK[1].p = state_n[1].p;
    uLINK[1].R = state_n[1].R;
    Dtime = h;
    SE3exp(uLINK, 1);
    Dtime = old_Dtime;

    EnforceLimits(uLINK, num_links);

    ForwardDynamics(uLINK, Status, (long int)(t + h));

    // Save k4
    for (int j = 1; j < num_links; j++)
    {
        k4_q[j] = uLINK[j].dq;
        k4_dq[j] = uLINK[j].ddq;
    }
    k4_vo = uLINK[1].dvo;
    k4_w = uLINK[1].dw;

    Eigen::Vector3d vo4 = uLINK[1].vo;
    Eigen::Vector3d w4 = uLINK[1].w;

    // 5. Final integration update
    // Restore initial state first
    RestoreRobotState(uLINK, state_n, num_links);

    for (int j = 1; j < num_links; j++)
    {
        uLINK[j].q += (h / 6.0) * (k1_q[j] + 2.0 * k2_q[j] + 2.0 * k3_q[j] + k4_q[j]);
        uLINK[j].dq += (h / 6.0) * (k1_dq[j] + 2.0 * k2_dq[j] + 2.0 * k3_dq[j] + k4_dq[j]);
    }
    uLINK[1].vo += (h / 6.0) * (k1_vo + 2.0 * k2_vo + 2.0 * k3_vo + k4_vo);
    uLINK[1].w += (h / 6.0) * (k1_w + 2.0 * k2_w + 2.0 * k3_w + k4_w);

    EnforceLimits(uLINK, num_links);

    // Apply SE3exp for the base pose with the weighted average velocities
    Eigen::Vector3d vo_avg = (1.0 / 6.0) * (vo1 + 2.0 * vo2 + 2.0 * vo3 + vo4);
    Eigen::Vector3d w_avg = (1.0 / 6.0) * (w1 + 2.0 * w2 + 2.0 * w3 + w4);

    uLINK[1].vo = vo_avg;
    uLINK[1].w = w_avg;
    SE3exp(uLINK, 1);
}
