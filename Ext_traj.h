#ifndef H_Ext_traj
#define H_Ext_traj


void Ext_trajectory_init(float *qd);

void Ext_q_trajectory(float *qd, double t);

void Ext_op_trajectory(float *opd, double t);

void Ext_op_trajectory_LIPM(float *opd, double t);

#endif
