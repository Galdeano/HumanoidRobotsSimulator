#ifndef H_Ext_traj
#define H_Ext_traj


void Ext_trajectory_init(double *qd);

void Ext_q_trajectory(double *qd, double t);

void Ext_op_trajectory(double *opd, int pos_in_file);

void Ext_op_trajectory2(double *opd, int pos_in_file);

void Ext_op_trajectory_LIPM(double *opd, double t);

void Ext_q_hoap_trajectory(short *oqd, int pos_in_file);

void Ext_walk_trajectory(double *opd, int pos_in_file);

#endif
