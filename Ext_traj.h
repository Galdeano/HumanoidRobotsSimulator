#ifndef H_Ext_traj
#define H_Ext_traj


void Ext_trajectory_init(float *qd);

void Ext_trajectory(float *qd, double t, int *desired_support, float *distribution_y);



#endif
