#ifndef H_StaticTrajectory_f
#define H_StaticTrajectory_f


void StaticTrajectory_f(float *qd, double t, int *desired_support, float *distribution_y);

void OneFoot_f(float *qd, double t, int *desired_support, float *distribution_y);

void GravityExperiment_f(float *qd, double t, int *desired_support, float *distribution_y);


#endif
