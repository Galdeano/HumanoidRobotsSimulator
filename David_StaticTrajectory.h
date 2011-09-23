#ifndef H_David_StaticTrajectory
#define H_David_StaticTrajectory


void David_StaticTrajectory(float *qd, double t, int *desired_support, float *distribution_y);

void David_OneFoot(float *qd, double t, int *desired_support, float *distribution_y);

void David_GravityExperiment(float *qd, double t, int *desired_support, float *distribution_y);


#endif
