#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "Ext_traj.h"
#include "Setup.h"

void Ext_trajectory_init(float *qd)
{

    FILE *f=fopen("./Trajectories/Human_q.txt","r");
    if (f == NULL) perror ("Error opening robot trajectory file");

    //fscanf (f, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", &qd[0], &qd[1], &qd[2], &qd[3], &qd[4], &qd[5], &qd[6], &qd[7], &qd[8], &qd[9], &qd[10], &qd[11], &qd[12], &qd[13], &qd[14], &qd[15], &qd[16], &qd[17], &qd[18], &qd[19], &qd[20]);

    fclose(f);
}

void Ext_trajectory(float *qd, double t, int *desired_support, float *distribution_y)
{
    int i;

    float *qt;
    qt = calloc(20,sizeof(float));

    FILE *f=fopen("./Trajectories/Human_q.txt","r");
    if (f == NULL) perror ("Error opening robot trajectory file");


    //fscanf (f, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", &qt[0], &qt[1], &qt[2], &qt[3], &qt[4], &qt[5], &qt[6], &qt[7], &qt[8], &qt[9], &qt[10], &qt[11], &qt[12], &qt[13], &qt[14], &qt[15], &qt[16], &qt[17], &qt[18], &qt[19], &qt[20]);


    for(i=0; i<20; i++)
    {
        //qd[i] = 0*qd[i];
    }

    *distribution_y=0.5f;	//repartition de l'effort de contact
    *desired_support=3;	//Pied de support: 0:none,1:right,2:left,3:both

    free (qt);
    fclose(f);
}











