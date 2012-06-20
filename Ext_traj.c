#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "Ext_traj.h"
#include "Setup.h"

void Ext_trajectory_init(float *qd)
{

    FILE *f=fopen("./Trajectories/Human_q3.txt","r");
    if (f == NULL) perror ("Error opening robot trajectory file");

    fscanf (f, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", &qd[0], &qd[1], &qd[2], &qd[3], &qd[4], &qd[5], &qd[6], &qd[7], &qd[8], &qd[9], &qd[10], &qd[11], &qd[12], &qd[13], &qd[14], &qd[15], &qd[16], &qd[17], &qd[18], &qd[19], &qd[20]);

    fclose(f);
}

void Ext_q_trajectory(float *qd, double t)
{
    int i;

    static float *qt;
    static FILE *f;
    static int init_tmp=1;
    if (init_tmp==1)
    {
        qt = calloc(22,sizeof(float));
        f=fopen("./Trajectories/Human_q3.txt","r");
        if (f == NULL) perror ("Error opening robot trajectory file");
        init_tmp=0;
    }


    //fscanf (f, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", &qt[0], &qt[1], &qt[2], &qt[3], &qt[4], &qt[5], &qt[6], &qt[7], &qt[8], &qt[9], &qt[10], &qt[11], &qt[12], &qt[13], &qt[14], &qt[15], &qt[16], &qt[17], &qt[18], &qt[19], &qt[20]);
    fscanf (f, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", &qt[0], &qt[1], &qt[2], &qt[3], &qt[4], &qt[5], &qt[6], &qt[7], &qt[8], &qt[9], &qt[10], &qt[11], &qt[12], &qt[13], &qt[14], &qt[15], &qt[16], &qt[17], &qt[18], &qt[19], &qt[20], &qt[21], &qt[22]);

    for(i=0; i<22; i++)
    {
        qd[i] = qt[i];
    }

    //free (qt);
    //fclose(f);
}


void Ext_op_trajectory(float *opd, double t)
{
    int i;

    static float *opt;
    static FILE *f;
    static int init_tmp=1;
    if (init_tmp==1)
    {
        opt = calloc(9,sizeof(float));
        f=fopen("./Trajectories/Human_op.txt","r");
        if (f == NULL) perror ("Error opening robot trajectory file");
        init_tmp=0;
    }


    fscanf (f, "%f,%f,%f,%f,%f,%f,%f,%f,%f", &opt[0], &opt[1], &opt[2], &opt[3], &opt[4], &opt[5], &opt[6], &opt[7], &opt[8]);

    for(i=0; i<9; i++)
    {
        opd[i] = opt[i];
    }

    //free (qt);
    //fclose(f);
}


void Ext_op_trajectory_LIPM(float *opd, double t)
{
    int i;

    static float *opt;
    static FILE *f;
    static int init_tmp=1;
    if (init_tmp==1)
    {
        opt = calloc(9,sizeof(float));
        f=fopen("./Trajectories/LIPM_op.txt","r");
        if (f == NULL) perror ("Error opening robot trajectory file");
        init_tmp=0;
    }


    fscanf (f, "%f,%f,%f,%f,%f,%f,%f,%f,%f", &opt[0], &opt[1], &opt[2], &opt[3], &opt[4], &opt[5], &opt[6], &opt[7], &opt[8]);

    for(i=0; i<9; i++)
    {
        opd[i] = opt[i];
    }

    //free (qt);
    //fclose(f);
}





