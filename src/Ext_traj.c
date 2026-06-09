#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "Ext_traj.h"
#include "Setup.h"

void Ext_trajectory_init(double *qd)
{

    FILE *f=fopen("./Trajectories/Human_q3.txt","r");
    if (f == NULL) perror ("Error opening robot trajectory file");

    fscanf (f, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &qd[0], &qd[1], &qd[2], &qd[3], &qd[4], &qd[5], &qd[6], &qd[7], &qd[8], &qd[9], &qd[10], &qd[11], &qd[12], &qd[13], &qd[14], &qd[15], &qd[16], &qd[17], &qd[18], &qd[19], &qd[20]);

    fclose(f);
}

void Ext_q_trajectory(double *qd, double t)
{
    int i;

    static double *qt;
    static FILE *f;
    static int init_tmp=1;
    if (init_tmp==1)
    {
        qt = calloc(22,sizeof(double));
        f=fopen("./Trajectories/Human_q3.txt","r");
        if (f == NULL) perror ("Error opening robot trajectory file");
        init_tmp=0;
    }


    //fscanf (f, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &qt[0], &qt[1], &qt[2], &qt[3], &qt[4], &qt[5], &qt[6], &qt[7], &qt[8], &qt[9], &qt[10], &qt[11], &qt[12], &qt[13], &qt[14], &qt[15], &qt[16], &qt[17], &qt[18], &qt[19], &qt[20]);
    fscanf (f, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &qt[0], &qt[1], &qt[2], &qt[3], &qt[4], &qt[5], &qt[6], &qt[7], &qt[8], &qt[9], &qt[10], &qt[11], &qt[12], &qt[13], &qt[14], &qt[15], &qt[16], &qt[17], &qt[18], &qt[19], &qt[20], &qt[21], &qt[22]);

    for(i=0; i<22; i++)
    {
        qd[i] = qt[i];
    }

    //free (qt);
    //fclose(f);
}


void Ext_op_trajectory(double *opd, int pos_in_file)
{
    int i,res;

    static double *opt, *gain;
    static double gain_x=0.5;
    static double gain_y=1.0;
    static double gain_z=1.0;

    static FILE *f;
    static int init_tmp=1;
    if (init_tmp==1)
    {
        opt = calloc(9,sizeof(double));
        gain = calloc(9,sizeof(double));
        if (opt == NULL)
        {
            perror ("Error allocation opt");
            printf("Error allocation opt");
        }
        f=fopen("./Trajectories/Human_op2.txt","r");
        if (f == NULL)
        {
            perror ("Error opening robot trajectory file");
            printf("Error opening robot trajectory file");
        }

        gain[0]=gain_x;
        gain[1]=gain_y;
        gain[2]=gain_z;
        gain[3]=gain_x;
        gain[4]=gain_y;
        gain[5]=gain_z;
        gain[6]=gain_x;
        gain[7]=gain_y;
        gain[8]=gain_z;

        init_tmp=0;
    }

    if (pos_in_file<485)
    {
        res=fscanf (f,"%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &opt[0], &opt[1], &opt[2], &opt[3], &opt[4], &opt[5], &opt[6], &opt[7], &opt[8]);
    }

//if t
    for(i=0; i<9; i++)
    {
        opd[i] = gain[i]*opt[i];
    }

    //free (qt);
    //fclose(f);
}

void Ext_op_trajectory2(double *opd, int pos_in_file)
{
    int i,res;

    static double *opt, *gain;
    static double gain_x=1.0;
    static double gain_y=1.0;
    static double gain_z=1.0;

    static FILE *f;
    static int init_tmp=1;
    if (init_tmp==1)
    {
        opt = calloc(9,sizeof(double));
        gain = calloc(9,sizeof(double));
        if (opt == NULL)
        {
            perror ("Error allocation opt");
            printf("Error allocation opt");
        }
        f=fopen("./Trajectories/Human_op3.txt","r");/////////////////////
        if (f == NULL)
        {
            perror ("Error opening robot trajectory file");
            printf("Error opening robot trajectory file");
        }

        gain[0]=gain_x;
        gain[1]=gain_y;
        gain[2]=gain_z;
        gain[3]=gain_x;
        gain[4]=gain_y;
        gain[5]=gain_z;
        gain[6]=gain_x;
        gain[7]=gain_y;
        gain[8]=gain_z;

        init_tmp=0;
    }


        res=fscanf (f,"%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &opt[0], &opt[1], &opt[2], &opt[3], &opt[4], &opt[5], &opt[6], &opt[7], &opt[8]);


//if t
    for(i=0; i<9; i++)
    {
        opd[i] = gain[i]*opt[i];
    }

    //free (qt);
    //fclose(f);
}

void Ext_op_trajectory_LIPM(double *opd, double t)
{
    int i;

    static double *opt;
    static FILE *f;
    static int init_tmp=1;
    if (init_tmp==1)
    {
        opt = calloc(9,sizeof(double));
        f=fopen("./Trajectories/LIPM_op.txt","r");
        if (f == NULL) perror ("Error opening robot trajectory file");
        init_tmp=0;
    }


    fscanf (f, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &opt[0], &opt[1], &opt[2], &opt[3], &opt[4], &opt[5], &opt[6], &opt[7], &opt[8]);

    for(i=0; i<9; i++)
    {
        opd[i] = opt[i];
    }

    //free (qt);
    //fclose(f);
}



void Ext_q_hoap_trajectory(short *oqd, int pos_in_file)
{
    int i,res;

    static short *qt;


    static FILE *f;
    static int init_tmp=1;
    if (init_tmp==1)
    {
        qt = calloc(21,sizeof(short));
        if (qt == NULL)
        {
            perror ("Error allocation opt");
            printf("Error allocation opt");
        }
        f=fopen("./Trajectories/m03.csv","r");
        if (f == NULL)
        {
            perror ("Error opening robot trajectory file");
            printf("Error opening robot trajectory file");
        }

        init_tmp=0;
    }
//    const double motor_command_init_p2[21] =
//    {
//        0, 40,  3697,  9537, -5840, -344,  18810, -2000,  0,  8800,
//        0, 40, -3727, -9536,  5809,  425, -18810,  2000,  0, -8800, 418
//    };
   fscanf (f, "%*hd,%*hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%*hd,%*hd,%*c,%*c,%*c,%*c",
           &qt[0], &qt[1], &qt[2], &qt[3], &qt[4], &qt[5], &qt[6], &qt[7], &qt[8], &qt[9], &qt[10], &qt[11], &qt[12], &qt[13], &qt[14],
           &qt[15], &qt[16], &qt[17], &qt[18], &qt[19], &qt[20]);

    for(i=0; i<21; i++)
    {
        oqd[i] = qt[i];
    }



    //free (qt);
    //fclose(f);
}



void Ext_walk_trajectory(double *opd, int pos_in_file)
{
    int i,res;

    static double *opt, *gain;
    static double gain_x=1.0;
    static double gain_y=1.0;
    static double gain_z=1.0;

    static FILE *f;
    static int init_tmp=1;
    if (init_tmp==1)
    {
        opt = calloc(9,sizeof(double));
        gain = calloc(9,sizeof(double));
        if (opt == NULL)
        {
            perror ("Error allocation opt");
            printf("Error allocation opt");
        }
        f=fopen("./Trajectories/Hoap_Walk4.txt","r");/////////////////////
        if (f == NULL)
        {
            perror ("Error opening robot trajectory file Walk");
            printf("Error opening robot trajectory file Walk");
        }

        gain[0]=gain_x;
        gain[1]=gain_y;
        gain[2]=gain_z;
        gain[3]=gain_x;
        gain[4]=gain_y;
        gain[5]=gain_z;
        gain[6]=gain_x;
        gain[7]=gain_y;
        gain[8]=gain_z;

        init_tmp=0;
    }


        res=fscanf (f,"%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &opt[0], &opt[1], &opt[2], &opt[3], &opt[4], &opt[5], &opt[6], &opt[7], &opt[8]);


//if t
    for(i=0; i<9; i++)
    {
        opd[i] = gain[i]*opt[i];
    }

    //free (qt);
    //fclose(f);
}
