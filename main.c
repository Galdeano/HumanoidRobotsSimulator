
/*!
 *  \file main.c
 *  \brief     This is the multi body simulator
 *  \details   This program use gsl for matrix coputation and SDL and openGL for visualization
 *  \author    David Galdeano
 *  \date      10/2011
 *  \pre       First install gsl, SDL and openGL for a proper compilation
 *  \warning   Do not modify this programm unless you know what you do.
 *  \copyright This program must not be modify, distributed or used in an other application without the autorisation of the autor and is distributed as is, without any guaranties in case of malfunctioning.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_math.h>
#include <SDL/SDL.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include<time.h>

#include "uLINK.h"
#include "DrawAllJoints.h"
#include "PrintState.h"
#include "bmp.h"
#include "IntegrateEuler.h"
#include "ForwardDynamics.h"
#include "ForwardKinematics.h"
#include "LoadRobot.h"

#include "InverseSherpaKinematics.h"
#include "CalcCoM.h"
#include "DrawMarker.h"
#include "Ext_traj.h"
#include "DrawIndicators.h"
#include "NodeForwardKinematics.h"
#include "DrawGround.h"

#include "ping.h"
#include "CalcJacobianModif.h"
#include "CalcVWerrOri.h"
#include "pinv.h"
#include "PrintGSLMatrix.h"
#include "CalcCoMJacobian.h"



#include "Setup.h"


#include "TotalMass.h"
#include "ObjLoader.h"
#include "DrawLight.h"
#include "DrawCamera.h"
#include "netHOAP.h"
#include "Rodrigues.h"
#include "rot2omega.h"

#include "Hoap_calc_zmp.h"
#include "zentimer.h"

#include <windows.h>



int main(int argc, char *argv[])
{
    freopen("CON", "w", stdout);
    freopen("CON", "r", stdin);
    freopen("CON", "w", stderr);

    printf("Sherpa Simulator\n");

#ifdef HAVE_INLINE
    printf("HAVE_INLINE\n");
#endif
#ifdef GSL_RANGE_CHECK_OFF
    printf("GSL_RANGE_CHECK_OFF\n");
#endif

    printf("\n Choose your robot: \n");

    DIR *d;
    struct dirent *dir;
    d = opendir("./Robots/");
    int i=0,j=0;
    char szInput [25];
    char RobotFile[255];
    strcpy( RobotFile, "./Robots/" );

    if (d == NULL)
        perror("");

    if (d)
    {
        while ((dir = readdir(d)) != NULL)
        {
            i++;
            printf("%ld -> %s\n",telldir(d), dir->d_name);
        }
        rewinddir(d);

        do
        {
            //*fgets ( szInput, 25, stdin );
            //j=atoi(szInput);
            //j=3;
            j=12;
        }
        while (!(j>2 && j<=i));

        for(i=0; i<j; i++)
        {
            dir = readdir(d);
        }
        strcat( RobotFile, dir->d_name );
        printf("\n%s\n", RobotFile);
        closedir(d);
    }



    FILE *f=fopen(RobotFile,"r");
    char mystring [100];
    int dof;
    if (f == NULL) perror ("Error opening robot description file");
    //fgets (mystring , 100 , f);
    //printf("%s \n",mystring);
    fscanf (f, "%s", mystring);
    //printf("%s \n",mystring);
    fscanf (f, "%s", mystring);
    //printf("%s \n",mystring);
    fscanf (f, "%i", &dof);
    printf("DoF: %i \n",dof);
    fclose(f);


    SuLINK *uLINK;
    uLINK = calloc(dof+2,sizeof(SuLINK));
    //SuLINK uLINK[NbLinks];

    State Status;

    strcpy(Status.RobotFile,RobotFile);

    //LoadRobotXML(uLINK,&Status,RobotFile);
    LoadRobotParserXML(uLINK,&Status,RobotFile);


    printf("Robot wheigth: %f \n",TotalMass(uLINK,1));
    //printf("%f \n",gsl_vector_get (uLINK[2].b,1));
    //printf("%8.1f \n",uLINK[8].mother);






    gsl_vector * com = gsl_vector_calloc (3);
    gsl_vector * CoP = gsl_vector_calloc (3);
    gsl_vector * q = gsl_vector_calloc (6);
    gsl_vector * pos = gsl_vector_calloc (3);
    long t=0;
    char titre[40] = "Visualisation t=0";
    double angular_z=0;


#if Video
    char buf[256];
    d = opendir("./../Simu_images/");

    while(dir = readdir(d))
    {
        //printf("%s\n",dir->d_name);
        sprintf(buf, "%s/%s", "./../Simu_images/", dir->d_name);
        remove(buf);
    }
#endif



//HWND Handle=NULL;

    SDL_Init(SDL_INIT_VIDEO); // Initialisation de la SDL
    atexit(SDL_Quit);

    //SDL_SetVideoMode(640, 480, 32, SDL_OPENGL); // vga// Ouverture de la fenêtre
    SDL_SetVideoMode(1024, 768, 32, SDL_OPENGL); // xga// Ouverture de la fenêtre
    //SDL_SetVideoMode(1400, 1050, 32, SDL_OPENGL); // sxga+// Ouverture de la fenêtre

    SDL_WM_SetCaption("Visualisation", NULL);
    SDL_Event event;

//Handle=FindWindow("SDL_app","Visualisation");
//SetWindowPos(Handle,HWND_TOPMOST,1,1,640,480,SWP_SHOWWINDOW);

    glMatrixMode( GL_PROJECTION );
    glLoadIdentity( );

    //gluPerspective(70, 640/480, 0.001, 1000);// vga
    gluPerspective(70, 1024/768, 0.001, 1000);// xga
    //gluPerspective(70, 1400/1050, 0.001, 1000);// sxga+

    //gluPerspective(50,(double)640/480,1,1000);
    //gluLookAt(1, 0, 0, 0, 0, 0, 0, 0, 1);
    //gluLookAt(2*gsl_vector_get(Status.com_old,2), -2*gsl_vector_get(Status.com_old,2), 1.4*gsl_vector_get(Status.com_old,2), 0, 0, 0, 0, 0, 1);
//    gluLookAt(2*gsl_vector_get(Status.com_old,2), -2*gsl_vector_get(Status.com_old,2), 1.4*gsl_vector_get(Status.com_old,2), 0, 0, 0.9*gsl_vector_get(Status.com_old,2), 0, 0, 1);
    glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    //glEnable(GL_LINE_SMOOTH);
    glEnable(GL_NORMALIZE);
//glShadeModel(GL_SMOOTH);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);	// Really Nice Perspective Calculations

    double Lc=0.5073;
    double Lt=0.510;
    double Lp=0.1104;
    int frame_skip = 20;
    int img_per_s = 25;
    //int frame_skip = 1;
//    for (i = 0; i < NbRobots; i++)
//    {
//        gsl_vector_set (uLINK[i][1].p, 2, Lc+Lt+Lp-0.0006);
//        gsl_vector_set (uLINK[i][1].p, 1, i);
//    }
    //gsl_vector_set (uLINK[0][1].p, 2, Lc+Lt+Lp+0.15);

#if Suspendu
    gsl_vector_set (uLINK[1].p, 2, Lc+Lt+Lp+0.1);
    uLINK[1].supportHeight=Lc+Lt+Lp+0.13;
#endif


#if Ext_traj
    double *qd;
    qd = calloc(dof,sizeof(double));
    Ext_trajectory_init(qd);

    for(i=1; i<(dof+2); i++)
    {
        uLINK[i].q = qd[i-1];
    }
    free (qd);
    ForwardKinematics(uLINK,1);
#endif



#if Task
    const int nDoF=dof+6;
    const double rad2deg=180/M_PI;
    const double deg2rad=M_PI/180;
    const double motor_command_init[21] =
    {
        0.0, -2.5, -0.33, 0.0, 0.29, 2.5, 90.0, -10.0, 10.0, 1.0,
        0.0, 2.5, 0.33, 0.0, -0.29, -2.5, 90.0, 10.0, -10.0, 1.0, 11.96
    };
    const double motor_command_init_p[21] =
    {
        0, -1, -2087,  4177, -2508, 0,  18807, -2091,  1045, 6268,
        -1, -3,  2090, -4179,  2505, 0, -18810,  2087, -1045,-6272, 2093
    };
    const double motor_command_init_p2[21] =
    {
        0, 40,  3697,  9537, -5840, -344,  18810, -2000,  0,  8800,
        0, 40, -3727, -9536,  5809,  425, -18810,  2000,  0, -8800, 418
    };
    const double motor_rotation[21] =
    {
        1.0, 1.0, -1.0, 1.0, 1.0, -1.0, 1.0, 1.0, -1.0, -1.0,
        1.0, 1.0, 1.0, -1.0, -1.0, -1.0, -1.0, 1.0, -1.0, 1.0, 1.0
    };
    const int map[23] =
    {
        2, 3, 4, 5, 6, 7, 15, 16, 17, 18,
        8, 9, 10, 11, 12, 13, 19, 20, 21, 22, 14,
        23, 24
    };

    const short motor_l_bound[21] =
    {
        -19019, -6479, -14839, -209, -12749, -5225, -19019, -20064, -19019, -209,
        -6479, -4389, -17138, -27170, -12749, -5225, -31559, -209, -19019, -24035,
        209
    };

    const short motor_u_bound[21] =
    {
        6479, 4389, 17138, 27170, 12249, 5225, 31559, 209, 19019, 24035,
        19019, 6479, 14839, 209, 12749, 5225, 19019, 20064, 19019, 209,
        18810
    };
#if file_motor
    static double *qd;
    qd = calloc(22,sizeof(double));
    FILE *file=fopen("./Trajectories/motor.dat","r");
    if (file== NULL) perror ("Error opening robot trajectory file");
    int nb_scan;
#endif


#if save_data_long
    FILE *q_file=fopen("./../Simu_data/q.txt","w");
    //fclose(q_file);

    FILE *qd_file=fopen("./../Simu_data/qd.txt","w");
    //fclose(qd_file);

    FILE *t_file=fopen("./../Simu_data/t.txt","w");
    //fclose(t_file);

    FILE *dq_file=fopen("./../Simu_data/dq.txt","w");
    //fclose(dq_file);

//    FILE *q_file=fopen("./../Simu_data/q.txt","a");
//    FILE *qd_file=fopen("./../Simu_data/qd.txt","a");
//    FILE *t_file=fopen("./../Simu_data/t.txt","a");
//    FILE *dq_file=fopen("./../Simu_data/dq.txt","a");
#endif
#if save_data_quick
    int buf_size=8000;
    HoapSensor buff_sensor[buf_size];
    HoapControl buff_control[buf_size];
    zmp_calc buff_zmp_c[buf_size];
    double  buff_t[buf_size];
#endif

    static gsl_vector * idx1;
    static gsl_vector * idx2;
    static gsl_matrix * J1;
    static gsl_matrix * J2;
    static gsl_matrix * JCoMR;
    static gsl_matrix * JCoML;
    static gsl_matrix * Jtilde;
    static gsl_vector * task1;
    static gsl_vector * task2;
    static gsl_vector * taskCoMR;
    static gsl_vector * taskCoML;
    static gsl_vector * CoM;
    static gsl_vector * vec3;
    static gsl_vector * vec3_2;
    static gsl_vector * p;
    static gsl_matrix * R;
    static gsl_matrix * P1;
    static gsl_matrix * P2;
    static gsl_matrix * PCoMR;
    static gsl_matrix * PCoML;
    static gsl_matrix * Ptilde;
    static gsl_matrix * Ptmp;
    static gsl_matrix * invJ;
    static gsl_matrix * invJCoM;
    static gsl_vector * dq;
    static gsl_vector * dqtmp;
    static gsl_vector * dqtmp2;
    static gsl_vector * dq_old;
    static gsl_vector * ddq;

    static double *opd;
    static gsl_vector * trace;

    static gsl_vector * adphi;
    static double *qdev;
    //static gsl_vector * CoP;
    //static double f=0.0;
    static gsl_vector * zmp;
    static gsl_vector * dzmp;

    static gsl_vector * q_pd;

    static gsl_vector * Init_task_CoM;
    static gsl_vector * Init_task_F2F;
    static gsl_vector * Init_task_F2F_p;
    static gsl_matrix * Init_task_F2F_R;

    static gsl_vector * debug;
    static gsl_vector * error;
    double dt;
    static gsl_vector * Stand_task_CoM;


    static int path1[8] = {7, 7, 6, 5, 4, 3, 2, 1};
    static int path2[14] = {13, 13, 12, 11, 10, 9, 8, 2, 3, 4, 5, 6, 7, 7};

    static int init_task=1;
    if (init_task==1)
    {
        idx1 = gsl_vector_calloc (8);
        idx2 = gsl_vector_calloc (14);
        J1 = gsl_matrix_calloc (6,nDoF-6);
        J2 = gsl_matrix_calloc (6,nDoF-6);
        JCoMR = gsl_matrix_calloc (3,nDoF-6);
        JCoML = gsl_matrix_calloc (3,nDoF-6);
        Jtilde = gsl_matrix_calloc (3,nDoF-6);
        task1 = gsl_vector_calloc (6);
        task2 = gsl_vector_calloc (6);
        taskCoMR = gsl_vector_calloc (3);
        taskCoML = gsl_vector_calloc (3);
        CoM = gsl_vector_calloc (3);
        vec3 = gsl_vector_calloc (3);
        vec3_2 = gsl_vector_calloc (3);
        p = gsl_vector_calloc (3);
        R = gsl_matrix_calloc (3,3);
        P1 = gsl_matrix_calloc (nDoF-6,nDoF-6);
        P2 = gsl_matrix_calloc (nDoF-6,nDoF-6);
        PCoMR = gsl_matrix_calloc (nDoF-6,nDoF-6);
        PCoML = gsl_matrix_calloc (nDoF-6,nDoF-6);
        Ptilde = gsl_matrix_calloc (nDoF-6,nDoF-6);
        Ptmp = gsl_matrix_calloc (nDoF-6,nDoF-6);
        invJ = gsl_matrix_calloc (nDoF-6,6);
        invJCoM = gsl_matrix_calloc (nDoF-6,3);
        dq = gsl_vector_calloc(nDoF-6);
        dqtmp = gsl_vector_calloc(nDoF-6);
        dqtmp2 = gsl_vector_calloc(nDoF-6);
        dq_old = gsl_vector_calloc(nDoF-6);
        ddq = gsl_vector_calloc(nDoF-6);

        opd = calloc(9,sizeof(double));
        trace = gsl_vector_calloc (3);
        adphi = gsl_vector_calloc(nDoF-6);
        qdev = calloc(nDoF-6,sizeof(double));
        //CoP = gsl_vector_calloc (3);
        zmp = gsl_vector_calloc (3);
        dzmp = gsl_vector_calloc (3);

        q_pd = gsl_vector_calloc(nDoF-6);

        Init_task_CoM=gsl_vector_calloc (3);
        Init_task_F2F=gsl_vector_calloc (6);
        Init_task_F2F_p=gsl_vector_calloc (3);
        Init_task_F2F_R=gsl_matrix_calloc (3,3);

        debug = gsl_vector_calloc(nDoF-6);
        error = gsl_vector_calloc (3);
        Stand_task_CoM=gsl_vector_calloc (3);

        init_task=0;


        for(i=0; i<8; i++)
        {
            gsl_vector_set(idx1,i,path1[i]);
        }

        for(i=0; i<14; i++)
        {
            gsl_vector_set(idx2,i,path2[i]);
        }

        for (i=0; i<(nDoF-6); i++)
        {
            qdev[i]=fmin(fabs(uLINK[i+2].qmin-uLINK[i+2].qmoy),fabs(uLINK[i+2].qmax-uLINK[i+2].qmoy))*2;
        }
    }

    CamParam_s CamParam;
    CamInit(&CamParam);

#if reseau
    HoapSensor sensor;
    HoapControl control;
    zmp_calc zmp_c;

    Hoap hoap = hoapConnect("10.59.145.197", 55000, &sensor);
    hoapSensor(hoap, &sensor);

    for(j=0; j<(dof); j++)
    {
        if(uLINK[map[j]].fixed==0)
        {
            uLINK[map[j]].q = deg2rad*sensor.q[j]*motor_rotation[j]/209;
        }
        else
        {
            uLINK[map[j]].q = 0;
        }
        control.q[j]=sensor.q[j];
        printf("%4.6f ",rad2deg*uLINK[map[j]].q);
    }
    printf("\n");
    hoapControl(hoap, &sensor, &control);
#else
    for(j=0; j<(dof); j++)
    {
        buff_data.val[j]=rad2deg*motor_rotation[j]*uLINK[map[j]].q*209;
        //printf("%4.6f ",buff_data.val[j]);
    }
    printf("\n");

    for(i=0; i<(dof); i++)
    {
        uLINK[map[i]].q = deg2rad*motor_rotation[i]*motor_command_init_p2[i]/209;
        printf("%4.6f ",rad2deg*uLINK[map[i]].q);
    }
    printf("\n");
#endif


    gsl_vector_set_zero(uLINK[Base_].p);
    //gsl_vector_set (uLINK[Base].p, 2, 0.066);
    gsl_vector_set (uLINK[Base_].p, 2, 0.04);
    gsl_matrix_set_identity(uLINK[Base_].R);
    NodeForwardKinematics(uLINK,Base_,0);

    CalcCoM(uLINK,com);

    gsl_matrix_set_identity(R);
    gsl_vector_set_zero(p);
    CalcVWerrOri(uLINK, task2, p, R,idx2);


    gsl_vector_memcpy(Init_task_CoM,com);
    gsl_vector_memcpy(Init_task_F2F,task2);

    gsl_vector_set(Stand_task_CoM,0,0.026);
    gsl_vector_set(Stand_task_CoM,1,-0.038);
    gsl_vector_set(Stand_task_CoM,2,0.29);

//    gsl_vector_set(Stand_task_CoM,0,0.015);
//    gsl_vector_set(Stand_task_CoM,1,-0.047);
//    gsl_vector_set(Stand_task_CoM,2,0.285);
//
//    gsl_vector_set(Stand_task_CoM,0,0.010);
//    gsl_vector_set(Stand_task_CoM,1,-0.039);
//    gsl_vector_set(Stand_task_CoM,2,0.27);

    PrintGSLVector(Init_task_CoM);
    PrintGSLVector(Stand_task_CoM);
    //PrintGSLVector(Init_task_F2F);

    gsl_vector_scale(Init_task_F2F,-1.0f);
    for(j=0; j<3; j++)
    {
        gsl_vector_set(Init_task_F2F_p,j,gsl_vector_get(Init_task_F2F,j));
        gsl_vector_set(vec3,j,gsl_vector_get(Init_task_F2F,j+3));
    }
    Rodrigues(Init_task_F2F_R,vec3,1.0);


    //PrintGSLVector(Init_task_F2F_p);
    //PrintGSLVector(Init_task_CoM);
    //PrintGSLMatrix(Init_task_F2F_R);

#if file_human
    double scale_task_F2F, scale_task_CoM;

    Ext_op_trajectory(opd, 0);

    gsl_vector_set (p, 0, (opd[3]-opd[6]));
    gsl_vector_set (p, 1, (opd[4]-opd[7]));
    gsl_vector_set (p, 2, (opd[5]-opd[8]));
    //gsl_vector_scale(p,-1.0);
    PrintGSLVector(p);
    scale_task_F2F=gsl_blas_dnrm2(Init_task_F2F_p)/gsl_blas_dnrm2(p);


    gsl_vector_set (p, 0, opd[0]);
    gsl_vector_set (p, 1, opd[1]);
    gsl_vector_set (p, 2, opd[2]-0.1);
    PrintGSLVector(p);
    scale_task_CoM=gsl_blas_dnrm2(Init_task_CoM)/gsl_blas_dnrm2(p);
#endif


//
//    PrintGSLVector(Init_task_CoM);
//    PrintGSLVector(Init_task_F2F);

    Init_light();

//clock_t begin,end;
//double elapsed;
    int quit;
    quit=0;
    i=0;
    double freq_;
    unsigned __int64 baseTime_;
    unsigned __int64 pf;
    QueryPerformanceFrequency( (LARGE_INTEGER *)&pf );
    freq_ = 1.0 / (double)pf;
    QueryPerformanceCounter( (LARGE_INTEGER *)&baseTime_ );

    while(1)
        //for (i = 0; i < 10000; i++)
    {
        i++;
//begin = clock();

#if save_data_quick
        if (i>=buf_size)
        {
            break;
        }
#endif
        if (quit==1)
        {
            break;
        }


        SDL_PollEvent(&event);
        switch (event.type)
        {
        case SDL_QUIT:
#if save_data_quick
            quit=1;
            continue;
#else
            exit(0);
#endif
            break;
#if !Light
        case SDL_KEYDOWN:
            switch (event.key.keysym.sym)
            {
            case SDLK_LEFT:
                angular_z-=M_PI*0.05;
                break;
            case SDLK_RIGHT:
                angular_z+=M_PI*0.05;
                break;
            default:
                break;
            }
            break;
        case SDL_MOUSEMOTION: //la souris est bougée, ça n'intéresse que la caméra
            OnMouseMotion(&CamParam,event.motion);
            break;
        case SDL_MOUSEBUTTONUP:
        case SDL_MOUSEBUTTONDOWN:
            OnMouseButton(&CamParam,event.button); //tous les événements boutons (up ou down) sont donnés à la caméra
            break;
#endif
        default:
            break;
        }

#if reseau
        hoapSensor(hoap, &sensor);

        for(j=0; j<(dof); j++)
        {
            if(uLINK[map[j]].fixed==0)
            {
                uLINK[map[j]].q = deg2rad*sensor.q[j]*motor_rotation[j]/209;
            }
            else
            {
                uLINK[map[j]].q = 0;
            }
        }
        Hoap_calc_zmp(&sensor,&zmp_c);

#endif

#if file_motor
        for(j=0; j<10; j++)
        {
            nb_scan=fscanf(file, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f", &qd[0], &qd[1], &qd[2], &qd[3], &qd[4], &qd[5], &qd[6], &qd[7], &qd[8], &qd[9], &qd[10], &qd[11], &qd[12], &qd[13], &qd[14], &qd[15], &qd[16], &qd[17], &qd[18], &qd[19], &qd[20], &qd[21]);
        }

        if(nb_scan==-1)
        {
            break;
        }

        for(j=0; j<(dof); j++)
        {
            uLINK[map[j]].q = deg2rad*qd[j+1]*motor_rotation[j]/209;
        }
#endif


#if Tasks



        t=i*10;

#if file_human
        Ext_op_trajectory(opd, 0);
#endif

        CalcJacobianModif(uLINK,J2,idx2);
//        CalcCoMJacobian(uLINK,&Status, JCoMR, Status.right_foot_ID);
        CalcCoMJacobian(uLINK,&Status, JCoML, Status.left_foot_ID);

        static double wO=2.0;
        static double amp=0.015;
        static double interp=0.5;


//        gsl_matrix_set_identity(R);
//        gsl_vector_set_zero(p);
//        gsl_vector_set (p, 1, -0.078);

        //gsl_vector_set_zero(p);

#if file_human
        gsl_vector_set (p, 0, (opd[3]-opd[6]));
        gsl_vector_set (p, 1, (opd[4]-opd[7]));
        gsl_vector_set (p, 2, (opd[5]-opd[8]));
        gsl_vector_scale(p,scale_task_F2F);
#else

        if((t*Dtime)>=interp)
        {
            gsl_vector_set_zero(p);
            gsl_vector_set (p, 1, -0.078);
        }
        else
        {
            gsl_vector_memcpy(p,Init_task_F2F_p);
            gsl_vector_set(p,0,gsl_vector_get(p,0)*(interp-(t*Dtime))/interp);
            gsl_vector_set(p,1,(gsl_vector_get(p,1)*(interp-(t*Dtime))/interp)+(-0.078*(t*Dtime)/interp));
            gsl_vector_set(p,2,gsl_vector_get(p,2)*(interp-(t*Dtime))/interp);

        }
#endif

        if((t*Dtime)>=interp)
        {
            gsl_matrix_set_identity(R);
        }
        else
        {
            gsl_matrix_memcpy(R,Init_task_F2F_R);
            rot2omega(R,error);
            dt = gsl_blas_dnrm2 (error);
            gsl_vector_scale (error, 1/dt);
            dt=dt*(interp-(t*Dtime))/interp;
            Rodrigues(R,error,dt);
        }
        CalcVWerrOri(uLINK, task2, p, R,idx2);
        //PrintGSLVector(task2);



#if file_human
        gsl_vector_set (p, 0, opd[0]);
        gsl_vector_set (p, 1, opd[1]);
        gsl_vector_set (p, 2, opd[2]-0.05);
        gsl_vector_scale(p,scale_task_CoM);
#else
        //gsl_vector_memcpy(p,Init_task_CoM);
        if((t*Dtime)>interp)
        {
            gsl_vector_memcpy(p,Stand_task_CoM);
            gsl_vector_set (p, 2,gsl_vector_get(p,2)+amp*(cos((1/wO)*M_PI*(t*Dtime-interp))-1));
        }
        else
        {
            gsl_vector_memcpy(p,Init_task_CoM);
//            gsl_vector_scale(p,(interp-(t*Dtime))/interp);
//            gsl_vector_add(p,Stand_task_CoM*(t*Dtime)/interp);
            gsl_vector_set(p,0,(gsl_vector_get(p,0)*(interp-(t*Dtime))/interp)+(gsl_vector_get(Stand_task_CoM,0)*(t*Dtime)/interp));
            gsl_vector_set(p,1,(gsl_vector_get(p,1)*(interp-(t*Dtime))/interp)+(gsl_vector_get(Stand_task_CoM,1)*(t*Dtime)/interp));
            gsl_vector_set(p,2,(gsl_vector_get(p,2)*(interp-(t*Dtime))/interp)+(gsl_vector_get(Stand_task_CoM,2)*(t*Dtime)/interp));
            //it's ok
        }

#endif

        //gsl_blas_dgemv(CblasNoTrans, 1.0, uLINK[Status.right_foot_ID].R, p, 0.0, taskCoMR);
        gsl_vector_memcpy(taskCoML,p);
        CalcCoM(uLINK,CoM);
        //PrintGSLVector(CoM);
        gsl_vector_sub(taskCoML,CoM);
        //PrintGSLVector(taskCoMR);


        //gsl_blas_dgemv(CblasNoTrans, 1.0, uLINK[Status.left_foot_ID].R, p, 0.0, taskCoML);
//        gsl_vector_memcpy(taskCoML,p);
//        gsl_vector_sub(taskCoML,CoM);
//        gsl_vector_memcpy(taskCoMR,taskCoML);
//        PrintGSLVector(taskCoML);


        for (j=0; j<(nDoF-6); j++)
        {
            gsl_vector_set(adphi,j,-0.01*(2*(uLINK[j+2].q-uLINK[j+2].qmoy)/(qdev[j]*qdev[j])));
        }
//ping(6);

        // first task
        pinv(invJ,J2);
        gsl_blas_dgemv(CblasNoTrans, 1.0, invJ, task2, 0.0, dq);

        // second task
        gsl_matrix_set_identity(P1);
        pinv(invJ,J2);
        gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, invJ, J2, 0.0, Ptmp);
        gsl_matrix_sub(P1,Ptmp);

        gsl_vector_memcpy(vec3,taskCoML);
        gsl_blas_dgemv(CblasNoTrans, 1.0, JCoML, dq, 0.0, vec3_2);
        gsl_vector_sub(vec3,vec3_2);

        gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, JCoML, P1, 0.0, Jtilde);
        pinv(invJCoM,Jtilde);
        gsl_blas_dgemv(CblasNoTrans, 1.0, invJCoM, vec3, 0.0, dqtmp);
        //gsl_blas_dgemv(CblasNoTrans, 1.0, P2, dqtmp2, 0.0, dqtmp);
        gsl_vector_add(dq,dqtmp);

////ping(1);
//gsl_vector_memcpy(debug,dq);
//
//        // third task
//        gsl_matrix_memcpy(P2,P1);
//        pinv(invJCoM,Jtilde);
//        gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, invJCoM, Jtilde, 0.0, Ptmp);
//        gsl_matrix_sub(P2,Ptmp);
//
//        gsl_vector_memcpy(vec3,taskCoMR);
//        gsl_blas_dgemv(CblasNoTrans, 1.0, JCoMR, dq, 0.0, vec3_2);
//        gsl_vector_sub(vec3,vec3_2);
//
//        gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, JCoMR, P2, 0.0, Jtilde);
//        pinv(invJCoM,Jtilde);
//        gsl_blas_dgemv(CblasNoTrans, 1.0, invJCoM, vec3, 0.0, dqtmp);
//        //gsl_blas_dgemv(CblasNoTrans, 1.0, P2, dqtmp2, 0.0, dqtmp);
//        gsl_vector_add(dq,dqtmp);
//
//gsl_vector_sub(debug,dq);
//PrintGSLVector(debug);

        // fourth task
        gsl_matrix_memcpy(Ptilde,P1);
        //gsl_matrix_set_identity(Ptilde);
        pinv(invJCoM,Jtilde);
        gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, invJCoM, Jtilde, 0.0, Ptmp);
        gsl_matrix_sub(Ptilde,Ptmp);

        gsl_vector_sub(adphi,dq);

        pinv(Ptmp,Ptilde);
        gsl_blas_dgemv(CblasNoTrans, 1.0, Ptmp, adphi, 0.0, dqtmp);
        gsl_vector_add(dq,dqtmp);


#if reseau
        for(j=0; j<(dof); j++)
        {
            if(uLINK[map[j]].fixed==0)
            {
                //uLINK[map[j]].q =uLINK[map[j]].q+0.2*gsl_vector_get(dq,map[j]-2);
                control.q[j]=rad2deg*motor_rotation[j]*(uLINK[map[j]].q+0.5*gsl_vector_get(dq,map[j]-2))*209;
                //printf("%d: %d %d\n",j+1,sensor.q[j],control.q[j]);
                if(control.q[j]<(motor_l_bound[j]+209))
                {
                    control.q[j]=motor_l_bound[j]+209;
                }
                if(control.q[j]>(motor_u_bound[j]-209))
                {
                    control.q[j]=motor_u_bound[j]-209;
                }
            }
        }



        hoapControl(hoap, &sensor, &control);

#if save_data_long

        for(j=0; j<dof; j++)
        {
            fprintf(q_file,"%d ",sensor.q[j]);
        }
        fprintf(q_file,"\n");

        for(j=0; j<dof; j++)
        {
            fprintf(qd_file,"%d ",control.q[j]);
        }
        fprintf(qd_file,"\n");

        fprintf(t_file,"%f \n",t*Dtime);

        for(j=0; j<dof; j++)
        {
            fprintf(dq_file,"%f ",rad2deg*motor_rotation[j]*(gsl_vector_get(dq,map[j]-2))*209);
        }
        fprintf(dq_file,"\n");

#endif
#if save_data_quick
        buff_sensor[i-1]=sensor;
        buff_control[i-1]=control;
        buff_zmp_c[i-1]=zmp_c;
        static unsigned __int64 val;
        QueryPerformanceCounter( (LARGE_INTEGER *)&val );
        buff_t[i-1]=(double)(val - baseTime_) * freq_;
#endif


//        for(j=0; j<(dof); j++)
//        {
//            uLINK[map[j]].q = deg2rad*control.q[j]*motor_rotation[j]/209;
//        }

#else
        for(j=0; j<(dof); j++)
        {
            buff_data.val[j]=rad2deg*motor_rotation[j]*(uLINK[map[j]].q+0.001*gsl_vector_get(dq,map[j]-2))*209;
            //buff_data.val[j]=rad2deg*motor_rotation[j]*uLINK[map[j]].q*209;
        }
#endif




//        for(j=0; j<(dof); j++)
//        {
//            if(motor_rotation[j]<0)
//            {
//                if (buff_data.val[j]<(rad2deg*motor_rotation[j]*uLINK[map[j]].qmax*209))
//                {
//                    printf("t= %lf maxq: %d %d %f\n",t*Dtime,map[j],buff_data.val[j]/209,rad2deg*motor_rotation[j]*uLINK[map[j]].qmax);
//                    buff_data.val[j]=rad2deg*motor_rotation[j]*uLINK[map[j]].qmax*209;
//                }
//                if (buff_data.val[j]>(rad2deg*motor_rotation[j]*uLINK[map[j]].qmin*209))
//                {
//                    printf("t= %lf minq: %d %d %f\n",t*Dtime,map[j],buff_data.val[j]/209,rad2deg*motor_rotation[j]*uLINK[map[j]].qmin);
//                    buff_data.val[j]=rad2deg*motor_rotation[j]*uLINK[map[j]].qmin*209;
//                }
//            }
//            else
//            {
//                if (buff_data.val[j]>(rad2deg*motor_rotation[j]*uLINK[map[j]].qmax*209))
//                {
//                    printf("t= %lf maxq: %d %d %f\n",t*Dtime,map[j],buff_data.val[j]/209,rad2deg*motor_rotation[j]*uLINK[map[j]].qmax);
//                    buff_data.val[j]=rad2deg*motor_rotation[j]*uLINK[map[j]].qmax*209;
//                }
//                if (buff_data.val[j]<(rad2deg*motor_rotation[j]*uLINK[map[j]].qmin*209))
//                {
//                    printf("t= %lf minq: %d %d %f\n",t*Dtime,map[j],buff_data.val[j]/209,rad2deg*motor_rotation[j]*uLINK[map[j]].qmin);
//                    buff_data.val[j]=rad2deg*motor_rotation[j]*uLINK[map[j]].qmin*209;
//                }
//            }
//        }


#if file_human
        gsl_vector_set (uLINK[Base_].p, 0, opd[6]);
        gsl_vector_set (uLINK[Base_].p, 1, opd[7]);
        gsl_vector_set (uLINK[Base_].p, 2, opd[8]+0.1);
        gsl_vector_scale(uLINK[Base_].p,scale_task_F2F);
#else
        gsl_vector_set_zero(uLINK[Base_].p);
        gsl_vector_set (uLINK[Base_].p, 2, 0.04);
#endif


        gsl_matrix_set_identity(uLINK[Base_].R);
        NodeForwardKinematics(uLINK,Base_,0);

#endif

        ForwardKinematics(uLINK,1);

#if !Light
        sprintf(titre,"Visualisation t= %3.3f", t*Dtime);
        SDL_WM_SetCaption(titre, NULL);



        //glClearStencil(0);
//        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT |GL_STENCIL_BUFFER_BIT);
//glCullFace(GL_BACK);
//glCullFace(GL_FRONT);


        Init_light();
        glMatrixMode( GL_MODELVIEW );
        glLoadIdentity();
        Camlook(&Status,&CamParam);

        //glRotated(angular_z,0,0,1);



#if shadow

        GLfloat light0_position [] = {2.0f, -2.0f, 2.0f, 1.0f};
        GLfloat shadowPlane [] = {0.0f, 0.0f, 1.0f, 0.0f};
        //GLfloat shadowPlane [] = {0.0f, 0.0f, 1.0f, 0.0f};
        buildShadowMatrix( g_shadowMatrix, light0_position , shadowPlane );


        //
        // Render the floor to the stencil buffer so we can use it later to trim
        // the shadow at the floor's edge...
        //
        glEnable( GL_STENCIL_TEST );
        glStencilFunc( GL_ALWAYS, 1, 0xFFFFFFFF );         // Write a 1 to the stencil buffer everywhere we are about to draw
        glStencilOp( GL_REPLACE, GL_REPLACE, GL_REPLACE ); // If a 1 is written to the stencil buffer - simply replace the current value stored there with the 1.

        glColorMask( GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE ); // Disable writing to the color buffer
        glDepthMask( GL_FALSE );                               // Disable writing to the depth buffer
        DrawGround(2.0,0.0,-0.05,6.0,6.0,0.1);

        glColorMask( GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE ); // Re-enable writing to the color buffer
        glDepthMask( GL_TRUE );                            // Re-enable writing to the depth buffer
        glDisable( GL_STENCIL_TEST );

        //
        // Render the floor...
        //
        DrawGround(2.0,0.0,-0.05,6.0,6.0,0.1);

        //
        // Render the shadow...
        //

        // Use our stencil to keep the shadow from running off the floor.
        glEnable( GL_STENCIL_TEST );
        glStencilFunc( GL_EQUAL, 1, 0xFFFFFFFF ); // Only write to areas where the stencil buffer has a 1.
        glStencilOp( GL_KEEP, GL_KEEP, GL_KEEP ); // Don't modify the contents of the stencil buffer

        glDisable( GL_LIGHTING );
        glDisable( GL_DEPTH_TEST );
        glEnable( GL_BLEND );
        glDisable(GL_COLOR_MATERIAL);

        glPushMatrix();
        {
            // Load the teapot's shadow matrix
            glMultMatrixf( g_shadowMatrix );

            glColor3f(0.1f, 0.1f, 0.1f); // shadow color

            {
                DrawAllJoints(uLINK,1);
            }
        }
        glPopMatrix();

        glEnable( GL_DEPTH_TEST );
        glDisable( GL_BLEND );
        glEnable( GL_LIGHTING );
        glDisable( GL_STENCIL_TEST );
        glEnable(GL_COLOR_MATERIAL);

        DrawAllJoints(uLINK,1);

#endif

#if !shadow
        DrawGround(2.0,0.0,-0.05,6.0,6.0,0.1);
        DrawAllJoints(uLINK,1);
#endif

//    glDisable(GL_DEPTH_TEST);
//    glDisable(GL_LIGHTING);
//    glColor3f(0.0f, 0.0f, 0.0f); // Shadow's color
//    glPushMatrix();
//    {
//        glMultMatrixf((GLfloat *)g_shadowMatrix);
//        DrawAllJoints(uLINK,1);
//    }
//    glPopMatrix();
//    glEnable(GL_DEPTH_TEST);
//    glEnable(GL_LIGHTING);




        CalcCoM(uLINK,com);
#if colors
        glColor3ub(0,0,255);
#endif
#if materials
        set_material(&turquoise);
#endif
        gsl_vector_set (com, 2, 0);
        DrawMarker(com);

#if colors
        glColor3ub(255,0,0);
#endif
#if materials
        set_material(&ruby);
#endif
        Hoap_calc_zmp_visu(uLINK,&Status,&zmp_c);

//DrawOBJ("./Robots/HOAP3mesh_old/BODY_LINK01.obj");



        glFlush();
        SDL_GL_SwapBuffers();
        //SDL_Delay(5);
#endif

//end = clock();
//
//elapsed = ((double)(end - begin))/ CLOCKS_PER_SEC;
//printf("t: %f %d\n",elapsed,CLOCKS_PER_SEC );

    }

#if file_motor
    fclose(file);
#endif

#if reseau
    hoapDisconnect(hoap);
#endif


#if save_data_long
    fclose(q_file);
    fclose(qd_file);
    fclose(t_file);
    fclose(dq_file);
#endif
#if save_data_quick

    FILE *sensor_file=fopen("./../../Simu_data/sensor.txt","w");
    FILE *control_file=fopen("./../../Simu_data/control.txt","w");
    FILE *zmp_file=fopen("./../../Simu_data/zmp.txt","w");
    FILE *t_file=fopen("./../../Simu_data/t.txt","w");

    int k;
    for(k=0; k<(i-2); k++)
    {

        for(j=0; j<21; j++)
        {
            fprintf(sensor_file,"%d ",buff_sensor[k].q[j]);
        }
        fprintf(sensor_file,"\n");
        for(j=0; j<21; j++)
        {
            fprintf(sensor_file,"%d ",buff_sensor[k].dq[j]);
        }
        fprintf(sensor_file,"\n");
        for(j=0; j<4; j++)
        {
            fprintf(sensor_file,"%d ",buff_sensor[k].fsr_right[j]);
        }
        fprintf(sensor_file,"\n");
        for(j=0; j<4; j++)
        {
            fprintf(sensor_file,"%d ",buff_sensor[k].fsr_left[j]);
        }
        fprintf(sensor_file,"\n");
        for(j=0; j<6; j++)
        {
            fprintf(sensor_file,"%d ",buff_sensor[k].acc_gyro[j]);
        }
        fprintf(sensor_file,"\n \n");

        for(j=0; j<21; j++)
        {
            fprintf(control_file,"%d ",buff_control[k].q[j]);
        }
        fprintf(control_file,"\n");

        fprintf(zmp_file,"%f ",buff_zmp_c[k].zmp_right.W);
        fprintf(zmp_file,"%f ",buff_zmp_c[k].zmp_right.x);
        fprintf(zmp_file,"%f ",buff_zmp_c[k].zmp_right.y);
        fprintf(zmp_file,"%f ",buff_zmp_c[k].zmp_left.W);
        fprintf(zmp_file,"%f ",buff_zmp_c[k].zmp_left.x);
        fprintf(zmp_file,"%f ",buff_zmp_c[k].zmp_left.y);
        fprintf(zmp_file,"\n");

        fprintf(t_file,"%f ",buff_t[k]);
        fprintf(t_file,"\n");
    }
//    fwrite(&buff_sensor, (i-2)*sizeof(HoapSensor), 1, sensor_file);
//    fwrite(&buff_control, (i-2)*sizeof(HoapControl), 1, control_file);
//    fwrite(&buff_zmp_c, (i-2)*sizeof(zmp_calc), 1, zmp_file);

    fclose(sensor_file);
    fclose(control_file);
    fclose(zmp_file);
    fclose(t_file);
#endif



    return EXIT_SUCCESS; // Fermeture du programme




















#if Ext_traj
    for (i = 0; i < 50; i++)
    {

        ping(i);

        int nDoF=dof+6;
        static gsl_vector * idx1;
        static gsl_vector * idx2;
        static gsl_matrix * J1;
        static gsl_matrix * J2;
        static gsl_matrix * JCoM;
        static gsl_matrix * Jtilde;
        static gsl_vector * task1;
        static gsl_vector * task2;
        static gsl_vector * taskCoM;
        static gsl_vector * CoM;
        static gsl_vector * vec3;
        static gsl_vector * vec3_2;
        static gsl_vector * p;
        static gsl_matrix * R;
        static gsl_matrix * P1;
        static gsl_matrix * P2;
        static gsl_matrix * PCoM;
        static gsl_matrix * Ptmp;
        static gsl_matrix * invJ;
        static gsl_matrix * invJCoM;
        static gsl_vector * dq;
        static gsl_vector * dqtmp;
        static gsl_vector * dqtmp2;
        static double *opd;
        static gsl_vector * trace;

        static int init_task=1;
        if (init_task==1)
        {
            opd = calloc(9,sizeof(double));
            trace = gsl_vector_calloc (3);

            idx1 = gsl_vector_calloc (8);
            idx2 = gsl_vector_calloc (14);
            J1 = gsl_matrix_calloc (6,nDoF-6);
            J2 = gsl_matrix_calloc (6,nDoF-6);
            JCoM = gsl_matrix_calloc (3,nDoF-6);
            Jtilde = gsl_matrix_calloc (3,nDoF-6);
            task1 = gsl_vector_calloc (6);
            task2 = gsl_vector_calloc (6);
            taskCoM = gsl_vector_calloc (3);
            CoM = gsl_vector_calloc (3);
            vec3 = gsl_vector_calloc (3);
            vec3_2 = gsl_vector_calloc (3);
            p = gsl_vector_calloc (3);
            R = gsl_matrix_calloc (3,3);
            P1 = gsl_matrix_calloc (nDoF-6,nDoF-6);
            P2 = gsl_matrix_calloc (nDoF-6,nDoF-6);
            PCoM = gsl_matrix_calloc (nDoF-6,nDoF-6);
            Ptmp = gsl_matrix_calloc (nDoF-6,nDoF-6);
            invJ = gsl_matrix_calloc (nDoF-6,6);
            invJCoM = gsl_matrix_calloc (nDoF-6,3);
            dq = gsl_vector_calloc(nDoF-6);
            dqtmp = gsl_vector_calloc(nDoF-6);
            dqtmp2 = gsl_vector_calloc(nDoF-6);
            init_task=0;

//            static int path1[8] = {7, 7, 6, 5, 4, 3, 2, 1};
//            //int path1[8] = {1, 2, 3, 4, 5, 6, 7, 7};
//            for(j=0; j<8; j++)State *Status,
//            {
//                gsl_vector_set(idx1,j,path1[j]);
//            }

            //static int path2[14] = {7, 7, 6, 5, 4, 3, 2, 8, 9, 10, 11, 12, 13, 13};
            static int path2[14] = {13, 13, 12, 11, 10, 9, 8, 2, 3, 4, 5, 6, 7, 7};
            for(j=0; j<14; j++)
            {
                gsl_vector_set(idx2,j,path2[j]);
            }
            Ext_op_trajectory(opd, 0);
        }



        gsl_vector_set (uLINK[base].p, 0, opd[6]);
        gsl_vector_set (uLINK[base].p, 1, opd[7]);
        gsl_vector_set (uLINK[base].p, 2, opd[8]+0.034);

        NodeForwardKinematics(uLINK,base,0);



//        CalcJacobianModif( uLINK,J1,idx1);
        CalcJacobianModif( uLINK,J2,idx2);
        CalcCoMJacobian(uLINK,&Status, JCoM, base);


//        gsl_matrix_set_identity(R);
//        gsl_vector_set_zero(p);
//        gsl_vector_set (p, 0, 0.0155);
//        gsl_vector_set (p, 1, 0.0798);
//        gsl_vector_set (p, 2, 0.8434);
//        CalcVWerrOri(uLINK, task1, p, R,idx1);

        gsl_matrix_set_identity(R);
        gsl_vector_set_zero(p);
        gsl_vector_set (p, 0, (opd[3]-opd[6]));
        gsl_vector_set (p, 1, (opd[4]-opd[7]));
        gsl_vector_set (p, 2, (opd[5]-opd[8]));
        CalcVWerrOri(uLINK, task2, p, R,idx2);

        gsl_vector_set (taskCoM, 0, opd[0]);
        gsl_vector_set (taskCoM, 1, opd[1]);
        gsl_vector_set (taskCoM, 2, opd[2]-0.1);
        CalcCoM(uLINK,CoM);
        gsl_vector_sub(taskCoM,CoM);



//        gsl_matrix_set_identity(P1);
//        pinv(invJ,J1);
//        gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, invJ, J1, 0.0, Ptmp);
//        gsl_matrix_sub(P1,Ptmp);

        gsl_matrix_set_identity(P2);
        pinv(invJ,J2);
        gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, invJ, J2, 0.0, Ptmp);
        gsl_matrix_sub(P2,Ptmp);

        gsl_matrix_set_identity(PCoM);
        pinv(invJCoM,JCoM);
        gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, invJCoM, JCoM, 0.0, Ptmp);
        gsl_matrix_sub(PCoM,Ptmp);



        pinv(invJ,J2);
        gsl_blas_dgemv(CblasNoTrans, 1.0, invJ, task2, 0.0, dq);

        gsl_vector_memcpy(vec3,taskCoM);
        gsl_blas_dgemv(CblasNoTrans, 1.0, JCoM, dq, 0.0, vec3_2);
        gsl_vector_sub(vec3,vec3_2);

        gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, JCoM, P2, 0.0, Jtilde);

        pinv(invJCoM,Jtilde);
        gsl_blas_dgemv(CblasNoTrans, 1.0, invJCoM, vec3, 0.0, dqtmp2);

        gsl_blas_dgemv(CblasNoTrans, 1.0, P2, dqtmp2, 0.0, dqtmp);
        gsl_vector_add(dq,dqtmp);



        for (j=0; j<(nDoF-6); j++)
        {
            uLINK[j+2].q =uLINK[j+2].q+gsl_vector_get(dq,j);//+gsl_vector_get (g, n+6);
        }



        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glMatrixMode( GL_MODELVIEW );
        glLoadIdentity( );
        //glRotated(angular_z,0,0,1);
        //ForwardKinematics(uLINK,1);
        NodeForwardKinematics(uLINK,base,0);
        DrawAllJoints(uLINK,1);


        for(j=0; j<3; j++)
        {
            gsl_vector_set (trace, 0, opd[0+3*j]);
            gsl_vector_set (trace, 1, opd[1+3*j]);
            gsl_vector_set (trace, 2, opd[2+3*j]);
            DrawMarker(trace);
        }
        DrawGround(2.0,0.0,-0.2,6.0,6.0,0.1);
        glFlush();
        SDL_GL_SwapBuffers();
        SDL_Delay(5);
    }

//    return EXIT_SUCCESS; // Fermeture du programme
#endif

#endif



//double EndTime = 3;


    char files[40];
    int frame=0;
    char pixel_data[3*640*480];



    int ground=0;
//SDL_EnableKeyRepeat(SDL_DEFAULT_REPEAT_DELAY*1000,SDL_DEFAULT_REPEAT_INTERVAL*1000);




    if (Visualisation)
    {
        FILE *q_file=fopen("./../Simu_data/q.txt","w");
        fclose(q_file);

        FILE *qd_file=fopen("./../Simu_data/qd.txt","w");
        fclose(qd_file);

        FILE *t_file=fopen("./../Simu_data/t.txt","w");
        fclose(t_file);

        FILE *pBody_file=fopen("./../Simu_data/pBody.txt","w");
        fclose(pBody_file);

        FILE *uq_file=fopen("./../Simu_data/uq.txt","w");
        fclose(uq_file);

        FILE *ustab_file=fopen("./../Simu_data/ustab.txt","w");
        fclose(ustab_file);

        FILE *ug_file=fopen("./../Simu_data/ug.txt","w");
        fclose(ug_file);

        FILE *ug1_file=fopen("./../Simu_data/ug1.txt","w");
        fclose(ug1_file);
        FILE *ug2_file=fopen("./../Simu_data/ug2.txt","w");
        fclose(ug2_file);

        FILE *deb_file=fopen("./../Simu_data/deb.txt","w");
        fclose(deb_file);

        FILE *deb2_file=fopen("./../Simu_data/deb2.txt","w");
        fclose(deb2_file);

        FILE *dq_file=fopen("./../Simu_data/dq.txt","w");
        fclose(dq_file);

        FILE *p_file=fopen("./../Simu_data/p.txt","w");
        fclose(p_file);

        FILE *CoM_file=fopen("./../Simu_data/CoM.txt","w");
        fclose(CoM_file);

        FILE *CoMp_file=fopen("./../Simu_data/CoMp.txt","w");
        fclose(CoMp_file);

        FILE *taskCoMR_file=fopen("./../Simu_data/taskCoMR.txt","w");
        fclose(taskCoMR_file);

        FILE *taskCoML_file=fopen("./../Simu_data/taskCoML.txt","w");
        fclose(taskCoML_file);

        FILE *CoP_file=fopen("./../Simu_data/CoP.txt","w");
        fclose(CoP_file);

        FILE *u_file=fopen("./../Simu_data/u.txt","w");
        fclose(u_file);
    }


//    for (t = 0; t < 55; t++)
//    {
//
//      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//      glMatrixMode( GL_MODELVIEW );
//      glLoadIdentity( );
//            //glRotated(angular_z,0,0,1);
//
//      ForwardKinematics(uLINK,1);
//      DrawAllJoints(uLINK,1);
//
//      DrawGround(2.0,0.0,-0.2,6.0,6.0,0.1);
//      glFlush();
//      SDL_GL_SwapBuffers();
//      SDL_Delay(100);
//      ForwardDynamics(uLINK,&Status,t);
//      IntegrateEuler(uLINK,1);
//    }
//    return EXIT_SUCCESS; // Fermeture du programme





    while(1)
        //for (i = 0; i < EndTime/Dtime-1; i++)
    {
        t++;
        SDL_PollEvent(&event);
        switch (event.type)
        {
        case SDL_QUIT:
            exit(0);
            break;
        case SDL_KEYDOWN:
            switch (event.key.keysym.sym)
            {
            case SDLK_LEFT:
                angular_z-=M_PI*0.02;
                break;
            case SDLK_RIGHT:
                angular_z+=M_PI*0.02;
                break;
            case SDLK_g:
                ground = !ground;
                break;
            case SDLK_p:
                fgets ( szInput, 25, stdin );
                break;
            case SDLK_s:
                SaveState(uLINK,&Status,&t);
                break;
            case SDLK_l:
                LoadState(uLINK,&Status,&t);
                break;
            default:
                break;
            }
            break;
        }


        if (t*Dtime>180) //Arret du programme à x sacondes de simulation pour vidéo
        {
            break;
        }

//        if (t==100)
//        {
//        SaveState(uLINK[0],t*Dtime);
//        }

        if (t%frame_skip==0)
        {
            sprintf(titre,"Visualisation t= %3.3f", t*Dtime);
            //sprintf(titre,"Visualisation t= %3.3f, %1.6f", t*Dtime, Lc+Lt+Lp-gsl_vector_get (uLINK[0][1].p, 2) );
            //sprintf(titre,"Visualisation t= %2.3f cop=%f center=%f", t*Dtime,Status[0].integral_R,Status[0].integral_L);
            SDL_WM_SetCaption(titre, NULL);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glMatrixMode( GL_MODELVIEW );
            glLoadIdentity( );
            glRotated(angular_z,0,0,1);


#if StaticCOM
            gsl_vector_set (uLINK[1].p, 2, Lc+Lt+Lp-0.07);
            gsl_vector_set (pos, 0, -0.10);
            gsl_vector_set (pos, 1, 0.21);
            gsl_vector_set (pos, 2, -0.95);
            InverseSherpaKinematics(q, pos);
            uLINK[2].q=gsl_vector_get (q,0);
            uLINK[3].q=gsl_vector_get (q,1);
            uLINK[4].q=gsl_vector_get (q,2);
            uLINK[5].q=gsl_vector_get (q,3);
            uLINK[6].q=gsl_vector_get (q,4);
            uLINK[7].q=gsl_vector_get (q,5);
            gsl_vector_set (pos, 0, -0.10);
            gsl_vector_set (pos, 1, 0.21);
            gsl_vector_set (pos, 2, -0.95);
            InverseSherpaKinematics(q, pos);
            uLINK[8].q=gsl_vector_get (q,0);
            uLINK[9].q=gsl_vector_get (q,1);
            uLINK[10].q=gsl_vector_get (q,2);
            uLINK[11].q=gsl_vector_get (q,3);
            uLINK[12].q=gsl_vector_get (q,4);
            uLINK[13].q=gsl_vector_get (q,5);
            ForwardKinematics(uLINK,1);
            DrawAllJoints(uLINK,1);
            DrawGround(2.0,0.0,-0.2,6.0,6.0,0.1);
            CalcCoM(uLINK,com);
            glColor3ub(0,0,255);
            if (!ground)
            {
                gsl_vector_set (com, 2, 0);
            }
            //sprintf(titre,"Visualisation t= %2.3f, x= %2.3f, y= %2.3f", t*Dtime,gsl_vector_get (com,0),gsl_vector_get (com,1));
            //SDL_WM_SetCaption(titre, NULL);
            DrawMarker(com);
            DrawIndicators(uLINK,&Status,com,CoP,ground);

#endif


#if !StaticCOM

            //ForwardDynamics(uLINK,&Status,t);
            //IntegrateEuler(uLINK,1);
            DrawAllJoints(uLINK,1);
            DrawIndicators(uLINK,&Status,com,CoP,ground);
            DrawGround(2.0,0.0,-0.2,6.0,6.0,0.1);

#endif



            glFlush();
            SDL_GL_SwapBuffers();

            if (Video==1)
            {
                if (frame==0)
                {
                    sprintf(files,"./../Simu_images/Test%ld.bmp", t/400);
                    //glReadBuffer(GL_FRONT);
                    glReadPixels(0, 0, 640, 480, GL_RGB, GL_UNSIGNED_BYTE, pixel_data);//GL_BGR
                    write_bmp(files, 640, 480, pixel_data);
                    frame=1/(Dtime*frame_skip*img_per_s);
                }
                frame--;
            }
            //angular_z+=M_PI*0.02;

        }
#if !StaticCOM
        //else
        {
            //gsl_vector_set_zero(uLINK[1].p);
            //gsl_vector_set (uLINK[1].p, 2, 1.2);
            //gsl_vector_set_zero(uLINK[1].vo);
            ForwardDynamics(uLINK,&Status,t);
            IntegrateEuler(uLINK,1);
            /// \todo Runge kuta
        }
#endif

    }

    SDL_Quit(); // Arrêt de la SDL

    gsl_vector_free(com);
    gsl_vector_free(CoP);
    gsl_vector_free(q);
    gsl_vector_free(pos);

    free(uLINK);

    return EXIT_SUCCESS; // Fermeture du programme

}



/*! \mainpage 	SimuSherpaConsole Reference Manual
 *
 * SimuSherpaConsole is a program which simulates a 3D biped robot with multi contact and multibody computations.
 *
 * \section author_sec Author
 *
 * This simulator is developped by David Galdeano (galdeano@lirmm.fr) under the supervision of Ahmed Chemori, Sebastien Krut and Philippe Fraisse at the
 *  <a href="http://www.lirmm.fr"> Montpellier Laboratory of Informatics, Robotics, and Microelectronics</a> (LIRMM in french).
 *
 * \section intro_sec Introduction
 *
 * This program is coded in C using the ezxml, SDL, OpenGL and gsl libraries functions. For now, SimuSherpaConsole is available for Windows and may work on other systems.
 *
 * Many functions for dynamics computations are adapted from Kajita's book: Humanoid Robot.
 *
 * ezxml librarie is used for xml file manipulation.
 * SDL and OpenGL libraries are used for graphical representations of the simulation.
 * gls librarie is used for vector and matrix computations.
 *
 * The project uses a code::blocs solution and may work on other IDE.
 *
 * \copyright This program must not be modified, distributed or used in an other application without the authorization of the autor and is distributed as is, without any guaranties in case of malfunctioning.
 *
 */




