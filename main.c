
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

//#include <windows.h>
#include "TotalMass.h"




int main(int argc, char *argv[])
{
    freopen("CON", "w", stdout);
    freopen("CON", "r", stdin);
    freopen("CON", "w", stderr);

    printf("Sherpa Simulator\n");

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
            //fgets ( szInput, 25, stdin );
            //j=atoi(szInput);
            j=3;
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
    SDL_SetVideoMode(640, 480, 32, SDL_OPENGL); // Ouverture de la fenêtre
    SDL_WM_SetCaption("Visualisation", NULL);

//Handle=FindWindow("SDL_app","Visualisation");
//SetWindowPos(Handle,HWND_TOPMOST,1,1,640,480,SWP_SHOWWINDOW);

    glMatrixMode( GL_PROJECTION );
    glLoadIdentity( );
    gluPerspective(50,(double)640/480,1,1000);
    gluLookAt(2*gsl_vector_get(Status.com_old,2), -2*gsl_vector_get(Status.com_old,2), 1.4*gsl_vector_get(Status.com_old,2), 0, 0, 0.9*gsl_vector_get(Status.com_old,2), 0, 0, 1);
    glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
    glEnable(GL_DEPTH_TEST | GL_LINE_SMOOTH);


    float Lc=0.5073;
    float Lt=0.510;
    float Lp=0.1104;
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
    float *qd;
    qd = calloc(dof,sizeof(float));
    Ext_trajectory_init(qd);
    0.0661375
    for(i=1; i<(dof+2); i++)
    {
        uLINK[i].q = qd[i-1];
    }
    free (qd);
    ForwardKinematics(uLINK,1);
#endif

#if Task
//    float *qd;
//    qd = calloc(dof,sizeof(float));
//    Ext_trajectory_init(qd);
//    for(i=1; i<(dof+2); i++)
//    {
//        uLINK[i].q = qd[i-1];
//    }
//    free (qd);

//    uLINK[2].q = -0.1745;
//    uLINK[5].q = 0.3491;
//    uLINK[6].q = -0.1745;
//
//    uLINK[8].q = -0.1745;
//    uLINK[11].q = 0.3491;
//    uLINK[12].q = -0.1745;

    for(i=1; i<(dof+2); i++)
    {
        uLINK[i].q = uLINK[i].qmoy;
    }


    gsl_vector_set_zero(uLINK[Base].p);
    gsl_vector_set (uLINK[Base].p, 2, 0.066);
    //gsl_vector_set (uLINK[base].p, 2, 0.2);
    gsl_matrix_set_identity(uLINK[Base].R);
    NodeForwardKinematics(uLINK,Base,0);

    CalcCoM(uLINK,com);
    PrintGSLVector(com);

////
//////return EXIT_SUCCESS; // Fermeture du programme
////
////
//////    for (i = 0; i < 540; i++)
//////    {
//////        float *kqd;
//////        kqd = calloc(nDoF-6,sizeof(float));
//////        Ext_q_trajectory(kqd, 0);
//////
//////        for (j=0; j<(nDoF-6); j++)
//////        {
//////            uLINK[j+2].q= kqd[j+1];
//////        }
//////
//////        free (kqd);return EXIT_SUCCESS; // Fermeture du programme
//////
//////        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//////        glMatrixMode( GL_MODELVIEW );
//////        glLoadIdentity( );
//////        //glRotated(angular_z,0,0,1);
//////        ForwardKinematics(uLINK,1);
//////        DrawAllJoints(uLINK,1);
//////        DrawGround(2.0,0.0,-0.2,6.0,6.0,0.1);
//////        glFlush();
//////        SDL_GL_SwapBuffers();
//////        SDL_Delay(5);
//////    }
//////
//////    return EXIT_SUCCESS; // Fermeture du programme
////

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
        static float *opd;
        static gsl_vector * trace;

        static int init_task=1;
        if (init_task==1)
        {
            opd = calloc(9,sizeof(float));
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
        CalcCoMJacobian(uLINK, JCoM, base);


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

////    return EXIT_SUCCESS; // Fermeture du programme
#endif

#endif



    //float EndTime = 3;
    long t=0;
    char titre[40] = "Visualisation t=0";

    char files[40];
    int frame=0;
    char pixel_data[3*640*480];


    SDL_Event event;
    int ground=0;
    double angular_z=0;
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

        FILE *taskCoM_file=fopen("./../Simu_data/taskCoM.txt","w");
        fclose(taskCoM_file);

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
            /// todo : Runge kuta
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
 * This program is coded in C using the SDL, OpenGL and gsl libraries functions. For now, SimuSherpaConsole is available for Windows and may work on other systems.
 *
 * Many functions for dynamics computations are adapted from Kajita's book: Humanoid Robot.
 *
 * SDL and OpenGL libraries are used for graphical representations of the simulation.
 * gls librarie is used for vector and matrix computations.
 *
 * The project uses a code::blocs solution and may work on other IDE.
 *
 * \copyright This program must not be modified, distributed or used in an other application without the authorization of the autor and is distributed as is, without any guaranties in case of malfunctioning.
 *
 */




