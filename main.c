
/*!
 *  \file main.c
 *  \brief     This is the multi body simulator
 *  \details   This program use gsl for matrix coputation and SDL and openGL for visualization
 *  \author    David Galdeano
 *  \date      10/2011
 *  \pre       First install gsl, SDL and openGL for a proper compilation
 *  \warning   Do not modify this programm unless you know what you do.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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

#include "bmp.h"
#include "IntegrateEuler.h"
#include "ForwardDynamics.h"
#include "ForwardKinematics.h"


#include "InverseSherpaKinematics.h"
#include "CalcCoM.h"
#include "DrawMarker.h"

#include "DrawIndicators.h"

#include "Setup.h"

//#include <windows.h>
#include "TotalMass.h"

// test traj pas


int main(int argc, char *argv[])
{
    printf("Sherpa Simulator\n");
    int i=0;

    SuLINK uLINK[NbRobots][NbLinks];
    State Status[NbRobots];
    for (i = 0; i < NbRobots; i++)
    {
        SetupRobot(uLINK[i],&Status[i]);
    }


    printf("Robot wheigth: %f \n",TotalMass(uLINK[0],1));
    //printf("%f \n",gsl_vector_get (uLINK[2].b,1));
    //printf("%8.1f \n",uLINK[8].mother);

    gsl_vector * com = gsl_vector_calloc (3);
    gsl_vector * CoP = gsl_vector_calloc (3);
    gsl_vector * q = gsl_vector_calloc (6);
    gsl_vector * pos = gsl_vector_calloc (3);
    gsl_vector * Visu = gsl_vector_calloc (3);

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
#if Sherpa
    gluLookAt(1.5, -1.5, 1, 0, 0, 0.6, 0, 0, 1);
#endif
#if Generic
    gluLookAt(1.8, -1.8, 1.4, 0, 0, 0.9, 0, 0, 1);
#endif
    glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
    glEnable(GL_DEPTH_TEST | GL_LINE_SMOOTH);


    float Lc=0.5073;
    float Lt=0.510;
    float Lp=0.1104;
    int frame_skip = 20;
    //int frame_skip = 1;
    for (i = 0; i < NbRobots; i++)
    {
        gsl_vector_set (uLINK[i][1].p, 2, Lc+Lt+Lp+0.0005);
        gsl_vector_set (uLINK[i][1].p, 1, i);
    }
    //gsl_vector_set (uLINK[0][1].p, 2, Lc+Lt+Lp+0.15);

#if Suspendu
    gsl_vector_set (uLINK[0][1].p, 2, Lc+Lt+Lp+0.1);
    uLINK[0][1].supportHeight=Lc+Lt+Lp+0.13;
#endif

    float Dtime = 0.0001;
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

    }


//    for (t = 0; t < 55; t++)
//    {
//
//      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//      glMatrixMode( GL_MODELVIEW );
//      glLoadIdentity( );
//            //glRotated(angular_z,0,0,1);
//
//      ForwardKinematics(uLINK[0],1);
//      DrawAllJoints(uLINK[0],1);
//      glFlush();
//      SDL_GL_SwapBuffers();
//      SDL_Delay(100);
//      ForwardDynamics(uLINK[0],&Status[0],Dtime,t);
//      IntegrateEuler(uLINK[0],1,Dtime);
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
            default:
                break;
            }
            break;
        }


        if (t*Dtime>105) //Arret du programme à x sacondes de simulation pour vidéo
        {
            break;
        }


        if (t%frame_skip==0)
        {

            sprintf(titre,"Visualisation t= %3.3f", t*Dtime);
            //sprintf(titre,"Visualisation t= %2.3f cop=%f center=%f", t*Dtime,Status[0].integral_R,Status[0].integral_L);
            SDL_WM_SetCaption(titre, NULL);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glMatrixMode( GL_MODELVIEW );
            glLoadIdentity( );
            glRotated(angular_z,0,0,1);


#if StaticCOM
            for (i = 0; i < NbRobots; i++)
            {
                gsl_vector_set (uLINK[i][1].p, 2, Lc+Lt+Lp-0.07);
                gsl_vector_set (pos, 0, -0.10);
                gsl_vector_set (pos, 1, 0.21);
                gsl_vector_set (pos, 2, -0.95);
                InverseSherpaKinematics(q, pos);
                uLINK[i][2].q=gsl_vector_get (q,0);
                uLINK[i][3].q=gsl_vector_get (q,1);
                uLINK[i][4].q=gsl_vector_get (q,2);
                uLINK[i][5].q=gsl_vector_get (q,3);
                uLINK[i][6].q=gsl_vector_get (q,4);
                uLINK[i][7].q=gsl_vector_get (q,5);
                gsl_vector_set (pos, 0, -0.10);
                gsl_vector_set (pos, 1, 0.21);
                gsl_vector_set (pos, 2, -0.95);
                InverseSherpaKinematics(q, pos);
                uLINK[i][8].q=gsl_vector_get (q,0);
                uLINK[i][9].q=gsl_vector_get (q,1);
                uLINK[i][10].q=gsl_vector_get (q,2);
                uLINK[i][11].q=gsl_vector_get (q,3);
                uLINK[i][12].q=gsl_vector_get (q,4);
                uLINK[i][13].q=gsl_vector_get (q,5);
                ForwardKinematics(uLINK[i],1);
                DrawAllJoints(uLINK[i],1);
                CalcCoM(uLINK[i],com);
                glColor3ub(0,0,255);
                if (!ground)
                {
                    gsl_vector_set (com, 2, 0);
                }
                //sprintf(titre,"Visualisation t= %2.3f, x= %2.3f, y= %2.3f", t*Dtime,gsl_vector_get (com,0),gsl_vector_get (com,1));
                //SDL_WM_SetCaption(titre, NULL);
                DrawMarker(com);
                DrawIndicators(uLINK[i],&Status[i],com,CoP,Visu,ground);
            }
#endif


#if !StaticCOM
            for (i = 0; i < NbRobots; i++)
            {
                ForwardDynamics(uLINK[i],&Status[i],Dtime,t);
                IntegrateEuler(uLINK[i],1,Dtime);
                //PrintState(uLINK);
                DrawAllJoints(uLINK[i],1);
                DrawIndicators(uLINK[i],&Status[i],com,CoP,Visu,ground);
            }
#endif



            glFlush();
            SDL_GL_SwapBuffers();

            if (Video==1)
            {
                if (frame==0)
                {
                    sprintf(files,"./../Simu_images/Test%ld.bmp", t/400);
                    glReadBuffer(GL_FRONT);
                    glReadPixels(0, 0, 640, 480, GL_RGB, GL_UNSIGNED_BYTE, pixel_data);//GL_BGR
                    write_bmp(files, 640, 480, pixel_data);
                    frame=20;
                }
                frame--;
            }

        }
#if !StaticCOM
        else
        {
            for (i = 0; i < NbRobots; i++)
            {
                ForwardDynamics(uLINK[i],&Status[i],Dtime,t);
                IntegrateEuler(uLINK[i],1,Dtime);
            }
        }
#endif

    }

    SDL_Quit(); // Arrêt de la SDL

    gsl_vector_free(com);
    gsl_vector_free(CoP);
    gsl_vector_free(q);
    gsl_vector_free(pos);
    gsl_vector_free(Visu);

    return EXIT_SUCCESS; // Fermeture du programme

}







