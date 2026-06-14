#include <winsock2.h>

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
#include <vector>
#include <Eigen/Dense>




#include <SDL2/SDL.h>
#include "Shader.h"
#include "DefaultShaders.h"
SDL_Window* window = NULL;
SDL_GLContext gl_context = NULL;
Shader defaultShader;
glm::vec3 activeColor(1.0f, 1.0f, 1.0f);
bool shadowPassActive = false;
double angular_z = 0.0;

void cleanup_sdl(void) {
    if (gl_context) {
        SDL_GL_DeleteContext(gl_context);
    }
    if (window) {
        SDL_DestroyWindow(window);
    }
    SDL_Quit();
}
#include "OpenGLHeaders.h"


#include<time.h>

#include "uLink.h"
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
#include "gsl_matrix_get_row_m.h"


#include "Setup.h"


#include "TotalMass.h"
#include "ObjLoader.h"
#include "DrawLight.h"
#include "DrawCamera.h"
#include "netHOAP.h"
#include "Rodrigues.h"
#include "rot2omega.h"
#include "DrawScene.h"

#include "Hoap_calc_zmp.h"
#include "butterworth.h"
#include "NPD.h"
#include "lecture_ecriture.h"

#if mathGL
#include <mgl2/mgl_cf.h>
#endif

#ifdef WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

//#include "Rdtsc.h"

void ReadTrajectory(double *opd, int pos) {
#if file_human
    Ext_op_trajectory2(opd, pos);
#elif file_walk
    Ext_walk_trajectory(opd, pos);
#else
    (void)opd;
    (void)pos;
#endif
}

int main(int argc, char *argv[])
{
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);


    printf("Sherpa Simulator\n");

#ifdef HAVE_INLINE
    printf("HAVE_INLINE\n");
#endif
#ifdef GSL_RANGE_CHECK_OFF
    printf("GSL_RANGE_CHECK_OFF\n");
#endif

    printf("\n Choose your robot: \n");

    int i=0, j=0;
    char szInput [25];
    char RobotFile[255];
#if ROBOT_SHERPA
    strcpy(RobotFile, "./Robots/RobotSherpa.xml");
#elif ROBOT_HUMAN
    strcpy(RobotFile, "./Robots/Human.xml");
#else
    strcpy(RobotFile, "./Robots/HOAP3v7.xml");
#endif
    printf("\nLoading robot file: %s\n", RobotFile);



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


    std::vector<SuLINK> uLINK_vec(dof + 2);
    SuLINK *uLINK = uLINK_vec.data();

    State Status;

    strcpy(Status.RobotFile,RobotFile);

    //LoadRobotXML(uLINK,&Status,RobotFile);
    LoadRobotParserXML(uLINK,&Status,RobotFile);
    //LoadSherpa(uLINK,&Status);


    printf("Robot wheigth: %f \n",TotalMass(uLINK,1)); fflush(stdout);
    printf("DEBUG 1: TotalMass done\n"); fflush(stdout);
    //printf("%f \n",uLINK[2].b(1));
    //printf("%8.1f \n",uLINK[8].mother);


    char files[40];
    int frame=0;
    //char pixel_data[3*1024*768];
    char *pixel_data;
    pixel_data=(char *)calloc(3*1024*768,sizeof(double));


    Eigen::Vector3d com = Eigen::Vector3d::Zero();
    Eigen::Vector3d CoP = Eigen::Vector3d::Zero();
    Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    long t=0;
    char titre[40] = "Visualisation t=0";
    angular_z=0;


#if Video
    char buf[256];
    d = opendir("./../Simu_images/");

    while(dir = readdir(d))
    {
        //printf("%s\n",dir->d_name);
        sprintf(buf, "%s/%s", "./../../Simu_images/", dir->d_name);
        remove(buf);
    }
#endif //Video



//HWND Handle=NULL;

    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        FILE *err_file = fopen("sdl_error.txt", "w");
        if (err_file) {
            fprintf(err_file, "SDL_Init failed: %s\n", SDL_GetError());
            fclose(err_file);
        }
        exit(101);
    }
    atexit(cleanup_sdl);
    printf("DEBUG 2: SDL_Init done\n"); fflush(stdout);

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

    window = SDL_CreateWindow("Visualisation", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 1024, 768, SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
    if (window == NULL) {
        FILE *err_file = fopen("sdl_error.txt", "w");
        if (err_file) {
            fprintf(err_file, "SDL_CreateWindow failed: %s\n", SDL_GetError());
            fclose(err_file);
        }
        exit(102);
    }
    printf("DEBUG 3: SDL_CreateWindow done\n"); fflush(stdout);
    gl_context = SDL_GL_CreateContext(window);
    if (gl_context == NULL) {
        FILE *err_file = fopen("sdl_error.txt", "w");
        if (err_file) {
            fprintf(err_file, "SDL_GL_CreateContext failed: %s\n", SDL_GetError());
            fclose(err_file);
        }
        exit(103);
    }
    printf("DEBUG 4: SDL_GL_CreateContext done\n"); fflush(stdout);

    glewExperimental = GL_TRUE;
    GLenum err = glewInit();
    if (err != GLEW_OK) {
        FILE *err_file = fopen("glew_error.txt", "w");
        if (err_file) {
            fprintf(err_file, "glewInit failed: %s\n", glewGetErrorString(err));
            fclose(err_file);
        }
        exit(104);
    }
    printf("DEBUG 5: glewInit done\n"); fflush(stdout);

    SDL_GL_SetSwapInterval(0); // vertical retrace sync off
    SDL_Event event;



    //gluPerspective(50,(double)640/480,1,1000);
    //gluLookAt(1, 0, 0, 0, 0, 0, 0, 0, 1);
    //gluLookAt(2*Status.com_old(2), -2*Status.com_old(2), 1.4*Status.com_old(2), 0, 0, 0, 0, 0, 1);
//    gluLookAt(2*Status.com_old(2), -2*Status.com_old(2), 1.4*Status.com_old(2), 0, 0, 0.9*Status.com_old(2), 0, 0, 1);
    glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    // gl_context init complete

    // Initialize defaultShader
    printf("DEBUG 6: Before Shader compile\n"); fflush(stdout);
    defaultShader = Shader(DEFAULT_VERTEX_SHADER, DEFAULT_FRAGMENT_SHADER);
    printf("DEBUG 7: Shader compiled successfully\n"); fflush(stdout);

    // Buffer robot model meshes
    for(i = 1; i < dof + 2; i++) {
        if (strlen(uLINK[i].obj) > 0) {
            printf("DEBUG: Buffering mesh %d: '%s', triangleCount=%d, gl_v=%p\n", i, uLINK[i].obj, uLINK[i].Mesh_obj.triangleCount, (void*)uLINK[i].Mesh_obj.gl_v); fflush(stdout);
            buffer_model(&(uLINK[i].Mesh_obj));
            printf("DEBUG: Buffering mesh %d done\n", i); fflush(stdout);
        }
    }
    printf("DEBUG 8: buffer_model loop done\n"); fflush(stdout);

    double Lc=0.5073;
    double Lt=0.510;
    double Lp=0.1104;
    int frame_skip = 20;
    int img_per_s = 25;
    //int frame_skip = 1;
//    for (i = 0; i < NbRobots; i++)
//    {
//        uLINK[i][1].p(2) = Lc+Lt+Lp-0.0006;
//        uLINK[i][1].p(1) = i;
//    }
    //uLINK[0][1].p(2) = Lc+Lt+Lp+0.15;


    static ButterworthData data_lx;
    ButterworthFilterInit(&data_lx,0.01);


#if 0
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

    double freq_,t2;

#ifdef WIN32
    static unsigned __int64 baseTime_;
    static unsigned __int64 pf_;
    QueryPerformanceFrequency( (LARGE_INTEGER *)&pf_ );
    QueryPerformanceCounter( (LARGE_INTEGER *)&baseTime_ );
    freq_ = 1.0 / (double)pf_;
#else
    static unsigned long long baseTime_;
    static unsigned long long pf_;
    struct timeval tv;
    gettimeofday (&tv, NULL);
    baseTime_ = ((unsigned long long) tv.tv_sec * 1000000) + tv.tv_usec;
    freq_ = 0.000001;
#endif

//    for(i=1; i<(dof+2); i++)
//    {
//        uLINK[i].q = 0;
//    }

    printf("DEBUG 9: Before ForwardKinematics\n"); fflush(stdout);
    ForwardKinematics(uLINK,1);
    printf("DEBUG 10: After ForwardKinematics\n"); fflush(stdout);

    if(uLINK[Status.right_foot_ID].p(2)<uLINK[Status.left_foot_ID].p(2))
    {
        uLINK[Status.right_foot_ID].p(2) = 0.11;


        uLINK[Status.right_foot_ID].R.setIdentity();
        NodeForwardKinematics(uLINK,Status.right_foot_ID,0);
    }
    else
    {
        uLINK[Status.left_foot_ID].p(2) = 0.11;
        //uLINK[Status.left_foot_ID].R.setIdentity();
        NodeForwardKinematics(uLINK,Status.left_foot_ID,0);
        uLINK[Status.right_foot_ID].R.setIdentity();
        NodeForwardKinematics(uLINK,Status.right_foot_ID,0);
    }

    int quitt=0;

    CamParam_s CamParamt;
    CamInit(&CamParamt);

    i=0;





    double dt;

    while(1)
        //for (i = 0; i < 10000; i++)
    {
        i++;
//begin = clock();


        if (quitt==1)
        {
            break;
        }

//SDL_PumpEvents() ;
        SDL_PollEvent(&event);
        switch (event.type)
        {
        case SDL_QUIT:
#if save_data_quick
            quitt=1;
            continue;
#else //save_data_quick
            exit(0);
#endif //save_data_quick
            break;
#if Light
        case SDL_KEYDOWN:
            quitt=1;
            break;
#endif
#if !Light
        case SDL_KEYDOWN:
            switch (event.key.keysym.sym)
            {

            default:
                break;
            }
            break;
        case SDL_MOUSEMOTION: //la souris est bougée, ça n'intéresse que la caméra
            OnMouseMotion(&CamParamt,event.motion);
            break;
        case SDL_MOUSEBUTTONUP:
        case SDL_MOUSEBUTTONDOWN:
            OnMouseButton(&CamParamt,event.button); //tous les événements boutons (up ou down) sont donnés à la caméra
            break;
        case SDL_MOUSEWHEEL:
            {
                SDL_MouseButtonEvent dummy_btn_event;
                memset(&dummy_btn_event, 0, sizeof(dummy_btn_event));
                dummy_btn_event.type = SDL_MOUSEBUTTONDOWN;
                dummy_btn_event.button = (event.wheel.y > 0) ? SDL_BUTTON_WHEELUP : SDL_BUTTON_WHEELDOWN;
                OnMouseButton(&CamParamt, dummy_btn_event);
            }
            break;
#endif //!Light
        default:
            break;
        }


#ifdef WIN32
        static unsigned __int64 val_;
        QueryPerformanceCounter( (LARGE_INTEGER *)&val_ );
#else //WIN32
        static unsigned long long val_;
        struct timeval tv;
        gettimeofday (&tv, NULL);
        val_ = ((unsigned long long) tv.tv_sec * 1000000) + tv.tv_usec;
#endif //WIN32

#if !Light
        printf("fps: %f \n",1/(((double)(val_ - baseTime_) * freq_ )-t2));

//        static uint64_t time,time2;
//        time=rdtsc();
//        printf("fps: %d %f \n",(double)(3200000000/(double)(time-time2)));
//        time2=time;

#endif //!Light
        t2=(double)(val_ - baseTime_) * freq_;






        const int nDoF=dof+6;

        static gsl_vector * idx1;
        static gsl_vector * idx2;
        static gsl_matrix * J1;
        static gsl_matrix * J2;
        static gsl_matrix * J1_l;
        static gsl_matrix * JCoML;
        static double *qdev;

        static gsl_vector * p;
        static gsl_matrix * R;
        static gsl_vector * Init_task_F2F_p;
        static gsl_matrix * Init_task_F2F_R;
        static gsl_vector * error;
        static gsl_vector * task2;
        static gsl_vector * task3;
        static gsl_vector * task3_l;
        static gsl_vector * Stand_task_CoM;
        static gsl_vector * Init_task_CoM;
        static gsl_vector * taskCoML;
        static gsl_vector * CoM;

        static gsl_vector * adphi;
        static gsl_matrix * invJ;
        static gsl_vector * dq;
        static gsl_matrix * P1;
        static gsl_matrix * P2;
        static gsl_matrix * Ptmp;
        static gsl_vector * vec3;
        static gsl_vector * vec3_2;
        static gsl_vector * vec1;
        static gsl_vector * vec1_2;
        static gsl_matrix * Jtilde;
        static gsl_matrix * Jtilde2;
        static gsl_matrix * invJtilde2;
        static gsl_matrix * invJCoM;
        static gsl_vector * dqtmp;
        static gsl_matrix * Ptilde;

        static gsl_vector * Init_task_F2F;

        static double F2F_y;

        static int path1[8] = {7, 7, 6, 5, 4, 3, 2, 1};
        static int path2[14] = {13, 13, 12, 11, 10, 9, 8, 2, 3, 4, 5, 6, 7, 7};

        static int init_tmp=1;
        if (init_tmp==1)
        {
            idx1 = gsl_vector_calloc (8);
            idx2 = gsl_vector_calloc (14);
            J1 = gsl_matrix_calloc (6,nDoF-6);
            J2 = gsl_matrix_calloc (6,nDoF-6);
            J1_l = gsl_matrix_calloc (1,nDoF-6);
            JCoML = gsl_matrix_calloc (3,nDoF-6);

            p = gsl_vector_calloc (3);
            R = gsl_matrix_calloc (3,3);
            Init_task_F2F_p=gsl_vector_calloc (3);
            Init_task_F2F_R=gsl_matrix_calloc (3,3);
            error = gsl_vector_calloc (3);
            task2 = gsl_vector_calloc (6);
            task3 = gsl_vector_calloc (3);
            task3_l = gsl_vector_calloc (1);
            Stand_task_CoM=gsl_vector_calloc (3);
            Init_task_CoM=gsl_vector_calloc (3);
            taskCoML = gsl_vector_calloc (3);
            CoM = gsl_vector_calloc (3);

            adphi = gsl_vector_calloc(nDoF-6);
            invJ = gsl_matrix_calloc (nDoF-6,6);
            dq = gsl_vector_calloc(nDoF-6);
            P1 = gsl_matrix_calloc (nDoF-6,nDoF-6);
            P2 = gsl_matrix_calloc (nDoF-6,nDoF-6);
            Ptmp = gsl_matrix_calloc (nDoF-6,nDoF-6);
            vec3 = gsl_vector_calloc (3);
            vec3_2 = gsl_vector_calloc (3);
            vec1 = gsl_vector_calloc (1);
            vec1_2 = gsl_vector_calloc (1);

            Jtilde = gsl_matrix_calloc (3,nDoF-6);
            Jtilde2 = gsl_matrix_calloc (1,nDoF-6);
            invJtilde2 = gsl_matrix_calloc (nDoF-6,1);
            invJCoM = gsl_matrix_calloc (nDoF-6,3);
            dqtmp = gsl_vector_calloc(nDoF-6);
            Ptilde = gsl_matrix_calloc (nDoF-6,nDoF-6);

            Init_task_F2F=gsl_vector_calloc (6);

            qdev = (double *)calloc(nDoF-6,sizeof(double));

            for(i=0; i<8; i++)
            {
                idx1(i) = path1[i];
            }


            for(i=0; i<14; i++)
            {
                idx2(i) = path2[i];
            }

            for (i=0; i<(nDoF-6); i++)
            {
                qdev[i]=fmin(fabs(uLINK[i+2].qmin-uLINK[i+2].qmoy),fabs(uLINK[i+2].qmax-uLINK[i+2].qmoy))*2;
                printf("%f \n",qdev[i]);
            }

            CalcCoM(uLINK,com);
//PrintGSLVector(com);

            R.setIdentity();
            p.setZero();
            CalcVWerrOri(uLINK, task2, p, R,idx2);
//PrintGSLVector(task2);

            Init_task_CoM = com;

            Init_task_F2F = task2;
            Init_task_CoM -= uLINK[baseFoot].p;
//PrintGSLVector(Init_task_CoM);

//
//            Stand_task_CoM(0) = 0.026;
//            Stand_task_CoM(1) = -0.038;
//            Stand_task_CoM(2) = 0.29-0.06;
            Stand_task_CoM = Init_task_CoM;


            Init_task_F2F *= -1.0f;
            for(j=0; j<3; j++)
            {
                Init_task_F2F_p(j) = Init_task_F2F(j);
                vec3(j) = Init_task_F2F(j+3);
            }
            Rodrigues(Init_task_F2F_R,vec3,1.0);
//PrintGSLVector(Init_task_F2F_p);
//PrintGSLVector(vec3);
//PrintGSLMatrix(Init_task_F2F_R);
            F2F_y=Init_task_F2F_p(1);

            init_tmp=0;
        }

        CalcJacobianModif(uLINK,J1,idx1);
        CalcJacobianModif(uLINK,J2,idx2);
        CalcCoMJacobian(uLINK,&Status, JCoML, Status.left_foot_ID);


//PrintGSLMatrix(J2);
        gsl_matrix_get_row_m(J1_l, J1, 4);
//gsl_matrix_get_column_m(J2_l, J2, 4);
//PrintGSLMatrix(J1_l);

        static double wO=3.0;
        static double amp=0.02;
        static double t_init=3.0;
        static double t_stand=5.0;
        static double t_stand_zmp=7.0;



        if((t2)>=t_init)
        {
            p.setZero();
            p(1) = F2F_y;
        }
        else
        {
            p = Init_task_F2F_p;
            p(0) = p(0*(t_init-(t2))/t_init);
            p(1) = (p(1*(t_init-(t2))/t_init)+(F2F_y*(t2)/t_init));
            p(2) = p(2*(t_init-(t2))/t_init);

        }

        if((t2)>=t_init)
        {
            R.setIdentity();
        }
        else
        {
            R = Init_task_F2F_R;
            rot2omega(R,error);
            dt = error.norm();
            if (dt>0.0000000001)
            {
                error *= 1/dt;
            }
            dt=dt*(t_init-(t2))/t_init;

            Rodrigues(R,error,dt);
        }
        //R = Init_task_F2F_R;
        CalcVWerrOri(uLINK, task2, p, R,idx2);
        //PrintGSLVector(task2);



        if(t2>t_stand_zmp)
        {
            p = Stand_task_CoM;
            p(2) = p(2+amp*(cos((1/wO)*M_PI*(t2-t_stand_zmp))-1));
        }
        else if((t2>t_init) && (t2<t_stand_zmp))
        {
            p = Stand_task_CoM;
            //p(1) = p(1);
            //p(2) = p(2);
        }
        else
        {
            p = Init_task_CoM;
//            p *= (t_init-(t*Dtime)/t_init);
//            p += Stand_task_CoM*(t*Dtime/t_init);
        }


        //taskCoMR = uLINK[Status.right_foot_ID].R * p;
        taskCoML = p;//-foot.p
        CalcCoM(uLINK,CoM);//-foot.p
        CoM -= uLINK[baseFoot].p;
        //PrintGSLVector(CoM);
        taskCoML -= CoM;
        //PrintGSLVector(taskCoML);

        rot2omega(uLINK[1].R,task3);
        task3_l(0) = -task3(1);





        for (j=0; j<(nDoF-6); j++)
        {
            adphi(j) = -0.05*(2*(uLINK[j+2].q-uLINK[j+2].qmoy/(qdev[j]*qdev[j])));
        }


        // first task
        pinv(invJ,J2);
        dq = invJ * task2;

        // second task
        P1.setIdentity();
        pinv(invJ,J2);
        Ptmp = invJ * J2;
        P1 -= Ptmp;

        vec3 = taskCoML;
        vec3_2 = JCoML * dq;
        vec3 -= vec3_2;

        Jtilde = JCoML * P1;
        pinv(invJCoM,Jtilde);
        dqtmp = invJCoM * vec3;
        //dqtmp = P2 * dqtmp2;
        dq += dqtmp;


//PrintGSLVector(task3_l);
        // third task
        P2 = P1;
        pinv(invJCoM,Jtilde);
        Ptmp = invJCoM * Jtilde;
        P2 -= Ptmp;

        vec1 = task3_l;
        vec1_2 = J1_l * dq;
        vec1 -= vec1_2;

        Jtilde2 = J1_l * P2;
        pinv(invJtilde2,Jtilde2);
        dqtmp = invJtilde2 * vec1;
        //dqtmp = P2 * dqtmp2;
        dq += dqtmp;



//debug = dq;
//
//        // third task
//        P2 = P1;
//        pinv(invJCoM,Jtilde);
//        Ptmp = invJCoM * Jtilde;
//        P2 -= Ptmp;
//
//        vec3 = taskCoMR;
//        vec3_2 = JCoMR * dq;
//        vec3 -= vec3_2;
//
//        Jtilde = JCoMR * P2;
//        pinv(invJCoM,Jtilde);
//        dqtmp = invJCoM * vec3;
//        //dqtmp = P2 * dqtmp2;
//        dq += dqtmp;
//
//debug -= dq;
//PrintGSLVector(debug);





        // fourth task
        Ptilde = P2;
        //Ptilde.setIdentity();
        pinv(invJtilde2,Jtilde2);
        Ptmp = invJtilde2 * Jtilde2;
//        pinv(invJCoM,Jtilde);
//        Ptmp = invJCoM * Jtilde;
        Ptilde -= Ptmp;

        adphi -= dq;

        pinv(Ptmp,Ptilde);
        //Ptmp = Ptilde;
        dqtmp = Ptmp * adphi;
        dq += dqtmp;




        for(j=0; j<dof; j++)
        {
            uLINK[j+2].q=uLINK[j+2].q+0.05*dq(j);
            //uLINK[j+2].q=uLINK[j+2].q;
        }


        if(uLINK[Status.right_foot_ID].p(2)<uLINK[Status.left_foot_ID].p(2))
        {
            uLINK[Status.right_foot_ID].p(2) = 0.11;
            uLINK[Status.right_foot_ID].R.setIdentity();
            NodeForwardKinematics(uLINK,Status.right_foot_ID,0);
        }
        else
        {
            uLINK[Status.left_foot_ID].p(2) = 0.11;
            //uLINK[Status.left_foot_ID].R.setIdentity();
            NodeForwardKinematics(uLINK,Status.left_foot_ID,0);
            uLINK[Status.right_foot_ID].R.setIdentity();
            NodeForwardKinematics(uLINK,Status.right_foot_ID,0);
        }



        ForwardKinematics(uLINK,1);

        DrawScene(uLINK,&Status, &CamParamt);


//        static int init_tmpz=1;
//        if (init_tmpz==1)
//        {
//            for(j=0; j<dof; j++)
//            {
//                ButterworthFilterInit(&(uLINK[j+2].filter));
//            }
//
//            init_tmpz=0;
//        }
//
//        if((t2)>=t_init)
//        {
//            for(j=0; j<dof; j++)
//            {
//                uLINK[j+2].q=ButterworthFilter (&(uLINK[j+2].filter), uLINK[j+2].q);
//            }
//        }
//        else
//        {
//            for(j=0; j<dof; j++)
//            {
//                ButterworthFilter (&(uLINK[j+2].filter), uLINK[j+2].q);
//            }
//        }


    }

    return EXIT_SUCCESS; // Fermeture du programme

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#endif






























#if 1






#if Suspendu
    uLINK[1].p(2) = Lc+Lt+Lp+0.1;
    uLINK[1].supportHeight=Lc+Lt+Lp+0.13;
#endif //Suspendu


#if Ext_traj
    double *qd;
    qd = (double *)calloc(dof,sizeof(double));
    Ext_trajectory_init(qd);

    for(i=1; i<(dof+2); i++)
    {
        uLINK[i].q = qd[i-1];
    }
    free (qd);
    ForwardKinematics(uLINK,1);
#endif //Ext_traj



#if CMD_TASK
    const int nDoF=dof+6;
    const double rad2deg=180/M_PI;
    const double deg2rad=M_PI/180;
    const double motor_command_init[23] =
    {
        0.0, -2.5, -0.33, 0.0, 0.29, 2.5, 90.0, -10.0, 10.0, 1.0,
        0.0, 2.5, 0.33, 0.0, -0.29, -2.5, 90.0, 10.0, -10.0, 1.0, 11.96,
        0.0, 0.0
    };
    const double motor_command_init_p[23] =
    {
        0, -1, -2087,  4177, -2508, 0,  18807, -2091,  1045, 6268,
        -1, -3,  2090, -4179,  2505, 0, -18810,  2087, -1045,-6272, 2093,
        0, 0
    };
    const double motor_command_init_p2[23] =
    {
        0, 40,  3697,  9537, -5840, -344,  18810, -2000,  0,  8800,
        0, 40, -3727, -9536,  5809,  425, -18810,  2000,  0, -8800, 418,
        0, 0
    };
    const double motor_rotation[23] =
    {
        1.0, 1.0, -1.0, 1.0, 1.0, -1.0, 1.0, 1.0, -1.0, -1.0,
        1.0, 1.0, 1.0, -1.0, -1.0, -1.0, -1.0, 1.0, -1.0, 1.0, 1.0,
        1.0, 1.0
    };
    const int map[23] =
    {
        2, 3, 4, 5, 6, 7, 15, 16, 17, 18,
        8, 9, 10, 11, 12, 13, 19, 20, 21, 22, 14,
        23, 24
    };

    const int motor_l_bound[23] =
    {
        -19019, -6479, -14839, -209, -12749, -5225, -19019, -20064, -19019, -209,
        -6479, -4389, -17138, -27170, -12749, -5225, -31559, -209, -19019, -24035,
        209,
        -30000, -30000
    };

    const int motor_u_bound [23] =
    {
        6479, 4389, 17138, 27170, 12749, 5225, 31559, 209, 19019, 24035,
        19019, 6479, 14839, 209, 12749, 5225, 19019, 20064, 19019, 209,
        18810,
        30000, 30000
    };

    double com_err[23]=     {
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0
    };
    double command[23]=     {
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0
    };



#if file_motor
    static double *qd;
    qd = (double *)calloc(22,sizeof(double));
    FILE *file=fopen("./Trajectories/motor.dat","r");
    if (file== NULL) perror ("Error opening robot trajectory file");
    int nb_scan;
#endif //file_motor


#if save_data_long
    FILE *q_file=fopen("./../../Simu_data/q.txt","w");
    //fclose(q_file);

    FILE *qd_file=fopen("./../../Simu_data/qd.txt","w");
    //fclose(qd_file);

    FILE *t_file=fopen("./../../Simu_data/t.txt","w");
    //fclose(t_file);

    FILE *dq_file=fopen("./../../Simu_data/dq.txt","w");
    //fclose(dq_file);

//    FILE *q_file=fopen("./../Simu_data/q.txt","a");
//    FILE *qd_file=fopen("./../Simu_data/qd.txt","a");
//    FILE *t_file=fopen("./../Simu_data/t.txt","a");
//    FILE *dq_file=fopen("./../Simu_data/dq.txt","a");
#endif //save_data_long
#if save_data_quick

    const int buf_size=200000;
    HoapSensor *buff_sensor;
    buff_sensor = (HoapSensor *)calloc(buf_size,sizeof(HoapSensor));
    zmp_calc *buff_zmp_c;
    buff_zmp_c = (zmp_calc *)calloc(buf_size,sizeof(zmp_calc));
    zmp_calc *buff_zmp_f;
    buff_zmp_f = (zmp_calc *)calloc(buf_size,sizeof(zmp_calc));
    double  *buff_t;
    buff_t = (double *)calloc(buf_size,sizeof(double));

#if save_data_quick_temp
    int temp_size=29;
    double  *buff_temp;
    buff_temp = (double *)calloc(buf_size*temp_size,sizeof(double));
    if (buff_temp==NULL)
    {
        printf("error allocation buff_temp");
    }
#endif //save_data_quick_temp

    for(i=0; i<=buf_size; i++)
    {
        buff_sensor[i].q[0]=0;
        //buff_control[i-1]=control;
        buff_zmp_c[i].zmp_right.W=0;
        buff_zmp_f[i].zmp_right.W=0;
        buff_t[i]=0;
#if save_data_quick_temp
        buff_temp[i*temp_size]=0;
#endif //save_data_quick_temp
    }


#endif //save_data_quick

    static Eigen::VectorXd idx1;
    static Eigen::VectorXd idx2;
    static Eigen::MatrixXd J1;
    static Eigen::MatrixXd J2;
    static Eigen::MatrixXd J1_l;
    static Eigen::MatrixXd J1_3;
    static Eigen::MatrixXd JCoMR;
    static Eigen::MatrixXd JCoML;
    static Eigen::MatrixXd Jtilde;
    static Eigen::MatrixXd Jtilde2;
    static Eigen::MatrixXd invJtilde;
    static Eigen::MatrixXd invJtilde2;
    static Eigen::VectorXd task1;
    static Eigen::VectorXd task2;
    static Eigen::VectorXd task2s;
    static Eigen::Vector3d task3;
    static Eigen::VectorXd task3_l;
    static Eigen::VectorXd taskCoMR;
    static Eigen::VectorXd taskCoML;
    static Eigen::Vector3d CoM;
    static Eigen::Vector3d vec3;
    static Eigen::Vector3d vec3_2;
    static Eigen::VectorXd vec1;
    static Eigen::VectorXd vec1_2;

    static Eigen::Vector3d p;
    static Eigen::Matrix3d R;
    static Eigen::MatrixXd P1;
    static Eigen::MatrixXd P2;
    static Eigen::MatrixXd PCoMR;
    static Eigen::MatrixXd PCoML;
    static Eigen::MatrixXd Ptilde;
    static Eigen::MatrixXd Ptmp;
    static Eigen::MatrixXd invJ;
    static Eigen::MatrixXd invJCoM;
    static Eigen::VectorXd dq;
    static Eigen::VectorXd dqtmp;
    static Eigen::VectorXd dqtmp2;
    static Eigen::VectorXd dq_old;
    static Eigen::VectorXd ddq;

    static double *opd;
    static double *opd_old;
    static double *opd_old2;

    static double *oqd;
    static short *oqd_old;
    static short *oqd_old2;

    static Eigen::Vector3d trace;

    static Eigen::VectorXd adphi;
    static double *qdev;
    //static Eigen::Vector3d CoP;
    //static double f=0.0;
    static Eigen::Vector3d zmp;
    static Eigen::Vector3d dzmp;

    static Eigen::VectorXd q_pd;

    static Eigen::Vector3d Init_task_CoM;
    static Eigen::VectorXd Init_task_F2F;
    static Eigen::Vector3d Init_task_F2F_p;
    static Eigen::Matrix3d Init_task_F2F_R;

    static Eigen::VectorXd debug;
    static Eigen::Vector3d error;
    double dt;
    static Eigen::Vector3d Stand_task_CoM;
    static Eigen::Vector3d taskZMP;


    //static int path1[8] = {7, 7, 6, 5, 4, 3, 2, 1};
    //static int path1[9] = {7, 7, 6, 5, 4, 3, 2, 14, 14};
    static int path1[10] = {14, 14, 8, 2, 3, 4, 5, 6, 7, 7};
    static int path2[14] = {13, 13, 12, 11, 10, 9, 8, 2, 3, 4, 5, 6, 7, 7};


    static int init_task=1;
    if (init_task==1)
    {
        idx1 = Eigen::VectorXd::Zero(10);
        idx2 = Eigen::VectorXd::Zero(14);
        J1 = Eigen::MatrixXd::Zero(6,nDoF-6);
        J2 = Eigen::MatrixXd::Zero(6,nDoF-6);
        J1_l = Eigen::MatrixXd::Zero(1,nDoF-6);
        J1_3 = Eigen::MatrixXd::Zero(3,nDoF-6);
        JCoMR = Eigen::MatrixXd::Zero(3,nDoF-6);
        JCoML = Eigen::MatrixXd::Zero(3,nDoF-6);
        Jtilde = Eigen::MatrixXd::Zero(3,nDoF-6);
        Jtilde2 = Eigen::MatrixXd::Zero(1,nDoF-6);
        invJtilde = Eigen::MatrixXd::Zero(nDoF-6,3);
        invJtilde2 = Eigen::MatrixXd::Zero(nDoF-6,1);

        task1 = Eigen::VectorXd::Zero(6);
        task2 = Eigen::VectorXd::Zero(6);
        task2s = Eigen::VectorXd::Zero(6);
        task3 = Eigen::Vector3d::Zero();
        task3_l = Eigen::VectorXd::Zero(1);
        taskCoMR = Eigen::VectorXd::Zero(3);
        taskCoML = Eigen::VectorXd::Zero(3);
        CoM = Eigen::Vector3d::Zero();
        vec3 = Eigen::Vector3d::Zero();
        vec3_2 = Eigen::Vector3d::Zero();
        vec1 = Eigen::VectorXd::Zero(1);
        vec1_2 = Eigen::VectorXd::Zero(1);

        p = Eigen::Vector3d::Zero();
        R = Eigen::Matrix3d::Zero();
        P1 = Eigen::MatrixXd::Zero(nDoF-6,nDoF-6);
        P2 = Eigen::MatrixXd::Zero(nDoF-6,nDoF-6);
        PCoMR = Eigen::MatrixXd::Zero(nDoF-6,nDoF-6);
        PCoML = Eigen::MatrixXd::Zero(nDoF-6,nDoF-6);
        Ptilde = Eigen::MatrixXd::Zero(nDoF-6,nDoF-6);
        Ptmp = Eigen::MatrixXd::Zero(nDoF-6,nDoF-6);
        invJ = Eigen::MatrixXd::Zero(nDoF-6,6);
        invJCoM = Eigen::MatrixXd::Zero(nDoF-6,3);
        dq = Eigen::VectorXd::Zero(nDoF-6);
        dqtmp = Eigen::VectorXd::Zero(nDoF-6);
        dqtmp2 = Eigen::VectorXd::Zero(nDoF-6);
        dq_old = Eigen::VectorXd::Zero(nDoF-6);
        ddq = Eigen::VectorXd::Zero(nDoF-6);

        opd = (double *)calloc(9,sizeof(double));
        opd_old = (double *)calloc(9,sizeof(double));
        opd_old2 = (double *)calloc(9,sizeof(double));

        oqd = (double *)calloc(nDoF-6,sizeof(double));
        oqd_old = (short *)calloc(nDoF-6,sizeof(short));
        oqd_old2 = (short *)calloc(nDoF-6,sizeof(short));

        trace = Eigen::Vector3d::Zero();
        adphi = Eigen::VectorXd::Zero(nDoF-6);
        qdev = (double *)calloc(nDoF-6,sizeof(double));
        //CoP = Eigen::Vector3d::Zero();
        zmp = Eigen::Vector3d::Zero();
        dzmp = Eigen::Vector3d::Zero();

        q_pd = Eigen::VectorXd::Zero(nDoF-6);

        Init_task_CoM = Eigen::Vector3d::Zero();
        Init_task_F2F = Eigen::VectorXd::Zero(6);
        Init_task_F2F_p = Eigen::Vector3d::Zero();
        Init_task_F2F_R = Eigen::Matrix3d::Zero();

        debug = Eigen::VectorXd::Zero(nDoF-6);
        error = Eigen::Vector3d::Zero();
        Stand_task_CoM = Eigen::Vector3d::Zero();
        taskZMP = Eigen::Vector3d::Zero();

        init_task=0;


        for(i=0; i<10; i++)
        {
            idx1(i) = path1[i];
        }

        for(i=0; i<14; i++)
        {
            idx2(i) = path2[i];
        }

        for (i=0; i<(nDoF-6); i++)
        {
            qdev[i]=fmin(fabs(uLINK[i+2].qmin-uLINK[i+2].qmoy),fabs(uLINK[i+2].qmax-uLINK[i+2].qmoy))*2;
        }
    }

#if zmp_filtering

    static Eigen::Vector3d zmp_left = Eigen::Vector3d::Zero();
    static Eigen::Vector3d zmp_right = Eigen::Vector3d::Zero();
    static Eigen::Vector3d zmp_moy = Eigen::Vector3d::Zero();
    static double zmp_x_magin;
    static double zmp_y_magin;

    static int init_task_zmp=1;
    if (init_task_zmp==1)
    {
        zmp_x_magin=0.005;
        zmp_y_magin=0.01;

        init_task_zmp=0;
    }
#endif //zmp_filtering


    CamParam_s CamParam;
    CamInit(&CamParam);

    zmp_calc zmp_c;
    zmp_calc zmp_f;
    zmp_f.zmp_right.W=0;
    zmp_f.zmp_right.x=0;
    zmp_f.zmp_right.y=0;
    zmp_f.zmp_left.W=0;
    zmp_f.zmp_left.x=0;
    zmp_f.zmp_left.y=0;

#if reseau
    HoapSensor sensor;
    HoapControl control;


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

        com_err[j]=sensor.q[j];
        command[j]=sensor.q[j];

        printf("%4.6f ",rad2deg*uLINK[map[j]].q);
    }
    printf("\n");
    hoapControl(hoap, &sensor, &control);
#else //reseau
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
#endif //reseau



#if file_human
    static double foot_Z_offset=0.04;
    static double CoM_Z_offset=0.00;
#endif

#if (file_human || file_walk)
    ReadTrajectory(opd, 0);
    for(j=0; j<9; j++)
    {
        opd_old[j]=opd[j];
        opd_old2[j]=opd[j];
    }
#endif

#if !file_human
    uLINK[baseFoot].p.setZero();
    uLINK[baseFoot].p(2) = 0.04;
#endif


#if file_hoap

    Ext_q_hoap_trajectory(oqd_old, 0);

    for(j=0; j<21; j++)
    {
        oqd[j]=(short)(oqd_old[j]);
        oqd_old2[j]=oqd_old[j];
    }
#endif //file_hoap



    uLINK[baseFoot].R.setIdentity();
    NodeForwardKinematics(uLINK,baseFoot,0);




    CalcCoM(uLINK,com);

    R.setIdentity();
    p.setZero();
    CalcVWerrOri(uLINK, task2, p, R,idx2);


    Init_task_CoM = com;
    Init_task_F2F = task2;
    Init_task_CoM -= uLINK[baseFoot].p;


    Stand_task_CoM(0) = 0.026;
    Stand_task_CoM(1) = -0.038;
    Stand_task_CoM(2) = 0.29-0.04;

//    Stand_task_CoM(0) = 0.015;
//    Stand_task_CoM(1) = -0.047;
//    Stand_task_CoM(2) = 0.28;


    PrintGSLVector(Init_task_CoM);
    PrintGSLVector(Stand_task_CoM);
    //PrintGSLVector(Init_task_F2F);

    Init_task_F2F *= -1.0f;
    for(j=0; j<3; j++)
    {
        Init_task_F2F_p(j) = Init_task_F2F(j);
        vec3(j) = Init_task_F2F(j+3);
    }
    Rodrigues(Init_task_F2F_R,vec3,1.0);


    PrintGSLVector(Init_task_F2F_p);
    //PrintGSLMatrix(Init_task_F2F_R);

#if file_human
    double scale_task_F2F, scale_task_CoM;

    p(0) = (opd[3]-opd[6]);
    p(1) = (opd[4]-opd[7]);
    p(2) = (opd[5]-opd[8]);
    //p *= -1.0;
    PrintGSLVector(p);
    scale_task_F2F=Init_task_F2F_p.norm()/p.norm();
    printf("scale_task_F2F %f \n",scale_task_F2F);

    p(0) = opd[0]-opd[6];
    p(1) = opd[1]-opd[7];
    p(2) = opd[2]-opd[8];
    PrintGSLVector(p);
    scale_task_CoM=Init_task_CoM.norm()/p.norm();
    printf("scale_task_CoM %f \n",scale_task_CoM);

#elif file_walk



#endif //file_human



    PrintGSLVector(Init_task_CoM);
//    PrintGSLVector(Init_task_F2F);

    Init_light();

//clock_t begin,end;
//double elapsed;
    int quit;
    quit=0;
    i=0;

    double freq_,t2;

#ifdef WIN32
    static unsigned __int64 baseTime_;
    static unsigned __int64 pf_;
    QueryPerformanceFrequency( (LARGE_INTEGER *)&pf_ );
    QueryPerformanceCounter( (LARGE_INTEGER *)&baseTime_ );
    freq_ = 1.0 / (double)pf_;
#else
    static unsigned long long baseTime_;
    static unsigned long long pf_;
    struct timeval tv;
    gettimeofday (&tv, NULL);
    baseTime_ = ((unsigned long long) tv.tv_sec * 1000000) + tv.tv_usec;
    freq_ = 0.000001;
#endif

    t2=0.0;

    static double move_com_x=0.0;
    static double move_com_y=0.0;
    static double move_com_z=0.0;
//SDL_KeyRepeat(1);



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
#endif //save_data_quick
        if (quit==1)
        {
            break;
        }

        if (t2>60)
        {
            break;
        }


//SDL_PumpEvents() ;
        SDL_PollEvent(&event);
        switch (event.type)
        {
        case SDL_QUIT:
#if save_data_quick
            quit=1;
            continue;
#else //save_data_quick
            exit(0);
#endif //save_data_quick
            break;
#if Light
        case SDL_KEYDOWN:
            quit=1;
            break;
#endif
#if !Light
        case SDL_KEYDOWN:
            switch (event.key.keysym.sym)
            {
            case SDLK_LEFT:
                move_com_y-=0.0005;
                break;
            case SDLK_RIGHT:
                move_com_y+=0.0005;
                break;
            case SDLK_DOWN:
                move_com_z-=0.0005;
                break;
            case SDLK_UP:
                move_com_z+=0.0005;
                break;
            case SDLK_PAGEDOWN:
                move_com_x-=0.0005;
                break;
            case SDLK_PAGEUP:
                move_com_x+=0.0005;
                break;
//            case SDLK_LEFT:
//                angular_z-=M_PI*0.05;
//                break;
//            case SDLK_RIGHT:
//                angular_z+=M_PI*0.05;
//                break;
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
        case SDL_MOUSEWHEEL:
            {
                SDL_MouseButtonEvent dummy_btn_event;
                memset(&dummy_btn_event, 0, sizeof(dummy_btn_event));
                dummy_btn_event.type = SDL_MOUSEBUTTONDOWN;
                dummy_btn_event.button = (event.wheel.y > 0) ? SDL_BUTTON_WHEELUP : SDL_BUTTON_WHEELDOWN;
                OnMouseButton(&CamParam, dummy_btn_event);
            }
            break;
#endif //!Light
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

#endif //reseau

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
#endif //file_motor


#if Tasks
#if !Video
#ifdef WIN32
        static unsigned __int64 val_;
        QueryPerformanceCounter( (LARGE_INTEGER *)&val_ );
#else //WIN32
        static unsigned long long val_;
        struct timeval tv;
        gettimeofday (&tv, NULL);
        val_ = ((unsigned long long) tv.tv_sec * 1000000) + tv.tv_usec;
#endif //WIN32

#if !Light
        printf("fps: %f \n",1/(((double)(val_ - baseTime_) * freq_ )-t2));
#endif //!Light
        t2=(double)(val_ - baseTime_) * freq_;
#else //!Video
        t2+=0.04;
#endif //!Video
        //t=i*10;



        CalcJacobianModif(uLINK,J2,idx2);
//        CalcCoMJacobian(uLINK,&Status, JCoMR, Status.right_foot_ID);
        CalcCoMJacobian(uLINK,&Status, JCoML, Status.left_foot_ID);

        static double wO=3.0;//3.0//1.O
        static double amp=0.015;
        static double t_init=3.0;
        static double t_stand=5.0;
        static double t_stand_zmp=70.0;


//        static double wO=2.0;
//        static double amp=0.015;
//        static double t_init=0.5;


//        R.setIdentity();
//        p.setZero();
//        p(1) = -0.078;

        //p.setZero();

#if (file_human || file_walk)
        static double vitesse =
#if file_human
            0.5;
#else
            1.0;
#endif

        static double t_start =
#if file_human
            t_stand_zmp;
#else
            t_init;
#endif

        static double gain_1, gain_2;
        static int pos_in_file = 1;
        if((t2) >= t_start)
        {
            while(((t2 - t_start) * vitesse) > (0.005 * pos_in_file))
            {
                for(j=0; j<9; j++)
                {
                    opd_old2[j] = opd_old[j];
                }
                ReadTrajectory(opd_old, pos_in_file);
                pos_in_file++;
            }

            // interpolation of data
            gain_1 = -(((t2 - t_start) * vitesse) - (0.005 * pos_in_file)) / 0.005;
            gain_2 = 1 + (((t2 - t_start) * vitesse) - (0.005 * pos_in_file)) / 0.005;
            for(j=0; j<9; j++)
            {
                opd[j] = gain_1 * opd_old2[j] + gain_2 * opd_old[j];
            }
        }
#endif




#if file_hoap
        static double vitesse=1.0;
        static double gain_1,gain_2;
        static int pos_in_file=1;
        if((t2)>=t_stand_zmp)
        {
            while(((t2-t_stand_zmp)*vitesse)>(0.001*pos_in_file))
            {
                for(j=0; j<dof; j++)
                {
                    oqd_old2[j]=oqd_old[j];
                }
                Ext_q_hoap_trajectory(oqd_old, pos_in_file);
                pos_in_file++;
            }

            // interpolation of data
            gain_1=-(((t2-t_stand_zmp)*vitesse)-(0.001*pos_in_file))/0.001;
            gain_2=1+(((t2-t_stand_zmp)*vitesse)-(0.001*pos_in_file))/0.001;
            for(j=0; j<dof; j++)
            {
                oqd[j]=gain_1*oqd_old2[j]+gain_2*oqd_old[j];
            }

            for(j=0; j<21; j++)
            {
                sensor.q[j]=(short)(oqd[j]);
            }

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
        }
        //CalcCoM(uLINK,CoM);

#endif //file_hoap


#if file_human
        static double z_f=1.0;
        static double F2F_y=-0.078;
        static double F2F_min=-0.067;
        static double correction;
        if(t2 > t_init)
        {
            p(0) = (opd[3]-opd[6]);
            p(1) = (opd[4]-opd[7]);
            p(2) = (opd[5]-opd[8]*z_f);
            p *= scale_task_F2F;
        }
        else
        {
            p = Init_task_F2F_p;
            p(0) = (p(0)*(t_init-(t2))/t_init)+((opd[3]-opd[6])*scale_task_F2F*(t2)/t_init);
            p(1) = (p(1)*(t_init-(t2))/t_init)+((opd[4]-opd[7])*scale_task_F2F*(t2)/t_init);
            p(2) = (p(2)*(t_init-(t2))/t_init)+((opd[5]-opd[8])*z_f*scale_task_F2F*(t2)/t_init);
        }

        if(p(1)>F2F_min)
        {
            correction=p(1)-F2F_min;
            p(1) = F2F_min;
        }
        else
        {
            correction=0.0;
        }

        if((t2)>=t_init)
        {
            R.setIdentity();
        }
        else
        {
            R = Init_task_F2F_R;
            rot2omega(R,error);
            dt = error.norm();
            error *= 1/dt;
            dt=dt*(t_init-(t2))/t_init;
            Rodrigues(R,error,dt);
        }

#elif file_walk

        if((t2)>=t_init)
        {
            p(0) = opd[3];
            p(1) = opd[4];
            p(2) = opd[5];
        }
        else
        {
            p = Init_task_F2F_p;
            p(0) = (p(0)*(t_init-(t2))/t_init)+(opd[3]*(t2)/t_init);
            p(1) = (p(1)*(t_init-(t2))/t_init)+(opd[4]*(t2)/t_init);
            p(2) = (p(2)*(t_init-(t2))/t_init)+(opd[5]*(t2)/t_init);
        }

        error(0) = -opd[6];
        error(1) = -opd[7];
        error(2) = -opd[8];
        dt=1.0;
        Rodrigues(R,error,dt);

#else //file_human
        static double F2F_y=-0.078;
        if((t2)>=t_init)
        {
            p.setZero();
            p(1) = F2F_y;
        }
        else
        {
            p = Init_task_F2F_p;
            p(0) = p(0)*(t_init-(t2))/t_init;
            p(1) = (p(1)*(t_init-(t2))/t_init)+(F2F_y*(t2)/t_init);
            p(2) = p(2)*(t_init-(t2))/t_init;
        }

        if((t2)>=t_init)
        {
            R.setIdentity();
        }
        else
        {
            R = Init_task_F2F_R;
            rot2omega(R,error);
            dt = error.norm();
            error *= 1/dt;
            dt=dt*(t_init-(t2))/t_init;
            Rodrigues(R,error,dt);
        }
#endif //file_human


//
//        if((t2)>=t_init)
//        {
//            R.setIdentity();
//        }
//        else
//        {
//            R = Init_task_F2F_R;
//            rot2omega(R,error);
//            dt = error.norm();
//            error *= 1/dt;
//            dt=dt*(t_init-(t2))/t_init;
//            Rodrigues(R,error,dt);
//        }
//        //R = Init_task_F2F_R;



        CalcVWerrOri(uLINK, task2, p, R,idx2);
        //PrintGSLVector(task2);



#if file_human

        static double foot_h;
        if(t2>t_init)
        {
            double z_val = opd[2]-opd[8]-CoM_Z_offset;
            if(t2>t_stand_zmp)
            {
                z_val -= foot_h;
            }
            p(0) = opd[0]-opd[6];
            p(1) = opd[1]-opd[7];
            p(2) = z_val;
            p *= scale_task_F2F;
        }
        else
        {
            p = Init_task_CoM;
            p(0) = ((p(0)*(t_init-(t2))/t_init)+((opd[0]-opd[6])*scale_task_F2F*(t2)/t_init));
            p(1) = ((p(1)*(t_init-(t2))/t_init)+((opd[1]-opd[7])*scale_task_F2F*(t2)/t_init));
            p(2) = ((p(2)*(t_init-(t2))/t_init)+((opd[2]-opd[8]-CoM_Z_offset)*scale_task_F2F*(t2)/t_init));
        }


        //p(1) = p(1+correction/2);


#elif file_walk


        if(t2>t_init)
        {
            p(0) = opd[0];
            p(1) = opd[1];
            p(2) = opd[2];
        }
        else
        {
            p = Init_task_CoM;
            p(0) = ((p(0)*(t_init-(t2))/t_init)+(opd[0]*(t2)/t_init));
            p(1) = ((p(1)*(t_init-(t2))/t_init)+(opd[1]*(t2)/t_init));
            p(2) = ((p(2)*(t_init-(t2))/t_init)+(opd[2]*(t2)/t_init));
        }

#else //file_human
static float wi;
//wi=wO-(t2-t_stand_zmp)*1/25;
//wi=3.0*exp(-(t2-t_stand_zmp)/50);
//wi=1.5*exp(-(t2-t_stand_zmp)/100);
wi=1.1*exp(-(t2-t_stand_zmp)/170);

#if save_data_quick_temp

        buff_temp[(i-1)*temp_size+19]=wi;

#endif //save_data_quick_temp

        if(t2>t_stand_zmp)
        {
            p = Stand_task_CoM;
            p(0) = p(0) + amp*(cos((1/wi)*M_PI*(t2-t_stand_zmp))-1)+move_com_x;
            p(1) = p(1) + move_com_y;
            p(2) = p(2) + move_com_z;
//            p(0) = p(0) + move_com_x;
//            p(1) = p(1) + move_com_y;
//            p(2) = p(2) + amp*(cos((1/wO)*M_PI*(t2-t_stand_zmp))-1)+move_com_z;
        }
        else if((t2>t_init) && (t2<t_stand_zmp))
        {
            p = Stand_task_CoM;
            p(0) = p(0) + move_com_x;
            p(1) = p(1) + move_com_y;
            p(2) = p(2) + move_com_z;
        }
        else
        {
            p = Init_task_CoM;
//            p *= (t_init-(t*Dtime)/t_init);
//            p += Stand_task_CoM*(t*Dtime/t_init);
            p(0) = (p(0)*(t_init-(t2))/t_init)+(Stand_task_CoM(0)*(t2)/t_init);
            p(1) = (p(1)*(t_init-(t2))/t_init)+(Stand_task_CoM(1)*(t2)/t_init);
            p(2) = (p(2)*(t_init-(t2))/t_init)+(Stand_task_CoM(2)*(t2)/t_init);
            //it's ok
        }

#endif //file_human

        //taskCoMR = uLINK[Status.right_foot_ID].R * p;
        taskCoML = p;//-foot.p
        CalcCoM(uLINK,CoM);//-foot.p
        CoM -= uLINK[baseFoot].p;
        //PrintGSLVector(CoM);
        taskCoML -= CoM;
        //PrintGSLVector(taskCoMR);





#if zmp_filtering
        static Eigen::Vector3d zmp_left = Eigen::Vector3d::Zero();
        static Eigen::Vector3d zmp_right = Eigen::Vector3d::Zero();
        static Eigen::Vector3d zmp_moy = Eigen::Vector3d::Zero();
        static Eigen::Vector3d izmp = Eigen::Vector3d::Zero(); // integral of ZMP
        static Eigen::Vector3d izmps = Eigen::Vector3d::Zero();
        static Eigen::Vector3d zmpsp = Eigen::Vector3d::Zero(); //sphere projection
        static double zmp_x_magin;
        static double zmp_y_magin;
        static double hcom;


        static int init_task_zmp=1;
        if (init_task_zmp==1)
        {
            hcom=Stand_task_CoM(2);
            zmp_x_magin=0.32*2;
            zmp_y_magin=0.64*2;


            init_task_zmp=0;
        }




        if( (zmp_c.zmp_left.W+zmp_c.zmp_right.W)>15 && (zmp_c.zmp_left.W+zmp_c.zmp_right.W)<100)
        {
//            if (zmp_c.zmp_left.W<10) //protect against false value due to too low pressure
//            {
//                zmp_c.zmp_left.W=0;
//            }
//            if (zmp_c.zmp_right.W<10)
//            {
//                zmp_c.zmp_right.W=0;
//            }
            Hoap_filter_zmp(&zmp_c,&zmp_f);

            zmp_left(0) = zmp_f.zmp_left.x/1000;
            zmp_left(1) = zmp_f.zmp_left.y/1000;
            zmp_right(0) = zmp_f.zmp_right.x/1000;
            zmp_right(1) = zmp_f.zmp_right.y/1000;


#if !old_zmp


            static double alpha;
            alpha=zmp_f.zmp_right.W/(zmp_f.zmp_left.W+zmp_f.zmp_right.W);
            zmp_left *= alpha;
            zmp_right *= 1-alpha;

            zmp_moy = zmp_left;
            zmp_moy += zmp_right;

            //zmp_moy(0) = CoM(0);
            //zmp_moy(1) = CoM(1);



            //if(t2>t_stand)
            {
//                p(0) = 0;
//                p(1) = 0;
//                p(2) = 0;

                taskZMP = zmp_moy;
//                taskZMP -= p;

                //taskZMP *= -0.03;//-0.02
                //taskZMP *= -0.03*npd(taskZMP.norm(),0.75,0.01);//-0.02
                taskZMP *= -0.03*npd(taskZMP.norm(),0.75,0.01);//-0.02
                //taskZMP(2) = 0;

                izmps += zmp_moy;
                izmp = izmps;
                izmp *= -0.0005;//-0.0002
                taskZMP += izmp;

                dzmp = zmp_moy;
                dzmp -= zmp;
                zmp = zmp_moy;
                dzmp *= Te;
                //dzmp *= -0.002;//-0.0002
                //dzmp *= -0.002*npd(dzmp.norm(),1.25,0.01);//-0.0002
                dzmp *= -0.002*npd(dzmp.norm(),1.25,0.01);//-0.0002
                taskZMP += dzmp;



                if (taskZMP(0)<-zmp_x_magin)
                {
                    taskZMP(0) = -zmp_x_magin;
                }
                if (taskZMP(0)>zmp_x_magin)
                {
                    taskZMP(0) = zmp_x_magin;
                }
                if (taskZMP(1)<-zmp_y_magin)
                {
                    taskZMP(1) = -zmp_y_magin;
                }
                if (taskZMP(1)>zmp_y_magin)
                {
                    taskZMP(1) = zmp_y_magin;
                }
                taskZMP(2) = 0.0;
#if zmp_feedback

            if(t2>t_init)
            {
//                zmpsp(0) = 0.0*hcom*sin(taskZMP(0/hcom));
//                zmpsp(1) = hcom*sin(taskZMP(1/hcom));
//                zmpsp(2) = 0.0*hcom*((cos(taskZMP(0/hcom)+cos(taskZMP(1)/hcom))/2-1));

                //taskCoML += zmpsp;
                taskCoML += taskZMP;
            }
#endif

//limites sur taskzmp et sa derivee
#if save_data_quick_temp
//                buff_temp[(i-1)*temp_size]=taskCoML(0);
//                buff_temp[(i-1)*temp_size+1]=taskCoML(1);
//                buff_temp[(i-1)*temp_size+2]=taskCoML(2);
//                buff_temp[(i-1)*temp_size+3]=zmp_moy(0);
//                buff_temp[(i-1)*temp_size+4]=zmp_moy(1);
//                buff_temp[(i-1)*temp_size+5]=zmp_moy(2);
//                buff_temp[(i-1)*temp_size+6]=taskZMP(0);
//                buff_temp[(i-1)*temp_size+7]=taskZMP(1);
//                buff_temp[(i-1)*temp_size+8]=taskZMP(2);
//        buff_temp[(i-1)*temp_size+19]=wi;
        buff_temp[(i-1)*temp_size+20]=taskCoML(0);
        buff_temp[(i-1)*temp_size+21]=taskCoML(1);
        buff_temp[(i-1)*temp_size+22]=taskCoML(2);
        buff_temp[(i-1)*temp_size+23]=zmp_moy(0);
        buff_temp[(i-1)*temp_size+24]=zmp_moy(1);
        buff_temp[(i-1)*temp_size+25]=zmp_moy(2);
        buff_temp[(i-1)*temp_size+26]=taskZMP(0);
        buff_temp[(i-1)*temp_size+27]=taskZMP(1);
        buff_temp[(i-1)*temp_size+28]=taskZMP(2);
#endif //save_data_quick_temp
            }
#endif //!old_zmp

#if old_zmp
            zmp_left += uLINK[Status.left_foot_ID].p;
            zmp_right += uLINK[Status.right_foot_ID].p;

            zmp_left(2) = 0;
            zmp_right(2) = 0;

            zmp_left *= zmp_f.zmp_left.W;
            zmp_right *= zmp_f.zmp_right.W;

            zmp_moy = zmp_left;
            zmp_moy += zmp_right;

            zmp_moy *= 1/(zmp_f.zmp_left.W+zmp_f.zmp_right.W);

            zmp_left *= 1/zmp_f.zmp_left.W;
            zmp_right *= 1/zmp_f.zmp_right.W;


            if(t2>t_stand)
            {

                p(2) = 0;
                taskZMP = zmp_moy;
                taskZMP -= p;
                taskZMP *= 0.05;
                //taskZMP(2) = 0;

                dzmp = zmp_moy;
                dzmp -= zmp;
                zmp = zmp_moy;
                dzmp *= Te;
                dzmp *= 0.005;
                taskZMP += dzmp;


                if (taskZMP(0)<-zmp_x_magin)
                {
                    taskZMP(0) = -zmp_x_magin;
                }
                if (taskZMP(0)>zmp_x_magin)
                {
                    taskZMP(0) = zmp_x_magin;
                }
                if (taskZMP(1)<-zmp_y_magin)
                {
                    taskZMP(1) = -zmp_y_magin;
                }
                if (taskZMP(1)>zmp_y_magin)
                {
                    taskZMP(1) = zmp_y_magin;
                }


#if zmp_feedback
                taskCoML += taskZMP;
#endif

//limites sur taskzmp et sa derivee
//#if save_data_quick_temp
//                buff_temp[(i-1)*temp_size]=taskCoML(0);
//                buff_temp[(i-1)*temp_size+1]=taskCoML(1);
//                buff_temp[(i-1)*temp_size+2]=taskCoML(2);
//                buff_temp[(i-1)*temp_size+3]=zmp_moy(0);
//                buff_temp[(i-1)*temp_size+4]=zmp_moy(1);
//                buff_temp[(i-1)*temp_size+5]=zmp_moy(2);
//                buff_temp[(i-1)*temp_size+6]=taskZMP(0);
//                buff_temp[(i-1)*temp_size+7]=taskZMP(1);
//                buff_temp[(i-1)*temp_size+8]=taskZMP(2);
//#endif //save_data_quick_temp


            }
#endif //old_zmp

        }

#endif //zmp_filtering



#if save_data_quick_temp
        CalcCoM(uLINK,CoM);//-foot.p

        p.setZero();
        //p(1) = F2F_y;
        R.setIdentity();
        CalcVWerrOri(uLINK, task2s, p, R,idx2);



        buff_temp[(i-1)*temp_size]=CoM(0);
        buff_temp[(i-1)*temp_size+1]=CoM(1);
        buff_temp[(i-1)*temp_size+2]=CoM(2);
        buff_temp[(i-1)*temp_size+3]=uLINK[Status.right_foot_ID].p(0);
        buff_temp[(i-1)*temp_size+4]=uLINK[Status.right_foot_ID].p(1);
        buff_temp[(i-1)*temp_size+5]=uLINK[Status.right_foot_ID].p(2);
        buff_temp[(i-1)*temp_size+6]=uLINK[Status.left_foot_ID].p(0);
        buff_temp[(i-1)*temp_size+7]=uLINK[Status.left_foot_ID].p(1);
        buff_temp[(i-1)*temp_size+8]=uLINK[Status.left_foot_ID].p(2);
        buff_temp[(i-1)*temp_size+9]=t2;
        buff_temp[(i-1)*temp_size+10]=task2s(3);
        buff_temp[(i-1)*temp_size+11]=task2s(4);
        buff_temp[(i-1)*temp_size+12]=task2s(5);

        CoM -= uLINK[baseFoot].p;
        buff_temp[(i-1)*temp_size+13]=-task2s(0);
        buff_temp[(i-1)*temp_size+14]=-task2s(1);
        buff_temp[(i-1)*temp_size+15]=-task2s(2);
        buff_temp[(i-1)*temp_size+16]=CoM(0);
        buff_temp[(i-1)*temp_size+17]=CoM(1);
        buff_temp[(i-1)*temp_size+18]=CoM(2);

#endif //save_data_quick_temp





        // first task
        pinv(invJ,J2);
        dq = invJ * task2;

        // second task
        P1.setIdentity();
        pinv(invJ,J2);
        Ptmp = invJ * J2;
        P1 -= Ptmp;

        vec3 = taskCoML;
        vec3_2 = JCoML * dq;
        vec3 -= vec3_2;

        Jtilde = JCoML * P1;
        pinv(invJCoM,Jtilde);
        dqtmp = invJCoM * vec3;
        //dqtmp = P2 * dqtmp2;
        dq += dqtmp;


//debug = dq;
//
//        // third task
//        P2 = P1;
//        pinv(invJCoM,Jtilde);
//        Ptmp = invJCoM * Jtilde;
//        P2 -= Ptmp;
//
//        vec3 = taskCoMR;
//        vec3_2 = JCoMR * dq;
//        vec3 -= vec3_2;
//
//        Jtilde = JCoMR * P2;
//        pinv(invJCoM,Jtilde);
//        dqtmp = invJCoM * vec3;
//        //dqtmp = P2 * dqtmp2;
//        dq += dqtmp;
//
//debug -= dq;
//PrintGSLVector(debug);

#if oritrunk
    CalcJacobianModif(uLINK,J1,idx1);
    gsl_matrix_get_row_m(J1_l, J1, 3);
    gsl_matrix_get_part2_m(J1_3, J1, 3, 6,0, dof);
    rot2omega(uLINK[14].R,task3);
    task3_l(0) = task3(0);


       // third task
       //printf("tache ori x \n");
        P2 = P1;
        pinv(invJCoM,Jtilde);
        Ptmp = invJCoM * Jtilde;
        P2 -= Ptmp;

////PrintGSLVector(task3);

//        //task3 *= -1.;
//        vec3 = task3;
//        vec3_2 = J1_3 * dq;
//        vec3 -= vec3_2;
//
//        Jtilde = J1_3 * P2;
//        pinv(invJtilde,Jtilde);
//        dqtmp = invJtilde * vec3;
//        //dqtmp = P2 * dqtmp2;
//        dq += dqtmp;

        vec1 = task3_l;
        vec1_2 = J1_l * dq;
        vec1 -= vec1_2;

        Jtilde2 = J1_l * P2;
        pinv(invJtilde2,Jtilde2);
        dqtmp = invJtilde2 * vec1;
        dq += dqtmp;


#endif // oritrunk


//        for (i=0; i<(nDoF-6); i++)
//        {
//            qdev[i]=fmin(fabs(uLINK[i+2].qmin-uLINK[i+2].qmoy),fabs(uLINK[i+2].qmax-uLINK[i+2].qmoy))*2;
//        }

        for (j=0; j<(nDoF-6); j++)
        {

            //adphi(j) = -0.05*(2*(uLINK[j+2].q-uLINK[j+2].qmoy)/(qdev[j]*qdev[j]) - 0.1*std::pow(2*(uLINK[j+2].q-uLINK[j+2].qmoy), 3.0)/(qdev[j]*qdev[j])); //O.O5//gsl_pow_3
            adphi(j) = -0.001*(2*(uLINK[j+2].q-uLINK[j+2].qmoy)/(qdev[j]*qdev[j]) - 0.01*std::pow(2*(uLINK[j+2].q-uLINK[j+2].qmoy), 3.0)/(qdev[j]*qdev[j])); //O.O5//gsl_pow_3
        }

        // fourth task
#if oritrunk
        Ptilde = P2;
//        pinv(invJCoM,Jtilde);
//        Ptmp = invJCoM * Jtilde;
        pinv(invJtilde2,Jtilde2);
        Ptmp = invJtilde2 * Jtilde2;
#else
        Ptilde = P1;
        pinv(invJCoM,Jtilde);
        Ptmp = invJCoM * Jtilde;
#endif // oritrunk
        Ptilde -= Ptmp;


        adphi -= dq;

        pinv(Ptmp,Ptilde);
        //Ptmp = Ptilde;
        dqtmp = Ptmp * adphi;
        dq += dqtmp;


        if(t2<t_init)
        {
            dq *= t2/t_init;
        }


#if reseau
static const float dqlim=0.12;


        for(j=0; j<(dof); j++)
        {
            if(uLINK[map[j]].fixed==0)
            {
                if(dq(map[j]-2)<(-dqlim))
                {
                    dq(map[j]-2) = (-dqlim);
                    //printf("dmin %d\n",map[j]);
                }
                if(dq(map[j]-2)>dqlim)
                {
                    dq(map[j]-2) = dqlim;
                    //printf("dmax %d\n",map[j]);
                }
                //uLINK[map[j]].q =uLINK[map[j]].q+0.2*dq(map[j]-2);
//                com_err[j]=command[j];
//                command[j]= rad2deg*motor_rotation[j]*(uLINK[map[j]].q+0.9*dq(map[j]-2))*209;
//                com_err[j]-=command[j];
//                control.q[j]=(short)(command[j]);
//                //control.q[j]=(short)(command[j]-0.05*com_err[j]);

                control.q[j]=(short)(rad2deg*motor_rotation[j]*(uLINK[map[j]].q+0.1*dq(map[j]-2))*209);//0.5
                //printf("%d: %d %d\n",j+1,sensor.q[j],control.q[j]);

                if(control.q[j]<(motor_l_bound[j]+209))
                {
                    control.q[j]=motor_l_bound[j]+209;
                    //printf("min %d %d %d [%d %d ] \n",map[j],j,control.q[j],motor_l_bound[j],motor_u_bound[j]);
                }
                if(control.q[j]>(motor_u_bound[j]-209))
                {
                    control.q[j]=motor_u_bound[j]-209;
                    //printf("max %d %d %d [%d %d ] \n",map[j],j,control.q[j],motor_l_bound[j],motor_u_bound[j]);
                }
//printf("test %d %d %d [%d %d]\n",map[j],j,control.q[j],motor_l_bound[j],motor_u_bound[j]);
            }


        }

#if file_hoap

//            for(j=0; j<21; j++)
//            {
//                control.q[j]=sensor.q[j];
//            }

        for(j=0; j<21; j++)
        {
            control.q[j]=(short)(oqd[j]);
        }

#endif //file_hoap

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
//        static unsigned __int64 val;
//        QueryPerformanceCounter( (LARGE_INTEGER *)&val );
        fprintf(t_file,"%f \n",(double)(val - baseTime_) * freq_);

        for(j=0; j<dof; j++)
        {
            fprintf(dq_file,"%f ",rad2deg*motor_rotation[j]*(dq(map[j]-2))*209);
        }
        fprintf(dq_file,"\n");

#endif //save_data_long
#if save_data_quick
        buff_sensor[i-1]=sensor;
        //buff_control[i-1]=control;
        buff_zmp_c[i-1]=zmp_c;
        buff_zmp_f[i-1]=zmp_f;
//        static unsigned __int64 val_;
//        QueryPerformanceCounter( (LARGE_INTEGER *)&val_ );
//        buff_t[i-1]=(double)(val_ - baseTime_) * freq_;
        buff_t[i-1]=t2;
#endif //save_data_quick


//        for(j=0; j<(dof); j++)
//        {
//            uLINK[map[j]].q = deg2rad*control.q[j]*motor_rotation[j]/209;
//        }

#else //reseau
        for(j=0; j<(dof); j++)
        {
            if(uLINK[map[j]].fixed==0)
            {
                uLINK[map[j]].q = uLINK[map[j]].q + 0.1 * dq(map[j]-2);
            }
            buff_data.val[j]=rad2deg*motor_rotation[j]*uLINK[map[j]].q*209;
        }
#endif //reseau



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
        uLINK[baseFoot].p(0) = opd[6];
        uLINK[baseFoot].p(1) = opd[7];
        uLINK[baseFoot].p(2) = opd[8]*z_f+foot_h+foot_Z_offset;
        uLINK[baseFoot].p *= scale_task_F2F;
        uLINK[baseFoot].R.setIdentity();
        NodeForwardKinematics(uLINK,baseFoot,0);
#else //file_human
        if(uLINK[Status.right_foot_ID].p(2)<uLINK[Status.left_foot_ID].p(2))
        {
            uLINK[Status.right_foot_ID].p(2) = 0.04;
            NodeForwardKinematics(uLINK,Status.right_foot_ID,0);
        }
        else
        {
            uLINK[Status.left_foot_ID].p(2) = 0.04;
            NodeForwardKinematics(uLINK,Status.left_foot_ID,0);
        }
            uLINK[Status.right_foot_ID].R.setIdentity();
            NodeForwardKinematics(uLINK,Status.right_foot_ID,0);
            //uLINK[Status.left_foot_ID].R.setIdentity();
            //NodeForwardKinematics(uLINK,Status.left_foot_ID,0);



//        uLINK[baseFoot].p.setZero();
//        uLINK[baseFoot].p(2) = 0.04;
#endif //file_human


//        uLINK[baseFoot].R.setIdentity();
//        NodeForwardKinematics(uLINK,baseFoot,0);



#endif //reseau

        ForwardKinematics(uLINK,1);

#if !Light
        sprintf(titre,"Visualisation t= %3.3f", t2);
        SDL_SetWindowTitle(window, titre);


        DrawScene(uLINK,&Status, &CamParam);





#endif //!Light


    }

#if file_motor
    fclose(file);
#endif //file_motor

#if reseau
    hoapDisconnect(hoap);
#endif //reseau


#if save_data_long
    fclose(q_file);
    fclose(qd_file);
    fclose(t_file);
    fclose(dq_file);
#endif //save_data_long

#if save_data_quick

    FILE *sensor_file=fopen("./../../Simu_data/sensor.txt","w");
    //FILE *control_file=fopen("./../../Simu_data/control.txt","w");
    FILE *zmp_file=fopen("./../../Simu_data/zmp.txt","w");
    FILE *zmpf_file=fopen("./../../Simu_data/zmpf.txt","w");
    FILE *t_file=fopen("./../../Simu_data/t.txt","w");

#if save_data_quick_temp
    FILE *temp_file=fopen("./../../Simu_data/temp.txt","w");
#endif //save_data_quick_temp

    double  *temp;
    temp = (double *)calloc(((i-2)*21),sizeof(double));
    if (temp==NULL)
    {
        printf("error");
    }
    double  *temp2;
    temp2 = (double *)calloc(i-2,sizeof(double));
    if (temp2==NULL)
    {
        printf("error");
    }


    int k;
    for(k=0; k<(i-2); k++)
    {

        for(j=0; j<21; j++)
        {
            fprintf(sensor_file,"%d ",buff_sensor[k].q[j]);
            temp[k+j*(i-2)]=(double)buff_sensor[k].q[j];
        }
        fprintf(sensor_file,"\n");
        //temp[k]=buff_sensor[k].q[2];


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

//        for(j=0; j<21; j++)
//        {
//            fprintf(control_file,"%d ",buff_control[k].q[j]);
//        }
//        fprintf(control_file,"\n");

        fprintf(zmp_file,"%f ",buff_zmp_c[k].zmp_right.W);
        fprintf(zmp_file,"%f ",buff_zmp_c[k].zmp_right.x);
        fprintf(zmp_file,"%f ",buff_zmp_c[k].zmp_right.y);
        fprintf(zmp_file,"%f ",buff_zmp_c[k].zmp_left.W);
        fprintf(zmp_file,"%f ",buff_zmp_c[k].zmp_left.x);
        fprintf(zmp_file,"%f ",buff_zmp_c[k].zmp_left.y);
        fprintf(zmp_file,"\n");

        fprintf(zmpf_file,"%f ",buff_zmp_f[k].zmp_right.W);
        fprintf(zmpf_file,"%f ",buff_zmp_f[k].zmp_right.x);
        fprintf(zmpf_file,"%f ",buff_zmp_f[k].zmp_right.y);
        fprintf(zmpf_file,"%f ",buff_zmp_f[k].zmp_left.W);
        fprintf(zmpf_file,"%f ",buff_zmp_f[k].zmp_left.x);
        fprintf(zmpf_file,"%f ",buff_zmp_f[k].zmp_left.y);
        fprintf(zmpf_file,"\n");

        //temp[k]=buff_zmp_c[k].zmp_right.x;
        temp2[k]=buff_zmp_f[k].zmp_right.x;

        fprintf(t_file,"%f ",buff_t[k]);
        fprintf(t_file,"\n");

#if save_data_quick_temp
        for(j=0; j<temp_size; j++)
        {
            fprintf(temp_file,"%f ",buff_temp[k*temp_size+j]);
        }
        fprintf(temp_file,"\n");
#endif //save_data_quick_temp

    }
//    fwrite(&buff_sensor, (i-2)*sizeof(HoapSensor), 1, sensor_file);
//    fwrite(&buff_control, (i-2)*sizeof(HoapControl), 1, control_file);
//    fwrite(&buff_zmp_c, (i-2)*sizeof(zmp_calc), 1, zmp_file);

    fclose(sensor_file);
//    fclose(control_file);
    fclose(zmp_file);
    fclose(t_file);
#if save_data_quick_temp
    fclose(temp_file);
#endif //save_data_quick_temp

#if mathGL
    int dat,dat2,datt;
    int dat_q[21];
    HMGL gr = mgl_create_graph(1024, 768);//600,400);


    //dat = mgl_create_data_size(i-2,1,1);
    dat = mgl_create_data();
    mgl_data_set_double (dat, temp, i-2, 21, 1);
    dat2 = mgl_create_data();
    //mgl_data_set_double (dat2, temp2, i-2, 1, 1);

    //datt = mgl_create_data_size(i-2,1,1);
    datt = mgl_create_data();
    mgl_data_set_double (datt, buff_t, i-2, 1, 1);

//dat = mgl_create_data_size(i-2,21+21+4+4+6,1);
//mgl_data_set_double (dat, buff_sensor, i-2, 21+21+4+4+6, 1);
//mgl_data_transpose(dat,"yxz");

//mgl_data_transpose(dat, "yxz");
    dat2=mgl_data_subdata (dat, -1, 2, -1);
//dat2=mgl_data_subdata (dat, 2, -1, -1);




//mgl_set_xrange(gr, datt, 0);
//mgl_set_yrange(gr, dat, 0);
    mgl_set_range_dat(gr,'x',datt,1);
    //mgl_set_range_dat(gr,'y',dat2,1);

    for(j=0; j<21; j++)
    {
        dat_q[j] = mgl_create_data();
        dat_q[j] = mgl_data_subdata (dat, -1, j, -1);
        mgl_set_range_dat(gr,'y',dat_q[j],1);
    }

    mgl_adjust_ticks (gr, "xy");

    mgl_axis (gr,"xy","");
    mgl_label(gr,'x', "\\i t");
    mgl_label(gr,'y', "\\i q");



    mgl_box_str(gr, "", 1);
    mgl_plot_xy(gr,datt,dat,"","");
    //mgl_plot_xy(gr,datt,dat2,"","");
//mgl_plot_xy(gr,datt,dat,"","");
//    mgl_plot(gr,dat,"","");
//    mgl_plot(gr,dat2,"","");

//    //mgl_puts (gr, -1, 1, 0, "q1", 0, 3);


    mgl_write_png(gr,"./plot/q.png","");
    mgl_write_eps(gr,"./plot/q.eps","");
    mgl_delete_data (dat);
    mgl_delete_data (dat2);
    mgl_delete_data (datt);
    mgl_delete_graph(gr);

#endif //mathGL

    free(buff_sensor);
    free(buff_zmp_c);
    free(buff_t);
#endif //save_data_quick



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
            opd = (double *)calloc(9,sizeof(double));
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
//                idx1(j) = path1[j];
//            }

            //static int path2[14] = {7, 7, 6, 5, 4, 3, 2, 8, 9, 10, 11, 12, 13, 13};
            static int path2[14] = {13, 13, 12, 11, 10, 9, 8, 2, 3, 4, 5, 6, 7, 7};
            for(j=0; j<14; j++)
            {
                idx2(j) = path2[j];
            }
            Ext_op_trajectory(opd, 0);
        }



        uLINK[base].p(0) = opd[6];
        uLINK[base].p(1) = opd[7];
        uLINK[base].p(2) = opd[8]+0.034;

        NodeForwardKinematics(uLINK,base,0);



//        CalcJacobianModif( uLINK,J1,idx1);
        CalcJacobianModif( uLINK,J2,idx2);
        CalcCoMJacobian(uLINK,&Status, JCoM, base);


//        R.setIdentity();
//        p.setZero();
//        p(0) = 0.0155;
//        p(1) = 0.0798;
//        p(2) = 0.8434;
//        CalcVWerrOri(uLINK, task1, p, R,idx1);

        R.setIdentity();
        p.setZero();
        p(0) = (opd[3]-opd[6]);
        p(1) = (opd[4]-opd[7]);
        p(2) = (opd[5]-opd[8]);
        CalcVWerrOri(uLINK, task2, p, R,idx2);

        taskCoM(0) = opd[0];
        taskCoM(1) = opd[1];
        taskCoM(2) = opd[2]-0.1;
        CalcCoM(uLINK,CoM);
        taskCoM -= CoM;



//        P1.setIdentity();
//        pinv(invJ,J1);
//        Ptmp = invJ * J1;
//        P1 -= Ptmp;

        P2.setIdentity();
        pinv(invJ,J2);
        Ptmp = invJ * J2;
        P2 -= Ptmp;

        PCoM.setIdentity();
        pinv(invJCoM,JCoM);
        Ptmp = invJCoM * JCoM;
        PCoM -= Ptmp;



        pinv(invJ,J2);
        dq = invJ * task2;

        vec3 = taskCoM;
        vec3_2 = JCoM * dq;
        vec3 -= vec3_2;

        Jtilde = JCoM * P2;

        pinv(invJCoM,Jtilde);
        dqtmp2 = invJCoM * vec3;

        dqtmp = P2 * dqtmp2;
        dq += dqtmp;



        for (j=0; j<(nDoF-6); j++)
        {
            uLINK[j+2].q =uLINK[j+2].q+dq(j);//+g(n+6);
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
            trace(0) = opd[0+3*j];
            trace(1) = opd[1+3*j];
            trace(2) = opd[2+3*j];
            DrawMarker(trace);
        }
        DrawGround(2.0,0.0,-0.05,6.0,6.0,0.1);
        glFlush();
        SDL_GL_SwapWindow(window);
        SDL_Delay(5);
    }

//    return EXIT_SUCCESS; // Fermeture du programme
#endif

#endif



//double EndTime = 3;


//    char files[40];
//    int frame=0;
//    char pixel_data[3*640*480];



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
//      SDL_GL_SwapWindow(window);
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
        case SDL_MOUSEMOTION: //la souris est bougée, ça n'intéresse que la caméra
            OnMouseMotion(&CamParam,event.motion);
            break;
        case SDL_MOUSEBUTTONUP:
        case SDL_MOUSEBUTTONDOWN:
            OnMouseButton(&CamParam,event.button); //tous les événements boutons (up ou down) sont donnés à la caméra
            break;
        case SDL_MOUSEWHEEL:
            {
                SDL_MouseButtonEvent dummy_btn_event;
                memset(&dummy_btn_event, 0, sizeof(dummy_btn_event));
                dummy_btn_event.type = SDL_MOUSEBUTTONDOWN;
                dummy_btn_event.button = (event.wheel.y > 0) ? SDL_BUTTON_WHEELUP : SDL_BUTTON_WHEELDOWN;
                OnMouseButton(&CamParam, dummy_btn_event);
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
            SDL_SetWindowTitle(window, titre);

#if StaticCOM
            uLINK[1].p(2) = Lc+Lt+Lp-0.07;
            pos(0) = -0.10;
            pos(1) = 0.21;
            pos(2) = -0.95;
            InverseSherpaKinematics(q, pos);
            uLINK[2].q=q(0);
            uLINK[3].q=q(1);
            uLINK[4].q=q(2);
            uLINK[5].q=q(3);
            uLINK[6].q=q(4);
            uLINK[7].q=q(5);
            pos(0) = -0.10;
            pos(1) = 0.21;
            pos(2) = -0.95;
            InverseSherpaKinematics(q, pos);
            uLINK[8].q=q(0);
            uLINK[9].q=q(1);
            uLINK[10].q=q(2);
            uLINK[11].q=q(3);
            uLINK[12].q=q(4);
            uLINK[13].q=q(5);
            ForwardKinematics(uLINK,1);
#endif

            DrawScene(uLINK, &Status, &CamParam);

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
            //uLINK[1].p.setZero();
            //uLINK[1].p(2) = 1.2;
            //uLINK[1].vo.setZero();
            ForwardDynamics(uLINK,&Status,t);
            IntegrateEuler(uLINK,1);
            /// \todo Runge kuta
        }
#endif

    }

    cleanup_sdl(); // Arrêt de la SDL

    // gsl_vector_free removed
    // gsl_vector_free removed
    // gsl_vector_free removed
    // gsl_vector_free removed

    free(uLINK);

//system("pause");
    return EXIT_SUCCESS; // Fermeture du programme

#endif

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




