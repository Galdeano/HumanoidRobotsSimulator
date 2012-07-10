#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_math.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include "uLINK.h"
#include "InvDyn.h"
#include "CalcCoM.h"
#include "Cross.h"
#include "DrawMarker.h"
#include "DrawForceMarker.h"
#include "Stabilizator.h"
#include "Gravity.h"
#include "ExternalForces.h"
#include "ForwardDynamics.h"

#include "LoadRobot.h"
#include "uLINK_f.h"
#include "Ext_traj.h"
#include "StaticTrajectory_f.h"
#include "Scenarios.h"
#include "ForwardKinematics_f.h"
#include "CalcCoM_f.h"
#include "Gravity_f.h"
#include "Stabilizator_f.h"
#include "DrawMarkerf.h"
#include "CalcJacobianModif.h"
#include "CalcVWerrOri.h"
#include "pinv.h"
#include "PrintGSLMatrix.h"
#include "CalcCoMJacobian.h"
#include "CalcCoP.h"

#ifndef MCSpline
#include "Mat.h"
#else
#include "d:\Mb\StLib\Mat.h"
#endif

#include "Setup.h"


void ForwardDynamics(SuLINK uLINK[],State *Status,long t)
{

    static Struct_uLINK *uLINKc;
    static gsl_matrix * Inertia_Motor;
    static gsl_matrix *A;
    static gsl_vector *b;
    static gsl_vector *tmp;
    static gsl_vector *g;
    static gsl_vector *ef;
    static gsl_vector *stab;
    static gsl_vector *u;

    static gsl_vector * ddq;
    static gsl_permutation * p;
    static gsl_matrix * A2;


    int nDoF = Status->ddl;
    int n,i,j;

//    gsl_matrix * A = gsl_matrix_calloc (nDoF, nDoF);
//    gsl_vector * b = gsl_vector_calloc (nDoF);
//    gsl_vector * tmp = gsl_vector_calloc (nDoF);
//    gsl_vector * g = gsl_vector_calloc (nDoF);
//    gsl_vector * ef = gsl_vector_calloc (nDoF);
//    gsl_vector * stab = gsl_vector_calloc (nDoF);
//    gsl_vector * u = gsl_vector_calloc (nDoF);



    static Struct_State Statusc;
    static int init=1;
    if (init==1)
    {

        uLINKc = calloc((Status->ddl)+2-6,sizeof(Struct_uLINK));
        Inertia_Motor = gsl_matrix_calloc (nDoF, nDoF);
        A = gsl_matrix_calloc (nDoF, nDoF);
        b = gsl_vector_calloc (nDoF);
        tmp = gsl_vector_calloc (nDoF);
        g = gsl_vector_calloc (nDoF);
        ef = gsl_vector_calloc (nDoF);
        stab = gsl_vector_calloc (nDoF);
        u = gsl_vector_calloc (nDoF);

        ddq = gsl_vector_alloc (nDoF);
        p = gsl_permutation_alloc (nDoF);
        A2 = gsl_matrix_calloc (nDoF, nDoF);

        for(n=7; n<nDoF; n++)
        {
            gsl_matrix_set(Inertia_Motor,n,n,0.5);
        }
        LoadRobotParserXML_f(uLINKc,&Statusc,Status->RobotFile);
        init=0;
    }

    gsl_vector_set_zero(u);


    InvDyn(uLINK,Status,0,b);
//b = InvDyn(0);
//--------------------------------------------------------------------------------
    for(n=1; n<(nDoF+1); n++)
    {
        InvDyn(uLINK,Status,n,tmp);
        gsl_vector_sub (tmp,b);
        gsl_matrix_set_col(A, n-1, tmp);
    }


//    Gravity
//    ExternalForces

    gsl_vector * f = gsl_vector_calloc (3);
    gsl_vector * tau = gsl_vector_calloc (3);

    Gravity(uLINK,Status,1,f,tau);
    if (Status->desired_support!=0 || Suspendu)
    {
        for (n=0; n<3; n++)
        {
            gsl_vector_set (g, n,gsl_vector_get (f,n));
        }
        for (n=0; n<3; n++)
        {
            gsl_vector_set (g, n+3,gsl_vector_get (tau,n));
        }
        for (n=0; n<nDoF-6; n++)
        {
            gsl_vector_set (g, n+6,uLINK[n+2].ug);
        }
    }

    gsl_vector_set_zero(f);
    gsl_vector_set_zero(tau);
    ExternalForces(uLINK,Status,1,f,tau);
    if (Status->desired_support!=0 || Suspendu)
    {
        for (n=0; n<3; n++)
        {
            gsl_vector_set (ef, n,gsl_vector_get (f,n));
        }
        for (n=0; n<3; n++)
        {
            gsl_vector_set (ef, n+3,gsl_vector_get (tau,n));
        }
        for (n=0; n<nDoF-6; n++)
        {
            gsl_vector_set (ef, n+6,uLINK[n+2].uef);
        }
    }
    gsl_vector_free(f);
    gsl_vector_free(tau);

//for n=1:nDoF
//    A(:,n) = InvDyn(n) - b;
//end

//    FILE *fil=fopen("test1.txt","w");
//    fprintf(fil,"%d \n",Status->support);
//    fclose(fil);
//
//    printf(" A: \n");
//    for(n=0; n<18; n++)
//    {
//        printf(" %d",n+1);
//        for(i=0; i<18; i++)
//        {
//            printf("%8.4f ",gsl_matrix_get (A, n,i));
//        }
//        printf(" \n");
//    }
//    printf(" \n");

//% Prise en compte de l inertie des moteurs
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//for n=7:nDoF
//    j = n-6+1;
//    A(n,n) = A(n,n) + uLINK(j).Ir * uLINK(j).gr^2;
//end

// Prise en compte de l inertie des moteurs
//PrintGSLMatrix(A);
    gsl_matrix_add(A,Inertia_Motor);


//      if ((t%50)==0)
    if ((t%(int)(Te/Dtime))==0)
    {
        static int init_com=1;
        static float *uPD;
        static float *qd;
        static float *dqd;
        if (init_com==1)
        {
            uPD = calloc((Status->ddl)-6,sizeof(float));
            qd = calloc((Status->ddl)-6,sizeof(float));
            dqd = calloc((Status->ddl)-6,sizeof(float));
            init_com=0;
        }

        static float com[3];


        for(n=2; n<nDoF-6+2; n++)
        {
            //uLINKc[n].dq = (uLINK[n].q - uLINKc[n].q)/Te;
            uLINKc[n].dq = uLINK[n].dq;
            uLINKc[n].q = uLINK[n].q;
        }


        for(i=0; i<3; i++)
        {
            for(j=0; j<3; j++)
            {
                uLINKc[1].R[i*3+j]=gsl_matrix_get (uLINK[1].R, i,j);
            }
        }

//        for(i=0; i<3; i++)
//        {
//                uLINKc[1].p[i]=gsl_vector_get (uLINK[1].p, i);
//        }


        ForwardKinematics_f(uLINKc,1);

        static float FootR[3]= {0.0853,0,-0.11};
        MatMulf( Statusc.FootCenter_R, uLINKc[7].R, FootR, 3, 3, 1) ;
        MatAddf( Statusc.FootCenter_R, Statusc.FootCenter_R, uLINKc[7].p, 3, 1) ;

        static float FootL[3]= {0.0853,0,-0.11};
        MatMulf( Statusc.FootCenter_L, uLINKc[13].R, FootL, 3, 3, 1) ;
        MatAddf( Statusc.FootCenter_L, Statusc.FootCenter_L, uLINKc[13].p, 3, 1) ;

//        DrawMarkerf(Statusc.FootCenter_R);
//        DrawMarkerf(Statusc.FootCenter_L);

        CalcCoM_f(uLINKc,com);
        //DrawMarkerf(com);

#if Trajectories
        OneFoot_f(qd, t*Dtime, &Statusc.desired_support, &Statusc.distribution_y);
        OneFoot_f(dqd, t*Dtime-Dtime, &Statusc.desired_support, &Statusc.distribution_y);

        OneFoot_f(qd, t*Dtime, &Status->desired_support, &Status->distribution_y);
        OneFoot_f(dqd, t*Dtime-Dtime, &Status->desired_support, &Status->distribution_y);
#endif

#if Scenarios
        Scenario_desired_trajectory(qd, t*Dtime, &Statusc.desired_support, &Statusc.distribution_y);
        Scenario_desired_trajectory(dqd, t*Dtime-Dtime, &Statusc.desired_support, &Statusc.distribution_y);

        Scenario_desired_trajectory(qd, t*Dtime, &Status->desired_support, &Status->distribution_y);
        Scenario_desired_trajectory(dqd, t*Dtime-Dtime, &Status->desired_support, &Status->distribution_y);
#endif

#if Ext_traj
        for(n=0; n<nDoF-6; n++)
        {
            dqd[n]=qd[n];
        }
        Ext_q_trajectory(qd,0);

#endif




#if PD
#if Sherpa
        float kd=3;
        float kp=1000;
#endif
#if Generic
//        float kd=1;
//        float kp=300;
        float kd=100;
        float kp=3000;
#endif
#if Human
        float kd=1;
        float kp=500;
#endif

        for(n=0; n<nDoF-6; n++)
        {
            uPD[n]=kd*(((qd[n+1]-dqd[n+1])/Te)-uLINKc[n+2].dq)+kp*(qd[n+1]-uLINKc[n+2].q);
            //uPD[n]=kd*(((0)/Te)-uLINKc[n+2].dq)+kp*(0-uLINKc[n+2].q);
            //gsl_vector_set(u,n+6,uPD[n]);
        }



        static float *uG;
        static float *fG;
        static float *tG;
        static float *uStab;
        static int init_G=1;
        if (init_G==1)
        {
            uG = calloc((Status->ddl)-6,sizeof(float));
            fG = calloc((Status->ddl)-6,sizeof(float));
            tG = calloc((Status->ddl)-6,sizeof(float));
            uStab = calloc((Status->ddl)-6,sizeof(float));
            init_G=0;
        }

        Gravity_f( uLINKc, &Statusc, 1, fG, tG);
        for (n=0; n<nDoF-6; n++)
        {
            uG[n]=uLINKc[n+2].ug;
        }

        //Stabilizator(uLINK,Status,stab,Dtime,t*Dtime);
        //gsl_vector_add (u,stab);


        //Stabilizator_f( uLINKc, &Statusc, uStab);
#endif


#if Dynamic
#if Sherpa
        float kd=3;
        float kp=1000;
#endif
#if Generic
        float kd=0.001;
        float kp=1;
#endif

        //static float uG[NbLinks-2];

        static float uG[NbLinks-2], fG[NbLinks-2], tG[NbLinks-2];
        Gravity_f( uLINKc, &Statusc, 1, fG, tG);
        for (n=0; n<nDoF-6; n++)
        {
            uG[n]=uLINKc[n+2].ug;
        }
        for(n=0; n<nDoF-6; n++)
        {
            uPD[n]=kd*(((0)/Te)-uLINKc[n+2].dq)+kp*(0-uLINKc[n+2].q);
            //gsl_vector_set(u,n+6,uPD[n]);
            uG[n]=gsl_vector_get(g,n+6)+gsl_vector_get(b,n+6);//+gsl_vector_get(ef,n+6)
        }

#endif



        if (Visualisation)
        {
            FILE *q_file=fopen("./../Simu_data/q.txt","a");
            for(n=0; n<nDoF-6; n++)
            {
                fprintf(q_file,"%f ",uLINKc[n+2].q);
            }
            fprintf(q_file,"\n");
            fclose(q_file);

            FILE *qd_file=fopen("./../Simu_data/qd.txt","a");
            for(n=0; n<nDoF-6; n++)
            {
                fprintf(qd_file,"%f ",qd[n]);
            }
            fprintf(qd_file,"\n");
            fclose(qd_file);

            FILE *t_file=fopen("./../Simu_data/t.txt","a");
            fprintf(t_file,"%f \n",t*Dtime);
            fclose(t_file);


            FILE *pBody_file=fopen("./../Simu_data/pBody.txt","a");
            for(n=0; n<3; n++)
            {
                fprintf(pBody_file,"%f ",gsl_vector_get(uLINK[1].p,n));
            }
            fprintf(pBody_file,"\n");
            fclose(pBody_file);


#if PD
            FILE *uq_file=fopen("./../Simu_data/uq.txt","a");
            for(n=0; n<nDoF-6; n++)
            {
                fprintf(uq_file,"%f ",uPD[n]);
            }
            fprintf(uq_file,"\n");
            fclose(uq_file);

            FILE *ug1_file=fopen("./../Simu_data/ug.txt","a");
            for(n=0; n<nDoF-6; n++)
            {
                DoF-6;
                fprintf(ug1_file,"%f ",uLINKc[n+2].ug);
            }
            fprintf(ug1_file,"\n");
            fclose(ug1_file);

            FILE *ustab_file=fopen("./../Simu_data/ustab.txt","a");
            for(n=0; n<nDoF-6; n++)
            {
                fprintf(ustab_file,"%f ",uStab[n]);
            }
            fprintf(ustab_file,"\n");
            fclose(ustab_file);
#endif

        }



#if Task



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
        static gsl_matrix * Ptilde;
        static gsl_matrix * Ptmp;
        static gsl_matrix * invJ;
        static gsl_matrix * invJCoM;
        static gsl_vector * dq;
        static gsl_vector * dqtmp;
        static gsl_vector * dqtmp2;
        static gsl_vector * dq_old;
        static gsl_vector * ddq;

        static float *opd;
        static gsl_vector * trace;

        static gsl_vector * adphi;
        static float *qdev;
        static gsl_vector * CoP;
        static double f=0.0;
        static gsl_vector * zmp;
        static gsl_vector * dzmp;

        static gsl_vector * q_pd;


        static int init_task=1;
        if (init_task==1)
        {
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
            Ptilde = gsl_matrix_calloc (nDoF-6,nDoF-6);
            Ptmp = gsl_matrix_calloc (nDoF-6,nDoF-6);
            invJ = gsl_matrix_calloc (nDoF-6,6);
            invJCoM = gsl_matrix_calloc (nDoF-6,3);
            dq = gsl_vector_calloc(nDoF-6);
            dqtmp = gsl_vector_calloc(nDoF-6);
            dqtmp2 = gsl_vector_calloc(nDoF-6);
            dq_old = gsl_vector_calloc(nDoF-6);
            ddq = gsl_vector_calloc(nDoF-6);

            opd = calloc(9,sizeof(float));
            trace = gsl_vector_calloc (3);
            adphi = gsl_vector_calloc(nDoF-6);
            qdev = calloc(nDoF-6,sizeof(float));
            CoP = gsl_vector_calloc (3);
            zmp = gsl_vector_calloc (3);
            dzmp = gsl_vector_calloc (3);

            q_pd = gsl_vector_calloc(nDoF-6);

            init_task=0;

            static int path1[8] = {7, 7, 6, 5, 4, 3, 2, 1};
            //int path1[8] = {1, 2, 3, 4, 5, 6, 7, 7};
            for(i=0; i<8; i++)
            {
                gsl_vector_set(idx1,i,path1[i]);
            }

            //static int path2[14] = {7, 7, 6, 5, 4, 3, 2, 8, 9, 10, 11, 12, 13, 13};
            static int path2[14] = {13, 13, 12, 11, 10, 9, 8, 2, 3, 4, 5, 6, 7, 7};
            for(i=0; i<14; i++)
            {
                gsl_vector_set(idx2,i,path2[i]);
            }

            for (i=0; i<(nDoF-6); i++)
            {
                qdev[i]=fmin(fabs(uLINK[i+2].qmin-uLINK[i+2].qmoy),fabs(uLINK[i+2].qmax-uLINK[i+2].qmoy))*2;
            }


        }


        CalcJacobianModif( uLINK,J1,idx1);
        CalcJacobianModif( uLINK,J2,idx2);
        CalcCoMJacobian(uLINK, JCoM, Base);



        gsl_matrix_set_identity(R);
        gsl_vector_set_zero(p);
        gsl_vector_set (p, 0, 0.0155);
        gsl_vector_set (p, 1, 0.0798);
        gsl_vector_set (p, 2, 0.8434);
        CalcVWerrOri(uLINK, task1, p, R,idx1);

#if 1
        gsl_matrix_set_identity(R);
        gsl_vector_set_zero(p);
        gsl_vector_set (p, 1, -0.1595);
        //gsl_vector_set (p, 2, -0.01);
        CalcVWerrOri(uLINK, task2, p, R,idx2);

//        gsl_vector_set (taskCoM, 0, 0.048516-0.01*(cos(0.5*M_PI*t*Dtime)-1));
//        gsl_vector_set (taskCoM, 1,-0.079750);
//        gsl_vector_set (taskCoM, 2, 0.884101+0.05*0.02*t*Dtime*(cos(0.5*M_PI*t*Dtime)-1));
        //gsl_vector_set (p, 0, 0.048516-0.01*(cos(0.5*M_PI*t*Dtime)-1));
        gsl_vector_set (p, 0, 0.048516);
        gsl_vector_set (p, 1,-0.079750);
        //gsl_vector_set (p, 2, 0.884101+0.05*0.05*t*Dtime*(cos(0.5*M_PI*t*Dtime)-1));
        gsl_vector_set (p, 2, 0.884101+0.05*(cos(0.25*sqrt(0.1*t*Dtime)*M_PI*t*Dtime)-1));
//        pinv(R,uLINK[base].R);
//        gsl_blas_dgemv(CblasNoTrans, 1.0, R, p, 0.0, taskCoM);
        CalcCoM(uLINK,CoM);
        gsl_vector_memcpy(taskCoM,p);
        gsl_vector_sub(taskCoM,CoM);

        if (Visualisation)
        {
            FILE *p_file=fopen("./../Simu_data/p.txt","a");
            for(n=0; n<3; n++)
            {
                fprintf(p_file,"%f ",gsl_vector_get(p,n));
            }
            fprintf(p_file,"\n");
            fclose(p_file);

            FILE *CoM_file=fopen("./../Simu_data/CoM.txt","a");
            for(n=0; n<3; n++)
            {
                fprintf(CoM_file,"%f ",gsl_vector_get(CoM,n));
            }
            fprintf(CoM_file,"\n");
            fclose(CoM_file);
        }

        gsl_vector_set (p, 2, 0);
        gsl_vector_set_zero(CoP);
        f=CalcCoP(uLINK,CoP,1);

//        if (f!=0.0 && t*Dtime>0.5)
//        {
//            gsl_vector_scale (CoP, 1/f);
//            gsl_vector_set (CoP, 2, 0);
//        }
//
//        if (Visualisation)
//        {
//            FILE *CoP_file=fopen("./../Simu_data/CoP.txt","a");
//            for(n=0; n<3; n++)
//            {
//                fprintf(CoP_file,"%f ",gsl_vector_get(CoP,n));
//            }
//            fprintf(CoP_file,"\n");
//            fclose(CoP_file);
//        }

        if (f!=0.0 && t*Dtime>0.5)
        {
            gsl_vector_scale (CoP, 1/f);
            gsl_vector_set (CoP, 2, 0);
            gsl_vector_sub(p,CoP);

            gsl_vector_memcpy(dzmp,p);
            gsl_vector_sub(dzmp,zmp);
            gsl_vector_scale (dzmp, Te);
            gsl_vector_scale (dzmp, 0.05);//0.15
            gsl_vector_add(taskCoM,dzmp);

            gsl_vector_memcpy(zmp,p);
            gsl_vector_scale(p,0.02);//0.04
            gsl_vector_add(taskCoM,p);
        }

        if (Visualisation)
        {
            FILE *CoP_file=fopen("./../Simu_data/CoP.txt","a");
            for(n=0; n<3; n++)
            {
                fprintf(CoP_file,"%f ",gsl_vector_get(CoP,n));
            }
            fprintf(CoP_file,"\n");
            fclose(CoP_file);

            FILE *taskCoM_file=fopen("./../Simu_data/taskCoM.txt","a");
            for(n=0; n<3; n++)
            {
                fprintf(taskCoM_file,"%f ",gsl_vector_get(taskCoM,n));
            }
            fprintf(taskCoM_file,"\n");
            fclose(taskCoM_file);
        }
#endif

#if 0
        gsl_matrix_set_identity(R);
        gsl_vector_set_zero(p);
        gsl_vector_set (p, 1, -0.1595);
        if ((t*Dtime)<1.0)
        {
            gsl_vector_set (p, 2,0);
        }
        if (((t*Dtime)>=1.0) && ((t*Dtime)<2.0))
        {
            gsl_vector_set (p, 2,0+0.003*(cos(1*M_PI*(t*Dtime-1.0))-1));
        }
        if ((t*Dtime)>=2.0)
        {
            gsl_vector_set (p, 2,0-0.006*(cos(1*M_PI*(t*Dtime-2.0))));
        }
        //gsl_vector_set (p, 2, -0.01);
        CalcVWerrOri(uLINK, task2, p, R,idx2);


        gsl_vector_set (p, 0, 0.048516);
        if ((t*Dtime)<1.0)
        {
            gsl_vector_set (p, 1,-0.079750);
        }
        if (((t*Dtime)>=1.0) && ((t*Dtime)<2.0))
        {
            gsl_vector_set (p, 1,-0.079750+0.012*(cos(1*M_PI*(t*Dtime-1.0))-1));
        }
        if ((t*Dtime)>=2.0)
        {
            gsl_vector_set (p, 1,-0.079750-0.024*(cos(1*M_PI*(t*Dtime-2.0))));
        }

        //gsl_vector_set (p, 2, 0.884101+0.05*0.05*t*Dtime*(cos(0.5*M_PI*t*Dtime)-1));
        gsl_vector_set (p, 2, 0.884101);
        gsl_matrix_memcpy(R,uLINK[Base].R);
        //pinv(R,uLINK[Base].R);
        gsl_blas_dgemv(CblasNoTrans, 1.0, R, p, 0.0, taskCoM);
        CalcCoM(uLINK,CoM);
        //gsl_vector_memcpy(taskCoM,p);
        gsl_vector_sub(taskCoM,CoM);

//        gsl_vector_set (p, 2, 0);
//        gsl_vector_set_zero(CoP);
//        f=CalcCoP(uLINK,CoP,1);
//        if (f!=0.0 && t*Dtime>0.5)
//        {
//            gsl_vector_scale (CoP, 1/f);
//            gsl_vector_set (CoP, 2, 0);
//            gsl_vector_sub(p,CoP);
//
//            gsl_vector_memcpy(dzmp,p);
//            gsl_vector_sub(dzmp,zmp);
//            gsl_vector_scale (dzmp, Te);
//            gsl_vector_scale (dzmp, 0.15);
//            gsl_vector_add(taskCoM,dzmp);
//
//            gsl_vector_memcpy(zmp,p);
//            gsl_vector_scale(p,0.04);
//            gsl_vector_add(taskCoM,p);
//        }
#endif

#if 0
        gsl_matrix_set_identity(R);
        gsl_vector_set_zero(p);
        gsl_vector_set (p, 1, -0.1595);
        if ((t*Dtime)<1.0)
        {
            gsl_vector_set (p, 2,0);
        }
        if (((t*Dtime)>=1.0) && ((t*Dtime)<11.0))
        {
            gsl_vector_set (p, 2,0+0.002*(cos(0.1*M_PI*(t*Dtime-1.0))-1));
        }
        if ((t*Dtime)>=11.0)
        {
            gsl_vector_set (p, 2,0-0.004*(cos(0.1*M_PI*(t*Dtime-11.0))));
        }
        CalcVWerrOri(uLINK, task2, p, R,idx2);


        gsl_vector_set (p, 0, 0.048516);
        if ((t*Dtime)<1.0)
        {
            gsl_vector_set (p, 1,-0.079750);
        }
        if (((t*Dtime)>=1.0) && ((t*Dtime)<11.0))
        {
            gsl_vector_set (p, 1,-0.079750+0.038*(cos(0.1*M_PI*(t*Dtime-1.0))-1));
        }
        if ((t*Dtime)>=11.0)
        {
            gsl_vector_set (p, 1,-0.079750-0.076*(cos(0.1*M_PI*(t*Dtime-11.0))));
        }

        //gsl_vector_set (p, 2, 0.884101+0.05*0.05*t*Dtime*(cos(0.5*M_PI*t*Dtime)-1));
        gsl_vector_set (p, 2, 0.884101);
        gsl_matrix_memcpy(R,uLINK[Base].R);
        //pinv(R,uLINK[Base].R);
        gsl_blas_dgemv(CblasNoTrans, 1.0, R, p, 0.0, taskCoM);
        CalcCoM(uLINK,CoM);
        //gsl_vector_memcpy(taskCoM,p);
        gsl_vector_sub(taskCoM,CoM);

//        gsl_vector_set (p, 2, 0);
//        gsl_vector_set_zero(CoP);
//        f=CalcCoP(uLINK,CoP,1);
//        if (f!=0.0 && t*Dtime>0.5)
//        {
//            gsl_vector_scale (CoP, 1/f);
//            gsl_vector_set (CoP, 2, 0);
//            gsl_vector_sub(p,CoP);
//
//            gsl_vector_memcpy(dzmp,p);
//            gsl_vector_sub(dzmp,zmp);
//            gsl_vector_scale (dzmp, Te);
//            gsl_vector_scale (dzmp, 0.15);
//            gsl_vector_add(taskCoM,dzmp);
//
//            gsl_vector_memcpy(zmp,p);
//            gsl_vector_scale(p,0.04);
//            gsl_vector_add(taskCoM,p);
//        }
#endif

#if Ext_traj
        Ext_op_trajectory(opd, 0);

        gsl_matrix_set_identity(R);
        gsl_vector_set_zero(p);
        gsl_vector_set (p, 0, (opd[3]-opd[6]));
        gsl_vector_set (p, 1, (opd[4]-opd[7]));
        gsl_vector_set (p, 2, (opd[5]-opd[8]));
        //gsl_vector_scale(p,-1.0);
        CalcVWerrOri(uLINK, task2, p, R,idx2);

        gsl_vector_set (taskCoM, 0, opd[0]);
        gsl_vector_set (taskCoM, 1, opd[1]);
        gsl_vector_set (taskCoM, 2, opd[2]-0.1);
        CalcCoM(uLINK,CoM);
        gsl_vector_sub(taskCoM,CoM);



#endif

#if 0
        if ((t*Dtime)>=0.5)
        {
            Ext_op_trajectory_LIPM(opd, 0);
        }

        gsl_matrix_set_identity(R);
        gsl_vector_set_zero(p);
        if ((t*Dtime)<0.5)
        {
            gsl_vector_set (p, 1, 0.0);
            gsl_vector_set (p, 1, -0.1595);
            gsl_vector_set (p, 2, 0.0);
        }
        if (((t*Dtime)>=0.5))
        {
            gsl_vector_set (p, 0, (opd[3]-opd[6]));
            gsl_vector_set (p, 1, (opd[4]-opd[7]));
            gsl_vector_set (p, 2, (opd[5]-opd[8])+0.09*(opd[1]));
        }
        CalcVWerrOri(uLINK, task2, p, R,idx2);


        if ((t*Dtime)<0.5)
        {
            gsl_vector_set (p, 0, 0.048516);
            gsl_vector_set (p, 1,-0.079750);
            gsl_vector_set (p, 2, 0.884101);
        }
        if (((t*Dtime)>=0.5))
        {
            gsl_vector_set (p, 0, (opd[0]-opd[6]));
            gsl_vector_set (p, 1, (opd[1]-opd[7]));
            gsl_vector_set (p, 2, (opd[2]-opd[8]));
        }
gsl_matrix_memcpy(R,uLINK[Base].R);
        //pinv(R,uLINK[Base].R);
gsl_blas_dgemv(CblasNoTrans, 1.0, R, p, 0.0, taskCoM);
        CalcCoM(uLINK,CoM);
//gsl_vector_memcpy(taskCoM,p);
        gsl_vector_sub(taskCoM,CoM);

//        gsl_vector_set (p, 2, 0);
//        gsl_vector_set_zero(CoP);
//        f=CalcCoP(uLINK,CoP,1);
//        if (f!=0.0 && t*Dtime>0.5)
//        {
//            gsl_vector_scale (CoP, 1/f);
//            gsl_vector_set (CoP, 2, 0);
//            gsl_vector_sub(p,CoP);
//
//            gsl_vector_memcpy(dzmp,p);
//            gsl_vector_sub(dzmp,zmp);
//            gsl_vector_scale (dzmp, Te);
//            gsl_vector_scale (dzmp, 0.15);
//            gsl_vector_add(taskCoM,dzmp);
//
//            gsl_vector_memcpy(zmp,p);
//            gsl_vector_scale(p,0.04);
//            gsl_vector_add(taskCoM,p);
//        }

//PrintGSLVector(task2);
//PrintGSLVector(taskCoM);

#endif





        for (i=0; i<(nDoF-6); i++)
        {
            gsl_vector_set(adphi,i,-0.2*(2*(uLINK[i+2].q-uLINK[i+2].qmoy)/(qdev[i]*qdev[i])));
        }



        //PrintGSLVector(task1);
        //PrintGSLVector(task2);
        //PrintGSLVector(taskCoM);


        gsl_matrix_set_identity(P1);
        pinv(invJ,J1);
        gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, invJ, J1, 0.0, Ptmp);
        gsl_matrix_sub(P1,Ptmp);

        gsl_matrix_set_identity(P2);
        pinv(invJ,J2);
        gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, invJ, J2, 0.0, Ptmp);
        gsl_matrix_sub(P2,Ptmp);

        gsl_matrix_set_identity(PCoM);
        pinv(invJCoM,JCoM);
        gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, invJCoM, JCoM, 0.0, Ptmp);
        gsl_matrix_sub(PCoM,Ptmp);


        // first task
        pinv(invJ,J2);
        gsl_blas_dgemv(CblasNoTrans, 1.0, invJ, task2, 0.0, dq);

        // second task
        gsl_vector_memcpy(vec3,taskCoM);
        gsl_blas_dgemv(CblasNoTrans, 1.0, JCoM, dq, 0.0, vec3_2);
        gsl_vector_sub(vec3,vec3_2);
        gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, JCoM, P2, 0.0, Jtilde);
        pinv(invJCoM,Jtilde);
        gsl_blas_dgemv(CblasNoTrans, 1.0, invJCoM, vec3, 0.0, dqtmp2);
        gsl_blas_dgemv(CblasNoTrans, 1.0, P2, dqtmp2, 0.0, dqtmp);
        gsl_vector_add(dq,dqtmp);

        // third task
        gsl_matrix_set_identity(Ptilde);
        pinv(invJCoM,Jtilde);
        gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, invJCoM, Jtilde, 0.0, Ptmp);
        gsl_matrix_sub(Ptilde,Ptmp);

        gsl_vector_memcpy(dqtmp2,adphi);
        gsl_vector_sub(dqtmp2,dqtmp);

        gsl_blas_dgemv(CblasNoTrans, 1.0, Ptilde, dqtmp2, 0.0, dqtmp);
        gsl_blas_dgemv(CblasNoTrans, 1.0, P2, dqtmp, 0.0, dqtmp2);
        gsl_vector_add(dq,dqtmp2);



//PrintGSLVector(dq);

        gsl_vector_memcpy(ddq,dq);
        gsl_vector_sub(ddq,dq_old);
        gsl_vector_scale(ddq,1/Te);


        for (i=0; i<(nDoF-6); i++)
        {
            //uLINK[i+2].u_joint =1000*gsl_vector_get(dq,i)+10*gsl_vector_get(ddq,i);
            uLINK[i+2].u_joint =1000*gsl_vector_get(dq,i)+10*gsl_vector_get(ddq,i);
        }

        gsl_vector_memcpy(dq_old,dq);


        for (i=0; i<(nDoF-6); i++)
        {
            if (uLINK[i+2].u_joint>uLINK[i+2].umax)
            {
                uLINK[i+2].u_joint=uLINK[i+2].umax;
                printf("t= %lf max: %d\n",t*Dtime,i+2);
            }
            if (uLINK[i+2].u_joint<uLINK[i+2].umin)
            {
                uLINK[i+2].u_joint=uLINK[i+2].umin;
                printf("t= %lf min: %d\n",t*Dtime,i+2);
            }
        }



        if (Visualisation)
        {
            FILE *q_file=fopen("./../Simu_data/q.txt","a");
            for(n=0; n<nDoF-6; n++)
            {
                fprintf(q_file,"%f ",uLINK[n+2].q);
            }
            fprintf(q_file,"\n");
            fclose(q_file);

            FILE *u_file=fopen("./../Simu_data/u.txt","a");
            for(n=0; n<nDoF-6; n++)
            {
                fprintf(u_file,"%f ",uLINK[n+2].u_joint);
            }
            fprintf(u_file,"\n");
            fclose(u_file);

            FILE *t_file=fopen("./../Simu_data/t.txt","a");
            fprintf(t_file,"%f \n",t*Dtime);
            fclose(t_file);

        }




//
//        float kd=1;
//        float kp=500;
//
//        float *kqd;
//        kqd = calloc(nDoF-6,sizeof(float));
//        Ext_q_trajectory(kqd, 0);
//
//        for (i=0; i<(nDoF-6); i++)
//        {
//            uLINK[i+2].u_joint = kd*(0.0-uLINKc[n+2].dq)+kp*(kqd[i+1]-uLINKc[i+2].q);
//        }
//
//        free (kqd);



//        float kd=300;
//        float kp=100;
//        for(n=0; n<nDoF-6; n++)
//        {
//            uLINK[n+2].u_joint=kd*(gsl_vector_get(dq,n)*Te-uLINKc[n+2].dq)+kp*((uLINKc[n+2].q+gsl_vector_get(dq,n)*Te)-uLINKc[n+2].q);
//        }



//            printf("\n P1: \n");
//            PrintGSLMatrix(P1);
//
//            printf("\n P2: \n");
//            PrintGSLMatrix(P2);
//
//            printf("\n PCoM: \n");
//            PrintGSLMatrix(PCoM);

        //PrintGSLVector(taskCoM);
        //PrintGSLVector(dqtmp);
        //PrintGSLVector(dq);


#endif





        for(n=2; n<nDoF-6+2; n++)
        {
            //uLINK[n].u_joint = gsl_vector_get (u,n-2+6) ;

            if (Statusc.desired_support!=0 || Suspendu)
            {
#if PD
                uLINK[n].u_joint = uPD[n-2]+uG[n-2]+uStab[n-2];
#endif
#if Dynamic
                uLINK[n].u_joint = uPD[n-2]+uG[n-2];
#endif

            }
            else
            {
#if PD
                uLINK[n].u_joint = uPD[n-2]+uStab[n-2];
#endif
#if Dynamic
                uLINK[n].u_joint = uPD[n-2]+uG[n-2];
#endif
            }
#if Task


#endif

            for (i=0; i<(nDoF-6); i++)
            {
                if (uLINK[i+2].u_joint>uLINK[i+2].umax)
                {
                    uLINK[i+2].u_joint=uLINK[i+2].umax;
                    printf("t= %lf max: %d\n",t*Dtime,i+2);
                }
                if (uLINK[i+2].u_joint<uLINK[i+2].umin)
                {
                    uLINK[i+2].u_joint=uLINK[i+2].umin;
                    printf("t= %lf min: %d\n",t*Dtime,i+2);
                }
            }
        }

    }

        for(n=2; n<nDoF-6+2; n++)
        {
            gsl_vector_set (u,n-2+6,uLINK[n].u_joint);
        }

//PrintGSLVector(u);
//gsl_vector_set_zero(u);

    gsl_vector_sub (u,g);
    gsl_vector_sub (u,ef);
    gsl_vector_sub (u,b);
//    gsl_vector * ddq = gsl_vector_alloc (nDoF);
//    gsl_permutation * p = gsl_permutation_alloc (nDoF);
//    gsl_matrix * A2 = gsl_matrix_calloc (nDoF, nDoF);
    gsl_matrix_memcpy(A2,A);
    int s;
    gsl_linalg_LU_decomp (A2, p, &s);
    gsl_linalg_LU_solve (A2, p, u, ddq);
//    gsl_permutation_free (p);

//ddq = A \ (-b + u);//LU decomp

////gsl_vector * test = gsl_vector_alloc (18);
////gsl_blas_dgemv(CblasNoTrans, 1.0, A, ddq, 0.0, test);
////
////    printf(" test: ");
////    for(n=0; n<18; n++)
////    {
////            printf("%8.4f ",gsl_vector_get (test, n));
////    }
////    printf(" \n");
////    printf(" u   : ");
////    for(n=0; n<18; n++)
////    {
////            printf("%8.4f ",gsl_vector_get (u, n));
////    }
////    printf(" \n");
////
////gsl_vector_free(test);
////    printf(" b: ");
////    for(n=0; n<18; n++)
////    {
////            printf("%8.4f ",gsl_vector_get (b, n));
////    }
////    printf(" \n");
////
////    printf(" ddq: ");
////    for(n=0; n<18; n++)
////    {
////            printf("%8.4f \n",gsl_vector_get (ddq, n));
////    }
////    printf(" \n");




    for (n=0; n<3; n++)
    {
        gsl_vector_set (uLINK[1].dvo, n,gsl_vector_get (ddq,n));
    }
//    printf(" dvo: ");
//    for(n=0; n<3; n++)
//    {
//            printf("%8.4f ",gsl_vector_get (uLINK[1].dvo, n));
//    }
//    printf(" \n");
//uLINK(1).dvo = ddq(1:3);
    for (n=0; n<3; n++)
    {
        gsl_vector_set (uLINK[1].dw, n,gsl_vector_get (ddq,n+3));
    }
//uLINK(1).dw  = ddq(4:6);
    for (n=2; n<nDoF-6+2; n++)
    {
        uLINK[n].ddq = gsl_vector_get (ddq,n+6-2);
    }
//for j=2:length(uLINK)
//    uLINK(j).ddq = ddq(j+6-1);
//end

//    gsl_matrix_free(A);
//    gsl_vector_free(b);
//    gsl_vector_free(tmp);
//    gsl_vector_free(g);
//    gsl_vector_free(ef);
//    gsl_vector_free(stab);0.0001
//    gsl_vector_free(u);
//    gsl_matrix_free(A2);
//    gsl_vector_free(ddq);


}



