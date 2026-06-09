#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_math.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include "uLink.h"
#include "InvDyn.h"
#include "CalcCoM.h"
#include "Cross.h"
#include "DrawMarker.h"
#include "DrawForceMarker.h"
#include "Stabilizator.h"
#include "Gravity.h"
#include "ExternalForces.h"
#include "ForwardDynamics.h"

#include "LoadRobot_f.h"
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

#include "CalcJacobianModif_f.h"
#include "CalcVWerrOri_f.h"
#include "vec2tab.h"

#ifndef MCSpline
#include "Mat.h"
#else
#include "d:\Mb\StLib\Mat.h"
#endif

#include "Setup.h"


void ForwardDynamics(SuLINK uLINK[],State *Status,ForwardDynamicsWorkspace *ws,long t)
{

// [Workspace Refactored]     static Struct_uLINK *ws->uLINKc;
// [Workspace Refactored]     static gsl_matrix * ws->Inertia_Motor;
// [Workspace Refactored]     static gsl_matrix *ws->A;
// [Workspace Refactored]     static gsl_vector *ws->b;
// [Workspace Refactored]     static gsl_vector *ws->tmp;
// [Workspace Refactored]     static gsl_vector *ws->g;
// [Workspace Refactored]     static gsl_vector *ws->ef;
// [Workspace Refactored]     static gsl_vector *ws->stab;
// [Workspace Refactored]     static gsl_vector *ws->u;

// [Workspace Refactored]     static gsl_vector * ws->ddq;
// [Workspace Refactored]     static gsl_permutation * ws->p;
// [Workspace Refactored]     static gsl_matrix * ws->A2;


    int nDoF = Status->ddl;
    int n,i,j;

//    gsl_matrix * ws->A = gsl_matrix_calloc (nDoF, nDoF);
//    gsl_vector * ws->b = gsl_vector_calloc (nDoF);
//    gsl_vector * ws->tmp = gsl_vector_calloc (nDoF);
//    gsl_vector * ws->g = gsl_vector_calloc (nDoF);
//    gsl_vector * ws->ef = gsl_vector_calloc (nDoF);
//    gsl_vector * ws->stab = gsl_vector_calloc (nDoF);
//    gsl_vector * ws->u = gsl_vector_calloc (nDoF);



// [Workspace Refactored]     static Struct_State ws->Statusc;
// [Workspace Refactored]     static int ws->init=1;
    if (ws->init==1)
    {

// [Workspace Refactored]         ws->uLINKc = calloc((Status->ddl)+2-6,sizeof(Struct_uLINK));
// [Workspace Refactored]         ws->Inertia_Motor = gsl_matrix_calloc (nDoF, nDoF);
// [Workspace Refactored]         ws->A = gsl_matrix_calloc (nDoF, nDoF);
// [Workspace Refactored]         ws->b = gsl_vector_calloc (nDoF);
// [Workspace Refactored]         ws->tmp = gsl_vector_calloc (nDoF);
// [Workspace Refactored]         ws->g = gsl_vector_calloc (nDoF);
// [Workspace Refactored]         ws->ef = gsl_vector_calloc (nDoF);
// [Workspace Refactored]         ws->stab = gsl_vector_calloc (nDoF);
// [Workspace Refactored]         ws->u = gsl_vector_calloc (nDoF);

// [Workspace Refactored]         ws->ddq = gsl_vector_alloc (nDoF);
// [Workspace Refactored]         ws->p = gsl_permutation_alloc (nDoF);
// [Workspace Refactored]         ws->A2 = gsl_matrix_calloc (nDoF, nDoF);

        for(n=7; n<nDoF; n++)
        {
            gsl_matrix_set(ws->Inertia_Motor,n,n,0.5);
        }
        LoadRobotParserXML_f(ws->uLINKc,&ws->Statusc,Status->RobotFile);
        ws->init=0;
    }

    gsl_vector_set_zero(ws->u);


    InvDyn(uLINK,Status,0,ws->b);
//ws->b = InvDyn(0);
//--------------------------------------------------------------------------------
    for(n=1; n<(nDoF+1); n++)
    {
        InvDyn(uLINK,Status,n,ws->tmp);
        gsl_vector_sub (ws->tmp,ws->b);
        gsl_matrix_set_col(ws->A, n-1, ws->tmp);
    }


//    Gravity
//    ExternalForces

// [Workspace Refactored]     gsl_vector * ws->f = gsl_vector_calloc (3);
// [Workspace Refactored]     gsl_vector * ws->tau = gsl_vector_calloc (3);

    Gravity(uLINK,Status,1,ws->f,ws->tau);
//    if (Status->desired_support!=0 || Suspendu)
//    {
        for (n=0; n<3; n++)
        {
            gsl_vector_set (ws->g, n,gsl_vector_get (ws->f,n));
        }
        for (n=0; n<3; n++)
        {
            gsl_vector_set (ws->g, n+3,gsl_vector_get (ws->tau,n));
        }
        for (n=0; n<nDoF-6; n++)
        {
            gsl_vector_set (ws->g, n+6,uLINK[n+2].ug);
        }
//    }

    gsl_vector_set_zero(ws->f);
    gsl_vector_set_zero(ws->tau);
    ExternalForces(uLINK,Status,1,ws->f,ws->tau);
    if (Status->desired_support!=0 || Suspendu)
    {
        for (n=0; n<3; n++)
        {
            gsl_vector_set (ws->ef, n,gsl_vector_get (ws->f,n));
        }
        for (n=0; n<3; n++)
        {
            gsl_vector_set (ws->ef, n+3,gsl_vector_get (ws->tau,n));
        }
        for (n=0; n<nDoF-6; n++)
        {
            gsl_vector_set (ws->ef, n+6,uLINK[n+2].uef);
        }
    }
// [Workspace Refactored]     gsl_vector_free(ws->f);
// [Workspace Refactored]     gsl_vector_free(ws->tau);

//for n=1:nDoF
//    ws->A(:,n) = InvDyn(n) - ws->b;
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
//            printf("%8.4f ",gsl_matrix_get (ws->A, n,i));
//        }
//        printf(" \n");
//    }
//    printf(" \n");

//% Prise en compte de l inertie des moteurs
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//for n=7:nDoF
//    j = n-6+1;
//    ws->A(n,n) = ws->A(n,n) + uLINK(j).Ir * uLINK(j).gr^2;
//end

// Prise en compte de l inertie des moteurs
//PrintGSLMatrix(ws->A);
    gsl_matrix_add(ws->A,ws->Inertia_Motor);


//      if ((t%50)==0)
    if ((t%(int)(Te/Dtime))==0)
    {
// [Workspace Refactored]         static int ws->init_com=1;
// [Workspace Refactored]         static double *ws->uPD;
// [Workspace Refactored]         static double *ws->qd;
// [Workspace Refactored]         static double *ws->dqd;
        if (ws->init_com==1)
        {
// [Workspace Refactored]             ws->uPD = calloc((Status->ddl)-6,sizeof(double));
// [Workspace Refactored]             ws->qd = calloc((Status->ddl)-6,sizeof(double));
// [Workspace Refactored]             ws->dqd = calloc((Status->ddl)-6,sizeof(double));
            ws->init_com=0;
        }

// [Workspace Refactored]         static double ws->com[3];


        for(n=2; n<nDoF-6+2; n++)
        {
            //ws->uLINKc[n].dq = (uLINK[n].q - ws->uLINKc[n].q)/Te;
            ws->uLINKc[n].dq = uLINK[n].dq;
            ws->uLINKc[n].q = uLINK[n].q;
        }


        for(i=0; i<3; i++)
        {
            for(j=0; j<3; j++)
            {
                ws->uLINKc[1].R[i*3+j]=gsl_matrix_get (uLINK[1].R, i,j);
            }
        }

//        for(i=0; i<3; i++)
//        {
//                ws->uLINKc[1].p[i]=gsl_vector_get (uLINK[1].p, i);
//        }


        ForwardKinematics_f(ws->uLINKc,1);

// [Workspace Refactored]         static double ws->FootR[3]= {0.0853,0,-0.11};
        MatMulf( ws->Statusc.FootCenter_R, ws->uLINKc[7].R, ws->FootR, 3, 3, 1) ;
        MatAddf( ws->Statusc.FootCenter_R, ws->Statusc.FootCenter_R, ws->uLINKc[7].p, 3, 1) ;

// [Workspace Refactored]         static double ws->FootL[3]= {0.0853,0,-0.11};
        MatMulf( ws->Statusc.FootCenter_L, ws->uLINKc[13].R, ws->FootL, 3, 3, 1) ;
        MatAddf( ws->Statusc.FootCenter_L, ws->Statusc.FootCenter_L, ws->uLINKc[13].p, 3, 1) ;

//        DrawMarkerf(ws->Statusc.FootCenter_R);
//        DrawMarkerf(ws->Statusc.FootCenter_L);

        CalcCoM_f(ws->uLINKc,ws->com);
        //DrawMarkerf(ws->com);

#if Trajectories
        OneFoot_f(ws->qd, t*Dtime, &ws->Statusc.desired_support, &ws->Statusc.distribution_y);
        OneFoot_f(ws->dqd, t*Dtime-Dtime, &ws->Statusc.desired_support, &ws->Statusc.distribution_y);

        OneFoot_f(ws->qd, t*Dtime, &Status->desired_support, &Status->distribution_y);
        OneFoot_f(ws->dqd, t*Dtime-Dtime, &Status->desired_support, &Status->distribution_y);
#endif

#if Scenarios
        Scenario_desired_trajectory(ws->qd, t*Dtime, &ws->Statusc.desired_support, &ws->Statusc.distribution_y);
        Scenario_desired_trajectory(ws->dqd, t*Dtime-Dtime, &ws->Statusc.desired_support, &ws->Statusc.distribution_y);

        Scenario_desired_trajectory(ws->qd, t*Dtime, &Status->desired_support, &Status->distribution_y);
        Scenario_desired_trajectory(ws->dqd, t*Dtime-Dtime, &Status->desired_support, &Status->distribution_y);
#endif

#if Ext_traj
        for(n=0; n<nDoF-6; n++)
        {
            ws->dqd[n]=ws->qd[n];
        }
        Ext_q_trajectory(ws->qd,0);

#endif




#if PD
#if Sherpa
        double kd=3;
        double kp=1000;
#endif
#if Generic
//        double kd=1;
//        double kp=300;
        double kd=100;
        double kp=3000;
#endif
#if Human
        double kd=1;
        double kp=500;
#endif

        for(n=0; n<nDoF-6; n++)
        {
            ws->uPD[n]=kd*(((ws->qd[n+1]-ws->dqd[n+1])/Te)-ws->uLINKc[n+2].dq)+kp*(ws->qd[n+1]-ws->uLINKc[n+2].q);
            //ws->uPD[n]=kd*(((0)/Te)-ws->uLINKc[n+2].dq)+kp*(0-ws->uLINKc[n+2].q);
            //gsl_vector_set(ws->u,n+6,ws->uPD[n]);
        }



// [Workspace Refactored]         static double *ws->uG;
// [Workspace Refactored]         static double *ws->fG;
// [Workspace Refactored]         static double *ws->tG;
// [Workspace Refactored]         static double *ws->uStab;
// [Workspace Refactored]         static int ws->init_G=1;
        if (ws->init_G==1)
        {
// [Workspace Refactored]             ws->uG = calloc((Status->ddl)-6,sizeof(double));
// [Workspace Refactored]             ws->fG = calloc((Status->ddl)-6,sizeof(double));
// [Workspace Refactored]             ws->tG = calloc((Status->ddl)-6,sizeof(double));
// [Workspace Refactored]             ws->uStab = calloc((Status->ddl)-6,sizeof(double));
            ws->init_G=0;
        }

        Gravity_f( ws->uLINKc, &ws->Statusc, 1, ws->fG, ws->tG);
        for (n=0; n<nDoF-6; n++)
        {
            ws->uG[n]=ws->uLINKc[n+2].ug;
        }

        //Stabilizator(uLINK,Status,ws->stab,Dtime,t*Dtime);
        //gsl_vector_add (ws->u,ws->stab);


        //Stabilizator_f( ws->uLINKc, &ws->Statusc, ws->uStab);
#endif


#if Dynamic
#if Sherpa
        double kd=3;
        double kp=1000;
#endif
#if Generic
        double kd=0.001;
        double kp=1;
#endif

        //static double ws->uG[NbLinks-2];

// [Workspace Refactored]         static double ws->uG[NbLinks-2], ws->fG[NbLinks-2], ws->tG[NbLinks-2];
        Gravity_f( ws->uLINKc, &ws->Statusc, 1, ws->fG, ws->tG);
        for (n=0; n<nDoF-6; n++)
        {
            ws->uG[n]=ws->uLINKc[n+2].ug;
        }
        for(n=0; n<nDoF-6; n++)
        {
            ws->uPD[n]=kd*(((0)/Te)-ws->uLINKc[n+2].dq)+kp*(0-ws->uLINKc[n+2].q);
            //gsl_vector_set(ws->u,n+6,ws->uPD[n]);
            ws->uG[n]=gsl_vector_get(ws->g,n+6)+gsl_vector_get(ws->b,n+6);//+gsl_vector_get(ws->ef,n+6)
        }

#endif



        if (Visualisation)
        {
            FILE *q_file=fopen("./../Simu_data/q.txt","a");
            for(n=0; n<nDoF-6; n++)
            {
                fprintf(q_file,"%f ",ws->uLINKc[n+2].q);
            }
            fprintf(q_file,"\n");
            fclose(q_file);

            FILE *qd_file=fopen("./../Simu_data/qd.txt","a");
            for(n=0; n<nDoF-6; n++)
            {
                fprintf(qd_file,"%f ",ws->qd[n]);
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
                fprintf(uq_file,"%f ",ws->uPD[n]);
            }
            fprintf(uq_file,"\n");
            fclose(uq_file);

            FILE *ug1_file=fopen("./../Simu_data/ug.txt","a");
            for(n=0; n<nDoF-6; n++)
            {
                DoF-6;
                fprintf(ug1_file,"%f ",ws->uLINKc[n+2].ug);
            }
            fprintf(ug1_file,"\n");
            fclose(ug1_file);

            FILE *ustab_file=fopen("./../Simu_data/ustab.txt","a");
            for(n=0; n<nDoF-6; n++)
            {
                fprintf(ustab_file,"%f ",ws->uStab[n]);
            }
            fprintf(ustab_file,"\n");
            fclose(ustab_file);
#endif

        }



#if Task



// [Workspace Refactored]         static gsl_vector * ws->idx1;
// [Workspace Refactored]         static gsl_vector * ws->idx2;
// [Workspace Refactored]         static gsl_matrix * ws->J1;
// [Workspace Refactored]         static gsl_matrix * ws->J2;
// [Workspace Refactored]         static gsl_matrix * ws->JCoMR;
// [Workspace Refactored]         static gsl_matrix * ws->JCoML;
// [Workspace Refactored]         static gsl_matrix * ws->Jtilde;
// [Workspace Refactored]         static gsl_vector * ws->task1;
// [Workspace Refactored]         static gsl_vector * ws->task2;
// [Workspace Refactored]         static gsl_vector * ws->taskCoMR;
// [Workspace Refactored]         static gsl_vector * ws->taskCoML;
// [Workspace Refactored]         static gsl_vector * ws->CoM;
// [Workspace Refactored]         static gsl_vector * ws->vec3;
// [Workspace Refactored]         static gsl_vector * ws->vec3_2;
// [Workspace Refactored]         static gsl_vector * ws->p_task;
// [Workspace Refactored]         static gsl_matrix * ws->R;
// [Workspace Refactored]         static gsl_matrix * ws->P1;
// [Workspace Refactored]         static gsl_matrix * ws->P2;
// [Workspace Refactored]         static gsl_matrix * ws->PCoMR;
// [Workspace Refactored]         static gsl_matrix * ws->PCoML;
// [Workspace Refactored]         static gsl_matrix * ws->Ptilde;
// [Workspace Refactored]         static gsl_matrix * ws->Ptmp;
// [Workspace Refactored]         static gsl_matrix * ws->invJ;
// [Workspace Refactored]         static gsl_matrix * ws->invJCoM;
// [Workspace Refactored]         static gsl_vector * ws->dq;
// [Workspace Refactored]         static gsl_vector * ws->dqtmp;
// [Workspace Refactored]         static gsl_vector * ws->dqtmp2;
// [Workspace Refactored]         static gsl_vector * ws->dq_old;
// [Workspace Refactored]         static gsl_vector * ws->ddq_task;

// [Workspace Refactored]         static double *ws->opd;
// [Workspace Refactored]         static gsl_vector * ws->trace;

// [Workspace Refactored]         static gsl_vector * ws->adphi;
// [Workspace Refactored]         static double *ws->qdev;
// [Workspace Refactored]         static gsl_vector * ws->CoP;
 //       static double ws->f=0.0;
// [Workspace Refactored]         static gsl_vector * ws->zmp;
// [Workspace Refactored]         static gsl_vector * ws->dzmp;

// [Workspace Refactored]         static gsl_vector * ws->q_pd;

        static int path1[8] = {7, 7, 6, 5, 4, 3, 2, 1};
        static int path2[14] = {13, 13, 12, 11, 10, 9, 8, 2, 3, 4, 5, 6, 7, 7};

// [Workspace Refactored]         static double *ws->Jf;
// [Workspace Refactored]         static double *ws->invf;
// [Workspace Refactored]         static double *ws->pf;
// [Workspace Refactored]         static double *ws->Rf;
// [Workspace Refactored]         static double *ws->taskf;
// [Workspace Refactored]         static double *ws->dqf;

// [Workspace Refactored]         static int ws->init_task=1;
        if (ws->init_task==1)
        {
// [Workspace Refactored]             ws->idx1 = gsl_vector_calloc (8);
// [Workspace Refactored]             ws->idx2 = gsl_vector_calloc (14);
// [Workspace Refactored]             ws->J1 = gsl_matrix_calloc (6,nDoF-6);
// [Workspace Refactored]             ws->J2 = gsl_matrix_calloc (6,nDoF-6);
// [Workspace Refactored]             ws->JCoMR = gsl_matrix_calloc (3,nDoF-6);
// [Workspace Refactored]             ws->JCoML = gsl_matrix_calloc (3,nDoF-6);
// [Workspace Refactored]             ws->Jtilde = gsl_matrix_calloc (3,nDoF-6);
// [Workspace Refactored]             ws->task1 = gsl_vector_calloc (6);
// [Workspace Refactored]             ws->task2 = gsl_vector_calloc (6);
// [Workspace Refactored]             ws->taskCoMR = gsl_vector_calloc (3);
// [Workspace Refactored]             ws->taskCoML = gsl_vector_calloc (3);
// [Workspace Refactored]             ws->CoM = gsl_vector_calloc (3);
// [Workspace Refactored]             ws->vec3 = gsl_vector_calloc (3);
// [Workspace Refactored]             ws->vec3_2 = gsl_vector_calloc (3);
// [Workspace Refactored]             ws->p_task = gsl_vector_calloc (3);
// [Workspace Refactored]             ws->R = gsl_matrix_calloc (3,3);
// [Workspace Refactored]             ws->P1 = gsl_matrix_calloc (nDoF-6,nDoF-6);
// [Workspace Refactored]             ws->P2 = gsl_matrix_calloc (nDoF-6,nDoF-6);
// [Workspace Refactored]             ws->PCoMR = gsl_matrix_calloc (nDoF-6,nDoF-6);
// [Workspace Refactored]             ws->PCoML = gsl_matrix_calloc (nDoF-6,nDoF-6);
// [Workspace Refactored]             ws->Ptilde = gsl_matrix_calloc (nDoF-6,nDoF-6);
// [Workspace Refactored]             ws->Ptmp = gsl_matrix_calloc (nDoF-6,nDoF-6);
// [Workspace Refactored]             ws->invJ = gsl_matrix_calloc (nDoF-6,6);
// [Workspace Refactored]             ws->invJCoM = gsl_matrix_calloc (nDoF-6,3);
// [Workspace Refactored]             ws->dq = gsl_vector_calloc(nDoF-6);
// [Workspace Refactored]             ws->dqtmp = gsl_vector_calloc(nDoF-6);
// [Workspace Refactored]             ws->dqtmp2 = gsl_vector_calloc(nDoF-6);
// [Workspace Refactored]             ws->dq_old = gsl_vector_calloc(nDoF-6);
// [Workspace Refactored]             ws->ddq_task = gsl_vector_calloc(nDoF-6);

// [Workspace Refactored]             ws->opd = calloc(9,sizeof(double));
// [Workspace Refactored]             ws->trace = gsl_vector_calloc (3);
// [Workspace Refactored]             ws->adphi = gsl_vector_calloc(nDoF-6);
// [Workspace Refactored]             ws->qdev = calloc(nDoF-6,sizeof(double));
// [Workspace Refactored]             ws->CoP = gsl_vector_calloc (3);
// [Workspace Refactored]             ws->zmp = gsl_vector_calloc (3);
// [Workspace Refactored]             ws->dzmp = gsl_vector_calloc (3);

// [Workspace Refactored]             ws->q_pd = gsl_vector_calloc(nDoF-6);

            ws->init_task=0;

            //static int path1[8] = {7, 7, 6, 5, 4, 3, 2, 1};
            //int path1[8] = {1, 2, 3, 4, 5, 6, 7, 7};
            for(i=0; i<8; i++)
            {
                gsl_vector_set(ws->idx1,i,path1[i]);
            }

            //static int path2[14] = {7, 7, 6, 5, 4, 3, 2, 8, 9, 10, 11, 12, 13, 13};

            for(i=0; i<14; i++)
            {
                gsl_vector_set(ws->idx2,i,path2[i]);
            }

            for (i=0; i<(nDoF-6); i++)
            {
                ws->qdev[i]=fmin(fabs(uLINK[i+2].qmin-uLINK[i+2].qmoy),fabs(uLINK[i+2].qmax-uLINK[i+2].qmoy))*2;
            }


// [Workspace Refactored]             ws->Jf = calloc(6*(nDoF-6),sizeof(double));
// [Workspace Refactored]             ws->pf = calloc(3,sizeof(double));
// [Workspace Refactored]             ws->Rf = calloc(9,sizeof(double));
// [Workspace Refactored]             ws->taskf = calloc(6,sizeof(double));
// [Workspace Refactored]             ws->invf = calloc((nDoF-6)*6,sizeof(double));
// [Workspace Refactored]             ws->dqf = calloc((nDoF-6),sizeof(double));
        }



//        CalcJacobianModif_f(ws->uLINKc,ws->Jf,path2,14,nDoF-6);
        CalcJacobianModif( uLINK,ws->J1,ws->idx1);
        CalcJacobianModif( uLINK,ws->J2,ws->idx2);
        CalcCoMJacobian(uLINK,Status, ws->JCoMR, Status->right_foot_ID);
        CalcCoMJacobian(uLINK,Status, ws->JCoML, Status->left_foot_ID);

//MatPrintf( "Jf", "%4.6f " , ws->Jf , 6, (nDoF-6)) ;
//printf(" \n \n");
//PrintGSLMatrix(ws->J2);


        gsl_matrix_set_identity(ws->R);
        gsl_vector_set_zero(ws->p_task);
        gsl_vector_set (ws->p_task, 0, 0.0155);
        gsl_vector_set (ws->p_task, 1, 0.0798);
        gsl_vector_set (ws->p_task, 2, 0.8434);
        CalcVWerrOri(uLINK, ws->task1, ws->p_task, ws->R,ws->idx1);


#if 0
        gsl_matrix_set_identity(ws->R);
        gsl_vector_set_zero(ws->p_task);
        gsl_vector_set (ws->p_task, 1, -0.1595);
        //gsl_vector_set (ws->p_task, 2, -0.01);
        CalcVWerrOri(uLINK, ws->task2, ws->p_task, ws->R,ws->idx2);

//        gsl_vector_set (taskCoM, 0, 0.048516-0.01*(cos(0.5*M_PI*t*Dtime)-1));
//        gsl_vector_set (taskCoM, 1,-0.079750);
//        gsl_vector_set (taskCoM, 2, 0.http://fr.farnell.com/raspberry-pi-accessories884101+0.05*0.02*t*Dtime*(cos(0.5*M_PI*t*Dtime)-1));
        //gsl_vector_set (ws->p_task, 0, 0.048516-0.01*(cos(0.5*M_PI*t*Dtime)-1));
        gsl_vector_set (ws->p_task, 0, 0.048516);
        gsl_vector_set (ws->p_task, 1,-0.079750);
        //gsl_vector_set (ws->p_task, 2, 0.884101+0.05*0.05*t*Dtime*(cos(0.5*M_PI*t*Dtime)-1));
        gsl_vector_set (ws->p_task, 2, 0.884101+0.05*(cos(0.25*sqrt(0.1*t*Dtime)*M_PI*t*Dtime)-1));
//        pinv(ws->R,uLINK[base].R);
//        gsl_blas_dgemv(CblasNoTrans, 1.0, ws->R, ws->p_task, 0.0, taskCoM);
        CalcCoM(uLINK,ws->CoM);
        gsl_vector_memcpy(ws->taskCoML,ws->p_task);
        gsl_vector_sub(ws->taskCoML,ws->CoM);

        if (Visualisation)
        {
            FILE *p_file=fopen("./../Simu_data/p_task.txt","a");
            for(n=0; n<3; n++)
            {
                fprintf(p_file,"%f ",gsl_vector_get(ws->p_task,n));
            }
            fprintf(p_file,"\n");
            fclose(p_file);

            FILE *CoM_file=fopen("./../Simu_data/CoM.txt","a");
            for(n=0; n<3; n++)
            {
                fprintf(CoM_file,"%f ",gsl_vector_get(ws->CoM,n));
            }
            fprintf(CoM_file,"\n");
            fclose(CoM_file);
        }

        gsl_vector_set (ws->p_task, 2, 0);
        gsl_vector_set_zero(ws->CoP);
        ws->f=CalcCoP(uLINK,ws->CoP,1);

//        if (ws->f!=0.0 && t*Dtime>0.5)
//        {
//            gsl_vector_scale (ws->CoP, 1/ws->f);
//            gsl_vector_set (ws->CoP, 2, 0);
//        }
//
//        if (Visualisation)
//        {
//            FILE *CoP_file=fopen("./../Simu_data/CoP.txt","a");
//            for(n=0; n<3; n++)
//            {
//                fprintf(CoP_file,"%f ",gsl_vector_get(ws->CoP,n));
//            }
//            fprintf(CoP_file,"\n");
//            fclose(CoP_file);
//        }

        if (ws->f!=0.0 && t*Dtime>0.5)
        {
            gsl_vector_scale (ws->CoP, 1/ws->f);
            gsl_vector_set (ws->CoP, 2, 0);
            gsl_vector_sub(ws->p_task,ws->CoP);

            gsl_vector_memcpy(ws->dzmp,ws->p_task);
            gsl_vector_sub(ws->dzmp,ws->zmp);
            gsl_vector_scale (ws->dzmp, Te);
            gsl_vector_scale (ws->dzmp, 0.05);//0.15
            gsl_vector_add(ws->taskCoML,ws->dzmp);

            gsl_vector_memcpy(ws->zmp,ws->p_task);
            gsl_vector_scale(ws->p_task,0.02);//0.04
            gsl_vector_add(ws->taskCoML,ws->p_task);
        }

        if (Visualisation)
        {
            FILE *CoP_file=fopen("./../Simu_data/CoP.txt","a");
            for(n=0; n<3; n++)
            {
                fprintf(CoP_file,"%f ",gsl_vector_get(ws->CoP,n));
            }
            fprintf(CoP_file,"\n");
            fclose(CoP_file);

            FILE *taskCoM_file=fopen("./../Simu_data/taskCoM.txt","a");
            for(n=0; n<3; n++)
            {
                fprintf(taskCoM_file,"%f ",gsl_vector_get(ws->taskCoML,n));
            }
            fprintf(taskCoM_file,"\n");
            fclose(taskCoM_file);
        }
#endif

#if 0
        gsl_matrix_set_identity(ws->R);
        gsl_vector_set_zero(ws->p_task);
        gsl_vector_set (ws->p_task, 1, -0.1595);
        if ((t*Dtime)<1.0)
        {
            gsl_vector_set (ws->p_task, 2,0);
        }
        if (((t*Dtime)>=1.0) && ((t*Dtime)<2.0))
        {
            gsl_vector_set (ws->p_task, 2,0+0.003*(cos(0.5*M_PI*(t*Dtime-1.0))-1));
        }
        if ((t*Dtime)>=2.0)
        {
            gsl_vector_set (ws->p_task, 2,0-0.006*(cos(0.5*M_PI*(t*Dtime-2.0))));
        }
        //gsl_vector_set (ws->p_task, 2, -0.01);
        CalcVWerrOri(uLINK, ws->task2, ws->p_task, ws->R,ws->idx2);


        gsl_vector_set (ws->p_task, 0, 0.048516);
        if ((t*Dtime)<1.0)
        {
            gsl_vector_set (ws->p_task, 1,-0.079750);
        }
        if (((t*Dtime)>=1.0) && ((t*Dtime)<2.0))
        {
            gsl_vector_set (ws->p_task, 1,-0.079750+0.012*(cos(0.5*M_PI*(t*Dtime-1.0))-1));
        }
        if ((t*Dtime)>=2.0)
        {
            gsl_vector_set (ws->p_task, 1,-0.079750-0.024*(cos(0.5*M_PI*(t*Dtime-2.0))));
        }

        //gsl_vector_set (ws->p_task, 2, 0.884101+0.05*0.05*t*Dtime*(cos(0.5*M_PI*t*Dtime)-1));
        gsl_vector_set (ws->p_task, 2, 0.884101);
        gsl_matrix_memcpy(ws->R,uLINK[Base].R);
        //pinv(ws->R,uLINK[Base].R);
        gsl_blas_dgemv(CblasNoTrans, 1.0, ws->R, ws->p_task, 0.0, ws->taskCoML);
        CalcCoM(uLINK,ws->CoM);
        //gsl_vector_memcpy(taskCoM,ws->p_task);
        gsl_vector_sub(ws->taskCoML,ws->CoM);

        gsl_vector_set (ws->p_task, 2, 0);
        gsl_vector_set_zero(ws->CoP);
        ws->f=CalcCoP(uLINK,ws->CoP,1);
        if (ws->f!=0.0 && t*Dtime>0.5)
        {
            gsl_vector_scale (ws->CoP, 1/ws->f);
            gsl_vector_set (ws->CoP, 2, 0);
            gsl_vector_sub(ws->p_task,ws->CoP);

            gsl_vector_memcpy(ws->dzmp,ws->p_task);
            gsl_vector_sub(ws->dzmp,ws->zmp);
            gsl_vector_scale (ws->dzmp, Te);
            gsl_vector_scale (ws->dzmp, 0.15);
            gsl_vector_add(ws->taskCoML,ws->dzmp);

            gsl_vector_memcpy(ws->zmp,ws->p_task);
            gsl_vector_scale(ws->p_task,0.04);
            gsl_vector_add(ws->taskCoML,ws->p_task);
        }
#endif

#if 1

// [Workspace Refactored]         static double ws->wO=5.0;
// [Workspace Refactored]         static double ws->amp=0.043;
        //static double ws->amp=0.038;

        gsl_matrix_set_identity(ws->R);
        gsl_vector_set_zero(ws->p_task);
        gsl_vector_set (ws->p_task, 1, -0.1595);
        if ((t*Dtime)<1.0)
        {
            gsl_vector_set (ws->p_task, 2,0);
        }
        if (((t*Dtime)>=1.0) && ((t*Dtime)<(ws->wO+1)))
        {
            gsl_vector_set (ws->p_task, 2,0+0.002*(cos((1/ws->wO)*M_PI*(t*Dtime-1.0))-1));
        }
        if ((t*Dtime)>=(ws->wO+1))
        {
            gsl_vector_set (ws->p_task, 2,0-0.004*(cos((1/ws->wO)*M_PI*(t*Dtime-(ws->wO+1)))));
        }
        CalcVWerrOri(uLINK, ws->task2, ws->p_task, ws->R,ws->idx2);


//vec2tab(ws->pf, ws->p_task);
//MatIf( ws->Rf, 3) ;
//CalcVWerrOri_f(ws->uLINKc,ws->taskf, ws->pf, ws->Rf, path2,14);

//MatPrintf( "taskf", "%4.6f " , ws->taskf , 6, 1) ;
//printf(" \n \n");
//PrintGSLVector(ws->task2);






        gsl_vector_set (ws->p_task, 0, 0.048516);
        if ((t*Dtime)<1.0)
        {
            gsl_vector_set (ws->p_task, 1,-0.079750);
        }
        if (((t*Dtime)>=1.0) && ((t*Dtime)<(ws->wO+1)))
        {
            gsl_vector_set (ws->p_task, 1,-0.079750+ws->amp*(cos((1/ws->wO)*M_PI*(t*Dtime-1.0))-1));
        }
        if ((t*Dtime)>=(ws->wO+1))
        {
            gsl_vector_set (ws->p_task, 1,-0.079750-2*ws->amp*(cos((1/ws->wO)*M_PI*(t*Dtime-(ws->wO+1)))));
        }

        gsl_vector_set (ws->p_task, 2, 0.884101);
        gsl_matrix_memcpy(ws->R,uLINK[Status->right_foot_ID].R);
        //pinv(ws->R,uLINK[Base].R);
        gsl_blas_dgemv(CblasNoTrans, 1.0, ws->R, ws->p_task, 0.0, ws->taskCoMR);
        CalcCoM(uLINK,ws->CoM);
//gsl_vector_memcpy(ws->taskCoMR,ws->p_task);
        gsl_vector_sub(ws->taskCoMR,ws->CoM);




        gsl_vector_set (ws->p_task, 0, 0.048516);
        if ((t*Dtime)<1.0)
        {
            gsl_vector_set (ws->p_task, 1,-0.079750);
        }
        if (((t*Dtime)>=1.0) && ((t*Dtime)<(ws->wO+1)))
        {
            gsl_vector_set (ws->p_task, 1,-0.079750+0.038*(cos((1/ws->wO)*M_PI*(t*Dtime-1.0))-1));
        }
        if ((t*Dtime)>=(ws->wO+1))
        {
            gsl_vector_set (ws->p_task, 1,-0.079750-0.076*(cos((1/ws->wO)*M_PI*(t*Dtime-(ws->wO+1)))));
        }

        gsl_vector_set (ws->p_task, 2, 0.884101);
        gsl_matrix_memcpy(ws->R,uLINK[Status->left_foot_ID].R);
        //pinv(ws->R,uLINK[Base].R);
        gsl_blas_dgemv(CblasNoTrans, 1.0, ws->R, ws->p_task, 0.0, ws->taskCoML);
        CalcCoM(uLINK,ws->CoM);
//gsl_vector_memcpy(ws->taskCoML,ws->p_task);
        gsl_vector_sub(ws->taskCoML,ws->CoM);

//        gsl_vector_set (ws->p_task, 2, 0);
//        gsl_vector_set_zero(ws->CoP);
//        ws->f=CalcCoP(uLINK,ws->CoP,1);
//        if (ws->f!=0.0 && t*Dtime>0.5)
//        {
//            gsl_vector_scale (ws->CoP, 1/ws->f);
//            gsl_vector_set (ws->CoP, 2, 0);
//            gsl_vector_sub(ws->p_task,ws->CoP);
//
//            gsl_vector_memcpy(ws->dzmp,ws->p_task);
//            gsl_vector_sub(ws->dzmp,ws->zmp);
//            gsl_vector_scale (ws->dzmp, Te);
//            gsl_vector_scale (ws->dzmp, 0.15);
//            gsl_vector_add(ws->taskCoML,ws->dzmp);
//
//            gsl_vector_memcpy(ws->zmp,ws->p_task);
//            gsl_vector_scale(ws->p_task,0.04);
//            gsl_vector_add(ws->taskCoMR,ws->p_task);
//            gsl_vector_add(ws->taskCoML,ws->p_task);
//        }

//        PrintGSLVector(ws->task2);
//        PrintGSLVector(ws->taskCoMR);
//        PrintGSLVector(ws->taskCoML);

//gsl_vector_scale(ws->taskCoMR,0.5);
//gsl_vector_scale(ws->taskCoML,0.5);

        if (Visualisation)
        {
            FILE *CoM_file=fopen("./../Simu_data/CoM.txt","a");
            for(n=0; n<3; n++)
            {
                fprintf(CoM_file,"%f ",gsl_vector_get(ws->CoM,n));
            }
            fprintf(CoM_file,"\n");
            fclose(CoM_file);

            FILE *CoMp_file=fopen("./../Simu_data/CoMp.txt","a");
            for(n=0; n<3; n++)
            {
                fprintf(CoMp_file,"%f ",gsl_vector_get(ws->p_task,n));
            }
            fprintf(CoMp_file,"\n");
            fclose(CoMp_file);

            FILE *CoP_file=fopen("./../Simu_data/CoP.txt","a");
            for(n=0; n<3; n++)
            {
                fprintf(CoP_file,"%f ",gsl_vector_get(ws->CoP,n));
            }
            fprintf(CoP_file,"\n");
            fclose(CoP_file);

            FILE *taskCoMR_file=fopen("./../Simu_data/taskCoMR.txt","a");
            for(n=0; n<3; n++)
            {
                fprintf(taskCoMR_file,"%f ",gsl_vector_get(ws->taskCoMR,n));
            }
            fprintf(taskCoMR_file,"\n");
            fclose(taskCoMR_file);

            FILE *taskCoML_file=fopen("./../Simu_data/taskCoML.txt","a");
            for(n=0; n<3; n++)
            {
                fprintf(taskCoML_file,"%f ",gsl_vector_get(ws->taskCoML,n));
            }
            fprintf(taskCoML_file,"\n");
            fclose(taskCoML_file);

        }

#endif

#if Ext_traj
        Ext_op_trajectory(ws->opd, 0);

        gsl_matrix_set_identity(ws->R);
        gsl_vector_set_zero(ws->p_task);
        gsl_vector_set (ws->p_task, 0, (ws->opd[3]-ws->opd[6]));
        gsl_vector_set (ws->p_task, 1, (ws->opd[4]-ws->opd[7]));
        gsl_vector_set (ws->p_task, 2, (ws->opd[5]-ws->opd[8]));
        //gsl_vector_scale(ws->p_task,-1.0);
        CalcVWerrOri(uLINK, ws->task2, ws->p_task, ws->R,ws->idx2);

        gsl_vector_set (ws->taskCoML, 0, ws->opd[0]);
        gsl_vector_set (ws->taskCoML, 1, ws->opd[1]);
        gsl_vector_set (ws->taskCoML, 2, ws->opd[2]-0.1);
        CalcCoM(uLINK,ws->CoM);
        gsl_vector_sub(ws->taskCoML,ws->CoM);



#endif

#if 0
        if ((t*Dtime)>=0.5)
        {
            Ext_op_trajectory_LIPM(ws->opd, 0);
        }

        gsl_matrix_set_identity(ws->R);
        gsl_vector_set_zero(ws->p_task);
        if ((t*Dtime)<0.5)
        {
            gsl_vector_set (ws->p_task, 1, 0.0);
            gsl_vector_set (ws->p_task, 1, -0.1595);
            gsl_vector_set (ws->p_task, 2, 0.0);
        }
        if (((t*Dtime)>=0.5))
        {
            gsl_vector_set (ws->p_task, 0, (ws->opd[3]-ws->opd[6]));
            gsl_vector_set (ws->p_task, 1, (ws->opd[4]-ws->opd[7]));
            gsl_vector_set (ws->p_task, 2, (ws->opd[5]-ws->opd[8])+0.09*(ws->opd[1]));
        }
        CalcVWerrOri(uLINK, ws->task2, ws->p_task, ws->R,ws->idx2);

        if ((t*Dtime)<0.5)
        {
            gsl_vector_set (ws->p_task, 0, 0.048516);
            gsl_vector_set (ws->p_task, 1,-0.079750);
            gsl_vector_set (ws->p_task, 2, 0.884101);
        }
        if (((t*Dtime)>=0.5))
        {
            gsl_vector_set (ws->p_task, 0, (ws->opd[0]-ws->opd[6]));
            gsl_vector_set (ws->p_task, 1, (ws->opd[1]-ws->opd[7]));
            gsl_vector_set (ws->p_task, 2, (ws->opd[2]-ws->opd[8]));
        }
        gsl_matrix_memcpy(ws->R,uLINK[Status->right_foot_ID].R);
        //pinv(ws->R,uLINK[Base].R);
        gsl_blas_dgemv(CblasNoTrans, 1.0, ws->R, ws->p_task, 0.0, ws->taskCoMR);
        CalcCoM(uLINK,ws->CoM);
        //gsl_vector_memcpy(taskCoM,ws->p_task);
        gsl_vector_sub(ws->taskCoMR,ws->CoM);


        if ((t*Dtime)<0.5)
        {
            gsl_vector_set (ws->p_task, 0, 0.048516);
            gsl_vector_set (ws->p_task, 1,-0.079750);
            gsl_vector_set (ws->p_task, 2, 0.884101);
        }
        if (((t*Dtime)>=0.5))
        {
            gsl_vector_set (ws->p_task, 0, (ws->opd[0]-ws->opd[6]));
            gsl_vector_set (ws->p_task, 1, (ws->opd[1]-ws->opd[7]));
            gsl_vector_set (ws->p_task, 2, (ws->opd[2]-ws->opd[8]));
        }
        gsl_matrix_memcpy(ws->R,uLINK[Status->left_foot_ID].R);
        //pinv(ws->R,uLINK[Base].R);
        gsl_blas_dgemv(CblasNoTrans, 1.0, ws->R, ws->p_task, 0.0, ws->taskCoML);
        CalcCoM(uLINK,ws->CoM);
        //gsl_vector_memcpy(taskCoM,ws->p_task);
        gsl_vector_sub(ws->taskCoML,ws->CoM);

//        gsl_vector_set (ws->p_task, 2, 0);
//        gsl_vector_set_zero(ws->CoP);
//        ws->f=CalcCoP(uLINK,ws->CoP,1);
//        if (ws->f!=0.0 && t*Dtime>0.5)
//        {
//            gsl_vector_scale (ws->CoP, 1/ws->f);
//            gsl_vector_set (ws->CoP, 2, 0);
//            gsl_vector_sub(ws->p_task,ws->CoP);
//
//            gsl_vector_memcpy(ws->dzmp,ws->p_task);
//            gsl_vector_sub(ws->dzmp,ws->zmp);
//            gsl_vector_scale (ws->dzmp, Te);
//            gsl_vector_scale (ws->dzmp, 0.15);
//            gsl_vector_add(ws->taskCoML,ws->dzmp);
//
//            gsl_vector_memcpy(ws->zmp,ws->p_task);
//            gsl_vector_scale(ws->p_task,0.04);
//            gsl_vector_add(ws->taskCoML,ws->p_task);
//        }

//PrintGSLVector(ws->task2);
//PrintGSLVector(ws->taskCoML);

#endif





        for (i=0; i<(nDoF-6); i++)
        {
            gsl_vector_set(ws->adphi,i,-0.2*(2*(uLINK[i+2].q-uLINK[i+2].qmoy)/(ws->qdev[i]*ws->qdev[i])));
        }



        //PrintGSLVector(ws->task1);
        //PrintGSLVector(ws->task2);
        //PrintGSLVector(taskCoM);


//        gsl_matrix_set_identity(ws->P1);
//        pinv(ws->invJ,ws->J2);
//        gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, ws->invJ, ws->J2, 0.0, ws->Ptmp);
//        gsl_matrix_sub(ws->P1,ws->Ptmp);
//
//        gsl_matrix_set_identity(ws->P2);
//        pinv(ws->invJ,ws->J2);
//        gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, ws->invJ, ws->J2, 0.0, ws->Ptmp);
//        gsl_matrix_sub(ws->P2,ws->Ptmp);
//
//        gsl_matrix_set_identity(ws->PCoMR);
//        pinv(ws->invJCoM,ws->JCoMR);
//        gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, ws->invJCoM, ws->JCoMR, 0.0, ws->Ptmp);
//        gsl_matrix_sub(ws->PCoMR,ws->Ptmp);
//
//        gsl_matrix_set_identity(ws->PCoML);
//        pinv(ws->invJCoM,ws->JCoML);
//        gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, ws->invJCoM, ws->JCoML, 0.0, ws->Ptmp);
//        gsl_matrix_sub(ws->PCoML,ws->Ptmp);


//MatPrintf( "Jf", "%4.6f " , ws->Jf , 6, (nDoF-6)) ;
//printf(" \n \n");
//PrintGSLMatrix(ws->J2);

//MatPrintf( "taskf", "%4.6f " , ws->taskf , 6, 1) ;
//printf(" \n \n");
//PrintGSLVector(ws->task2);

        // first task
        pinv(ws->invJ,ws->J2);
        gsl_blas_dgemv(CblasNoTrans, 1.0, ws->invJ, ws->task2, 0.0, ws->dq);


//MatPseudoInvf( ws->invf , ws->Jf , 6, (nDoF-6)) ;
//MatMulf( ws->dqf , ws->invf , ws->taskf, (nDoF-6), 6, 1 ) ;

//MatPrintf( "invf", "%4.6f " , ws->invf , (nDoF-6), 6) ;
////MatPrintf( "invf", "%4.6f " , ws->invf , 6, (nDoF-6)) ;
//printf(" \n \n");
//PrintGSLMatrix(ws->invJ);


//MatPrintf( "dqf", "%4.6f " , ws->dqf , 1, (nDoF-6)) ;
//printf(" \n \n");
//PrintGSLVector(ws->dq);




        // second task
        gsl_matrix_set_identity(ws->P1);
        pinv(ws->invJ,ws->J2);
        gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, ws->invJ, ws->J2, 0.0, ws->Ptmp);
        gsl_matrix_sub(ws->P1,ws->Ptmp);

        gsl_vector_memcpy(ws->vec3,ws->taskCoML);
        gsl_blas_dgemv(CblasNoTrans, 1.0, ws->JCoML, ws->dq, 0.0, ws->vec3_2);
        gsl_vector_sub(ws->vec3,ws->vec3_2);
        gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, ws->JCoML, ws->P1, 0.0, ws->Jtilde);
        pinv(ws->invJCoM,ws->Jtilde);
        gsl_blas_dgemv(CblasNoTrans, 1.0, ws->invJCoM, ws->vec3, 0.0, ws->dqtmp);
        //gsl_blas_dgemv(CblasNoTrans, 1.0, ws->P2, ws->dqtmp2, 0.0, ws->dqtmp);
        gsl_vector_add(ws->dq,ws->dqtmp);


        // third task
        gsl_matrix_memcpy(ws->P2,ws->P1);
        pinv(ws->invJCoM,ws->Jtilde);
        gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, ws->invJCoM, ws->Jtilde, 0.0, ws->Ptmp);
        gsl_matrix_sub(ws->P2,ws->Ptmp);

        gsl_vector_memcpy(ws->vec3,ws->taskCoMR);
        gsl_blas_dgemv(CblasNoTrans, 1.0, ws->JCoMR, ws->dq, 0.0, ws->vec3_2);
        gsl_vector_sub(ws->vec3,ws->vec3_2);

        gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, ws->JCoMR, ws->P2, 0.0, ws->Jtilde);
        pinv(ws->invJCoM,ws->Jtilde);
        gsl_blas_dgemv(CblasNoTrans, 1.0, ws->invJCoM, ws->vec3, 0.0, ws->dqtmp);
        //gsl_blas_dgemv(CblasNoTrans, 1.0, ws->P2, ws->dqtmp2, 0.0, ws->dqtmp);
        gsl_vector_add(ws->dq,ws->dqtmp);


        // fourth task
        gsl_matrix_set_identity(ws->Ptilde);
        pinv(ws->invJCoM,ws->Jtilde);
        gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, ws->invJCoM, ws->Jtilde, 0.0, ws->Ptmp);
        gsl_matrix_sub(ws->Ptilde,ws->Ptmp);

        gsl_blas_dgemv(CblasNoTrans, 1.0, ws->Ptilde, ws->adphi, 0.0, ws->dqtmp);
        gsl_vector_add(ws->dq,ws->dqtmp);


//PrintGSLVector(ws->dq);

        gsl_vector_memcpy(ws->ddq_task,ws->dq);
        gsl_vector_sub(ws->ddq_task,ws->dq_old);
        gsl_vector_scale(ws->ddq_task,1/Te);


        for (i=0; i<(nDoF-6); i++)
        {
            //uLINK[i+2].u_joint =1000*gsl_vector_get(ws->dq,i)+10*gsl_vector_get(ws->ddq_task,i);
            uLINK[i+2].u_joint =1000*gsl_vector_get(ws->dq,i)+10*gsl_vector_get(ws->ddq_task,i);
        }

        gsl_vector_memcpy(ws->dq_old,ws->dq);


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

            FILE *u_file=fopen("./../Simu_data/u.txt","a");
            for(n=0; n<nDoF-6; n++)
            {
                fprintf(u_file,"%f ",uLINK[n+2].u_joint);
            }
            fprintf(u_file,"\n");
            fclose(u_file);

        }




//
//        double kd=1;
//        double kp=500;
//
//        double *kqd;
//        kqd = calloc(nDoF-6,sizeof(double));
//        Ext_q_trajectory(kqd, 0);
//
//        for (i=0; i<(nDoF-6); i++)
//        {
//            uLINK[i+2].u_joint = kd*(0.0-ws->uLINKc[n+2].dq)+kp*(kqd[i+1]-ws->uLINKc[i+2].q);
//        }
//
//        free (kqd);



//        double kd=300;
//        double kp=100;
//        for(n=0; n<nDoF-6; n++)
//        {
//            uLINK[n+2].u_joint=kd*(gsl_vector_get(ws->dq,n)*Te-ws->uLINKc[n+2].dq)+kp*((ws->uLINKc[n+2].q+gsl_vector_get(ws->dq,n)*Te)-ws->uLINKc[n+2].q);
//        }



//            printf("\n P1: \n");w_c = gsl_vector_calloc (3);
//            PrintGSLMatrix(ws->P1);
//
//            printf("\n P2: \n");
//            PrintGSLMatrix(ws->P2);
//
//            printf("\n PCoM: \n");
//            PrintGSLMatrix(PCoM);

        //PrintGSLVector(taskCoM);
        //PrintGSLVector(ws->dqtmp);
        //PrintGSLVector(ws->dq);


#endif





        for(n=2; n<nDoF-6+2; n++)
        {
            //uLINK[n].u_joint = gsl_vector_get (ws->u,n-2+6) ;

            if (ws->Statusc.desired_support!=0 || Suspendu)
            {
#if PD
                uLINK[n].u_joint = ws->uPD[n-2]+ws->uG[n-2]+ws->uStab[n-2];
#endif
#if Dynamic
                uLINK[n].u_joint = ws->uPD[n-2]+ws->uG[n-2];
#endif

            }
            else
            {
#if PD
                uLINK[n].u_joint = ws->uPD[n-2]+ws->uStab[n-2];
#endif
#if Dynamic
                uLINK[n].u_joint = ws->uPD[n-2]+ws->uG[n-2];
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
            gsl_vector_set (ws->u,n-2+6,uLINK[n].u_joint);
        }

//PrintGSLVector(ws->u);
//gsl_vector_set_zero(ws->u);

    gsl_vector_add (ws->u,ws->g);
    gsl_vector_sub (ws->u,ws->ef);
    gsl_vector_sub (ws->u,ws->b);
//    gsl_vector * ws->ddq = gsl_vector_alloc (nDoF);
//    gsl_permutation * ws->p = gsl_permutation_alloc (nDoF);
//    gsl_matrix * ws->A2 = gsl_matrix_calloc (nDoF, nDoF);
    gsl_matrix_memcpy(ws->A2,ws->A);
    int s;
    gsl_linalg_LU_decomp (ws->A2, ws->p, &s);
    gsl_linalg_LU_solve (ws->A2, ws->p, ws->u, ws->ddq);
//    gsl_permutation_free (ws->p);

//ws->ddq = ws->A \ (-ws->b + ws->u);//LU decomp

////gsl_vector * test = gsl_vector_alloc (18);
////gsl_blas_dgemv(CblasNoTrans, 1.0, ws->A, ws->ddq, 0.0, test);
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
////            printf("%8.4f ",gsl_vector_get (ws->u, n));
////    }
////    printf(" \n");
////
////gsl_vector_free(test);
////    printf(" b: ");
////    for(n=0; n<18; n++)
////    {
////            printf("%8.4f ",gsl_vector_get (ws->b, n));
////    }
////    printf(" \n");
////
////    printf(" ddq: ");
////    for(n=0; n<18; n++)
////    {
////            printf("%8.4f \n",gsl_vector_get (ws->ddq, n));
////    }
////    printf(" \n");




    for (n=0; n<3; n++)
    {
        gsl_vector_set (uLINK[1].dvo, n,gsl_vector_get (ws->ddq,n));
    }
//    printf(" dvo: ");
//    for(n=0; n<3; n++)
//    {
//            printf("%8.4f ",gsl_vector_get (uLINK[1].dvo, n));
//    }
//    printf(" \n");
//uLINK(1).dvo = ws->ddq(1:3);
    for (n=0; n<3; n++)
    {
        gsl_vector_set (uLINK[1].dw, n,gsl_vector_get (ws->ddq,n+3));
    }
//uLINK(1).dw  = ws->ddq(4:6);
    for (n=2; n<nDoF-6+2; n++)
    {
        uLINK[n].ddq = gsl_vector_get (ws->ddq,n+6-2);
    }
//for j=2:length(uLINK)
//    uLINK(j).ddq = ws->ddq(j+6-1);
//end

//    gsl_matrix_free(ws->A);
//    gsl_vector_free(ws->b);
//    gsl_vector_free(ws->tmp);
//    gsl_vector_free(ws->g);
//    gsl_vector_free(ws->ef);
//    gsl_vector_free(ws->stab);0.0001
//    gsl_vector_free(ws->u);
//    gsl_matrix_free(ws->A2);
//    gsl_vector_free(ws->ddq);


}



