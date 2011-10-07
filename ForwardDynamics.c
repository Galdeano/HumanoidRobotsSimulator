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


#include "David_uLINK.h"
#include "David_StaticTrajectory.h"
#include "David_ForwardKinematics.h"
#include "David_CalcCoM.h"
#include "David_Gravity.h"
#include "David_Stabilizator.h"
#include "DrawMarkerf.h"

#ifndef MCSpline
#include "Mat.h"
#else
#include "d:\Mb\StLib\Mat.h"
#endif

#include "Setup.h"


void ForwardDynamics(SuLINK uLINK[],State *Status,double Dtime,long t)
{
    static Struct_uLINK uLINKc[NbLinks];
    static Struct_State Statusc;
    if (t==1)
    {
        David_SetupRobot(uLINKc,&Statusc);
    }



    int nDoF = Status->ddl;
    int n,i,j;
//    double tmp2;
    gsl_matrix * A = gsl_matrix_calloc (nDoF, nDoF);
//nDoF = length(uLINK)-1+6;
//A = zeros(nDoF,nDoF);
    gsl_vector * b = gsl_vector_calloc (nDoF);
    gsl_vector * tmp = gsl_vector_calloc (nDoF);
    gsl_vector * g = gsl_vector_calloc (nDoF);
    gsl_vector * ef = gsl_vector_calloc (nDoF);
    gsl_vector * stab = gsl_vector_calloc (nDoF);

    //gsl_vector * qd = gsl_vector_calloc (nDoF);
    //gsl_vector * dqd = gsl_vector_calloc (nDoF);
    gsl_vector * u = gsl_vector_calloc (nDoF);

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


    if ((t%50)==0)
    {

        static float uPD[NbLinks-2], qd[NbLinks-2], dqd[NbLinks-2];  // rad
        static float com[3];


        FILE *dq_file=fopen("./../Simu_data/dq.txt","a");
        for(n=0; n<nDoF-6; n++)
        {
            fprintf(dq_file,"%f .",uLINK[n+2].dq);
        }
        fprintf(dq_file,"\n");
        for(n=0; n<nDoF-6; n++)
        {
            fprintf(dq_file,"%f .",(uLINK[n+2].q - uLINKc[n+2].q)/Te);
        }
        fprintf(dq_file,"\n");
        fprintf(dq_file,"\n");
        fclose(dq_file);



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


        David_ForwardKinematics(uLINKc,1);

        static float FootR[3]= {0.0853,0,-0.11};
        MatMulf( Statusc.FootCenter_R, uLINKc[7].R, FootR, 3, 3, 1) ;
        MatAddf( Statusc.FootCenter_R, Statusc.FootCenter_R, uLINKc[7].p, 3, 1) ;

        static float FootL[3]= {0.0853,0,-0.11};
        MatMulf( Statusc.FootCenter_L, uLINKc[13].R, FootL, 3, 3, 1) ;
        MatAddf( Statusc.FootCenter_L, Statusc.FootCenter_L, uLINKc[13].p, 3, 1) ;

//        DrawMarkerf(Statusc.FootCenter_R);
//        DrawMarkerf(Statusc.FootCenter_L);

        David_CalcCoM(uLINKc,com);
        //DrawMarkerf(com);



        David_OneFoot(qd, t*Dtime, &Statusc.desired_support, &Statusc.distribution_y);
        David_OneFoot(dqd, t*Dtime-Dtime, &Statusc.desired_support, &Statusc.distribution_y);

//            FILE *dist_file=fopen("./../Simu_data/dist.txt","a");
//            for(n=0; n<nDoF-6; n++)
//            {
//                fprintf(dist_file,"%f ",Statusc.distribution_y);
//            }
//            fprintf(dist_file,"\n");
//            fclose(dist_file);

        David_OneFoot(qd, t*Dtime, &Status->desired_support, &Status->distribution_y);
        David_OneFoot(dqd, t*Dtime-Dtime, &Status->desired_support, &Status->distribution_y);


#if PD
#if Sherpa
        float kd=3;
        float kp=1000;
#endif
#if Generic
        float kd=1;
        float kp=300;
#endif

        for(n=0; n<nDoF-6; n++)
        {
            uPD[n]=kd*(((qd[n]-dqd[n])/Te)-uLINKc[n+2].dq)+kp*(qd[n]-uLINKc[n+2].q);
            //uPD[n]=kd*(((0)/Te)-uLINKc[n+2].dq)+kp*(0-uLINKc[n+2].q);
            //gsl_vector_set(u,n+6,uPD[n]);
        }

        static float uG[NbLinks-2], fG[NbLinks-2], tG[NbLinks-2];
        David_Gravity( uLINKc, &Statusc, 1, fG, tG);
        for (n=0; n<nDoF-6; n++)
        {
            uG[n]=uLINKc[n+2].ug;
        }

        //Stabilizator(uLINK,Status,stab,Dtime,t*Dtime);
        //gsl_vector_add (u,stab);
        static float uStab[NbLinks-2];
        //David_Stabilizator( uLINKc, &Statusc, uStab);
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
        David_Gravity( uLINKc, &Statusc, 1, fG, tG);
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
            gsl_vector_set (u,n-2+6,uLINK[n].u_joint);

        }

    }
    else
    {
        for(n=2; n<nDoF-6+2; n++)
        {
            gsl_vector_set (u,n-2+6,uLINK[n].u_joint);
        }
    }


//gsl_vector_set_zero(u);

    gsl_vector_sub (u,g);
    gsl_vector_sub (u,ef);
    gsl_vector_sub (u,b);
    gsl_vector * ddq = gsl_vector_alloc (nDoF);
    gsl_permutation * p = gsl_permutation_alloc (nDoF);
    gsl_matrix * A2 = gsl_matrix_calloc (nDoF, nDoF);
    gsl_matrix_memcpy(A2,A);
    int s;
    gsl_linalg_LU_decomp (A2, p, &s);
    gsl_linalg_LU_solve (A2, p, u, ddq);
    gsl_permutation_free (p);

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

    gsl_matrix_free(A);
    gsl_vector_free(b);
    gsl_vector_free(tmp);
    gsl_vector_free(g);
    gsl_vector_free(ef);
    gsl_vector_free(stab);
    gsl_vector_free(u);
    //gsl_vector_free(qd);
    //gsl_vector_free(dqd);
    gsl_matrix_free(A2);
    gsl_vector_free(ddq);

}



