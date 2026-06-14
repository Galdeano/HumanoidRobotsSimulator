#include <Eigen/Dense>




#include <vector>
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

#include "LoadRobot.h"

#include "Ext_traj.h"
#include "StaticTrajectory.h"
#include "OneFootTrajectory.h"
#include "Scenarios.h"
#include "ForwardKinematics.h"
#include "CalcCoM.h"
#include "Gravity.h"
#include "Stabilizator.h"
#include "DrawMarker.h"
#include "CalcJacobianModif.h"
#include "CalcVWerrOri.h"
#include "pinv.h"
#include "PrintGSLMatrix.h"
#include "CalcCoMJacobian.h"
#include "CalcCoP.h"

#include "CalcJacobianModif.h"
#include "CalcVWerrOri.h"


#ifndef MCSpline

#else
#include "d:\Mb\StLib\Mat.h"
#endif

#include "Setup.h"


void ForwardDynamics(SuLINK uLINK[],State *Status,long t)
{

    static std::vector<SuLINK> uLINKc_vec;
    static SuLINK *uLINKc = nullptr;
    static Eigen::MatrixXd Inertia_Motor;
    static Eigen::MatrixXd A;
    static Eigen::VectorXd b;
    static Eigen::VectorXd tmp;
    static Eigen::VectorXd g;
    static Eigen::VectorXd ef;
    static Eigen::VectorXd stab;
    static Eigen::VectorXd u;
    static Eigen::VectorXd ddq;
    static Eigen::MatrixXd A2;


    int nDoF = Status->ddl;
    int n,i,j;

//    gsl_matrix * A = gsl_matrix_calloc (nDoF, nDoF);
//    gsl_vector * b = gsl_vector_calloc (nDoF);
//    gsl_vector * tmp = gsl_vector_calloc (nDoF);
//    gsl_vector * g = gsl_vector_calloc (nDoF);
//    gsl_vector * ef = gsl_vector_calloc (nDoF);
//    gsl_vector * stab = gsl_vector_calloc (nDoF);
//    gsl_vector * u = gsl_vector_calloc (nDoF);



    static State Statusc;
    static int init=1;
    if (init==1)
    {

        uLINKc_vec.resize((Status->ddl)+2-6);
        uLINKc = uLINKc_vec.data();
        Inertia_Motor.resize(nDoF, nDoF); Inertia_Motor.setZero();
        A.resize(nDoF, nDoF); A.setZero();
        b.resize(nDoF); b.setZero();
        tmp.resize(nDoF); tmp.setZero();
        g.resize(nDoF); g.setZero();
        ef.resize(nDoF); ef.setZero();
        stab.resize(nDoF); stab.setZero();
        u.resize(nDoF); u.setZero();
        ddq.resize(nDoF); ddq.setZero();
        A2.resize(nDoF, nDoF); A2.setZero();

        for(n=7; n<nDoF; n++)
        {
            Inertia_Motor(n,n) = 0.5;
        }
        LoadRobotParserXML(uLINKc,&Statusc,Status->RobotFile);
        init=0;
    }

    u.setZero();


    InvDyn(uLINK,Status,0,b);
//b = InvDyn(0);
//--------------------------------------------------------------------------------
    for(n=1; n<(nDoF+1); n++)
    {
        InvDyn(uLINK,Status,n,tmp);
        tmp -= b;
        A.col(n-1) = tmp;
    }


//    Gravity
//    ExternalForces

    Eigen::Vector3d f_grav = Eigen::Vector3d::Zero();
    Eigen::Vector3d tau_grav = Eigen::Vector3d::Zero();

    Gravity(uLINK,Status,1,f_grav,tau_grav);
    for (n=0; n<3; n++)
    {
        g(n) = f_grav(n);
    }
    for (n=0; n<3; n++)
    {
        g(n+3) = tau_grav(n);
    }
    for (n=0; n<nDoF-6; n++)
    {
        g(n+6) = uLINK[n+2].ug;
    }

    f_grav.setZero();
    tau_grav.setZero();
    ExternalForces(uLINK,Status,1,f_grav,tau_grav);
    if (Status->desired_support!=0 || Suspendu)
    {
        for (n=0; n<3; n++)
        {
            ef(n) = f_grav(n);
        }
        for (n=0; n<3; n++)
        {
            ef(n+3) = tau_grav(n);
        }
        for (n=0; n<nDoF-6; n++)
        {
            ef(n+6) = uLINK[n+2].uef;
        }
    }

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
    A += Inertia_Motor;


//      if ((t%50)==0)
    if ((t%(int)(Te/Dtime))==0)
    {
        static int init_com=1;
        static Eigen::VectorXd uPD;
        static Eigen::VectorXd qd;
        static Eigen::VectorXd dqd;
        if (init_com==1)
        {
            uPD.resize(Status->ddl); uPD.setZero();
            qd.resize(Status->ddl); qd.setZero();
            dqd.resize(Status->ddl); dqd.setZero();
            init_com=0;
        }

        static Eigen::Vector3d com;


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
                uLINKc[1].R(i, j) = uLINK[1].R(i, j);
            }
        }

//        for(i=0; i<3; i++)
//        {
//                uLINKc[1].p[i]=gsl_vector_get (uLINK[1].p, i);
//        }


        ForwardKinematics(uLINKc,1);

        static Eigen::Vector3d FootR; FootR << 0.0853, 0, -0.11;
        Statusc.FootCenter_R = uLINKc[7].R * FootR + uLINKc[7].p;

        static Eigen::Vector3d FootL; FootL << 0.0853, 0, -0.11;
        Statusc.FootCenter_L = uLINKc[13].R * FootL + uLINKc[13].p;

        CalcCoM(uLINKc,com);
        //DrawMarkerf(com);

#if Trajectories
        OneFootTrajectory(uLINKc, &Statusc, u, t*Dtime);
        

        OneFootTrajectory(uLINK, Status, u, t*Dtime);
        
#endif

#if Scenarios
        Scenario_desired_trajectory(qd.data(), t*Dtime, &Statusc.desired_support, &Statusc.distribution_y);
        Scenario_desired_trajectory(dqd.data(), t*Dtime-Dtime, &Statusc.desired_support, &Statusc.distribution_y);

        Scenario_desired_trajectory(qd.data(), t*Dtime, &Status->desired_support, &Status->distribution_y);
        Scenario_desired_trajectory(dqd.data(), t*Dtime-Dtime, &Status->desired_support, &Status->distribution_y);
#endif

#if Ext_traj
        for(n=0; n<nDoF-6; n++)
        {
            dqd[n]=qd[n];
        }
        Ext_q_trajectory(qd,0);

#endif




#if CMD_PD
#if ROBOT_SHERPA
        double kd=3;
        double kp=1000;
#endif
#if ROBOT_GENERIC
//        double kd=1;
//        double kp=300;
        double kd=100;
        double kp=3000;
#endif
#if ROBOT_HUMAN
        double kd=1;
        double kp=500;
#endif

        for(n=0; n<nDoF-6; n++)
        {
            uPD[n]=kd*(((qd(n+1)-dqd(n+1))/Te)-uLINKc[n+2].dq)+kp*(qd(n+1)-uLINKc[n+2].q);
            //uPD[n]=kd*(((0)/Te)-uLINKc[n+2].dq)+kp*(0-uLINKc[n+2].q);
            
        }



        static std::vector<double> uG_vec;
        static std::vector<double> fG_vec;
        static std::vector<double> tG_vec;
        static std::vector<double> uStab_vec;
        static double *uG = nullptr;
        static double *fG = nullptr;
        static double *tG = nullptr;
        static double *uStab = nullptr;
        static int init_G=1;
        if (init_G==1)
        {
            uG_vec.resize((Status->ddl)-6);
            fG_vec.resize((Status->ddl)-6);
            tG_vec.resize((Status->ddl)-6);
            uStab_vec.resize((Status->ddl)-6);
            uG = uG_vec.data();
            fG = fG_vec.data();
            tG = tG_vec.data();
            uStab = uStab_vec.data();
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


#if CMD_DYNAMIC
#if ROBOT_SHERPA
        double kd=3;
        double kp=1000;
#endif
#if ROBOT_GENERIC
        double kd=0.001;
        double kp=1;
#endif

        //static double uG[NbLinks-2];

        static double uG[NbLinks-2], fG[NbLinks-2], tG[NbLinks-2];
        Gravity_f( uLINKc, &Statusc, 1, fG, tG);
        for (n=0; n<nDoF-6; n++)
        {
            uG[n]=uLINKc[n+2].ug;
        }
        for(n=0; n<nDoF-6; n++)
        {
            uPD[n]=kd*(((0)/Te)-uLINKc[n+2].dq)+kp*(0-uLINKc[n+2].q);
            uG[n] = g(n+6) + b(n+6);
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
                fprintf(pBody_file,"%f ",uLINK[1].p(n));
            }
            fprintf(pBody_file,"\n");
            fclose(pBody_file);


#if CMD_PD
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



#if CMD_TASK



        static Eigen::VectorXd idx1;
        static Eigen::VectorXd idx2;
        static Eigen::MatrixXd J1;
        static Eigen::MatrixXd J2;
        static Eigen::MatrixXd JCoMR;
        static Eigen::MatrixXd JCoML;
        static Eigen::MatrixXd Jtilde;
        static Eigen::VectorXd task1;
        static Eigen::VectorXd task2;
        static Eigen::VectorXd taskCoMR;
        static Eigen::VectorXd taskCoML;
        static Eigen::Vector3d CoM;
        static Eigen::Vector3d vec3;
        static Eigen::Vector3d vec3_2;
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
        static Eigen::VectorXd ddq_task;

        static double *opd;
        static Eigen::Vector3d trace;

        static Eigen::VectorXd adphi;
        static double *qdev;
        static Eigen::Vector3d CoP;
 //       static double f=0.0;
        static Eigen::Vector3d zmp;
        static Eigen::Vector3d dzmp;

        static Eigen::VectorXd q_pd;

        static int path1[8] = {7, 7, 6, 5, 4, 3, 2, 1};
        static int path2[14] = {13, 13, 12, 11, 10, 9, 8, 2, 3, 4, 5, 6, 7, 7};

        static double *Jf;
        static double *invf;
        static double *pf;
        static double *Rf;
        static double *taskf;
        static double *dqf;

        static int init_task=1;
        if (init_task==1)
        {
            idx1.resize(8); idx1.setZero();
            idx2.resize(14); idx2.setZero();
            J1.resize(6,nDoF-6); J1.setZero();
            J2.resize(6,nDoF-6); J2.setZero();
            JCoMR.resize(3,nDoF-6); JCoMR.setZero();
            JCoML.resize(3,nDoF-6); JCoML.setZero();
            Jtilde.resize(3,nDoF-6); Jtilde.setZero();
            task1.resize(6); task1.setZero();
            task2.resize(6); task2.setZero();
            taskCoMR.resize(3); taskCoMR.setZero();
            taskCoML.resize(3); taskCoML.setZero();
            CoM.setZero();
            vec3.setZero();
            vec3_2.setZero();
            p.setZero();
            R.setZero();
            P1.resize(nDoF-6,nDoF-6); P1.setZero();
            P2.resize(nDoF-6,nDoF-6); P2.setZero();
            PCoMR.resize(nDoF-6,nDoF-6); PCoMR.setZero();
            PCoML.resize(nDoF-6,nDoF-6); PCoML.setZero();
            Ptilde.resize(nDoF-6,nDoF-6); Ptilde.setZero();
            Ptmp.resize(nDoF-6,nDoF-6); Ptmp.setZero();
            invJ.resize(nDoF-6,6); invJ.setZero();
            invJCoM.resize(nDoF-6,3); invJCoM.setZero();
            dq.resize(nDoF-6); dq.setZero();
            dqtmp.resize(nDoF-6); dqtmp.setZero();
            dqtmp2.resize(nDoF-6); dqtmp2.setZero();
            dq_old.resize(nDoF-6); dq_old.setZero();
            ddq_task.resize(nDoF-6); ddq_task.setZero();

            opd = (double *)calloc(9,sizeof(double));
            trace.setZero();
            adphi.resize(nDoF-6); adphi.setZero();
            qdev = (double *)calloc(nDoF-6,sizeof(double));
            CoP.setZero();
            zmp.setZero();
            dzmp.setZero();

            q_pd.resize(nDoF-6); q_pd.setZero();

            init_task=0;

            //static int path1[8] = {7, 7, 6, 5, 4, 3, 2, 1};
            //int path1[8] = {1, 2, 3, 4, 5, 6, 7, 7};
            for(i=0; i<8; i++)
            {
                idx1(i) = path1[i];
            }

            //static int path2[14] = {7, 7, 6, 5, 4, 3, 2, 8, 9, 10, 11, 12, 13, 13};

            for(i=0; i<14; i++)
            {
                idx2(i) = path2[i];
            }

            for (i=0; i<(nDoF-6); i++)
            {
                qdev[i]=fmin(fabs(uLINK[i+2].qmin-uLINK[i+2].qmoy),fabs(uLINK[i+2].qmax-uLINK[i+2].qmoy))*2;
            }


            Jf = (double *)calloc(6*(nDoF-6),sizeof(double));
            pf = (double *)calloc(3,sizeof(double));
            Rf = (double *)calloc(9,sizeof(double));
            taskf = (double *)calloc(6,sizeof(double));
            invf = (double *)calloc((nDoF-6)*6,sizeof(double));
            dqf = (double *)calloc((nDoF-6),sizeof(double));
        }



//        CalcJacobianModif_f(uLINKc,Jf,path2,14,nDoF-6);
        CalcJacobianModif( uLINK,J1,idx1);
        CalcJacobianModif( uLINK,J2,idx2);
        CalcCoMJacobian(uLINK,Status, JCoMR, Status->right_foot_ID);
        CalcCoMJacobian(uLINK,Status, JCoML, Status->left_foot_ID);

//MatPrintf( "Jf", "%4.6f " , Jf , 6, (nDoF-6)) ;
//printf(" \n \n");
//PrintGSLMatrix(J2);


        R.setIdentity();
        p.setZero();
        p(0) = 0.0155;
        p(1) = 0.0798;
        p(2) = 0.8434;
        CalcVWerrOri(uLINK, task1, p, R,idx1);


#if 0
        R.setIdentity();
        p.setZero();
        p(1) = -0.1595;
        //gsl_vector_set (p, 2, -0.01);
        CalcVWerrOri(uLINK, task2, p, R,idx2);

//        gsl_vector_set (taskCoM, 0, 0.048516-0.01*(cos(0.5*M_PI*t*Dtime)-1));
//        gsl_vector_set (taskCoM, 1,-0.079750);
//        gsl_vector_set (taskCoM, 2, 0.http://fr.farnell.com/raspberry-pi-accessories884101+0.05*0.02*t*Dtime*(cos(0.5*M_PI*t*Dtime)-1));
        //gsl_vector_set (p, 0, 0.048516-0.01*(cos(0.5*M_PI*t*Dtime)-1));
        p(0) = 0.048516;
        p(1) = -0.079750;
        //gsl_vector_set (p, 2, 0.884101+0.05*0.05*t*Dtime*(cos(0.5*M_PI*t*Dtime)-1));
        gsl_vector_set (p, 2, 0.884101+0.05*(cos(0.25*sqrt(0.1*t*Dtime)*M_PI*t*Dtime)-1));
//        pinv(R,uLINK[base].R);
//        gsl_blas_dgemv(CblasNoTrans, 1.0, R, p, 0.0, taskCoM);
        CalcCoM(uLINK,CoM);
        taskCoML = p;
        taskCoML -= CoM;

        if (Visualisation)
        {
            FILE *p_file=fopen("./../Simu_data/p.txt","a");
            for(n=0; n<3; n++)
            {
                fprintf(p_file,"%f ",p(n));
            }
            fprintf(p_file,"\n");
            fclose(p_file);

            FILE *CoM_file=fopen("./../Simu_data/CoM.txt","a");
            for(n=0; n<3; n++)
            {
                fprintf(CoM_file,"%f ",CoM(n));
            }
            fprintf(CoM_file,"\n");
            fclose(CoM_file);
        }

        gsl_vector_set (p, 2, 0);
        gsl_vector_set_zero(CoP);
        f=CalcCoP(uLINK,CoP,1);

//        if (f!=0.0 && t*Dtime>0.5)
//        {
//            CoP *= 1/f;
//            CoP(2) = 0;
//        }
//
//        if (Visualisation)
//        {
//            FILE *CoP_file=fopen("./../Simu_data/CoP.txt","a");
//            for(n=0; n<3; n++)
//            {
//                fprintf(CoP_file,"%f ",CoP(n));
//            }
//            fprintf(CoP_file,"\n");
//            fclose(CoP_file);
//        }

        if (f!=0.0 && t*Dtime>0.5)
        {
            CoP *= 1/f;
            CoP(2) = 0;
            p -= CoP;

            dzmp = p;
            dzmp -= zmp;
            dzmp *= Te;
            dzmp *= 0.05;
            taskCoML += dzmp;

            zmp = p;
            p *= 0.02;
            taskCoML += p;
        }

        if (Visualisation)
        {
            FILE *CoP_file=fopen("./../Simu_data/CoP.txt","a");
            for(n=0; n<3; n++)
            {
                fprintf(CoP_file,"%f ",CoP(n));
            }
            fprintf(CoP_file,"\n");
            fclose(CoP_file);

            FILE *taskCoM_file=fopen("./../Simu_data/taskCoM.txt","a");
            for(n=0; n<3; n++)
            {
                fprintf(taskCoM_file,"%f ",taskCoML(n));
            }
            fprintf(taskCoM_file,"\n");
            fclose(taskCoM_file);
        }
#endif

#if 0
        R.setIdentity();
        p.setZero();
        p(1) = -0.1595;
        if ((t*Dtime)<1.0)
        {
            p(2) = 0;
        }
        if (((t*Dtime)>=1.0) && ((t*Dtime)<2.0))
        {
            gsl_vector_set (p, 2,0+0.003*(cos(0.5*M_PI*(t*Dtime-1.0))-1));
        }
        if ((t*Dtime)>=2.0)
        {
            gsl_vector_set (p, 2,0-0.006*(cos(0.5*M_PI*(t*Dtime-2.0))));
        }
        //gsl_vector_set (p, 2, -0.01);
        CalcVWerrOri(uLINK, task2, p, R,idx2);


        p(0) = 0.048516;
        if ((t*Dtime)<1.0)
        {
            p(1) = -0.079750;
        }
        if (((t*Dtime)>=1.0) && ((t*Dtime)<2.0))
        {
            gsl_vector_set (p, 1,-0.079750+0.012*(cos(0.5*M_PI*(t*Dtime-1.0))-1));
        }
        if ((t*Dtime)>=2.0)
        {
            gsl_vector_set (p, 1,-0.079750-0.024*(cos(0.5*M_PI*(t*Dtime-2.0))));
        }

        //gsl_vector_set (p, 2, 0.884101+0.05*0.05*t*Dtime*(cos(0.5*M_PI*t*Dtime)-1));
        p(2) = 0.884101;
        gsl_matrix_memcpy(R,uLINK[Base].R);
        //pinv(R,uLINK[Base].R);
        taskCoML = R * p;
        CalcCoM(uLINK,CoM);
        //gsl_vector_memcpy(taskCoM,p);
        taskCoML -= CoM;

        gsl_vector_set (p, 2, 0);
        gsl_vector_set_zero(CoP);
        f=CalcCoP(uLINK,CoP,1);
        if (f!=0.0 && t*Dtime>0.5)
        {
            CoP *= 1/f;
            CoP(2) = 0;
            p -= CoP;

            dzmp = p;
            dzmp -= zmp;
            dzmp *= Te;
            gsl_vector_scale (dzmp, 0.15);
            taskCoML += dzmp;

            zmp = p;
            gsl_vector_scale(p,0.04);
            taskCoML += p;
        }
#endif

#if 1

        static double wO=5.0;
        static double amp=0.043;
        //static double amp=0.038;

        R.setIdentity();
        p.setZero();
        p(1) = -0.1595;
        if ((t*Dtime)<1.0)
        {
            p(2) = 0;
        }
        if (((t*Dtime)>=1.0) && ((t*Dtime)<(wO+1)))
        {
            p(2) = 0+0.002*(cos((1/wO)*M_PI*(t*Dtime-1.0))-1);
        }
        if ((t*Dtime)>=(wO+1))
        {
            p(2) = 0-0.004*(cos((1/wO)*M_PI*(t*Dtime-(wO+1))));
        }
        CalcVWerrOri(uLINK, task2, p, R,idx2);


//vec2tab(pf, p);
//MatIf( Rf, 3) ;
//CalcVWerrOri_f(uLINKc,taskf, pf, Rf, path2,14);

//MatPrintf( "taskf", "%4.6f " , taskf , 6, 1) ;
//printf(" \n \n");
//PrintGSLVector(task2);






        p(0) = 0.048516;
        if ((t*Dtime)<1.0)
        {
            p(1) = -0.079750;
        }
        if (((t*Dtime)>=1.0) && ((t*Dtime)<(wO+1)))
        {
            p(1) = -0.079750+amp*(cos((1/wO)*M_PI*(t*Dtime-1.0))-1);
        }
        if ((t*Dtime)>=(wO+1))
        {
            p(1) = -0.079750-2*amp*(cos((1/wO)*M_PI*(t*Dtime-(wO+1))));
        }

        p(2) = 0.884101;
        R = uLINK[Status->right_foot_ID].R;
        //pinv(R,uLINK[Base].R);
        taskCoMR = R * p;
        CalcCoM(uLINK,CoM);
//gsl_vector_memcpy(taskCoMR,p);
        taskCoMR -= CoM;




        p(0) = 0.048516;
        if ((t*Dtime)<1.0)
        {
            p(1) = -0.079750;
        }
        if (((t*Dtime)>=1.0) && ((t*Dtime)<(wO+1)))
        {
            p(1) = -0.079750+0.038*(cos((1/wO)*M_PI*(t*Dtime-1.0))-1);
        }
        if ((t*Dtime)>=(wO+1))
        {
            p(1) = -0.079750-0.076*(cos((1/wO)*M_PI*(t*Dtime-(wO+1))));
        }

        p(2) = 0.884101;
        R = uLINK[Status->left_foot_ID].R;
        //pinv(R,uLINK[Base].R);
        taskCoML = R * p;
        CalcCoM(uLINK,CoM);
//taskCoML = p;
        taskCoML -= CoM;

//        gsl_vector_set (p, 2, 0);
//        gsl_vector_set_zero(CoP);
//        f=CalcCoP(uLINK,CoP,1);
//        if (f!=0.0 && t*Dtime>0.5)
//        {
//            CoP *= 1/f;
//            CoP(2) = 0;
//            p -= CoP;
//
//            dzmp = p;
//            dzmp -= zmp;
//            dzmp *= Te;
//            gsl_vector_scale (dzmp, 0.15);
//            taskCoML += dzmp;
//
//            zmp = p;
//            gsl_vector_scale(p,0.04);
//            gsl_vector_add(taskCoMR,p);
//            taskCoML += p;
//        }

//        PrintGSLVector(task2);
//        PrintGSLVector(taskCoMR);
//        PrintGSLVector(taskCoML);

//gsl_vector_scale(taskCoMR,0.5);
//gsl_vector_scale(taskCoML,0.5);

        if (Visualisation)
        {
            FILE *CoM_file=fopen("./../Simu_data/CoM.txt","a");
            for(n=0; n<3; n++)
            {
                fprintf(CoM_file,"%f ",CoM(n));
            }
            fprintf(CoM_file,"\n");
            fclose(CoM_file);

            FILE *CoMp_file=fopen("./../Simu_data/CoMp.txt","a");
            for(n=0; n<3; n++)
            {
                fprintf(CoMp_file,"%f ",p(n));
            }
            fprintf(CoMp_file,"\n");
            fclose(CoMp_file);

            FILE *CoP_file=fopen("./../Simu_data/CoP.txt","a");
            for(n=0; n<3; n++)
            {
                fprintf(CoP_file,"%f ",CoP(n));
            }
            fprintf(CoP_file,"\n");
            fclose(CoP_file);

            FILE *taskCoMR_file=fopen("./../Simu_data/taskCoMR.txt","a");
            for(n=0; n<3; n++)
            {
                fprintf(taskCoMR_file,"%f ",taskCoMR(n));
            }
            fprintf(taskCoMR_file,"\n");
            fclose(taskCoMR_file);

            FILE *taskCoML_file=fopen("./../Simu_data/taskCoML.txt","a");
            for(n=0; n<3; n++)
            {
                fprintf(taskCoML_file,"%f ",taskCoML(n));
            }
            fprintf(taskCoML_file,"\n");
            fclose(taskCoML_file);

        }

#endif

#if Ext_traj
        Ext_op_trajectory(opd, 0);

        R.setIdentity();
        p.setZero();
        p(0) = (opd[3]-opd[6]);
        p(1) = (opd[4]-opd[7]);
        p(2) = (opd[5]-opd[8]);
        //gsl_vector_scale(p,-1.0);
        CalcVWerrOri(uLINK, task2, p, R,idx2);

        taskCoML(0) = opd[0];
        taskCoML(1) = opd[1];
        taskCoML(2) = opd[2]-0.1;
        CalcCoM(uLINK,CoM);
        taskCoML -= CoM;



#endif

#if 0
        if ((t*Dtime)>=0.5)
        {
            Ext_op_trajectory_LIPM(opd, 0);
        }

        R.setIdentity();
        p.setZero();
        if ((t*Dtime)<0.5)
        {
            gsl_vector_set (p, 1, 0.0);
            p(1) = -0.1595;
            gsl_vector_set (p, 2, 0.0);
        }
        if (((t*Dtime)>=0.5))
        {
            p(0) = (opd[3]-opd[6]);
            p(1) = (opd[4]-opd[7]);
            gsl_vector_set (p, 2, (opd[5]-opd[8])+0.09*(opd[1]));
        }
        CalcVWerrOri(uLINK, task2, p, R,idx2);

        if ((t*Dtime)<0.5)
        {
            p(0) = 0.048516;
            p(1) = -0.079750;
            p(2) = 0.884101;
        }
        if (((t*Dtime)>=0.5))
        {
            gsl_vector_set (p, 0, (opd[0]-opd[6]));
            gsl_vector_set (p, 1, (opd[1]-opd[7]));
            gsl_vector_set (p, 2, (opd[2]-opd[8]));
        }
        R = uLINK[Status->right_foot_ID].R;
        //pinv(R,uLINK[Base].R);
        taskCoMR = R * p;
        CalcCoM(uLINK,CoM);
        //gsl_vector_memcpy(taskCoM,p);
        taskCoMR -= CoM;


        if ((t*Dtime)<0.5)
        {
            p(0) = 0.048516;
            p(1) = -0.079750;
            p(2) = 0.884101;
        }
        if (((t*Dtime)>=0.5))
        {
            gsl_vector_set (p, 0, (opd[0]-opd[6]));
            gsl_vector_set (p, 1, (opd[1]-opd[7]));
            gsl_vector_set (p, 2, (opd[2]-opd[8]));
        }
        R = uLINK[Status->left_foot_ID].R;
        //pinv(R,uLINK[Base].R);
        taskCoML = R * p;
        CalcCoM(uLINK,CoM);
        //gsl_vector_memcpy(taskCoM,p);
        taskCoML -= CoM;

//        gsl_vector_set (p, 2, 0);
//        gsl_vector_set_zero(CoP);
//        f=CalcCoP(uLINK,CoP,1);
//        if (f!=0.0 && t*Dtime>0.5)
//        {
//            CoP *= 1/f;
//            CoP(2) = 0;
//            p -= CoP;
//
//            dzmp = p;
//            dzmp -= zmp;
//            dzmp *= Te;
//            gsl_vector_scale (dzmp, 0.15);
//            taskCoML += dzmp;
//
//            zmp = p;
//            gsl_vector_scale(p,0.04);
//            taskCoML += p;
//        }

//PrintGSLVector(task2);
//PrintGSLVector(taskCoML);

#endif





        for (i=0; i<(nDoF-6); i++)
        {
            adphi(i) = -0.2*(2*(uLINK[i+2].q-uLINK[i+2].qmoy)/(qdev[i]*qdev[i]));
        }



        //PrintGSLVector(task1);
        //PrintGSLVector(task2);
        //PrintGSLVector(taskCoM);


//        P1.setIdentity();
//        pinv(invJ,J2);
//        Ptmp = invJ * J2;
//        P1 -= Ptmp;
//
//        gsl_matrix_set_identity(P2);
//        pinv(invJ,J2);
//        Ptmp = invJ * J2;
//        P2 -= Ptmp;
//
//        gsl_matrix_set_identity(PCoMR);
//        pinv(invJCoM,JCoMR);
//        gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, invJCoM, JCoMR, 0.0, Ptmp);
//        gsl_matrix_sub(PCoMR,Ptmp);
//
//        gsl_matrix_set_identity(PCoML);
//        pinv(invJCoM,JCoML);
//        gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, invJCoM, JCoML, 0.0, Ptmp);
//        gsl_matrix_sub(PCoML,Ptmp);


//MatPrintf( "Jf", "%4.6f " , Jf , 6, (nDoF-6)) ;
//printf(" \n \n");
//PrintGSLMatrix(J2);

//MatPrintf( "taskf", "%4.6f " , taskf , 6, 1) ;
//printf(" \n \n");
//PrintGSLVector(task2);

        // first task
        pinv(invJ,J2);
        dq = invJ * task2;


//MatPseudoInvf( invf , Jf , 6, (nDoF-6)) ;
//MatMulf( dqf , invf , taskf, (nDoF-6), 6, 1 ) ;

//MatPrintf( "invf", "%4.6f " , invf , (nDoF-6), 6) ;
////MatPrintf( "invf", "%4.6f " , invf , 6, (nDoF-6)) ;
//printf(" \n \n");
//PrintGSLMatrix(invJ);


//MatPrintf( "dqf", "%4.6f " , dqf , 1, (nDoF-6)) ;
//printf(" \n \n");
//PrintGSLVector(dq);




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
        //gsl_blas_dgemv(CblasNoTrans, 1.0, P2, dqtmp2, 0.0, dqtmp);
        dq += dqtmp;


        // third task
        P2 = P1;
        pinv(invJCoM,Jtilde);
        Ptmp = invJCoM * Jtilde;
        P2 -= Ptmp;

        vec3 = taskCoMR;
        vec3_2 = JCoMR * dq;
        vec3 -= vec3_2;

        Jtilde = JCoMR * P2;
        pinv(invJCoM,Jtilde);
        dqtmp = invJCoM * vec3;
        //gsl_blas_dgemv(CblasNoTrans, 1.0, P2, dqtmp2, 0.0, dqtmp);
        dq += dqtmp;


        // fourth task
        Ptilde.setIdentity();
        pinv(invJCoM,Jtilde);
        Ptmp = invJCoM * Jtilde;
        Ptilde -= Ptmp;

        dqtmp = Ptilde * adphi;
        dq += dqtmp;


//PrintGSLVector(dq);

        ddq_task = dq;
        ddq_task -= dq_old;
        ddq_task *= 1/Te;


        for (i=0; i<(nDoF-6); i++)
        {
            //uLINK[i+2].u_joint = 1000*dq(i)+10*ddq_task(i);
            uLINK[i+2].u_joint = 1000*dq(i)+10*ddq_task(i);
        }

        dq_old = dq;


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
//            uLINK[i+2].u_joint = kd*(0.0-uLINKc[n+2].dq)+kp*(kqd[i+1]-uLINKc[i+2].q);
//        }
//
//        free (kqd);



//        double kd=300;
//        double kp=100;
//        for(n=0; n<nDoF-6; n++)
//        {
//            uLINK[n+2].u_joint=kd*(gsl_vector_get(dq,n)*Te-uLINKc[n+2].dq)+kp*((uLINKc[n+2].q+gsl_vector_get(dq,n)*Te)-uLINKc[n+2].q);
//        }



//            printf("\n P1: \n");w_c = gsl_vector_calloc (3);
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
#if CMD_PD
                uLINK[n].u_joint = uPD[n-2]+uG[n-2]+uStab[n-2];
#endif
#if CMD_DYNAMIC
                uLINK[n].u_joint = uPD[n-2]+uG[n-2];
#endif

            }
            else
            {
#if CMD_PD
                uLINK[n].u_joint = uPD[n-2]+uStab[n-2];
#endif
#if CMD_DYNAMIC
                uLINK[n].u_joint = uPD[n-2]+uG[n-2];
#endif
            }
#if CMD_TASK


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
            u(n-2+6) = uLINK[n].u_joint;
        }

//PrintGSLVector(u);
//u.setZero();

    u += g;
    u -= ef;
    u -= b;
//    gsl_vector * ddq = gsl_vector_alloc (nDoF);
//    gsl_permutation * p = gsl_permutation_alloc (nDoF);
//    gsl_matrix * A2 = gsl_matrix_calloc (nDoF, nDoF);
    A2 = A;
    ddq = A2.partialPivLu().solve(u);
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
//
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
        uLINK[1].dvo(n) = ddq(n);
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
        uLINK[1].dw(n) = ddq(n+3);
    }
//uLINK(1).dw  = ddq(4:6);
    for (n=2; n<nDoF-6+2; n++)
    {
        uLINK[n].ddq = ddq(n+6-2);
    }
//for j=2:length(uLINK)
//    uLINK(j).ddq = ddq(j+6-1);
//end












}



