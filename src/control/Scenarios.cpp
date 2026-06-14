
#include "InverseSherpaKinematics.h"
#include "Spline.h"
#include "Scenarios.h"

#include "Setup.h"


#if Sc1
void Scenario_desired_trajectory(double *qd, double t, int *desired_support, double *distribution)
{

    double t1=1;
    double t2=5;
    double t3=9;

    if (t<t1)
    {
        double pos[6] = { 0.f , 0.f , -1.01729f ,
                         0.f , 0.f , -1.01729f
                       } ;// Posture in operational space
        *distribution=0.5f;	// contact force distribution
        *desired_support=3;	// Support foot: 0:none,1:right,2:left,3:both
        InverseSherpaKinematics(qd, pos);
        InverseSherpaKinematics(qd+6, pos+3);
    }

    if (t>=t1 && t<t2)
    {
        double pos[6] = { 0.f , 0.f , Spline((double)t,t1,t2,-1.01729f,-0.9f,0.f,0.f) ,
                         0.f , 0.f , Spline((double)t,t1,t2,-1.01729f,-0.9f,0.f,0.f)
                       } ;// Posture in operational space
        *distribution=0.5f;
        *desired_support=3;
        InverseSherpaKinematics(qd, pos);
        InverseSherpaKinematics(qd+6, pos+3);
    }

    if (t>=t2 && t<t3)
    {
        double pos[6] = { 0.f , 0.f , Spline((double)t,t2,t3,-0.9f,-1.01729f,0.f,0.f) ,
                         0.f , 0.f , Spline((double)t,t2,t3,-0.9f,-1.01729f,0.f,0.f)
                       } ;// Posture in operational space
        *distribution=0.5f;
        *desired_support=3;
        InverseSherpaKinematics(qd, pos);
        InverseSherpaKinematics(qd+6, pos+3);
    }

    if (t>=t3)
    {
        double pos[6] = { 0.f , 0.f , -1.01729f ,
                         0.f , 0.f , -1.01729f
                       } ;
        *distribution=0.5;
        *desired_support=3;
        InverseSherpaKinematics(qd, pos);
        InverseSherpaKinematics(qd+6, pos+3);
    }

}
#endif


#if Sc2
void Scenario_desired_trajectory(double *qd, double t, int *desired_support, double *distribution)
{

    double t1=1;
    double t2=5;
    double t3=9;
    double t4=13;
    double t5=17;


    double com_y =0.20f;



    if (t<t1)
    {
        double pos[6] = { 0.f , 0.f , -1.01729f ,
                         0.f , 0.f , -1.01729f
                       } ;// Posture in operational space
        *distribution=0.5f;	// contact force distribution
        *desired_support=3;	// Support foot: 0:none,1:right,2:left,3:both
        InverseSherpaKinematics(qd, pos);
        InverseSherpaKinematics(qd+6, pos+3);
    }

    if (t>=t1 && t<t2)
    {
        double pos[6] = { 0.f , 0.f , C2Spline((double)t,t1,(t1+t2)/2,t2,-1.01729f,-1.f,-0.95f,0.f,0.f) ,
                         0.f , 0.f , C2Spline((double)t,t1,(t1+t2)/2,t2,-1.01729f,-1.f,-0.95f,0.f,0.f)
                       } ;// Posture in operational space
        *distribution=0.5f;
        *desired_support=3;
        InverseSherpaKinematics(qd, pos);
        InverseSherpaKinematics(qd+6, pos+3);
    }

    if (t>=t2 && t<t3)
    {
        double pos[6] = { Spline((double)t,t2,t3,0.f,-0.05f,0.f,0.f) , Spline((double)t,t2,t3,0.f,com_y,0.f,0.f) , -0.95f ,
                         Spline((double)t,t2,t3,0.f,-0.05f,0.f,0.f) , Spline((double)t,t2,t3,0.f,com_y,0.f,0.f) , -0.95f
                       } ;
        *distribution=Spline((double)t,t2,t3,0.5f,0.f,0.f,0.f);
        *desired_support=3;
        InverseSherpaKinematics(qd, pos);
        InverseSherpaKinematics(qd+6, pos+3);
    }


    if (t>=t3 && t<t4)
    {
        double pos[6] = { Spline((double)t,t3,t4,-0.05f,0.f,0.f,0.f) , Spline((double)t,t3,t4,com_y,0.f,0.f,0.f) , -0.95f ,
                         Spline((double)t,t3,t4,-0.05f,0.f,0.f,0.f) , Spline((double)t,t3,t4,com_y,0.f,0.f,0.f) , -0.95f
                       } ;
        *distribution=Spline((double)t,t3,t4,0.f,0.5f,0.f,0.f);
        *desired_support=3;
        InverseSherpaKinematics(qd, pos);
        InverseSherpaKinematics(qd+6, pos+3);
    }


    if (t>=t4 && t<t5)
    {
        double pos[6] = { 0.f , 0.f , C2Spline((double)t,t4,(t4+t5)/2,t5,-0.95f,-1.f,-1.01729f,0.f,0.f) ,
                         0.f , 0.f , C2Spline((double)t,t4,(t4+t5)/2,t5,-0.95f,-1.f,-1.01729f,0.f,0.f)
                       } ;
        *distribution=0.5;
        *desired_support=3;
        InverseSherpaKinematics(qd, pos);
        InverseSherpaKinematics(qd+6, pos+3);
    }

    if (t>=t5)
    {
        double pos[6] = { 0.f , 0.f , -1.01729f ,
                         0.f , 0.f , -1.01729f
                       } ;
        *distribution=0.5;
        *desired_support=3;
        InverseSherpaKinematics(qd, pos);
        InverseSherpaKinematics(qd+6, pos+3);
    }

}
#endif



#if Sc3
void Scenario_desired_trajectory(double *qd, double t, int *desired_support, double *distribution)
{

    double t1=1;
    double t2=5;
    double t3=9;
    double temp;

    if (t<t1)
    {
//        double pos[6] = { 0.f , 0.f , -1.01729f ,
//                         0.f , 0.f , -1.01729f
//                       } ;// Posture in operational space
//        *distribution=0.5f;	// contact force distribution
//        *desired_support=3;	// Support foot: 0:none,1:right,2:left,3:both
//        InverseSherpaKinematics(qd, pos);
//        InverseSherpaKinematics(qd+6, pos+3);

        qd[1]=0;
        qd[4]=0;
        qd[7]=0;
        qd[10]=0;


        *distribution=0.5f;	// contact force distribution
        *desired_support=3;	// Support foot: 0:none,1:right,2:left,3:both
    }

    if (t>=t1 && t<t2)
    {
//        double pos[6] = { 0.f , 0.f , Spline((double)t,t1,t2,-1.01729f,-0.9f,0.f,0.f) ,
//                         0.f , 0.f , Spline((double)t,t1,t2,-1.01729f,-0.9f,0.f,0.f)
//                       } ;// Posture in operational space
//        *distribution=0.5f;
//        *desired_support=3;
//        InverseSherpaKinematics(qd, pos);
//        InverseSherpaKinematics(qd+6, pos+3);

        temp=Spline((double)t,t1,t2,0.f,.1f,0.f,0.f);
        qd[1]=temp;
        qd[4]=-temp;
        qd[7]=temp;
        qd[10]=-temp;

        *distribution=0.5f;	// contact force distribution
        *desired_support=3;	// Support foot: 0:none,1:right,2:left,3:both

    }

    if (t>=t2 && t<t3)
    {
//        double pos[6] = { 0.f , 0.f , Spline((double)t,t2,t3,-0.9f,-1.01729f,0.f,0.f) ,
//                         0.f , 0.f , Spline((double)t,t2,t3,-0.9f,-1.01729f,0.f,0.f)
//                       } ;// Posture in operational space
//        *distribution=0.5f;
//        *desired_support=3;
//        InverseSherpaKinematics(qd, pos);
//        InverseSherpaKinematics(qd+6, pos+3);
        temp=Spline((double)t,t2,t3,0.1f,0.02f,0.f,0.f);
        qd[1]=temp;
        qd[4]=-temp;
        qd[7]=temp;
        qd[10]=-temp;

        *distribution=0.5f;	// contact force distribution
        *desired_support=3;	// Support foot: 0:none,1:right,2:left,3:both

    }

    if (t>=t3)
    {
//        double pos[6] = { 0.f , 0.f , -1.01729f ,
//                         0.f , 0.f , -1.01729f
//                       } ;
//        *distribution=0.5;
//        *desired_support=3;
//        InverseSherpaKinematics(qd, pos);
//        InverseSherpaKinematics(qd+6, pos+3);

        //qd[1]=0;
        //qd[4]=0;
        //qd[7]=0;
        //qd[10]=0;


        *distribution=0.5f;	// contact force distribution
        *desired_support=3;	// Support foot: 0:none,1:right,2:left,3:both

    }

}
#endif



#if Sc4
void Scenario_desired_trajectory(double *qd, double t, int *desired_support, double *distribution)
{

    double t1=4;
    double t2=16;
    double t3=17;
    double t4=29;
    double t5=30;
    double t6=34;

    double temp;
    double offset=0.2f;
    double amplitude=0.2f;
    //double amplitude=0.25f;

    if (t<t1)
    {

        temp=Spline((double)t,0,t1,0.f,-offset,0.f,0.f);
        qd[RARM-1]=temp;
        qd[LARM-1]=temp;

        *distribution=0.5f;	// contact force distribution
        *desired_support=3;	// Support foot: 0:none,1:right,2:left,3:both
    }

    if (t>=t1 && t<t2)
    {

        temp=amplitude*(1-cos((t-t1)*(M_PI)));
        qd[RARM-1]=-temp-offset;
        qd[LARM-1]=-temp-offset;


        *distribution=0.5f;	// contact force distribution
        *desired_support=3;	// Support foot: 0:none,1:right,2:left,3:both

    }

    if (t>=t2 && t<t3)
    {

        temp=amplitude*(1-cos((t-t1)*(M_PI)));
        qd[RARM-1]=0-offset;
        qd[LARM-1]=-temp-offset;

        *distribution=0.5f;	// contact force distribution
        *desired_support=3;	// Support foot: 0:none,1:right,2:left,3:both

    }

    if (t>=t3 && t<t4)
    {

        temp=amplitude*(1-cos((t-t1)*(M_PI)));
        qd[RARM-1]=temp-2*amplitude-offset;
        qd[LARM-1]=-temp-offset;

        *distribution=0.5f;	// contact force distribution
        *desired_support=3;	// Support foot: 0:none,1:right,2:left,3:both

    }




    if (t>=t4 && t<t5)
    {

        temp=amplitude*(1-cos((t-t1)*(M_PI)));
        qd[RARM-1]=0-offset;
        qd[LARM-1]=-temp-offset;

        *distribution=0.5f;	// contact force distribution
        *desired_support=3;	// Support foot: 0:none,1:right,2:left,3:both

    }


    if (t>=t5 && t<t6)
    {

        temp=Spline((double)t,t5,t6,-offset,0.f,0.f,0.f);
        qd[RARM-1]=temp;
        qd[LARM-1]=temp;

        *distribution=0.5f;	// contact force distribution
        *desired_support=3;	// Support foot: 0:none,1:right,2:left,3:both

    }

    if (t>=t6)
    {

        qd[RARM-1]=0;
        qd[LARM-1]=0;

        *distribution=0.5f;	// contact force distribution
        *desired_support=3;	// Support foot: 0:none,1:right,2:left,3:both

    }

}
#endif




#if Sc5
void Scenario_desired_trajectory(double *qd, double t, int *desired_support, double *distribution)
{

    double t1=2;
    double t2=8;

    double temp;
    //double offset=0.2f;
    double amplitude=0.2f;
    //double amplitude=0.25f;

    if (t<t1)
    {

        temp=0;//Spline((double)t,0,t1,0.f,-offset,0.f,0.f);
        qd[SPINE-2]=temp;

        *distribution=0.5f;	// contact force distribution
        *desired_support=3;	// Support foot: 0:none,1:right,2:left,3:both
    }

    if (t>=t1 && t<t2)
    {

        temp=amplitude*(1-cos((t-t1)*(M_PI)));
        qd[SPINE-2]=temp;


        *distribution=0.5f;	// contact force distribution
        *desired_support=3;	// Support foot: 0:none,1:right,2:left,3:both

    }

    if (t>=t2)
    {

        qd[SPINE-2]=0;

        *distribution=0.5f;	// contact force distribution
        *desired_support=3;	// Support foot: 0:none,1:right,2:left,3:both

    }

}
#endif





#if Sc6
void Scenario_desired_trajectory(double *qd, double t, int *desired_support, double *distribution)
{

    double t1=2;
    double t2=8;

    double temp;
    double offset=0.2f;
    double amplitude=0.1f;
    //double amplitude=0.25f;

    if (t<t1)
    {

        temp=Spline((double)t,0,t1,0.f,offset,0.f,0.f);
        qd[SPINE-2]=temp;

        *distribution=0.5f;	// contact force distribution
        *desired_support=3;	// Support foot: 0:none,1:right,2:left,3:both
    }

    if (t>=t1)
    {

        temp=amplitude*(1-cos((t-t1)*(M_PI)));
        qd[SPINE-2]=amplitude*(1-cos((t-t1)*(M_PI)))+offset;

        qd[RARM-1]=-amplitude*(1-cos((t-t1)*(M_PI)*1.1));
        qd[LARM-1]=-amplitude*(1-cos((t-t1)*(M_PI)*1.2));

        *distribution=0.5f;	// contact force distribution
        *desired_support=3;	// Support foot: 0:none,1:right,2:left,3:both

    }


}
#endif







#if Sc7
void Scenario_desired_trajectory(double *qd, double t, int *desired_support, double *distribution)
{

    double t1=1;
    double t2=5;
    double t3=9;
    double temp;

    if (t<t1)
    {
//        double pos[6] = { 0.f , 0.f , -1.01729f ,
//                         0.f , 0.f , -1.01729f
//                       } ;// Posture in operational space
//        *distribution=0.5f;	// contact force distribution
//        *desired_support=3;	// Support foot: 0:none,1:right,2:left,3:both
//        InverseSherpaKinematics(qd, pos);
//        InverseSherpaKinematics(qd+6, pos+3);

        qd[0]=0;
        qd[5]=0;
        qd[6]=0;
        qd[11]=0;


        *distribution=0.5f;	// contact force distribution
        *desired_support=3;	// Support foot: 0:none,1:right,2:left,3:both
    }

    if (t>=t1 && t<t2)
    {
//        double pos[6] = { 0.f , 0.f , Spline((double)t,t1,t2,-1.01729f,-0.9f,0.f,0.f) ,
//                         0.f , 0.f , Spline((double)t,t1,t2,-1.01729f,-0.9f,0.f,0.f)
//                       } ;// Posture in operational space
//        *distribution=0.5f;
//        *desired_support=3;
//        InverseSherpaKinematics(qd, pos);
//        InverseSherpaKinematics(qd+6, pos+3);

        temp=Spline((double)t,t1,t2,0.f,.1f,0.f,0.f);
        qd[0]=-temp;
        qd[5]=temp;
        qd[6]=-temp;
        qd[11]=temp;

        *distribution=0.5f+tan(temp)*1.01729f;	// contact force distribution
        *desired_support=3;	// Support foot: 0:none,1:right,2:left,3:both

    }

    if (t>=t2 && t<t3)
    {
//        double pos[6] = { 0.f , 0.f , Spline((double)t,t2,t3,-0.9f,-1.01729f,0.f,0.f) ,
//                         0.f , 0.f , Spline((double)t,t2,t3,-0.9f,-1.01729f,0.f,0.f)
//                       } ;// Posture in operational space
//        *distribution=0.5f;
//        *desired_support=3;
//        InverseSherpaKinematics(qd, pos);
//        InverseSherpaKinematics(qd+6, pos+3);
        temp=Spline((double)t,t2,t3,0.1f,0.0f,0.f,0.f);
        qd[0]=-temp;
        qd[5]=temp;
        qd[6]=-temp;
        qd[11]=temp;

        *distribution=0.5f+tan(temp)*1.01729f;	// contact force distribution
        *desired_support=3;	// Support foot: 0:none,1:right,2:left,3:both

    }

    if (t>=t3)
    {
//        double pos[6] = { 0.f , 0.f , -1.01729f ,
//                         0.f , 0.f , -1.01729f
//                       } ;
//        *distribution=0.5;
//        *desired_support=3;
//        InverseSherpaKinematics(qd, pos);
//        InverseSherpaKinematics(qd+6, pos+3);

        qd[0]=0;
        qd[5]=0;
        qd[6]=0;
        qd[11]=0;


        *distribution=0.5f;	// contact force distribution
        *desired_support=3;	// Support foot: 0:none,1:right,2:left,3:both

    }

}
#endif



