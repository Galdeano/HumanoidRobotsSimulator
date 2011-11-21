
#include "InverseSherpaKinematics_f.h"
#include "Spline_f.h"
#include "Scenarios.h"
#include <gsl/gsl_math.h>

#include "Setup.h"


#if Sc1
void Scenario_desired_trajectory(float *qd, double t, int *desired_support, float *distribution)
{

    float t1=1;
    float t2=5;
    float t3=9;

    if (t<t1)
    {
        float pos[6] = { 0.f , 0.f , -1.01729f ,
                         0.f , 0.f , -1.01729f
                       } ;//Posture dans l'espace operationel
        *distribution=0.5f;	//repartition de l'effort de contact
        *desired_support=3;	//Pied de support: 0:none,1:right,2:left,3:both
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }

    if (t>=t1 && t<t2)
    {
        float pos[6] = { 0.f , 0.f , Spline_f((float)t,t1,t2,-1.01729f,-0.9f,0.f,0.f) ,
                         0.f , 0.f , Spline_f((float)t,t1,t2,-1.01729f,-0.9f,0.f,0.f)
                       } ;//Posture dans l'espace operationel
        *distribution=0.5f;
        *desired_support=3;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }

    if (t>=t2 && t<t3)
    {
        float pos[6] = { 0.f , 0.f , Spline_f((float)t,t2,t3,-0.9f,-1.01729f,0.f,0.f) ,
                         0.f , 0.f , Spline_f((float)t,t2,t3,-0.9f,-1.01729f,0.f,0.f)
                       } ;//Posture dans l'espace operationel
        *distribution=0.5f;
        *desired_support=3;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }

    if (t>=t3)
    {
        float pos[6] = { 0.f , 0.f , -1.01729f ,
                         0.f , 0.f , -1.01729f
                       } ;
        *distribution=0.5;
        *desired_support=3;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }

}
#endif


#if Sc2
void Scenario_desired_trajectory(float *qd, double t, int *desired_support, float *distribution)
{

    float t1=1;
    float t2=5;
    float t3=9;
    float t4=13;
    float t5=17;


    float com_y =0.20f;



    if (t<t1)
    {
        float pos[6] = { 0.f , 0.f , -1.01729f ,
                         0.f , 0.f , -1.01729f
                       } ;//Posture dans l'espace operationel
        *distribution=0.5f;	//repartition de l'effort de contact
        *desired_support=3;	//Pied de support: 0:none,1:right,2:left,3:both
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }

    if (t>=t1 && t<t2)
    {
        float pos[6] = { 0.f , 0.f , C2Spline_f((float)t,t1,(t1+t2)/2,t2,-1.01729f,-1.f,-0.95f,0.f,0.f) ,
                         0.f , 0.f , C2Spline_f((float)t,t1,(t1+t2)/2,t2,-1.01729f,-1.f,-0.95f,0.f,0.f)
                       } ;//Posture dans l'espace operationel
        *distribution=0.5f;
        *desired_support=3;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }

    if (t>=t2 && t<t3)
    {
        float pos[6] = { Spline_f((float)t,t2,t3,0.f,-0.05f,0.f,0.f) , Spline_f((float)t,t2,t3,0.f,com_y,0.f,0.f) , -0.95f ,
                         Spline_f((float)t,t2,t3,0.f,-0.05f,0.f,0.f) , Spline_f((float)t,t2,t3,0.f,com_y,0.f,0.f) , -0.95f
                       } ;
        *distribution=Spline_f((float)t,t2,t3,0.5f,0.f,0.f,0.f);
        *desired_support=3;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }


    if (t>=t3 && t<t4)
    {
        float pos[6] = { Spline_f((float)t,t3,t4,-0.05f,0.f,0.f,0.f) , Spline_f((float)t,t3,t4,com_y,0.f,0.f,0.f) , -0.95f ,
                         Spline_f((float)t,t3,t4,-0.05f,0.f,0.f,0.f) , Spline_f((float)t,t3,t4,com_y,0.f,0.f,0.f) , -0.95f
                       } ;
        *distribution=Spline_f((float)t,t3,t4,0.f,0.5f,0.f,0.f);
        *desired_support=3;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }


    if (t>=t4 && t<t5)
    {
        float pos[6] = { 0.f , 0.f , C2Spline_f((float)t,t4,(t4+t5)/2,t5,-0.95f,-1.f,-1.01729f,0.f,0.f) ,
                         0.f , 0.f , C2Spline_f((float)t,t4,(t4+t5)/2,t5,-0.95f,-1.f,-1.01729f,0.f,0.f)
                       } ;
        *distribution=0.5;
        *desired_support=3;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }

    if (t>=t5)
    {
        float pos[6] = { 0.f , 0.f , -1.01729f ,
                         0.f , 0.f , -1.01729f
                       } ;
        *distribution=0.5;
        *desired_support=3;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }

}
#endif



#if Sc3
void Scenario_desired_trajectory(float *qd, double t, int *desired_support, float *distribution)
{

    float t1=1;
    float t2=5;
    float t3=9;
    float temp;

    if (t<t1)
    {
//        float pos[6] = { 0.f , 0.f , -1.01729f ,
//                         0.f , 0.f , -1.01729f
//                       } ;//Posture dans l'espace operationel
//        *distribution=0.5f;	//repartition de l'effort de contact
//        *desired_support=3;	//Pied de support: 0:none,1:right,2:left,3:both
//        InverseSherpaKinematics_f(qd, pos);
//        InverseSherpaKinematics_f(qd+6, pos+3);

        qd[1]=0;
        qd[4]=0;
        qd[7]=0;
        qd[10]=0;


        *distribution=0.5f;	//repartition de l'effort de contact
        *desired_support=3;	//Pied de support: 0:none,1:right,2:left,3:both
    }

    if (t>=t1 && t<t2)
    {
//        float pos[6] = { 0.f , 0.f , Spline_f((float)t,t1,t2,-1.01729f,-0.9f,0.f,0.f) ,
//                         0.f , 0.f , Spline_f((float)t,t1,t2,-1.01729f,-0.9f,0.f,0.f)
//                       } ;//Posture dans l'espace operationel
//        *distribution=0.5f;
//        *desired_support=3;
//        InverseSherpaKinematics_f(qd, pos);
//        InverseSherpaKinematics_f(qd+6, pos+3);

        temp=Spline_f((float)t,t1,t2,0.f,.1f,0.f,0.f);
        qd[1]=temp;
        qd[4]=-temp;
        qd[7]=temp;
        qd[10]=-temp;

        *distribution=0.5f;	//repartition de l'effort de contact
        *desired_support=3;	//Pied de support: 0:none,1:right,2:left,3:both

    }

    if (t>=t2 && t<t3)
    {
//        float pos[6] = { 0.f , 0.f , Spline_f((float)t,t2,t3,-0.9f,-1.01729f,0.f,0.f) ,
//                         0.f , 0.f , Spline_f((float)t,t2,t3,-0.9f,-1.01729f,0.f,0.f)
//                       } ;//Posture dans l'espace operationel
//        *distribution=0.5f;
//        *desired_support=3;
//        InverseSherpaKinematics_f(qd, pos);
//        InverseSherpaKinematics_f(qd+6, pos+3);
        temp=Spline_f((float)t,t2,t3,0.1f,0.02f,0.f,0.f);
        qd[1]=temp;
        qd[4]=-temp;
        qd[7]=temp;
        qd[10]=-temp;

        *distribution=0.5f;	//repartition de l'effort de contact
        *desired_support=3;	//Pied de support: 0:none,1:right,2:left,3:both

    }

    if (t>=t3)
    {
//        float pos[6] = { 0.f , 0.f , -1.01729f ,
//                         0.f , 0.f , -1.01729f
//                       } ;
//        *distribution=0.5;
//        *desired_support=3;
//        InverseSherpaKinematics_f(qd, pos);
//        InverseSherpaKinematics_f(qd+6, pos+3);

        //qd[1]=0;
        //qd[4]=0;
        //qd[7]=0;
        //qd[10]=0;


        *distribution=0.5f;	//repartition de l'effort de contact
        *desired_support=3;	//Pied de support: 0:none,1:right,2:left,3:both

    }

}
#endif



#if Sc4
void Scenario_desired_trajectory(float *qd, double t, int *desired_support, float *distribution)
{

    float t1=4;
    float t2=16;
    float t3=17;
    float t4=29;
    float t5=30;
    float t6=34;

    float temp;
    float offset=0.2f;
    float amplitude=0.2f;
    //float amplitude=0.25f;

    if (t<t1)
    {

        temp=Spline_f((float)t,0,t1,0.f,-offset,0.f,0.f);
        qd[RARM-1]=temp;
        qd[LARM-1]=temp;

        *distribution=0.5f;	//repartition de l'effort de contact
        *desired_support=3;	//Pied de support: 0:none,1:right,2:left,3:both
    }

    if (t>=t1 && t<t2)
    {

        temp=amplitude*(1-cos((t-t1)*(M_PI)));
        qd[RARM-1]=-temp-offset;
        qd[LARM-1]=-temp-offset;


        *distribution=0.5f;	//repartition de l'effort de contact
        *desired_support=3;	//Pied de support: 0:none,1:right,2:left,3:both

    }

    if (t>=t2 && t<t3)
    {

        temp=amplitude*(1-cos((t-t1)*(M_PI)));
        qd[RARM-1]=0-offset;
        qd[LARM-1]=-temp-offset;

        *distribution=0.5f;	//repartition de l'effort de contact
        *desired_support=3;	//Pied de support: 0:none,1:right,2:left,3:both

    }

    if (t>=t3 && t<t4)
    {

        temp=amplitude*(1-cos((t-t1)*(M_PI)));
        qd[RARM-1]=temp-2*amplitude-offset;
        qd[LARM-1]=-temp-offset;

        *distribution=0.5f;	//repartition de l'effort de contact
        *desired_support=3;	//Pied de support: 0:none,1:right,2:left,3:both

    }




    if (t>=t4 && t<t5)
    {

        temp=amplitude*(1-cos((t-t1)*(M_PI)));
        qd[RARM-1]=0-offset;
        qd[LARM-1]=-temp-offset;

        *distribution=0.5f;	//repartition de l'effort de contact
        *desired_support=3;	//Pied de support: 0:none,1:right,2:left,3:both

    }


    if (t>=t5 && t<t6)
    {

        temp=Spline_f((float)t,t5,t6,-offset,0.f,0.f,0.f);
        qd[RARM-1]=temp;
        qd[LARM-1]=temp;

        *distribution=0.5f;	//repartition de l'effort de contact
        *desired_support=3;	//Pied de support: 0:none,1:right,2:left,3:both

    }

    if (t>=t6)
    {

        qd[RARM-1]=0;
        qd[LARM-1]=0;

        *distribution=0.5f;	//repartition de l'effort de contact
        *desired_support=3;	//Pied de support: 0:none,1:right,2:left,3:both

    }

}
#endif




#if Sc5
void Scenario_desired_trajectory(float *qd, double t, int *desired_support, float *distribution)
{

    float t1=2;
    float t2=8;

    float temp;
    //float offset=0.2f;
    float amplitude=0.2f;
    //float amplitude=0.25f;

    if (t<t1)
    {

        temp=0;//Spline_f((float)t,0,t1,0.f,-offset,0.f,0.f);
        qd[SPINE-2]=temp;

        *distribution=0.5f;	//repartition de l'effort de contact
        *desired_support=3;	//Pied de support: 0:none,1:right,2:left,3:both
    }

    if (t>=t1 && t<t2)
    {

        temp=amplitude*(1-cos((t-t1)*(M_PI)));
        qd[SPINE-2]=temp;


        *distribution=0.5f;	//repartition de l'effort de contact
        *desired_support=3;	//Pied de support: 0:none,1:right,2:left,3:both

    }

    if (t>=t2)
    {

        qd[SPINE-2]=0;

        *distribution=0.5f;	//repartition de l'effort de contact
        *desired_support=3;	//Pied de support: 0:none,1:right,2:left,3:both

    }

}
#endif





#if Sc6
void Scenario_desired_trajectory(float *qd, double t, int *desired_support, float *distribution)
{

    float t1=2;
    float t2=8;

    float temp;
    float offset=0.2f;
    float amplitude=0.1f;
    //float amplitude=0.25f;

    if (t<t1)
    {

        temp=Spline_f((float)t,0,t1,0.f,offset,0.f,0.f);
        qd[SPINE-2]=temp;

        *distribution=0.5f;	//repartition de l'effort de contact
        *desired_support=3;	//Pied de support: 0:none,1:right,2:left,3:both
    }

    if (t>=t1)
    {

        temp=amplitude*(1-cos((t-t1)*(M_PI)));
        qd[SPINE-2]=amplitude*(1-cos((t-t1)*(M_PI)))+offset;

        qd[RARM-1]=-amplitude*(1-cos((t-t1)*(M_PI)*1.1));
        qd[LARM-1]=-amplitude*(1-cos((t-t1)*(M_PI)*1.2));

        *distribution=0.5f;	//repartition de l'effort de contact
        *desired_support=3;	//Pied de support: 0:none,1:right,2:left,3:both

    }


}
#endif







#if Sc7
void Scenario_desired_trajectory(float *qd, double t, int *desired_support, float *distribution)
{

    float t1=1;
    float t2=5;
    float t3=9;
    float temp;

    if (t<t1)
    {
//        float pos[6] = { 0.f , 0.f , -1.01729f ,
//                         0.f , 0.f , -1.01729f
//                       } ;//Posture dans l'espace operationel
//        *distribution=0.5f;	//repartition de l'effort de contact
//        *desired_support=3;	//Pied de support: 0:none,1:right,2:left,3:both
//        InverseSherpaKinematics_f(qd, pos);
//        InverseSherpaKinematics_f(qd+6, pos+3);

        qd[0]=0;
        qd[5]=0;
        qd[6]=0;
        qd[11]=0;


        *distribution=0.5f;	//repartition de l'effort de contact
        *desired_support=3;	//Pied de support: 0:none,1:right,2:left,3:both
    }

    if (t>=t1 && t<t2)
    {
//        float pos[6] = { 0.f , 0.f , Spline_f((float)t,t1,t2,-1.01729f,-0.9f,0.f,0.f) ,
//                         0.f , 0.f , Spline_f((float)t,t1,t2,-1.01729f,-0.9f,0.f,0.f)
//                       } ;//Posture dans l'espace operationel
//        *distribution=0.5f;
//        *desired_support=3;
//        InverseSherpaKinematics_f(qd, pos);
//        InverseSherpaKinematics_f(qd+6, pos+3);

        temp=Spline_f((float)t,t1,t2,0.f,.1f,0.f,0.f);
        qd[0]=-temp;
        qd[5]=temp;
        qd[6]=-temp;
        qd[11]=temp;

        *distribution=0.5f+tan(temp)*1.01729f;	//repartition de l'effort de contact
        *desired_support=3;	//Pied de support: 0:none,1:right,2:left,3:both

    }

    if (t>=t2 && t<t3)
    {
//        float pos[6] = { 0.f , 0.f , Spline_f((float)t,t2,t3,-0.9f,-1.01729f,0.f,0.f) ,
//                         0.f , 0.f , Spline_f((float)t,t2,t3,-0.9f,-1.01729f,0.f,0.f)
//                       } ;//Posture dans l'espace operationel
//        *distribution=0.5f;
//        *desired_support=3;
//        InverseSherpaKinematics_f(qd, pos);
//        InverseSherpaKinematics_f(qd+6, pos+3);
        temp=Spline_f((float)t,t2,t3,0.1f,0.0f,0.f,0.f);
        qd[0]=-temp;
        qd[5]=temp;
        qd[6]=-temp;
        qd[11]=temp;

        *distribution=0.5f+tan(temp)*1.01729f;	//repartition de l'effort de contact
        *desired_support=3;	//Pied de support: 0:none,1:right,2:left,3:both

    }

    if (t>=t3)
    {
//        float pos[6] = { 0.f , 0.f , -1.01729f ,
//                         0.f , 0.f , -1.01729f
//                       } ;
//        *distribution=0.5;
//        *desired_support=3;
//        InverseSherpaKinematics_f(qd, pos);
//        InverseSherpaKinematics_f(qd+6, pos+3);

        qd[0]=0;
        qd[5]=0;
        qd[6]=0;
        qd[11]=0;


        *distribution=0.5f;	//repartition de l'effort de contact
        *desired_support=3;	//Pied de support: 0:none,1:right,2:left,3:both

    }

}
#endif



