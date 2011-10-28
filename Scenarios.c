
#include "InverseSherpaKinematics_f.h"
#include "Spline_f.h"
#include "Scenarios.h"

#include "Setup.h"


#if Sc1
void Scenario_desired_trajectory(float *qd, double t, int *desired_support, float *distribution)
{


    float t1=5;
    float t2=15;
    float t3=25;
    float t4=35;
    float t5=45;
    float t6=55;

//    float t1=5;
//    float t2=15;
//    float t3=15;
//    float t4=15;
//    float t5=15;
//    float t6=25;

//    float t1=0.5;
//    float t2=2;
//    float t3=5;
//    float t4=8;
//    float t5=11;
//    float t6=14;


    float com_y =0.210f;
    //float com_y =0.29;


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
        float pos[6] = { Spline_f((float)t,t2,t3,0.f,-0.10f,0.f,0.f) , Spline_f((float)t,t2,t3,0.f,com_y,0.f,0.f) , -0.95f ,
                         Spline_f((float)t,t2,t3,0.f,-0.10f,0.f,0.f) , Spline_f((float)t,t2,t3,0.f,com_y,0.f,0.f) , -0.95f
                       } ;
        *distribution=Spline_f((float)t,t2,t3,0.5f,0.f,0.035f,0.030f);
        *desired_support=3;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }

    if (t>=t3 && t<t4)
    {
        float pos[6] = { -0.10f , com_y , -0.95f ,
                         -0.10f , com_y , C2Spline_f((float)t,t3,(t3+t4)/2,t4,-0.95f,-0.90f,-0.95f,0.f,0.f)
                       } ;
        *distribution=0.f;
        *desired_support=1;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }


    if (t>=t4 && t<t5)
    {
        float pos[6] = { Spline_f((float)t,t4,t5,-0.10f,0.f,0.f,0.f) , Spline_f((float)t,t4,t5,com_y,0.f,0.f,0.f) , -0.95f ,
                         Spline_f((float)t,t4,t5,-0.10f,0.f,0.f,0.f) , Spline_f((float)t,t4,t5,com_y,0.f,0.f,0.f) , -0.95f
                       } ;
        *distribution=Spline_f((float)t,t4,t5,0.f,0.5f,-0.035f,-0.035f);
        *desired_support=3;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }


    if (t>=t5 && t<t6)
    {
        float pos[6] = { 0.f , 0.f , C2Spline_f((float)t,t5,(t5+t6)/2,t6,-0.95f,-1.f,-1.01729f,0.f,0.f) ,
                         0.f , 0.f , C2Spline_f((float)t,t5,(t5+t6)/2,t6,-0.95f,-1.f,-1.01729f,0.f,0.f)
                       } ;
        *distribution=0.5;
        *desired_support=3;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }

    if (t>=t6)
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

