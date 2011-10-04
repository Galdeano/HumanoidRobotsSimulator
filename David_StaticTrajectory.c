//#include "d:\Mb\StLib\Mat.h"
#include "David_InverseSherpaKinematics.h"
#include "David_Spline.h"
#include "David_StaticTrajectory.h"

#include "Setup.h"

void David_StaticTrajectory(float *qd, double t, int *desired_support, float *distribution)
{
//   int n;

    float t1=2;
    float t2=5;
    float t3=15;
    float t4=25;
    float t5=35;
    float t6=45;
    float t7=55;
    float t8=65;
    float t9=75;
    float t10=85;
    float t11=95;
    float t12=105;

    /*
        float t1=2;
        float t2=5;
        float t3=10;
        float t4=15;
        float t5=20;
        float t6=25;
        float t7=30;
        float t8=35;
        float t9=40;
        float t10=45;
        float t11=50;
        float t12=55;
        */

//    float t1=0.5;
//    float t2=2;
//    float t3=6;
//    float t4=10;
//    float t5=14;
//    float t6=18;
//    float t7=22;
//    float t8=26;

//    float t1=0.5;
//    float t2=2;
//    float t3=5;
//    float t4=8;
//    float t5=11;
//    float t6=14;
//    float t7=17;
//    float t8=20;
//    float t9=23;
//    float t10=26;
//    float t11=29;
//    float t12=32;

    float com_y =0.29f;
    //float com_y =0.29;
    float adjust_x =-0.03f;
    //float steep_x =0.05;


//        while(t>=t8) //Repete le pas
//        {
//            t -= t8-t4;
//        }





    if (t<t1)
    {
        float pos[6] = { 0.f , 0.f , -1.01729f ,
                         0.f , 0.f , -1.01729f
                       } ;//Posture dans l'espace operationel
        *distribution=0.5;	//repartition de l'effort de contact
        *desired_support=3;	//Pied de support: 0:none,1:right,2:left,3:both
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }

    if (t>=t1 && t<t2)
    {
        float pos[6] = { 0.f , 0.f , David_C2Spline((float)t,t1,(t1+t2)/2,t2,-1.01729f,-1.f,-0.95f,0.f,0.f) ,
                         0.f , 0.f , David_C2Spline((float)t,t1,(t1+t2)/2,t2,-1.01729f,-1.f,-0.95f,0.f,0.f)
                       } ;//Posture dans l'espace operationel
        *distribution=0.5f;
        *desired_support=3;
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }

    if (t>=t2 && t<t3)
    {
        float pos[6] = { David_Spline((float)t,t2,t3,0.f,-0.05f,0.f,0.f) , David_Spline((float)t,t2,t3,0.f,com_y,0.f,0.f) , -0.95f ,
                         David_Spline((float)t,t2,t3,0.f,-0.05f,0.f,0.f) , David_Spline((float)t,t2,t3,0.f,com_y,0.f,0.f) , -0.95f
                       } ;
        *distribution=David_Spline((float)t,t2,t3,0.5f,0.f,0.f,0.f);
        *desired_support=3;
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }

    if (t>=t3 && t<t4)
    {
        float pos[6] = { -0.05f+David_Spline((float)t,t3,t4,0.f,-adjust_x,0.f,0.f) , com_y , -0.95f ,
                         David_Spline((float)t,t3,t4,-0.05f,0.05f,0.f,0.f)+David_Spline((float)t,t3,t4,0.f,-adjust_x,0.f,0.f) , com_y , David_C2Spline((float)t,t3,(t3+t4)/2,t4,-0.95f,-0.9f,-0.95f,0.f,0.f)
                       } ;
        *distribution=0.f;
        *desired_support=1;
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }


    if (t>=t4 && t<t5)
    {
        float pos[6] = { David_Spline((float)t,t4,t5,-0.05f,-0.15f,0.f,0.f)+David_Spline((float)t,t4,t5,-adjust_x,adjust_x,0.f,0.f) , David_Spline((float)t,t4,t5,com_y,-com_y,0.f,0.f) , -0.95f ,
                         David_Spline((float)t,t4,t5, 0.05f,-0.05f,0.f,0.f)+David_Spline((float)t,t4,t5,-adjust_x,adjust_x,0.f,0.f) , David_Spline((float)t,t4,t5,com_y,-com_y,0.f,0.f) , -0.95f
                       } ;
        *distribution=David_Spline((float)t,t4,t5,0.f,1.f,0.f,0.f);
        *desired_support=3;
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }


    if (t>=t5 && t<t6)
    {
        float pos[6] = { David_Spline((float)t,t5,t6,-0.15f,0.05f,0.f,0.f)+David_Spline((float)t,t5,t6,adjust_x,-adjust_x,0.f,0.f) , -com_y , David_C2Spline((float)t,t5,(t5+t6)/2,t6,-0.95f,-0.9f,-0.95f,0.f,0.f) ,
                         -0.05f+David_Spline((float)t,t5,t6,adjust_x,-adjust_x,0.f,0.f) , -com_y , -0.95f
                       } ;
        *distribution=1.f;
        *desired_support=2;
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }


    if (t>=t6 && t<t7)
    {
        float pos[6] = { David_Spline((float)t,t6,t7, 0.05f,-0.05f,0.f,0.f)+David_Spline((float)t,t6,t7,-adjust_x,adjust_x,0.f,0.f) , David_Spline((float)t,t6,t7,-com_y,com_y,0.f,0.f) , -0.95f ,
                         David_Spline((float)t,t6,t7,-0.05f,-0.15f,0.f,0.f)+David_Spline((float)t,t6,t7,-adjust_x,adjust_x,0.f,0.f) , David_Spline((float)t,t6,t7,-com_y,com_y,0.f,0.f) , -0.95f
                       } ;
        *distribution=David_Spline((float)t,t6,t7,1.f,0.f,0.f,0.f);
        *desired_support=3;
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }


    if (t>=t7 && t<t8)
    {
        float pos[6] = { -0.05f+David_Spline((float)t,t7,t8,adjust_x,-adjust_x,0.f,0.f) , com_y , -0.95f ,
                         David_Spline((float)t,t7,t8,-0.15f,0.05f,0.f,0.f)+David_Spline((float)t,t7,t8,adjust_x,-adjust_x,0.f,0.f) , com_y , David_C2Spline((float)t,t7,(t7+t8)/2,t8,-0.95f,-0.9f,-0.95f,0.f,0.f)
                       } ;
        *distribution=0.f;
        *desired_support=1;
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }


    if (t>=t8 && t<t9)
    {
        float pos[6] = { David_Spline((float)t,t8,t9,-0.05f,-0.15f,0.f,0.f)+David_Spline((float)t,t8,t9,-adjust_x,adjust_x,0.f,0.f) , David_Spline((float)t,t8,t9,com_y,-com_y,0.f,0.f) , -0.95f ,
                         David_Spline((float)t,t8,t9, 0.05f,-0.05f,0.f,0.f)+David_Spline((float)t,t8,t9,-adjust_x,adjust_x,0.f,0.f) , David_Spline((float)t,t8,t9,com_y,-com_y,0.f,0.f) , -0.95f
                       } ;
        *distribution=David_Spline((float)t,t8,t9,0.f,1.f,0.f,0.f);
        *desired_support=3;
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }


    if (t>=t9 && t<t10)
    {
        float pos[6] = { David_Spline((float)t,t9,t10,-0.15f,-0.05f,0.f,0.f)+David_Spline((float)t,t9,t10,adjust_x,0.f,0.f,0.f) , -com_y , David_C2Spline((float)t,t9,(t9+t10)/2,t10,-0.95f,-0.9f,-0.95f,0.f,0.f) ,
                         -0.05f+David_Spline((float)t,t9,t10,adjust_x,0.f,0.f,0.f) , -com_y , -0.95f
                       } ;
        *distribution=1.f;
        *desired_support=2;
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }


    if (t>=t10 && t<t11)
    {
        float pos[6] = { David_Spline((float)t,t10,t11,-0.05f,0.f,0.f,0.f) , David_Spline((float)t,t10,t11,-com_y,0.f,0.f,0.f) , -0.95f ,
                         David_Spline((float)t,t10,t11,-0.05f,0.f,0.f,0.f) , David_Spline((float)t,t10,t11,-com_y,0.f,0.f,0.f) , -0.95f
                       } ;
        *distribution=David_Spline((float)t,t10,t11,1.f,0.5f,0.f,0.f);
        *desired_support=3;
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }


    if (t>=t11 && t<t12)
    {
        float pos[6] = { 0.f , 0.f , David_C2Spline((float)t,t11,(t11+t12)/2,t12,-0.95f,-1.f,-1.01729f,0.f,0.f) ,
                         0.f , 0.f , David_C2Spline((float)t,t11,(t11+t12)/2,t12,-0.95f,-1.f,-1.01729f,0.f,0.f)
                       };
        *distribution=0.5f;
        *desired_support=3;
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }



    //David_InverseSherpaKinematics(qd, pos);

    //David_InverseSherpaKinematics(qd+6, pos+3);


}



#if Sherpa
void David_OneFoot(float *qd, double t, int *desired_support, float *distribution)
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


    float com_y =0.29f;
    //float com_y =0.29;


    if (t<t1)
    {
        float pos[6] = { 0.f , 0.f , -1.01729f ,
                         0.f , 0.f , -1.01729f
                       } ;//Posture dans l'espace operationel
        *distribution=0.5f;	//repartition de l'effort de contact
        *desired_support=3;	//Pied de support: 0:none,1:right,2:left,3:both
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }

    if (t>=t1 && t<t2)
    {
        float pos[6] = { 0.f , 0.f , David_C2Spline((float)t,t1,(t1+t2)/2,t2,-1.01729f,-1.f,-0.95f,0.f,0.f) ,
                         0.f , 0.f , David_C2Spline((float)t,t1,(t1+t2)/2,t2,-1.01729f,-1.f,-0.95f,0.f,0.f)
                       } ;//Posture dans l'espace operationel
        *distribution=0.5f;
        *desired_support=3;
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }

    if (t>=t2 && t<t3)
    {
        float pos[6] = { David_Spline((float)t,t2,t3,0.f,-0.05f,0.f,0.f) , David_Spline((float)t,t2,t3,0.f,com_y,0.f,0.f) , -0.95f ,
                         David_Spline((float)t,t2,t3,0.f,-0.05f,0.f,0.f) , David_Spline((float)t,t2,t3,0.f,com_y,0.f,0.f) , -0.95f
                       } ;
        *distribution=David_Spline((float)t,t2,t3,0.5f,0.f,0.f,0.f);
        *desired_support=3;
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }

    if (t>=t3 && t<t4)
    {
        float pos[6] = { -0.05f , com_y , -0.95f ,
                         -0.05f , com_y , David_C2Spline((float)t,t3,(t3+t4)/2,t4,-0.95f,-0.90f,-0.95f,0.f,0.f)
                       } ;
        *distribution=0.f;
        *desired_support=1;
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }


    if (t>=t4 && t<t5)
    {
        float pos[6] = { David_Spline((float)t,t4,t5,-0.05f,0.f,0.f,0.f) , David_Spline((float)t,t4,t5,com_y,0.f,0.f,0.f) , -0.95f ,
                         David_Spline((float)t,t4,t5,-0.05f,0.f,0.f,0.f) , David_Spline((float)t,t4,t5,com_y,0.f,0.f,0.f) , -0.95f
                       } ;
        *distribution=David_Spline((float)t,t4,t5,0.f,0.5f,0.f,0.f);
        *desired_support=3;
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }


    if (t>=t5 && t<t6)
    {
        float pos[6] = { 0.f , 0.f , David_C2Spline((float)t,t5,(t5+t6)/2,t6,-0.95f,-1.f,-1.01729f,0.f,0.f) ,
                         0.f , 0.f , David_C2Spline((float)t,t5,(t5+t6)/2,t6,-0.95f,-1.f,-1.01729f,0.f,0.f)
                       } ;
        *distribution=0.5;
        *desired_support=3;
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }

    if (t>=t6)
    {
        float pos[6] = { 0.f , 0.f , -1.01729f ,
                         0.f , 0.f , -1.01729f
                       } ;
        *distribution=0.5;
        *desired_support=3;
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }

}
#endif

#if Generic
void David_OneFoot(float *qd, double t, int *desired_support, float *distribution)
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


    float com_y =0.215f;
    //float com_y =0.29;


    if (t<t1)
    {
        float pos[6] = { 0.f , 0.f , -1.01729f ,
                         0.f , 0.f , -1.01729f
                       } ;//Posture dans l'espace operationel
        *distribution=0.5f;	//repartition de l'effort de contact
        *desired_support=3;	//Pied de support: 0:none,1:right,2:left,3:both
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }

    if (t>=t1 && t<t2)
    {
        float pos[6] = { 0.f , 0.f , David_C2Spline((float)t,t1,(t1+t2)/2,t2,-1.01729f,-1.f,-0.95f,0.f,0.f) ,
                         0.f , 0.f , David_C2Spline((float)t,t1,(t1+t2)/2,t2,-1.01729f,-1.f,-0.95f,0.f,0.f)
                       } ;//Posture dans l'espace operationel
        *distribution=0.5f;
        *desired_support=3;
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }

    if (t>=t2 && t<t3)
    {
        float pos[6] = { David_Spline((float)t,t2,t3,0.f,-0.10f,0.f,0.f) , David_Spline((float)t,t2,t3,0.f,com_y,0.f,0.f) , -0.95f ,
                         David_Spline((float)t,t2,t3,0.f,-0.10f,0.f,0.f) , David_Spline((float)t,t2,t3,0.f,com_y,0.f,0.f) , -0.95f
                       } ;
        *distribution=David_Spline((float)t,t2,t3,0.5f,0.f,0.035f,0.035f);
        *desired_support=3;
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }

    if (t>=t3 && t<t4)
    {
        float pos[6] = { -0.10f , com_y , -0.95f ,
                         -0.10f , com_y , David_C2Spline((float)t,t3,(t3+t4)/2,t4,-0.95f,-0.90f,-0.95f,0.f,0.f)
                       } ;
        *distribution=0.f;
        *desired_support=1;
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }


    if (t>=t4 && t<t5)
    {
        float pos[6] = { David_Spline((float)t,t4,t5,-0.10f,0.f,0.f,0.f) , David_Spline((float)t,t4,t5,com_y,0.f,0.f,0.f) , -0.95f ,
                         David_Spline((float)t,t4,t5,-0.10f,0.f,0.f,0.f) , David_Spline((float)t,t4,t5,com_y,0.f,0.f,0.f) , -0.95f
                       } ;
        *distribution=David_Spline((float)t,t4,t5,0.f,0.5f,-0.035f,-0.035f);
        *desired_support=3;
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }


    if (t>=t5 && t<t6)
    {
        float pos[6] = { 0.f , 0.f , David_C2Spline((float)t,t5,(t5+t6)/2,t6,-0.95f,-1.f,-1.01729f,0.f,0.f) ,
                         0.f , 0.f , David_C2Spline((float)t,t5,(t5+t6)/2,t6,-0.95f,-1.f,-1.01729f,0.f,0.f)
                       } ;
        *distribution=0.5;
        *desired_support=3;
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }

    if (t>=t6)
    {
        float pos[6] = { 0.f , 0.f , -1.01729f ,
                         0.f , 0.f , -1.01729f
                       } ;
        *distribution=0.5;
        *desired_support=3;
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }

}
#endif



void David_GravityExperiment(float *qd, double t, int *desired_support, float *distribution)
{
//   int n;
    /*
        float t1=2;
        float t2=5;
        float t3=15;
        float t4=25;
        float t5=35;
        float t6=45;
        float t7=55;
        float t8=65;
        float t9=75;
        float t10=85;
        float t11=95;
        float t12=105;
        */


    float t1=2;
    float t2=5;
    float t3=10;
    float t4=15;
    float t5=20;
    float t6=25;
    float t7=30;
    float t8=35;
    float t9=40;
    float t10=45;
    float t11=50;
    float t12=55;


//    float t1=0.5;
//    float t2=2;
//    float t3=6;
//    float t4=10;
//    float t5=14;
//    float t6=18;
//    float t7=22;
//    float t8=26;

//    float t1=0.5;
//    float t2=2;
//    float t3=5;
//    float t4=8;
//    float t5=11;
//    float t6=14;
//    float t7=17;
//    float t8=20;


    float com_y =0.10f;
    //float com_y =0.29;

    *distribution=0.5f;	//repartition de l'effort de contact
    *desired_support=0;	//Pied de support: 0:none,1:right,2:left,3:both

    if (t<t1)
    {
        float pos[6] = { 0.f , 0.f , -1.01729f ,
                         0.f , 0.f , -1.01729f
                       } ;//Posture dans l'espace operationel
        *distribution=0.5f;	//repartition de l'effort de contact
        *desired_support=0;	//Pied de support: 0:none,1:right,2:left,3:both
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }

    if (t>=t1 && t<t2)
    {
        float pos[6] = { David_Spline((float)t,t1,t2,0.f,-0.1f,0.f,0.f) , 0.f , David_C2Spline((float)t,t1,(t1+t2)/2,t2,-1.01729f,-1.f,-0.95f,0.f,0.f) ,
                         David_Spline((float)t,t1,t2,0.f,-0.1f,0.f,0.f) , 0.f , David_C2Spline((float)t,t1,(t1+t2)/2,t2,-1.01729f,-1.f,-0.95f,0.f,0.f)
                       } ;//Posture dans l'espace operationel
        *distribution=0.5f;
        *desired_support=0;
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }

    if (t>=t2 && t<t3)
    {
        float pos[6] = { David_Spline((float)t,t2,t3,-0.1f,-0.25f,0.f,0.f) , David_Spline((float)t,t2,t3,0.f,com_y,0.f,0.f) , -0.95f ,
                         David_Spline((float)t,t2,t3,-0.1f,-0.25f,0.f,0.f) , David_Spline((float)t,t2,t3,0.f,com_y,0.f,0.f) , -0.95f
                       } ;
        *distribution=David_Spline((float)t,t2,t3,0.5f,0.f,0.f,0.f);
        *desired_support=0;
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }

    if (t>=t3 && t<t4)
    {
        float pos[6] = { -0.25f , com_y , -0.95f ,
                         -0.25f , com_y , -0.95f
                       } ;
        *distribution=0.f;
        *desired_support=0;
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }


    if (t>=t4 && t<t5)
    {
        float pos[6] = { David_Spline((float)t,t4,t5,-0.25f,-0.35f,0.f,0.f) , David_Spline((float)t,t4,t5,com_y,0.f,0.f,0.f) , -0.95f ,
                         David_Spline((float)t,t4,t5,-0.25f,-0.15f,0.f,0.f) , David_Spline((float)t,t4,t5,com_y,0.f,0.f,0.f) , -0.95f
                       } ;
        *distribution=David_Spline((float)t,t4,t5,0.f,1.f,0.f,0.f);
        *desired_support=0;
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }


    if (t>=t5 && t<t6)
    {
        float pos[6] = { -0.35f , 0.0f , -0.95f ,
                         -0.15f , 0.0f , -0.95f
                       } ;
        *distribution=1.f;
        *desired_support=0;
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }


    if (t>=t6 && t<t7)
    {
        float pos[6] = { David_Spline((float)t,t6,t7,-0.35f,-0.25f,0.f,0.f) , David_Spline((float)t,t6,t7,0.f,-com_y,0.f,0.f) , -0.95f ,
                         David_Spline((float)t,t6,t7,-0.15f,-0.25f,0.f,0.f) , David_Spline((float)t,t6,t7,0.f,-com_y,0.f,0.f) , -0.95f
                       } ;
        *distribution=David_Spline((float)t,t6,t7,1.f,0.f,0.f,0.f);
        *desired_support=0;
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }


    if (t>=t7 && t<t8)
    {
        float pos[6] = { -0.25f , -com_y , -0.95f ,
                         -0.25f , -com_y , -0.95f
                       } ;
        *distribution=0.f;
        *desired_support=0;
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }


    if (t>=t8 && t<t9)
    {
        float pos[6] = { David_Spline((float)t,t8,t9,-0.25f,-0.15f,0.f,0.f) , David_Spline((float)t,t8,t9,-com_y,0.f,0.f,0.f) , -0.95f ,
                         David_Spline((float)t,t8,t9,-0.25f,-0.35f,0.f,0.f) , David_Spline((float)t,t8,t9,-com_y,0.f,0.f,0.f) , -0.95f
                       } ;
        *distribution=David_Spline((float)t,t8,t9,0.f,1.f,0.f,0.f);
        *desired_support=0;
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }


    if (t>=t9 && t<t10)
    {
        float pos[6] = { -0.15f , 0.0f , -0.95f ,
                         -0.35f , 0.0f , -0.95f
                       } ;
        *distribution=1.f;
        *desired_support=0;
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }


    if (t>=t10 && t<t11)
    {
        float pos[6] = { David_Spline((float)t,t10,t11,-0.15f,0.f,0.f,0.f) , 0.0f , -0.95f ,
                         David_Spline((float)t,t10,t11,-0.35f,0.f,0.f,0.f) , 0.0f , -0.95f
                       } ;
        *distribution=David_Spline((float)t,t10,t11,1.f,0.5f,0.f,0.f);
        *desired_support=0;
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }


    if (t>=t11 && t<t12)
    {
        float pos[6] = { 0.f , 0.f , David_C2Spline((float)t,t11,(t11+t12)/2,t12,-0.95f,-1.f,-1.01729f,0.f,0.f) ,
                         0.f , 0.f , David_C2Spline((float)t,t11,(t11+t12)/2,t12,-0.95f,-1.f,-1.01729f,0.f,0.f)
                       };
        *distribution=0.5f;
        *desired_support=0;
        David_InverseSherpaKinematics(qd, pos);
        David_InverseSherpaKinematics(qd+6, pos+3);
    }

}

