//#include "d:\Mb\StLib\Mat.h"
#include "InverseSherpaKinematics_f.h"
#include "Spline_f.h"
#include "StaticTrajectory_f.h"

#include "Setup.h"

void StaticTrajectory_f(double *qd, double t, int *desired_support, double *distribution)
{
//   int n;

    double t1=2;
    double t2=5;
    double t3=15;
    double t4=25;
    double t5=35;
    double t6=45;
    double t7=55;
    double t8=65;
    double t9=75;
    double t10=85;
    double t11=95;
    double t12=105;

    /*
        double t1=2;
        double t2=5;
        double t3=10;
        double t4=15;
        double t5=20;
        double t6=25;
        double t7=30;
        double t8=35;
        double t9=40;
        double t10=45;
        double t11=50;
        double t12=55;
        */

//    double t1=0.5;
//    double t2=2;
//    double t3=6;
//    double t4=10;
//    double t5=14;
//    double t6=18;
//    double t7=22;
//    double t8=26;

//    double t1=0.5;
//    double t2=2;
//    double t3=5;
//    double t4=8;
//    double t5=11;
//    double t6=14;
//    double t7=17;
//    double t8=20;
//    double t9=23;
//    double t10=26;
//    double t11=29;
//    double t12=32;

    double com_y =0.29f;
    //double com_y =0.29;
    double adjust_x =-0.03f;
    //double steep_x =0.05;


//        while(t>=t8) //Repete le pas
//        {
//            t -= t8-t4;
//        }





    if (t<t1)
    {
        double pos[6] = { 0.f , 0.f , -1.01729f ,
                         0.f , 0.f , -1.01729f
                       } ;//Posture dans l'espace operationel
        *distribution=0.5;	//repartition de l'effort de contact
        *desired_support=3;	//Pied de support: 0:none,1:right,2:left,3:both
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }

    if (t>=t1 && t<t2)
    {
        double pos[6] = { 0.f , 0.f , C2Spline_f((double)t,t1,(t1+t2)/2,t2,-1.01729f,-1.f,-0.95f,0.f,0.f) ,
                         0.f , 0.f , C2Spline_f((double)t,t1,(t1+t2)/2,t2,-1.01729f,-1.f,-0.95f,0.f,0.f)
                       } ;//Posture dans l'espace operationel
        *distribution=0.5f;
        *desired_support=3;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }

    if (t>=t2 && t<t3)
    {
        double pos[6] = { Spline_f((double)t,t2,t3,0.f,-0.05f,0.f,0.f) , Spline_f((double)t,t2,t3,0.f,com_y,0.f,0.f) , -0.95f ,
                         Spline_f((double)t,t2,t3,0.f,-0.05f,0.f,0.f) , Spline_f((double)t,t2,t3,0.f,com_y,0.f,0.f) , -0.95f
                       } ;
        *distribution=Spline_f((double)t,t2,t3,0.5f,0.f,0.f,0.f);
        *desired_support=3;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }

    if (t>=t3 && t<t4)
    {
        double pos[6] = { -0.05f+Spline_f((double)t,t3,t4,0.f,-adjust_x,0.f,0.f) , com_y , -0.95f ,
                         Spline_f((double)t,t3,t4,-0.05f,0.05f,0.f,0.f)+Spline_f((double)t,t3,t4,0.f,-adjust_x,0.f,0.f) , com_y , C2Spline_f((double)t,t3,(t3+t4)/2,t4,-0.95f,-0.9f,-0.95f,0.f,0.f)
                       } ;
        *distribution=0.f;
        *desired_support=1;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }


    if (t>=t4 && t<t5)
    {
        double pos[6] = { Spline_f((double)t,t4,t5,-0.05f,-0.15f,0.f,0.f)+Spline_f((double)t,t4,t5,-adjust_x,adjust_x,0.f,0.f) , Spline_f((double)t,t4,t5,com_y,-com_y,0.f,0.f) , -0.95f ,
                         Spline_f((double)t,t4,t5, 0.05f,-0.05f,0.f,0.f)+Spline_f((double)t,t4,t5,-adjust_x,adjust_x,0.f,0.f) , Spline_f((double)t,t4,t5,com_y,-com_y,0.f,0.f) , -0.95f
                       } ;
        *distribution=Spline_f((double)t,t4,t5,0.f,1.f,0.f,0.f);
        *desired_support=3;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }


    if (t>=t5 && t<t6)
    {
        double pos[6] = { Spline_f((double)t,t5,t6,-0.15f,0.05f,0.f,0.f)+Spline_f((double)t,t5,t6,adjust_x,-adjust_x,0.f,0.f) , -com_y , C2Spline_f((double)t,t5,(t5+t6)/2,t6,-0.95f,-0.9f,-0.95f,0.f,0.f) ,
                         -0.05f+Spline_f((double)t,t5,t6,adjust_x,-adjust_x,0.f,0.f) , -com_y , -0.95f
                       } ;
        *distribution=1.f;
        *desired_support=2;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }


    if (t>=t6 && t<t7)
    {
        double pos[6] = { Spline_f((double)t,t6,t7, 0.05f,-0.05f,0.f,0.f)+Spline_f((double)t,t6,t7,-adjust_x,adjust_x,0.f,0.f) , Spline_f((double)t,t6,t7,-com_y,com_y,0.f,0.f) , -0.95f ,
                         Spline_f((double)t,t6,t7,-0.05f,-0.15f,0.f,0.f)+Spline_f((double)t,t6,t7,-adjust_x,adjust_x,0.f,0.f) , Spline_f((double)t,t6,t7,-com_y,com_y,0.f,0.f) , -0.95f
                       } ;
        *distribution=Spline_f((double)t,t6,t7,1.f,0.f,0.f,0.f);
        *desired_support=3;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }


    if (t>=t7 && t<t8)
    {
        double pos[6] = { -0.05f+Spline_f((double)t,t7,t8,adjust_x,-adjust_x,0.f,0.f) , com_y , -0.95f ,
                         Spline_f((double)t,t7,t8,-0.15f,0.05f,0.f,0.f)+Spline_f((double)t,t7,t8,adjust_x,-adjust_x,0.f,0.f) , com_y , C2Spline_f((double)t,t7,(t7+t8)/2,t8,-0.95f,-0.9f,-0.95f,0.f,0.f)
                       } ;
        *distribution=0.f;
        *desired_support=1;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }


    if (t>=t8 && t<t9)
    {
        double pos[6] = { Spline_f((double)t,t8,t9,-0.05f,-0.15f,0.f,0.f)+Spline_f((double)t,t8,t9,-adjust_x,adjust_x,0.f,0.f) , Spline_f((double)t,t8,t9,com_y,-com_y,0.f,0.f) , -0.95f ,
                         Spline_f((double)t,t8,t9, 0.05f,-0.05f,0.f,0.f)+Spline_f((double)t,t8,t9,-adjust_x,adjust_x,0.f,0.f) , Spline_f((double)t,t8,t9,com_y,-com_y,0.f,0.f) , -0.95f
                       } ;
        *distribution=Spline_f((double)t,t8,t9,0.f,1.f,0.f,0.f);
        *desired_support=3;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }


    if (t>=t9 && t<t10)
    {
        double pos[6] = { Spline_f((double)t,t9,t10,-0.15f,-0.05f,0.f,0.f)+Spline_f((double)t,t9,t10,adjust_x,0.f,0.f,0.f) , -com_y , C2Spline_f((double)t,t9,(t9+t10)/2,t10,-0.95f,-0.9f,-0.95f,0.f,0.f) ,
                         -0.05f+Spline_f((double)t,t9,t10,adjust_x,0.f,0.f,0.f) , -com_y , -0.95f
                       } ;
        *distribution=1.f;
        *desired_support=2;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }


    if (t>=t10 && t<t11)
    {
        double pos[6] = { Spline_f((double)t,t10,t11,-0.05f,0.f,0.f,0.f) , Spline_f((double)t,t10,t11,-com_y,0.f,0.f,0.f) , -0.95f ,
                         Spline_f((double)t,t10,t11,-0.05f,0.f,0.f,0.f) , Spline_f((double)t,t10,t11,-com_y,0.f,0.f,0.f) , -0.95f
                       } ;
        *distribution=Spline_f((double)t,t10,t11,1.f,0.5f,0.f,0.f);
        *desired_support=3;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }


    if (t>=t11 && t<t12)
    {
        double pos[6] = { 0.f , 0.f , C2Spline_f((double)t,t11,(t11+t12)/2,t12,-0.95f,-1.f,-1.01729f,0.f,0.f) ,
                         0.f , 0.f , C2Spline_f((double)t,t11,(t11+t12)/2,t12,-0.95f,-1.f,-1.01729f,0.f,0.f)
                       };
        *distribution=0.5f;
        *desired_support=3;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }



    //InverseSherpaKinematics_f(qd, pos);

    //InverseSherpaKinematics_f(qd+6, pos+3);


}



#if Sherpa
void OneFoot_f(double *qd, double t, int *desired_support, double *distribution)
{


    double t1=5;
    double t2=15;
    double t3=25;
    double t4=35;
    double t5=45;
    double t6=55;

//    double t1=5;
//    double t2=15;
//    double t3=15;
//    double t4=15;
//    double t5=15;
//    double t6=25;

//    double t1=0.5;
//    double t2=2;
//    double t3=5;
//    double t4=8;
//    double t5=11;
//    double t6=14;


    double com_y =0.29f;
    //double com_y =0.29;


    if (t<t1)
    {
        double pos[6] = { 0.f , 0.f , -1.01729f ,
                         0.f , 0.f , -1.01729f
                       } ;//Posture dans l'espace operationel
        *distribution=0.5f;	//repartition de l'effort de contact
        *desired_support=3;	//Pied de support: 0:none,1:right,2:left,3:both
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }

    if (t>=t1 && t<t2)
    {
        double pos[6] = { 0.f , 0.f , C2Spline_f((double)t,t1,(t1+t2)/2,t2,-1.01729f,-1.f,-0.95f,0.f,0.f) ,
                         0.f , 0.f , C2Spline_f((double)t,t1,(t1+t2)/2,t2,-1.01729f,-1.f,-0.95f,0.f,0.f)
                       } ;//Posture dans l'espace operationel
        *distribution=0.5f;
        *desired_support=3;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }

    if (t>=t2 && t<t3)
    {
        double pos[6] = { Spline_f((double)t,t2,t3,0.f,-0.05f,0.f,0.f) , Spline_f((double)t,t2,t3,0.f,com_y,0.f,0.f) , -0.95f ,
                         Spline_f((double)t,t2,t3,0.f,-0.05f,0.f,0.f) , Spline_f((double)t,t2,t3,0.f,com_y,0.f,0.f) , -0.95f
                       } ;
        *distribution=Spline_f((double)t,t2,t3,0.5f,0.f,0.f,0.f);
        *desired_support=3;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }

    if (t>=t3 && t<t4)
    {
        double pos[6] = { -0.05f , com_y , -0.95f ,
                         -0.05f , com_y , C2Spline_f((double)t,t3,(t3+t4)/2,t4,-0.95f,-0.90f,-0.95f,0.f,0.f)
                       } ;
        *distribution=0.f;
        *desired_support=1;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }


    if (t>=t4 && t<t5)
    {
        double pos[6] = { Spline_f((double)t,t4,t5,-0.05f,0.f,0.f,0.f) , Spline_f((double)t,t4,t5,com_y,0.f,0.f,0.f) , -0.95f ,
                         Spline_f((double)t,t4,t5,-0.05f,0.f,0.f,0.f) , Spline_f((double)t,t4,t5,com_y,0.f,0.f,0.f) , -0.95f
                       } ;
        *distribution=Spline_f((double)t,t4,t5,0.f,0.5f,0.f,0.f);
        *desired_support=3;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }


    if (t>=t5 && t<t6)
    {
        double pos[6] = { 0.f , 0.f , C2Spline_f((double)t,t5,(t5+t6)/2,t6,-0.95f,-1.f,-1.01729f,0.f,0.f) ,
                         0.f , 0.f , C2Spline_f((double)t,t5,(t5+t6)/2,t6,-0.95f,-1.f,-1.01729f,0.f,0.f)
                       } ;
        *distribution=0.5;
        *desired_support=3;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }

    if (t>=t6)
    {
        double pos[6] = { 0.f , 0.f , -1.01729f ,
                         0.f , 0.f , -1.01729f
                       } ;
        *distribution=0.5;
        *desired_support=3;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }

}
#endif

#if Generic
void OneFoot_f(double *qd, double t, int *desired_support, double *distribution)
{


//    double t1=5;
//    double t2=15;
//    double t3=25;
//    double t4=35;
//    double t5=45;
//    double t6=55;

//    double t1=5;
//    double t2=15;
//    double t3=15;
//    double t4=15;
//    double t5=15;
//    double t6=25;

    double t1=0.5;
    double t2=2;
    double t3=5;
    double t4=8;
    double t5=11;
    double t6=14;

    double com_y =0.290f;
    //double com_y =0.210f;
    //double com_y =0.29;


    if (t<t1)
    {
        double pos[6] = { 0.f , 0.f , -1.01729f ,
                         0.f , 0.f , -1.01729f
                       } ;//Posture dans l'espace operationel
        *distribution=0.5f;	//repartition de l'effort de contact
        *desired_support=3;	//Pied de support: 0:none,1:right,2:left,3:both
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }

    if (t>=t1 && t<t2)
    {
        double pos[6] = { 0.f , 0.f , C2Spline_f((double)t,t1,(t1+t2)/2,t2,-1.01729f,-1.f,-0.95f,0.f,0.f) ,
                         0.f , 0.f , C2Spline_f((double)t,t1,(t1+t2)/2,t2,-1.01729f,-1.f,-0.95f,0.f,0.f)
                       } ;//Posture dans l'espace operationel
        *distribution=0.5f;
        *desired_support=3;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }

    if (t>=t2 && t<t3)
    {
        double pos[6] = { Spline_f((double)t,t2,t3,0.f,-0.10f,0.f,0.f) , Spline_f((double)t,t2,t3,0.f,com_y,0.f,0.f) , -0.95f ,
                         Spline_f((double)t,t2,t3,0.f,-0.10f,0.f,0.f) , Spline_f((double)t,t2,t3,0.f,com_y,0.f,0.f) , -0.95f
                       } ;
        *distribution=Spline_f((double)t,t2,t3,0.5f,0.f,0.035f,0.030f);
        *desired_support=3;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }

    if (t>=t3 && t<t4)
    {
        double pos[6] = { -0.10f , com_y , -0.95f ,
                         -0.10f , com_y , C2Spline_f((double)t,t3,(t3+t4)/2,t4,-0.95f,-0.90f,-0.95f,0.f,0.f)
                       } ;
        *distribution=0.f;
        *desired_support=1;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }


    if (t>=t4 && t<t5)
    {
        double pos[6] = { Spline_f((double)t,t4,t5,-0.10f,0.f,0.f,0.f) , Spline_f((double)t,t4,t5,com_y,0.f,0.f,0.f) , -0.95f ,
                         Spline_f((double)t,t4,t5,-0.10f,0.f,0.f,0.f) , Spline_f((double)t,t4,t5,com_y,0.f,0.f,0.f) , -0.95f
                       } ;
        *distribution=Spline_f((double)t,t4,t5,0.f,0.5f,-0.035f,-0.035f);
        *desired_support=3;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }


    if (t>=t5 && t<t6)
    {
        double pos[6] = { 0.f , 0.f , C2Spline_f((double)t,t5,(t5+t6)/2,t6,-0.95f,-1.f,-1.01729f,0.f,0.f) ,
                         0.f , 0.f , C2Spline_f((double)t,t5,(t5+t6)/2,t6,-0.95f,-1.f,-1.01729f,0.f,0.f)
                       } ;
        *distribution=0.5;
        *desired_support=3;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }

    if (t>=t6)
    {
        double pos[6] = { 0.f , 0.f , -1.01729f ,
                         0.f , 0.f , -1.01729f
                       } ;
        *distribution=0.5;
        *desired_support=3;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }

}
#endif



void GravityExperiment_f(double *qd, double t, int *desired_support, double *distribution)
{
//   int n;
    /*
        double t1=2;
        double t2=5;
        double t3=15;
        double t4=25;
        double t5=35;
        double t6=45;
        double t7=55;
        double t8=65;
        double t9=75;
        double t10=85;
        double t11=95;
        double t12=105;
        */


    double t1=2;
    double t2=5;
    double t3=10;
    double t4=15;
    double t5=20;
    double t6=25;
    double t7=30;
    double t8=35;
    double t9=40;
    double t10=45;
    double t11=50;
    double t12=55;


//    double t1=0.5;
//    double t2=2;
//    double t3=6;
//    double t4=10;
//    double t5=14;
//    double t6=18;
//    double t7=22;
//    double t8=26;

//    double t1=0.5;
//    double t2=2;
//    double t3=5;
//    double t4=8;
//    double t5=11;
//    double t6=14;
//    double t7=17;
//    double t8=20;


    double com_y =0.10f;
    //double com_y =0.29;

    *distribution=0.5f;	//repartition de l'effort de contact
    *desired_support=0;	//Pied de support: 0:none,1:right,2:left,3:both

    if (t<t1)
    {
        double pos[6] = { 0.f , 0.f , -1.01729f ,
                         0.f , 0.f , -1.01729f
                       } ;//Posture dans l'espace operationel
        *distribution=0.5f;	//repartition de l'effort de contact
        *desired_support=0;	//Pied de support: 0:none,1:right,2:left,3:both
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }

    if (t>=t1 && t<t2)
    {
        double pos[6] = { Spline_f((double)t,t1,t2,0.f,-0.1f,0.f,0.f) , 0.f , C2Spline_f((double)t,t1,(t1+t2)/2,t2,-1.01729f,-1.f,-0.95f,0.f,0.f) ,
                         Spline_f((double)t,t1,t2,0.f,-0.1f,0.f,0.f) , 0.f , C2Spline_f((double)t,t1,(t1+t2)/2,t2,-1.01729f,-1.f,-0.95f,0.f,0.f)
                       } ;//Posture dans l'espace operationel
        *distribution=0.5f;
        *desired_support=0;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }

    if (t>=t2 && t<t3)
    {
        double pos[6] = { Spline_f((double)t,t2,t3,-0.1f,-0.25f,0.f,0.f) , Spline_f((double)t,t2,t3,0.f,com_y,0.f,0.f) , -0.95f ,
                         Spline_f((double)t,t2,t3,-0.1f,-0.25f,0.f,0.f) , Spline_f((double)t,t2,t3,0.f,com_y,0.f,0.f) , -0.95f
                       } ;
        *distribution=Spline_f((double)t,t2,t3,0.5f,0.f,0.f,0.f);
        *desired_support=0;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }

    if (t>=t3 && t<t4)
    {
        double pos[6] = { -0.25f , com_y , -0.95f ,
                         -0.25f , com_y , -0.95f
                       } ;
        *distribution=0.f;
        *desired_support=0;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }


    if (t>=t4 && t<t5)
    {
        double pos[6] = { Spline_f((double)t,t4,t5,-0.25f,-0.35f,0.f,0.f) , Spline_f((double)t,t4,t5,com_y,0.f,0.f,0.f) , -0.95f ,
                         Spline_f((double)t,t4,t5,-0.25f,-0.15f,0.f,0.f) , Spline_f((double)t,t4,t5,com_y,0.f,0.f,0.f) , -0.95f
                       } ;
        *distribution=Spline_f((double)t,t4,t5,0.f,1.f,0.f,0.f);
        *desired_support=0;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }


    if (t>=t5 && t<t6)
    {
        double pos[6] = { -0.35f , 0.0f , -0.95f ,
                         -0.15f , 0.0f , -0.95f
                       } ;
        *distribution=1.f;
        *desired_support=0;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }


    if (t>=t6 && t<t7)
    {
        double pos[6] = { Spline_f((double)t,t6,t7,-0.35f,-0.25f,0.f,0.f) , Spline_f((double)t,t6,t7,0.f,-com_y,0.f,0.f) , -0.95f ,
                         Spline_f((double)t,t6,t7,-0.15f,-0.25f,0.f,0.f) , Spline_f((double)t,t6,t7,0.f,-com_y,0.f,0.f) , -0.95f
                       } ;
        *distribution=Spline_f((double)t,t6,t7,1.f,0.f,0.f,0.f);
        *desired_support=0;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }


    if (t>=t7 && t<t8)
    {
        double pos[6] = { -0.25f , -com_y , -0.95f ,
                         -0.25f , -com_y , -0.95f
                       } ;
        *distribution=0.f;
        *desired_support=0;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }


    if (t>=t8 && t<t9)
    {
        double pos[6] = { Spline_f((double)t,t8,t9,-0.25f,-0.15f,0.f,0.f) , Spline_f((double)t,t8,t9,-com_y,0.f,0.f,0.f) , -0.95f ,
                         Spline_f((double)t,t8,t9,-0.25f,-0.35f,0.f,0.f) , Spline_f((double)t,t8,t9,-com_y,0.f,0.f,0.f) , -0.95f
                       } ;
        *distribution=Spline_f((double)t,t8,t9,0.f,1.f,0.f,0.f);
        *desired_support=0;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }


    if (t>=t9 && t<t10)
    {
        double pos[6] = { -0.15f , 0.0f , -0.95f ,
                         -0.35f , 0.0f , -0.95f
                       } ;
        *distribution=1.f;
        *desired_support=0;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }


    if (t>=t10 && t<t11)
    {
        double pos[6] = { Spline_f((double)t,t10,t11,-0.15f,0.f,0.f,0.f) , 0.0f , -0.95f ,
                         Spline_f((double)t,t10,t11,-0.35f,0.f,0.f,0.f) , 0.0f , -0.95f
                       } ;
        *distribution=Spline_f((double)t,t10,t11,1.f,0.5f,0.f,0.f);
        *desired_support=0;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }


    if (t>=t11 && t<t12)
    {
        double pos[6] = { 0.f , 0.f , C2Spline_f((double)t,t11,(t11+t12)/2,t12,-0.95f,-1.f,-1.01729f,0.f,0.f) ,
                         0.f , 0.f , C2Spline_f((double)t,t11,(t11+t12)/2,t12,-0.95f,-1.f,-1.01729f,0.f,0.f)
                       };
        *distribution=0.5f;
        *desired_support=0;
        InverseSherpaKinematics_f(qd, pos);
        InverseSherpaKinematics_f(qd+6, pos+3);
    }

}

