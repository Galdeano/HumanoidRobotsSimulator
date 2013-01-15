
#include "butterworth.h"
#include "netHOAP.h"
#include "Hoap_calc_zmp.h"



void Hoap_calc_zmp_right(HoapSensor* sensor,zmp_calc* zmp)
{
    int i;
    double a[4]= {-0.004523279, -0.004067359, 0.000604722, -0.00196956};

    double b[4]= {28.1478107, 26.3690884, 16.79400767, 23.20293315};

    double c[4]= {-6121.966103, -5716.92565, -3945.717084, -5259.220736};

    double t=8.19522E-10;
    double u=-7.73897E-06;
    double v=0.957774143;

    double Lx=47;
    double Mx=18.5;
    double Ly=83;
    double My=27.5;

    double P[4];
    double Q[4];
    double R;

    double D[4];

    double W;

    for (i=0; i<4; i++)
    {
        D[i]=sensor->fsr_right[i];
    }

    for (i=0; i<4; i++)
    {
        P[i]=a[i]*D[i]*D[i]+b[i]*D[i]+c[i];
    }

    for (i=0; i<4; i++)
    {
        Q[i]=t*P[i]*P[i]*P[i]+u*P[i]*P[i]+v*P[i];
    }

    R=P[0]+P[1]+P[2]+P[3];

    W=(t*R*R*R+u*R*R+v*R)*0.00981;

    zmp->zmp_right.W=W;

    if(zmp->zmp_right.W<0.01)
    {
        zmp->zmp_right.W=0.01;
    }

    zmp->zmp_right.y=-((Q[0]+Q[2])*Lx*0.00981/W - Mx);

    zmp->zmp_right.x=(Q[0]+Q[1])*Ly*0.00981/W - My;


}


void Hoap_calc_zmp_left(HoapSensor* sensor,zmp_calc* zmp)
{
    int i;
    double a[4]= {-0.00016777, -0.005978587, -0.000913583, -0.003810901};

    double b[4]= {19.21302846, 31.79738164, 21.96814481, 23.97477592};

    double c[4]= {-4382.901663, -6895.472873, -5046.774609, -5119.711075};


    double t=-1.23328E-09;
    double u=1.79435E-05;
    double v=0.871825383;



    double Lx=47;
    double Mx=28.5;
    double Ly=83;
    double My=27.5;

    double P[4];
    double Q[4];
    double R;

    double D[4];

    double W;

    for (i=0; i<4; i++)
    {
        D[i]=sensor->fsr_left[i];
    }

    for (i=0; i<4; i++)
    {
        P[i]=a[i]*D[i]*D[i]+b[i]*D[i]+c[i];
    }

    for (i=0; i<4; i++)
    {
        Q[i]=t*P[i]*P[i]*P[i]+u*P[i]*P[i]+v*P[i];
    }

    R=P[0]+P[1]+P[2]+P[3];

    W=(t*R*R*R+u*R*R+v*R)*0.00981;

    zmp->zmp_left.W=W;

    if(zmp->zmp_left.W<0.01)
    {
        zmp->zmp_left.W=0.01;
    }

    zmp->zmp_left.y=-((Q[0]+Q[2])*Lx*0.00981/W - Mx);

    zmp->zmp_left.x=(Q[0]+Q[1])*Ly*0.00981/W - My;



}

void Hoap_calc_zmp(HoapSensor* sensor,zmp_calc* zmp)
{

    Hoap_calc_zmp_right(sensor,zmp);
    Hoap_calc_zmp_left(sensor,zmp);
    //printf("right: %f %f %f left: %f %f %f \n",zmp->zmp_right.W,zmp->zmp_right.x,zmp->zmp_right.y,zmp->zmp_left.W,zmp->zmp_left.x,zmp->zmp_left.y);

}

void Hoap_filter_zmp(zmp_calc* zmp_c, zmp_calc* zmp_f)
{
static ButterworthData data_lx;
static ButterworthData data_ly;
static ButterworthData data_lW;
static ButterworthData data_rx;
static ButterworthData data_ry;
static ButterworthData data_rW;

    static int init_tmp=1;
    if (init_tmp==1)
    {
        ButterworthFilterInit(&data_lx);
        ButterworthFilterInit(&data_ly);
        ButterworthFilterInit(&data_lW);
        ButterworthFilterInit(&data_rx);
        ButterworthFilterInit(&data_ry);
        ButterworthFilterInit(&data_rW);
        init_tmp=0;
    }


    if (zmp_c->zmp_right.x>56)
    {
        zmp_c->zmp_right.x=56;
    }
    else if (zmp_c->zmp_right.x<-42)
    {
        zmp_c->zmp_right.x=-42;
    }

    if (zmp_c->zmp_right.y>31.5)
    {
        zmp_c->zmp_right.y=31.5;
    }
    else if (zmp_c->zmp_right.y<-31.5)
    {
        zmp_c->zmp_right.y=-31.5;
    }


    if (zmp_c->zmp_left.x>56)
    {
        zmp_c->zmp_left.x=56;
    }
    else if (zmp_c->zmp_left.x<-42)
    {
        zmp_c->zmp_left.x=-42;
    }

    if (zmp_c->zmp_left.y>31.5)
    {
        zmp_c->zmp_left.y=31.5;
    }
    else if (zmp_c->zmp_left.y<-31.5)
    {
        zmp_c->zmp_left.y=-31.5;
    }

zmp_f->zmp_left.x=ButterworthFilter (&data_lx, zmp_c->zmp_left.x);
zmp_f->zmp_left.y=ButterworthFilter (&data_ly, zmp_c->zmp_left.y);
zmp_f->zmp_left.W=ButterworthFilter (&data_lW, zmp_c->zmp_left.W);
zmp_f->zmp_right.x=ButterworthFilter (&data_rx, zmp_c->zmp_right.x);
zmp_f->zmp_right.y=ButterworthFilter (&data_ry, zmp_c->zmp_right.y);
zmp_f->zmp_right.W=ButterworthFilter (&data_rW, zmp_c->zmp_right.W);

}










