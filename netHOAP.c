
#if defined (WIN32)
#include <winsock2.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <fcntl.h>
#include "netHOAP.h"
#include "Setup.h"
#include "ping.h"

Hoap hoapConnect(const char* addr, short port, HoapSensor* sensor)
{
    Hoap hoap;
#if !local
    WSADATA WSAData;
    WSAStartup(MAKEWORD(2,2), &WSAData);

    SOCKADDR_IN sock_sin;
//struct sockaddr_in sock_sin;
    int sock;

    memset(&sock_sin, sizeof(sock_sin), 0);

    sock = socket(AF_INET, SOCK_STREAM, 0);

    sock_sin.sin_addr.s_addr = inet_addr(addr);
    sock_sin.sin_family = AF_INET;
    sock_sin.sin_port = htons(port);

//    int flag = 1;
//    int result = setsockopt(sock,            /* socket affected */
//                            IPPROTO_TCP,     /* set option at TCP level */
//                            TCP_NODELAY,     /* name of option */
//                            (char *) &flag,  /* the cast is historical cruft */
//                            sizeof(int));    /* length of option value */

    connect(sock, (struct sockaddr *)&sock_sin, sizeof(SOCKADDR_IN));


    hoap.sock = sock;
#else
    int i;
    const double motor_command_init_p2[21] =
    {
        0, 40,  3697,  9537, -5840, -344,  18810, -2000,  0,  8800,
        0, 40, -3727, -9536,  5809,  425, -18810,  2000,  0, -8800, 418
    };
    const double fsr_right_init[4] =
    {
        //273, 434,  450,  558
        216,272,229,274

    };
    const double fsr_left_init[4] =
    {
        //273, 434,  450,  558
        309,217,264,220
    };

    for(i=0; i<21; i++)
    {
        sensor->q[i]=motor_command_init_p2[i];
    }

    for(i=0; i<4; i++)
    {
        sensor->fsr_right[i]=fsr_right_init[i];
        sensor->fsr_left[i]=fsr_left_init[i];
    }
#endif
    return hoap;
}

void hoapDisconnect(Hoap hoap)
{
#if !local
    close(hoap.sock);
    WSACleanup();
#endif
}

void hoapSensor(Hoap hoap, HoapSensor* sensor)
{
#if !local
    int debug=recv(hoap.sock, (char*)sensor, sizeof(HoapSensor),0);
    //printf("%d\n",debug);
#endif
}

void hoapControl(Hoap hoap, HoapSensor* sensor, const HoapControl* control)
{
#if !local
    send(hoap.sock,(char*)control,sizeof(HoapControl),0);
#endif
#if local
#if !replay
    int j;
    for(j=0; j<21; j++)
    {
        sensor->q[j]=control->q[j];
    }
#else
    static int j,k,size;
    static FILE *sensor_file;
    static FILE *control_file;
    static FILE *zmp_file;
    static FILE *t_file;
    static int init_tmp=1;
    if (init_tmp==1)
    {
        sensor_file=fopen("./../../Simu_data/sensor.txt","r");
        control_file=fopen("./../../Simu_data/control.txt","r");
        zmp_file=fopen("./../../Simu_data/zmp.txt","r");
        t_file=fopen("./../../Simu_data/t.txt","r");
        init_tmp=0;
    }


    //printf("q: ");
    for(j=0; j<21; j++)
    {
        fscanf(sensor_file,"%d",&(sensor->q[j]));
        //printf("%d ",sensor->q[j]);
    }
    //printf("\n");
    //printf("dq: ");
    for(j=0; j<21; j++)
    {
        fscanf(sensor_file,"%d",&(sensor->dq[j]));
        //printf("%d ",sensor->dq[j]);
    }
    //printf("\n");
    //printf("fsrr: ");
    for(j=0; j<4; j++)
    {
        fscanf(sensor_file,"%d",&(sensor->fsr_right[j]));
        //printf("%d ",sensor->fsr_right[j]);
    }
    //printf("\n");
    //printf("fsrl: ");
    for(j=0; j<4; j++)
    {
        fscanf(sensor_file,"%d",&(sensor->fsr_left[j]));
        //printf("%d ",sensor->fsr_left[j]);
    }
    //printf("\n");
    //printf("acc: ");
    for(j=0; j<6; j++)
    {
        fscanf(sensor_file,"%d",&(sensor->acc_gyro[j]));
        //printf("%d ",sensor->acc_gyro[j]);
    }
    //printf("\n");


#endif
#endif
}




//void hoapReadSensor(SuLINK uLINK[],Hoap hoap, HoapSensor* sensor)
//{
//    hoapSensor(hoap, sensor);
//    for(j=0; j<(dof); j++)
//    {
//        uLINK[map[j]].q = deg2rad*buff_data.val[j]*motor_rotation[j]/209;
//    }
//
//}


