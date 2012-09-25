
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
//    const double deg2rad=M_PI/180;
    const double motor_command_init_p2[21] =
    {
        0, 40,  3697,  9537, -5840, -344,  18810, -2000,  0,  8800,
        0, 40, -3727, -9536,  5809,  425, -18810,  2000,  0, -8800, 418
    };
//    const double motor_rotation[21] =
//    {
//        1.0, 1.0, -1.0, 1.0, 1.0, -1.0, 1.0, 1.0, -1.0, -1.0,
//        1.0, 1.0, 1.0, -1.0, -1.0, -1.0, -1.0, 1.0, -1.0, 1.0, 1.0
//    };

    for(i=0; i<21; i++)
    {
        sensor->q[i]=motor_command_init_p2[i];
        //sensor->q[i]=deg2rad*motor_rotation[i]*motor_command_init_p2[i]/209;
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
#else
    int j;
    for(j=0; j<21; j++)
    {
        sensor->q[j]=control->q[j];
    }
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


