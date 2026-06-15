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

#if !defined(WIN32)
#include <unistd.h>
#define closesocket close
#endif

Hoap hoapConnect(const char* addr, short port, HoapSensor* sensor)
{
    Hoap hoap;
    hoap.sock = -1;
    if (!local)
    {
#if defined (WIN32)
        WSADATA WSAData;
        WSAStartup(MAKEWORD(2,2), &WSAData);
#endif

        SOCKADDR_IN sock_sin;
        int sock;

        memset(&sock_sin, 0, sizeof(sock_sin));

        sock = socket(AF_INET, SOCK_STREAM, 0);

        sock_sin.sin_addr.s_addr = inet_addr(addr);
        sock_sin.sin_family = AF_INET;
        sock_sin.sin_port = htons(port);

        connect(sock, (struct sockaddr *)&sock_sin, sizeof(SOCKADDR_IN));

        hoap.sock = sock;
    }
    else
    {
        int i;
        const double motor_command_init_p2[21] =
        {
            0, 40,  3697,  9537, -5840, -344,  18810, -2000,  0,  8800,
            0, 40, -3727, -9536,  5809,  425, -18810,  2000,  0, -8800, 418
        };
        const double fsr_right_init[4] =
        {
            216,272,229,274
        };
        const double fsr_left_init[4] =
        {
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
    }
    return hoap;
}

void hoapDisconnect(Hoap hoap)
{
    if (!local)
    {
#if defined(WIN32)
        closesocket(hoap.sock);
        WSACleanup();
#else
        close(hoap.sock);
#endif
    }
}

void hoapSensor(Hoap hoap, HoapSensor* sensor)
{
    if (!local)
    {
        recv(hoap.sock, (char*)sensor, sizeof(HoapSensor), 0);
    }
}

void hoapControl(Hoap hoap, HoapSensor* sensor, const HoapControl* control)
{
    if (!local)
    {
        send(hoap.sock, (char*)control, sizeof(HoapControl), 0);
    }
    if (local)
    {
        if (!replay)
        {
            int j;
            for(j=0; j<21; j++)
            {
                sensor->q[j]=control->q[j];
            }
        }
        if (replay)
        {
            static int j;
            static FILE *sensor_file;
            static int init_tmp=1;
            if (init_tmp==1)
            {
                sensor_file=fopen("./../../Simu_data/sensor.txt","r");
                init_tmp=0;
            }

            for(j=0; j<21; j++)
            {
                if (sensor_file) {
                    fscanf(sensor_file,"%hd",&(sensor->q[j]));
                }
            }
            for(j=0; j<21; j++)
            {
                if (sensor_file) {
                    fscanf(sensor_file,"%hd",&(sensor->dq[j]));
                }
            }
            for(j=0; j<4; j++)
            {
                if (sensor_file) {
                    fscanf(sensor_file,"%hd",&(sensor->fsr_right[j]));
                }
            }
            for(j=0; j<4; j++)
            {
                if (sensor_file) {
                    fscanf(sensor_file,"%hd",&(sensor->fsr_left[j]));
                }
            }
            for(j=0; j<6; j++)
            {
                if (sensor_file) {
                    fscanf(sensor_file,"%hd",&(sensor->acc_gyro[j]));
                }
            }
        }
    }
}
