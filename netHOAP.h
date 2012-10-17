#ifndef H_netHOAP
#define H_netHOAP


typedef struct
{
    short q[21];
    short dq[21];
    short fsr_right[4];
    short fsr_left[4];
    short acc_gyro[6];
} HoapSensor;

typedef struct
{
    short q[21];
} HoapControl;

typedef struct
{
    int sock;
} Hoap;


Hoap hoapConnect(const char* addr, short port, HoapSensor* sensor);

void hoapDisconnect(Hoap hoap);

void hoapSensor(Hoap hoap, HoapSensor* sensor);

void hoapControl(Hoap hoap, HoapSensor* sensor, const HoapControl* control);




//
//void hoapReadSensor(SuLINK uLINK[],Hoap hoap, HoapSensor* sensor);
//
//void hoapSendControl(SuLINK uLINK[],Hoap hoap, const HoapControl* control);


#endif
