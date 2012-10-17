#ifndef H_Hoap_calc_zmp
#define H_Hoap_calc_zmp
#include "netHOAP.h"

typedef struct
{
    double W;   //Foot bottom vertical loading (N)
    double x;   //Zero moment point (mm)
    double y;
} zmp_calc_foot;

typedef struct
{
    zmp_calc_foot zmp_right;
    zmp_calc_foot zmp_left;
} zmp_calc;

void Hoap_calc_zmp_right(HoapSensor* sensor,zmp_calc* zmp);
void Hoap_calc_zmp_left(HoapSensor* sensor,zmp_calc* zmp);
void Hoap_calc_zmp(HoapSensor* sensor,zmp_calc* zmp);


#endif
