
#include "butterworth.h"

void ButterworthFilterInit (ButterworthData *data)
{
    data->xv[0] = 0;
    data->xv[1] = 0;
    data->xv[2] = 0;
    data->xv[3] = 0;
    data->yv[0] = 0;
    data->yv[1] = 0;
    data->yv[2] = 0;
    data->yv[3] = 0;
}

/*******************************************************************************
 * Function:      ButterworthFilter
 *
 * Description:   This function uses the Butterworth filter and returns a new
 *                value for an individual floating point value.
 *
 * Access:        PRIVATE
 *
 * Parameters:    double input - value to be converted
 *
 * Globals:       None
 *
 * Returns:       double - new value that has been converted
 ******************************************************************************/
double ButterworthFilter (ButterworthData *data, double input)
{
    // Matlab : [b,a] = butter(3,.01); //order 3 lowpass digital Butterworth filter with normalized cutoff frequency 0.01
    static double b1= 0.037568380197861*1.0e-004;
    static double b2= 0.112705140593583*1.0e-004;
    static double b3= 0.112705140593583*1.0e-004;
    static double b4= 0.037568380197861*1.0e-004;
    static double a1= 1.000000000000000;
    static double a2= -2.937170728449891;
    static double a3= 2.876299723479332;
    static double a4= -0.939098940325283;


    data->xv[0] = data->xv[1];
    data->xv[1] = data->xv[2];
    data->xv[2] = data->xv[3];
    data->xv[3] = input;
    data->yv[0] = data->yv[1];
    data->yv[1] = data->yv[2];
    data->yv[2] = data->yv[3];
    data->yv[3] = (double)((b1*data->xv[3]+b2*data->xv[2]+b3*data->xv[1]+b4*data->xv[0])
                        +(-a2*data->yv[2]-a3*data->yv[1]-a4*data->yv[0]));
    return (data->yv[3]);
}








