#include <gsl/gsl_math.h>
#include "butterworth.h"

void ButterworthFilterInit (ButterworthData *data,double wc)
{
    int i;

    data->xv[0] = 0;
    data->xv[1] = 0;
    data->xv[2] = 0;
    data->xv[3] = 0;
    data->yv[0] = 0;
    data->yv[1] = 0;
    data->yv[2] = 0;
    data->yv[3] = 0;


//    // Matlab : [b,a] = butter(3,.01); //order 3 lowpass digital Butterworth filter with normalized cutoff frequency 0.01
//    data->b[0]= 0.037568380197861*1.0e-004;
//    data->b[1]= 0.112705140593583*1.0e-004;
//    data->b[2]= 0.112705140593583*1.0e-004;
//    data->b[3]= 0.037568380197861*1.0e-004;
//    data->a[0]= 1.000000000000000;
//    data->a[1]= -2.937170728449891;
//    data->a[2]= 2.876299723479332;
//    data->a[3]= -0.939098940325283;
//
//    MatPrintf( "a:" , " %f ", data->a , 1 , 4 ) ;
//    MatPrintf( "b:" , " %f ", data->b , 1 , 4 ) ;


    wc=wc*M_PI;

//    bk2(4) = wc^3;
//    bk2(3) = 3*wc^3;
//    bk2(2) = 3*wc^3;
//    bk2(1) = wc^3;
//
//    ak2(4) = -8+8*wc-4*wc^2+wc^3;
//    ak2(3) = 24-8*wc-4*wc^2+3*wc^3;
//    ak2(2) = -24-8*wc+4*wc^2+3*wc^3;
//    ak2(1) = 8+8*wc+4*wc^2+wc^3;
//
//    bk2=bk2./ak2(1)
//    ak2=ak2./ak2(1)

    data->a[0]= 8+8*wc+4*gsl_pow_2(wc)+gsl_pow_3(wc);
    data->a[1]= -24-8*wc+4*gsl_pow_2(wc)+3*gsl_pow_3(wc);
    data->a[2]= 24-8*wc-4*gsl_pow_2(wc)+3*gsl_pow_3(wc);
    data->a[3]= -8+8*wc-4*gsl_pow_2(wc)+gsl_pow_3(wc);
    data->b[0]= gsl_pow_3(wc);
    data->b[1]= 3*gsl_pow_3(wc);
    data->b[2]= 3*gsl_pow_3(wc);
    data->b[3]= gsl_pow_3(wc);

    for(i=4;i--; )
    {
        data->b[i]=data->b[i]/data->a[0];
        data->a[i]=data->a[i]/data->a[0];
    }

//
//    MatPrintf( "a:" , " %f ", data->a , 1 , 4 ) ;
//    MatPrintf( "b:" , " %f ", data->b , 1 , 4 ) ;

}

void ButterworthFilterValue (ButterworthData *data,double value)
{
    data->xv[0] = value;
    data->xv[1] = value;
    data->xv[2] = value;
    data->xv[3] = value;
    data->yv[0] = value;
    data->yv[1] = value;
    data->yv[2] = value;
    data->yv[3] = value;
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

    data->xv[0] = data->xv[1];
    data->xv[1] = data->xv[2];
    data->xv[2] = data->xv[3];
    data->xv[3] = input;
    data->yv[0] = data->yv[1];
    data->yv[1] = data->yv[2];
    data->yv[2] = data->yv[3];
    data->yv[3] = (double)((data->b[0]*data->xv[3]
                            +data->b[1]*data->xv[2]
                            +data->b[2]*data->xv[1]
                            +data->b[3]*data->xv[0])
                            +(-data->a[1]*data->yv[2]
                              -data->a[2]*data->yv[1]
                              -data->a[3]*data->yv[0]));
    return (data->yv[3]);
}

// checker: http://www.exstrom.com/journal/sigproc/






