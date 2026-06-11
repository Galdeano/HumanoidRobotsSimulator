#include <math.h>
#include "rot2omega_f.h"
#include "Mat.h"

void rot2omega_f(double *R,double *w)
{

    double alpha,th;

    alpha=(R[0]+R[4]+R[8]-1)/2;


    if (fabs(alpha-1) <0.0000001)
    {
        MatClrf( w, 3, 1) ;
        //gsl_vector_set_zero(w);
        return;
    }

    th= acos(alpha);

//w[0]=R[5]-R[7];
//w[1]=R[6]-R[2];
//w[2]=R[1]-R[3];

w[0]=R[7]-R[5];
w[1]=R[2]-R[6];
w[2]=R[3]-R[1];

MatMulScalf( w, 0.5*th/sin(th), w, 3, 1) ;


//    gsl_vector_set(w,0,gsl_matrix_get(R,2,1)-gsl_matrix_get(R,1,2));
//    gsl_vector_set(w,1,gsl_matrix_get(R,0,2)-gsl_matrix_get(R,2,0));
//    gsl_vector_set(w,2,gsl_matrix_get(R,1,0)-gsl_matrix_get(R,0,1));
//
//    gsl_vector_scale (w, 0.5*th/sin(th));

}

