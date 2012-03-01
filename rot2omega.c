
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_math.h>
#include "rot2omega.h"



void rot2omega(gsl_matrix * R,gsl_vector * w)
{

    float alpha,th;

    alpha=(gsl_matrix_get(R,0,0)+gsl_matrix_get(R,1,1)+gsl_matrix_get(R,2,2)-1)/2;

    if (fabs(alpha-1) <0.0000001)
    {
        gsl_vector_set_zero(w);
        return;
    }

    th= acos(alpha);
    gsl_vector_set(w,0,gsl_matrix_get(R,2,1)-gsl_matrix_get(R,1,2));
    gsl_vector_set(w,1,gsl_matrix_get(R,0,2)-gsl_matrix_get(R,2,0));
    gsl_vector_set(w,2,gsl_matrix_get(R,1,0)-gsl_matrix_get(R,0,1));

    gsl_vector_scale (w, 0.5*th/sin(th));

}

