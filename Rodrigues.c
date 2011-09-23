#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_math.h>
#include "Rodrigues.h"


void Rodrigues(gsl_matrix * rot,gsl_vector * w,double dt)
{
    double d,th;
    gsl_vector * wn = gsl_vector_calloc (3);
    gsl_matrix * w_wedge = gsl_matrix_calloc (3, 3);
    gsl_matrix * w_wedge2 = gsl_matrix_calloc (3, 3);
    gsl_matrix * eye = gsl_matrix_calloc (3, 3);

    d = gsl_blas_dnrm2 (w);
    th = d*dt;
    gsl_vector_memcpy (wn, w);
//wn = w/d;		// normarized vector
    gsl_vector_scale (wn, 1/d);

    gsl_matrix_set (w_wedge, 0, 1, -gsl_vector_get (wn,2));
    gsl_matrix_set (w_wedge, 0, 2, gsl_vector_get (wn,1));
    gsl_matrix_set (w_wedge, 1, 0, gsl_vector_get (wn,2));
    gsl_matrix_set (w_wedge, 1, 2, -gsl_vector_get (wn,0));
    gsl_matrix_set (w_wedge, 2, 0, -gsl_vector_get (wn,1));
    gsl_matrix_set (w_wedge, 2, 1, gsl_vector_get (wn,0));

    gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, w_wedge, w_wedge, 0.0, w_wedge2);

//w_wedge = [0 -wn(3) wn(2);wn(3) 0 -wn(1);-wn(2) wn(1) 0];


    gsl_matrix_set_identity (eye);

    gsl_matrix_add (rot, eye);

    gsl_matrix_scale ( w_wedge, (sin(th)));
    gsl_matrix_add (rot, w_wedge);

    gsl_matrix_scale ( w_wedge2, (1-cos(th)));
    gsl_matrix_add (rot, w_wedge2);

//R = eye(3) + w_wedge * sin(th) + w_wedge^2 * (1-cos(th));
    gsl_vector_free(wn);
    gsl_matrix_free(w_wedge);
    gsl_matrix_free(w_wedge2);
    gsl_matrix_free(eye);
//$R = $eye3 + sin($angle)*$axis_skewed + (1-cos($angle))*$axis_skewed*$axis_skewed;

}



