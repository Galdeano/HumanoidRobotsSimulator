#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_math.h>
#include "uLINK.h"
#include "ForwardKinematics.h"
#include "Rodrigues.h"

void ForwardKinematics(SuLINK uLINK[],int j)
{

    if (j == 0)
    {
        return;
    }

    static gsl_matrix * rot;
    static int init_tmp=1;
    if (init_tmp==1)
    {
        rot = gsl_matrix_calloc (3, 3);
        init_tmp=0;
    }
//    gsl_matrix_set_zero(rot);

    if (j != 1)
    {
        gsl_blas_dgemv(CblasNoTrans, 1.0, uLINK[uLINK[j].mother].R, uLINK[j].b, 0.0, uLINK[j].p);
        gsl_vector_add (uLINK[j].p, uLINK[uLINK[j].mother].p);
        //uLINK[j].p = uLINK[uLINK[j].mother].R * uLINK[j].b + uLINK[uLINK[j].mother].p;

//        gsl_matrix * rot = gsl_matrix_calloc (3, 3);
        Rodrigues( rot ,uLINK[j].a, uLINK[j].q);

//printf("%8.1f ",gsl_vector_get (uLINK[j].a,0));
//printf("%8.1f ",gsl_vector_get (uLINK[j].a,1));
//printf("%8.1f \n",gsl_vector_get (uLINK[j].a,2));
//
//printf("%d: %8.1f \n",j,uLINK[j].q);
//
//printf("%8.4f ",gsl_vector_get (uLINK[j].p,0));
//printf("%8.4f ",gsl_vector_get (uLINK[j].p,1));
//printf("%8.4f \n \n",gsl_vector_get (uLINK[j].p,2));

//printf("%8.1f ",gsl_matrix_get (rot, 0,0));
//printf("%8.1f ",gsl_matrix_get (rot, 0,1));
//printf("%8.1f \n",gsl_matrix_get (rot, 0,2));
//printf("%8.1f ",gsl_matrix_get (rot, 1,0));
//printf("%8.1f ",gsl_matrix_get (rot, 1,1));
//printf("%8.1f \n",gsl_matrix_get (rot, 1,2));
//printf("%8.1f ",gsl_matrix_get (rot, 2,0));
//printf("%8.1f ",gsl_matrix_get (rot, 2,1));
//printf("%8.1f \n \n",gsl_matrix_get (rot, 2,2));

        gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, uLINK[uLINK[j].mother].R, rot, 0.0, uLINK[j].R);

//printf("%8.1f ",gsl_matrix_get (uLINK[uLINK[j].mother].R, 0,0));
//printf("%8.1f ",gsl_matrix_get (uLINK[uLINK[j].mother].R, 0,1));
//printf("%8.1f \n",gsl_matrix_get (uLINK[uLINK[j].mother].R, 0,2));
//printf("%8.1f ",gsl_matrix_get (uLINK[uLINK[j].mother].R, 1,0));
//printf("%8.1f ",gsl_matrix_get (uLINK[uLINK[j].mother].R, 1,1));
//printf("%8.1f \n",gsl_matrix_get (uLINK[uLINK[j].mother].R, 1,2));
//printf("%8.1f ",gsl_matrix_get (uLINK[uLINK[j].mother].R, 2,0));
//printf("%8.1f ",gsl_matrix_get (uLINK[uLINK[j].mother].R, 2,1));
//printf("%8.1f \n \n",gsl_matrix_get (uLINK[uLINK[j].mother].R, 2,2));
//
//printf("%8.1f ",gsl_matrix_get (uLINK[j].R, 0,0));
//printf("%8.1f ",gsl_matrix_get (uLINK[j].R, 0,1));
//printf("%8.1f \n",gsl_matrix_get (uLINK[j].R, 0,2));
//printf("%8.1f ",gsl_matrix_get (uLINK[j].R, 1,0));
//printf("%8.1f ",gsl_matrix_get (uLINK[j].R, 1,1));
//printf("%8.1f \n",gsl_matrix_get (uLINK[j].R, 1,2));
//printf("%8.1f ",gsl_matrix_get (uLINK[j].R, 2,0));
//printf("%8.1f ",gsl_matrix_get (uLINK[j].R, 2,1));
//printf("%8.1f \n \n",gsl_matrix_get (uLINK[j].R, 2,2));
        //uLINK[j].R = uLINK[uLINK[j].mother].R * Rodrigues(uLINK[j].a, uLINK[j].q);
//        gsl_matrix_free(rot);
    }

    ForwardKinematics(uLINK,uLINK[j].sister);
    ForwardKinematics(uLINK,uLINK[j].child);


}
