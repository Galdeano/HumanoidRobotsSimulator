#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include "uLINK.h"
#include "CalcVWerrOri.h"
#include "rot2omega.h"
#include "gsl_length_v.h"
#include "pinv.h"


void CalcVWerrOri( SuLINK uLINK[], gsl_vector * err, gsl_vector * p, gsl_matrix * R, gsl_vector * idx)
{
    int ori,now,i;

    ori=gsl_vector_get(idx,0);
    now=gsl_vector_get(idx,gsl_length_v(idx)-1);


    static gsl_vector * error;
    static gsl_vector * error2;
    static gsl_matrix * rot;
    static gsl_matrix * rot2;
    static int init_tmp=1;
    if (init_tmp==1)
    {
        error = gsl_vector_calloc (3);
        error2 = gsl_vector_calloc (3);
        rot = gsl_matrix_calloc (3, 3);
        rot2 = gsl_matrix_calloc (3, 3);
        init_tmp=0;
    }


    gsl_vector_memcpy(error,p);
    gsl_vector_sub(error,uLINK[now].p);
    gsl_vector_add(error,uLINK[ori].p);
 //   pinv( rot, uLINK[ori].R);
 //   gsl_blas_dgemv(CblasNoTrans, 1.0, rot, error, 0.0, error2);

    for (i=0; i<3; i++)
    {
        gsl_vector_set(err,i,gsl_vector_get(error,i));
    }

    pinv( rot, uLINK[now].R);
    gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, rot, uLINK[ori].R, 0.0, rot2);
    //gsl_matrix_memcpy(rot,rot2);
    gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, rot2, R, 0.0, rot);
    rot2omega(rot,error);
    gsl_blas_dgemv(CblasNoTrans, 1.0, uLINK[now].R, error, 0.0, error2);

    for (i=0; i<3; i++)
    {
        gsl_vector_set(err,i+3,gsl_vector_get(error2,i));
    }
}


//
//function err = CalcVWerrOri(Cref, Cnow, Cori)
//global uLINK
//
//perr = Cref.p - Cnow.p+Cori.p;
//Rerr = Cnow.R^-1 * Cori.R*Cref.R;
//werr = Cnow.R * rot2omega(Rerr);
//
//err = [perr; werr];
//

