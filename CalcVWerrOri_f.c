#include "uLINK_f.h"
#include "CalcVWerrOri_f.h"
#include "rot2omega_f.h"
#include "Mat.h"

void CalcVWerrOri_f(Struct_uLINK uLINK[],double *err, double *p, double *R, int *idx,int idx_size)
{
    int ori,now,i;

    ori=idx[0];
    now=idx[idx_size-1];

    static double error[3];
    static double error2[3];
    static double rot[3*3];
    static double rot2[3*3];

//    static gsl_vector * error;
//    static gsl_vector * error2;
//    static gsl_matrix * rot;
//    static gsl_matrix * rot2;
//    static int init_tmp=1;
//    if (init_tmp==1)
//    {
//        error = gsl_vector_calloc (3);
//        error2 = gsl_vector_calloc (3);
//        rot = gsl_matrix_calloc (3, 3);
//        rot2 = gsl_matrix_calloc (3, 3);
//        init_tmp=0;
//    }

//MatCpyf( error , p, 3 , 1);
MatSubf( error , p , uLINK[now].p, 3, 1) ;
MatAddf( error, error, uLINK[ori].p, 3, 1) ;
//    gsl_vector_memcpy(error,p);
//    gsl_vector_sub(error,uLINK[now].p);
//    gsl_vector_add(error,uLINK[ori].p);

//MatCpyf( err , error , 3 , 1);
    for (i=0; i<3; i++)
    {
        err[i]=error[i];
        //gsl_vector_set(err,i,gsl_vector_get(error,i));
    }

MatPseudoInvf( rot, uLINK[now].R, 3, 3) ;
MatMulf( rot2, rot, uLINK[ori].R , 3, 3, 3) ;
MatMulf( rot, rot2, R , 3, 3, 3) ;
rot2omega_f(rot,error);
MatMulf( error2, uLINK[now].R, error, 3, 3, 1);
//MatCpyf( *err+3 , error2 , 3 , 1);
//    pinv( rot, uLINK[now].R);
//    gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, rot, uLINK[ori].R, 0.0, rot2);
//    gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, rot2, R, 0.0, rot);
//    rot2omega(rot,error);
//    gsl_blas_dgemv(CblasNoTrans, 1.0, uLINK[now].R, error, 0.0, error2);
//
    for (i=0; i<3; i++)
    {
        err[i+3]=error2[i];
        //gsl_vector_set(err,i+3,gsl_vector_get(error2,i));
    }


}

