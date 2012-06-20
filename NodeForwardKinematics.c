#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_math.h>
#include "uLINK.h"
#include "NodeForwardKinematics.h"
#include "Rodrigues.h"
#include "pinv.h"

void NodeForwardKinematics(SuLINK uLINK[],int j, int sender)
{
//printf("Robot!!! %d %d\n",j,sender);
//printf("Robot!!!\n");
    if (j == 0)
    {
        return;
    }

    static gsl_matrix * rot;
    static gsl_matrix * rot2;
    static int init_tmp=1;
    if (init_tmp==1)
    {
        rot = gsl_matrix_calloc (3, 3);
        rot2 = gsl_matrix_calloc (3, 3);
        init_tmp=0;
    }
//    gsl_matrix_set_zero(rot);
//    gsl_matrix_set_zero(rot2);

    if (sender!=0)
    {
        if (uLINK[sender].upper==j)
        {
            if (uLINK[j].child==sender)
            {
//                gsl_matrix * rot = gsl_matrix_calloc (3, 3);
//                gsl_matrix * rot2 = gsl_matrix_calloc (3, 3);
                Rodrigues( rot ,uLINK[sender].a, uLINK[sender].q);
                pinv(rot2,rot);
                gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, uLINK[sender].R, rot2, 0.0, uLINK[j].R);
//                gsl_matrix_free(rot);
//                gsl_matrix_free(rot2);
                gsl_blas_dgemv(CblasNoTrans, 1.0, uLINK[j].R, uLINK[sender].b, 0.0, uLINK[j].p);
                gsl_vector_scale (uLINK[j].p, -1.0);
                gsl_vector_add (uLINK[j].p, uLINK[sender].p);//pas add
            }
            else
            {
//                gsl_matrix * rot = gsl_matrix_calloc (3, 3);
//                gsl_matrix * rot2 = gsl_matrix_calloc (3, 3);
                Rodrigues( rot ,uLINK[sender].a, uLINK[sender].q);
                pinv(rot2,rot);
                gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, uLINK[sender].R, rot2, 0.0, uLINK[uLINK[sender].mother].R);
//                gsl_matrix_free(rot);
//                gsl_matrix_free(rot2);
                gsl_blas_dgemv(CblasNoTrans, 1.0, uLINK[uLINK[sender].mother].R, uLINK[sender].b, 0.0, uLINK[uLINK[sender].mother].p);
                gsl_vector_scale (uLINK[uLINK[sender].mother].p, -1.0);
                gsl_vector_add (uLINK[uLINK[sender].mother].p, uLINK[sender].p);

                NodeForwardKinematics(uLINK,1,0);
            }
        }
        else
        {
            gsl_blas_dgemv(CblasNoTrans, 1.0, uLINK[uLINK[j].mother].R, uLINK[j].b, 0.0, uLINK[j].p);
            gsl_vector_add (uLINK[j].p, uLINK[uLINK[j].mother].p);
//            gsl_matrix * rot = gsl_matrix_calloc (3, 3);
            Rodrigues( rot ,uLINK[j].a, uLINK[j].q);
            gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, uLINK[uLINK[j].mother].R, rot, 0.0, uLINK[j].R);
//            gsl_matrix_free(rot);
        }
    }

    if (uLINK[j].upper!=sender)
    {
        NodeForwardKinematics(uLINK,uLINK[j].upper,j);
    }

    if (uLINK[j].sister!=sender)
    {
        NodeForwardKinematics(uLINK,uLINK[j].sister,j);
    }

    if (uLINK[j].child!=sender)
    {
        NodeForwardKinematics(uLINK,uLINK[j].child,j);
    }

}
