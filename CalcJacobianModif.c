
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include "uLINK.h"
#include "Cross.h"
#include "gsl_length_v.h"
#include "CalcJacobianModif.h"




void CalcJacobianModif( SuLINK uLINK[], gsl_matrix * J, gsl_vector * idx)
{
    int i,n,j,k;
//gsl_length_v(idx)

    gsl_vector * a = gsl_vector_calloc (3);
    gsl_vector * error = gsl_vector_calloc (3);

    for(n=1; n<(gsl_length_v(idx)-1); n++)//attention aux indices qui partent de zero en c
    {
        j=gsl_vector_get(idx,n);
        k=gsl_vector_get(idx,n+1);

        if (j==uLINK[k].mother)
        {
            gsl_blas_dgemv(CblasNoTrans, 1.0, uLINK[j].R, uLINK[j].a, 0.0, a);
        }
        else if (k==uLINK[j].mother)
        {
            gsl_blas_dgemv(CblasNoTrans, 1.0, uLINK[j].R, uLINK[j].a, 0.0, a);
            gsl_vector_scale (a, -1.0);
        }
        else if (k==j)
        {
            gsl_blas_dgemv(CblasNoTrans, 1.0, uLINK[j].R, uLINK[j].a, 0.0, a);
        }
        else
        {
            gsl_blas_dgemv(CblasNoTrans, 1.0, uLINK[j].R, uLINK[j].a, 0.0, a);
            gsl_vector_scale (a, -1.0);
        }

        gsl_vector_memcpy(error,uLINK[(int)gsl_vector_get(idx,gsl_length_v(idx)-1)].p);
        gsl_vector_sub(error,uLINK[j].p);

        Cross(a, error, 1);


        for (i=0; i<3; i++)
        {
            gsl_matrix_set(J,i,j-1,gsl_vector_get(error,i));
            gsl_matrix_set(J,i+3,j-1,gsl_vector_get(a,i));
        }

        //J(:,n-1) = [cross(a, target - uLINK(j).p) ; a ];

    }
    gsl_vector_free(a);
    gsl_vector_free(error);

}



//function J = CalcJacobianModif(idx)
//% Jacobian matrix of current configration in World frame
//global uLINK
//
//
//jsize = length(idx);
//target = uLINK(idx(end)).p;   % absolute target position
//J = zeros(6,jsize-2);
//
//for n=2:jsize-1
//    j = idx(n);
//    k = idx(n+1);
//
//    if j==uLINK(k).mother
//        a = uLINK(j).R * uLINK(j).a;
//    elseif k==uLINK(j).mother
//        a = uLINK(j).R * (-uLINK(j).a);
//    elseif j==k
//        a = uLINK(j).R * uLINK(j).a;
//    else
//        a = uLINK(j).R * (-uLINK(j).a);
//    end
//    J(:,n-1) = [cross(a, target - uLINK(j).p) ; a ];
//%     a = uLINK(j).R * uLINK(j).a;  % joint axis vector in world frame
//%     J(:,n) = [cross(a, target - uLINK(j).p) ; a ];
//end



