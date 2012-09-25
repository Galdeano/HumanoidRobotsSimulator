#include "uLINK_f.h"
#include "Mat.h"
#include "Cross_f.h"
#include "CalcJacobianModif_f.h"




void CalcJacobianModif_f(Struct_uLINK uLINK[],double *J,int *idx,int idx_size,int nDoF)
{
    int i,n,j,k;


    static double a[3];
    static double error[3];

    for(n=1; n<(idx_size-1); n++)//attention aux indices qui partent de zero en c
    {
        j=idx[n];
        k=idx[n+1];


        if (j==uLINK[k].mother)
        {
            MatMulf( a, uLINK[j].R, uLINK[j].a, 3 , 3 , 1);
            //gsl_blas_dgemv(CblasNoTrans, 1.0, uLINK[j].R, uLINK[j].a, 0.0, a);
        }
        else if (k==uLINK[j].mother)
        {
            MatMulf( a, uLINK[j].R, uLINK[j].a, 3 , 3 , 1);
            //gsl_blas_dgemv(CblasNoTrans, 1.0, uLINK[j].R, uLINK[j].a, 0.0, a);
            MatMulScalf( a, -1.0, a, 3, 1);
            //gsl_vector_scale (a, -1.0);
        }
        else if (k==j)
        {
            MatMulf( a, uLINK[j].R, uLINK[j].a, 3 , 3 , 1);
            //gsl_blas_dgemv(CblasNoTrans, 1.0, uLINK[j].R, uLINK[j].a, 0.0, a);
        }
        else
        {
            MatMulf( a, uLINK[j].R, uLINK[j].a, 3 , 3 , 1);
            //gsl_blas_dgemv(CblasNoTrans, 1.0, uLINK[j].R, uLINK[j].a, 0.0, a);
            MatMulScalf( a, -1.0, a, 3, 1);
            //gsl_vector_scale (a, -1.0);
        }

        MatCpyf( error , uLINK[idx[idx_size-1]].p, 3 , 1);
        //gsl_vector_memcpy(error,uLINK[(int)gsl_vector_get(idx,gsl_length_v(idx)-1)].p);
        MatSubf( error, error, uLINK[j].p, 3, 1);
        //gsl_vector_sub(error,uLINK[j].p);

        Cross_f( a, error, 1);
        //Cross(a, error, 1);


        for (i=0; i<3; i++)
        {
            J[i*nDoF+(j-2)]=error[i];
            J[(i+3)*nDoF+(j-2)]=a[i];
            //gsl_matrix_set(J,i,j-2,gsl_vector_get(error,i));
            //gsl_matrix_set(J,i+3,j-2,gsl_vector_get(a,i));
        }

    }

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



