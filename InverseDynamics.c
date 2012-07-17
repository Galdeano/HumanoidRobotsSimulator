#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include "uLINK.h"
#include "Hat.h"
#include "Cross.h"
#include "InverseDynamics.h"


void InverseDynamics(SuLINK uLINK[],State *Status,int j,gsl_vector * f,gsl_vector * t)
{
    if (j == 0)
    {
        gsl_vector_set_zero(f);
        gsl_vector_set_zero(t);
        return;
    }
//if j == 0
//    f=[0,0,0]';
//    t=[0,0,0]';
//    return;
//end
//
    double res;

    static gsl_vector * c;
    static gsl_vector * P;
    static gsl_vector * L;
    static gsl_vector * tmp3;
    static gsl_matrix * I;
    static gsl_matrix * tmp;
    static gsl_matrix * tmp2;
    static int init_tmp=1;
    if (init_tmp==1)
    {
        c = gsl_vector_calloc (3);
        P = gsl_vector_calloc (3);
        L = gsl_vector_calloc (3);
        tmp3 = gsl_vector_calloc (3);
        I = gsl_matrix_calloc (3, 3);
        tmp = gsl_matrix_calloc (3, 3);
        tmp2 = gsl_matrix_calloc (3, 3);

        init_tmp=0;
    }

    gsl_vector_set_zero(f);
    gsl_vector_set_zero(t);
//    gsl_vector * c = gsl_vector_calloc (3);
//    gsl_matrix * I = gsl_matrix_calloc (3,3);
//    gsl_matrix * tmp = gsl_matrix_calloc (3,3);
//    gsl_matrix * tmp2 = gsl_matrix_calloc (3,3);
//
//    gsl_vector * P = gsl_vector_calloc (3);
//    gsl_vector * L = gsl_vector_calloc (3);
//    gsl_vector * tmp3 = gsl_vector_calloc (3);

    gsl_vector * ftmp = gsl_vector_calloc (3);
    gsl_vector * ttmp = gsl_vector_calloc (3);

    gsl_blas_dgemv(CblasNoTrans, 1.0, uLINK[j].R, uLINK[j].c, 0.0, c);
    gsl_vector_add (c, uLINK[j].p);
    gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, uLINK[j].R, uLINK[j].I, 0.0, tmp);
    gsl_blas_dgemm (CblasNoTrans, CblasTrans, 1.0, tmp, uLINK[j].R, 0.0, I);
//c = uLINK(j).R * uLINK(j).c + uLINK(j).p;   % center of mass
//I = uLINK(j).R * uLINK(j).I * uLINK(j).R';  % inertia in world frame

    Hat(tmp,c);
//c_hat = hat(c);
    gsl_blas_dgemm (CblasNoTrans, CblasTrans, 1.0, tmp, tmp, 0.0, tmp2);
    gsl_matrix_scale (tmp2, uLINK[j].m);
    gsl_matrix_add (I, tmp2);
//I = I + uLINK(j).m * c_hat * c_hat';

    gsl_vector_memcpy (P, c);
    Cross(uLINK[j].w,P,1);
    gsl_vector_add (P, uLINK[j].vo);
    gsl_vector_scale (P, uLINK[j].m);
//P = uLINK(j).m * (uLINK(j).vo + cross(uLINK(j).w,c));   % linear  momentum
    gsl_vector_memcpy (L, c);
    Cross(L,uLINK[j].vo,0);
    gsl_vector_scale (L, uLINK[j].m);
    gsl_blas_dgemv(CblasNoTrans, 1.0, I, uLINK[j].w, 0.0, tmp3);
    gsl_vector_add (L, tmp3);
//L = uLINK(j).m * cross(c,uLINK(j).vo) + I * uLINK(j).w; % angular momentum

    gsl_vector_memcpy (f, c);
    Cross(uLINK[j].dw,f,1);
    gsl_vector_add (f, uLINK[j].dvo);
    gsl_vector_scale (f, uLINK[j].m);

    gsl_vector_memcpy (tmp3, P);
    Cross(uLINK[j].w,tmp3,1);
    gsl_vector_add (f, tmp3);
//f = uLINK(j).m * (uLINK(j).dvo + cross(uLINK(j).dw,c))   + cross(uLINK(j).w,P);

    gsl_vector_memcpy (t, c);
    Cross(t,uLINK[j].dvo,0);
    gsl_vector_scale (t, uLINK[j].m);

    gsl_blas_dgemv(CblasNoTrans, 1.0, I, uLINK[j].dw, 0.0, tmp3);
    gsl_vector_add (t, tmp3);

    gsl_vector_memcpy (tmp3, P);
    Cross(uLINK[j].vo,tmp3,1);
    gsl_vector_add (t, tmp3);

    gsl_vector_memcpy (tmp3, L);
    Cross(uLINK[j].w,tmp3,1);
    gsl_vector_add (t, tmp3);
//t = uLINK(j).m * cross(c,uLINK(j).dvo) + I * uLINK(j).dw + cross(uLINK(j).vo,P) + cross(uLINK(j).w,L);






//% from child link
    gsl_vector_set_zero(ftmp);
    gsl_vector_set_zero(ttmp);
    InverseDynamics(uLINK,Status,uLINK[j].child,ftmp,ttmp);
//[f1,t1] = InverseDynamics(uLINK(j).child);
//
    gsl_vector_add (f, ftmp);
    gsl_vector_add (t, ttmp);
//f = f + f1;
//t = t + t1;



    if (j != 1)
    {
        gsl_blas_ddot (uLINK[j].hv, f, &res);
        uLINK[j].u = res;
        gsl_blas_ddot (uLINK[j].hw, t, &res);
        uLINK[j].u = res + uLINK[j].u ;
    }
//if j ~= 1
//    uLINK(j).u = uLINK(j).hv' * f + uLINK(j).hw' * t;  % joint driving force
//end
//






//% return force to mother, with sisters
    gsl_vector_set_zero(ftmp);
    gsl_vector_set_zero(ttmp);
    InverseDynamics(uLINK,Status,uLINK[j].sister,ftmp,ttmp);
//[f2,t2] = InverseDynamics(uLINK(j).sister);
    gsl_vector_add (f, ftmp);
    gsl_vector_add (t, ttmp);
//f = f + f2;
//t = t + t2;






    gsl_vector_free(ftmp);
    gsl_vector_free(ttmp);
}


