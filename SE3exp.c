#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_math.h>
#include "uLink.h"
#include "Cross.h"
#include "Hat.h"
#include "SE3exp.h"

#include "Setup.h"

void SE3exp(SuLINK uLINK[],int j)
{
//function [p2, R2] = SE3exp(j, dt)
//SE3exp.m actualise la position et lÅforientation par lÅfutilisation de la vitesse spatiale.
// see Murray, Li, Sastry p.42

    static gsl_vector * tmp;
    static gsl_vector * wn;
    static gsl_vector * vo;
    static gsl_matrix * w_wedge;
    static gsl_matrix * w_wedge2;
    static gsl_matrix * eye;
    static gsl_matrix * rot;
    static gsl_matrix * drot;
    static gsl_matrix * tmp4;
    static gsl_vector * tmp2;
    static gsl_vector * tmp3;
    static int init_tmp=1;
    if (init_tmp==1)
    {
        tmp = gsl_vector_calloc (3);
        wn = gsl_vector_calloc (3);
        vo = gsl_vector_calloc (3);
        w_wedge = gsl_matrix_calloc (3, 3);
        w_wedge2 = gsl_matrix_calloc (3, 3);
        eye = gsl_matrix_calloc (3, 3);
        rot = gsl_matrix_calloc (3, 3);
        drot = gsl_matrix_calloc (3, 3);
        tmp4 = gsl_matrix_calloc (3, 3);
        tmp2 = gsl_vector_calloc (3);
        tmp3 = gsl_vector_calloc (3);

        gsl_matrix_set_identity (eye);
        init_tmp=0;
    }

//    gsl_vector * wn = gsl_vector_calloc (3);
//    gsl_vector * vo = gsl_vector_calloc (3);
//    gsl_matrix * w_wedge = gsl_matrix_calloc (3, 3);
//    gsl_matrix * w_wedge2 = gsl_matrix_calloc (3, 3);
//    gsl_matrix * eye = gsl_matrix_calloc (3, 3);
//    gsl_matrix * rot = gsl_matrix_calloc (3, 3);
//    gsl_matrix * drot = gsl_matrix_calloc (3, 3);
//    gsl_matrix * tmp4 = gsl_matrix_calloc (3, 3);
//    gsl_vector * tmp2 = gsl_vector_calloc (3);
//    gsl_vector * tmp3 = gsl_vector_calloc (3);
//
//    gsl_matrix_set_identity (eye);


    double norm_w = gsl_blas_dnrm2 (uLINK[j].w);
    if (norm_w < pow (2, - 53))//0.000000001
    {
//        gsl_vector * tmp = gsl_vector_calloc (3);
        gsl_vector_memcpy (tmp, uLINK[j].vo);
        gsl_vector_scale (tmp, Dtime);
        gsl_vector_add (uLINK[j].p, tmp);
//        gsl_vector_free(tmp);
//p2 = uLINK(j).p + dt * uLINK(j).vo;
//R2 = uLINK(j).R;
    }
    else
    {
        double th = norm_w*Dtime;

        gsl_vector_memcpy (wn, uLINK[j].w);
        gsl_vector_scale (wn, 1/norm_w);
//        wn = uLINK(j).w/norm_w;
//        // normarized vector

        gsl_vector_memcpy (vo, uLINK[j].vo);
        gsl_vector_scale (vo, 1/norm_w);
//        vo = uLINK(j).vo/norm_w;
//
        Hat(w_wedge,wn);
//        w_wedge = [0 -wn(3) wn(2); wn(3) 0 -wn(1); -wn(2) wn(1) 0];
        gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, w_wedge, w_wedge, 0.0, w_wedge2);


        gsl_matrix_scale ( w_wedge, (sin(th)));
        gsl_matrix_memcpy(drot, w_wedge);

        gsl_matrix_scale ( w_wedge2, (1-cos(th)));
        gsl_matrix_add (drot, w_wedge2);

        gsl_matrix_memcpy (rot, drot);

        gsl_matrix_add (rot, eye);
//        drot = w_wedge * sin(th) + w_wedge^2 * (1-cos(th));
//        rot  = eye(3) + drot;


        gsl_blas_dgemv(CblasNoTrans, 1.0, rot, uLINK[j].p, 0.0, tmp2);
        gsl_vector_memcpy (uLINK[j].p, tmp2);

        gsl_vector_memcpy (tmp2, wn);
        Cross(tmp2,vo,0);
        gsl_blas_dgemv(CblasNoTrans, 1.0, drot, tmp2, 0.0, tmp3);
        gsl_vector_sub (uLINK[j].p, tmp3);

        gsl_matrix_set_zero(tmp4);
        gsl_blas_dger (1.0, wn, wn, tmp4);
        gsl_blas_dgemv(CblasNoTrans, 1.0, tmp4, vo, 0.0, tmp2);
        gsl_vector_scale ( tmp2, th);
        gsl_vector_add (uLINK[j].p, tmp2);

//        p2 = rot * uLINK(j).p -drot*cross(wn, vo) + wn * wn' * vo * th;
        gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, rot, uLINK[j].R, 0.0, tmp4);
        gsl_matrix_memcpy (uLINK[j].R, tmp4);
//             R2 = rot * uLINK(j).R;
//             end

//        gsl_vector_free(wn);
//        gsl_vector_free(vo);
//        gsl_matrix_free(w_wedge);
//        gsl_matrix_free(w_wedge2);
//
//        gsl_matrix_free(eye);
//        gsl_matrix_free(rot);
//        gsl_matrix_free(drot);
//
//        gsl_matrix_free(tmp4);
//        gsl_vector_free(tmp2);
//        gsl_vector_free(tmp3);
    }

}


