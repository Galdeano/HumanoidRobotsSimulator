#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include "uLINK.h"
#include "Cross.h"
#include "Rodrigues.h"
#include "ForwardAllKinematics.h"



void ForwardAllKinematics(SuLINK uLINK[],int j)
{

    if (j == 0)
    {
        return;
    }
    if (j != 1)
    {
        //position and orientation
        gsl_blas_dgemv(CblasNoTrans, 1.0, uLINK[uLINK[j].mother].R, uLINK[j].b, 0.0, uLINK[j].p);
        gsl_vector_add (uLINK[j].p, uLINK[uLINK[j].mother].p);
//      uLINK[j].p = uLINK[uLINK[j].mother].R * uLINK[j].b + uLINK[uLINK[j].mother].p;
        gsl_matrix * rot = gsl_matrix_calloc (3, 3);
        Rodrigues( rot ,uLINK[j].a, uLINK[j].q);
        gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, uLINK[uLINK[j].mother].R, rot, 0.0, uLINK[j].R);
        gsl_matrix_free(rot);
//      uLINK(j).R = uLINK(mom).R * Rodrigues(uLINK(j).a, uLINK(j).q);
//if (kawa==1)
//{
//
//printf(" j: %d",j);
//printf(" p: \n");
//printf("%8.4f \n",gsl_vector_get (uLINK[j].p,0));
//printf("%8.4f \n",gsl_vector_get (uLINK[j].p,1));
//printf("%8.4f \n \n",gsl_vector_get (uLINK[j].p,2));
//
//    printf(" R: \n");
//printf("%8.4f ",gsl_matrix_get (uLINK[j].R, 0,0));
//printf("%8.4f ",gsl_matrix_get (uLINK[j].R, 0,1));
//printf("%8.4f \n",gsl_matrix_get (uLINK[j].R, 0,2));
//printf("%8.4f ",gsl_matrix_get (uLINK[j].R, 1,0));
//printf("%8.4f ",gsl_matrix_get (uLINK[j].R, 1,1));
//printf("%8.4f \n",gsl_matrix_get (uLINK[j].R, 1,2));
//printf("%8.4f ",gsl_matrix_get (uLINK[j].R, 2,0));
//printf("%8.4f ",gsl_matrix_get (uLINK[j].R, 2,1));
//printf("%8.4f \n \n",gsl_matrix_get (uLINK[j].R, 2,2));
//
//}

        //spatial velocity
        gsl_blas_dgemv(CblasNoTrans, 1.0, uLINK[uLINK[j].mother].R, uLINK[j].a, 0.0, uLINK[j].hw);
        gsl_vector_memcpy (uLINK[j].hv, uLINK[j].hw);
        Cross(uLINK[j].p, uLINK[j].hv, 1);
//        hw = uLINK(mom).R * uLINK(j).a;  //axis vector in world frame
//        hv = cross(uLINK(j).p, hw);      //p_i x axis
//        uLINK(j).hw = hw;                //store h1 and h2 for future use
//        uLINK(j).hv = hv;

        gsl_vector_memcpy (uLINK[j].w, uLINK[j].hw);
        gsl_vector_scale (uLINK[j].w, uLINK[j].dq);
        gsl_vector_add (uLINK[j].w, uLINK[uLINK[j].mother].w);

        gsl_vector_memcpy (uLINK[j].vo, uLINK[j].hv);
        gsl_vector_scale (uLINK[j].vo, uLINK[j].dq);
        gsl_vector_add (uLINK[j].vo, uLINK[uLINK[j].mother].vo);
//        uLINK(j).w  = uLINK(mom).w  + hw * uLINK(j).dq;
//        uLINK(j).vo = uLINK(mom).vo + hv * uLINK(j).dq;
//if (kawa==1)
//{
//
//printf(" j: %d",j);
//printf(" w: \n");
//printf(" dq: %f",uLINK[j].dq);
//printf("%8.4f \n",gsl_vector_get (uLINK[j].hw,0));
//printf("%8.4f \n",gsl_vector_get (uLINK[j].hw,1));
//printf("%8.4f \n \n",gsl_vector_get (uLINK[j].hw,2));
//printf(" vo: \n");
//printf("%8.4f \n",gsl_vector_get (uLINK[j].hv,0));
//printf("%8.4f \n",gsl_vector_get (uLINK[j].hv,1));
//printf("%8.4f \n \n",gsl_vector_get (uLINK[j].hv,2));
//}


//
        //spatial acceleration

        gsl_vector * dhv = gsl_vector_calloc (3);
        gsl_vector * dhw = gsl_vector_calloc (3);
        gsl_vector * tmp = gsl_vector_calloc (3);
        gsl_vector_memcpy (dhv, uLINK[j].hv);
        Cross(uLINK[uLINK[j].mother].w, dhv, 1);
        gsl_vector_memcpy (tmp, uLINK[j].hw);
        Cross(uLINK[uLINK[j].mother].vo, tmp, 1);
        gsl_vector_add (dhv, tmp);
        gsl_vector_memcpy (dhw, uLINK[j].hw);
        Cross(uLINK[uLINK[j].mother].w, dhw, 1);
//        dhv = cross(uLINK(mom).w, hv) + cross(uLINK(mom).vo, hw);
//        dhw = cross(uLINK(mom).w, hw);
//

        gsl_vector_memcpy (uLINK[j].dw, uLINK[j].hw);
        gsl_vector_scale (uLINK[j].dw, uLINK[j].ddq);
        gsl_vector_add (uLINK[j].dw, uLINK[uLINK[j].mother].dw);
        gsl_vector_scale (dhw, uLINK[j].dq);
        gsl_vector_add (uLINK[j].dw, dhw);

        gsl_vector_memcpy (uLINK[j].dvo, uLINK[j].hv);
        gsl_vector_scale (uLINK[j].dvo, uLINK[j].ddq);
        gsl_vector_add (uLINK[j].dvo, uLINK[uLINK[j].mother].dvo);
        gsl_vector_scale (dhv, uLINK[j].dq);
        gsl_vector_add (uLINK[j].dvo, dhv);
//        uLINK(j).dw  = uLINK(mom).dw  + dhw * uLINK(j).dq + hw * uLINK(j).ddq;
//        uLINK(j).dvo = uLINK(mom).dvo + dhv * uLINK(j).dq + hv * uLINK(j).ddq;

        gsl_vector_free(dhv);
        gsl_vector_free(dhw);
        gsl_vector_free(tmp);
    }

    ForwardAllKinematics(uLINK,uLINK[j].sister);
    ForwardAllKinematics(uLINK,uLINK[j].child);

}

