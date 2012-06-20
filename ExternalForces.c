#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_math.h>
#include "uLINK.h"
#include "Cross.h"
#include "CalcCoM.h"
#include "TotalMass.h"
#include "ExternalForces.h"

#include "Setup.h"


void ExternalForces(SuLINK uLINK[],State *Status,int j,gsl_vector * f,gsl_vector * t)
{

    gsl_vector_set_zero(f);
    gsl_vector_set_zero(t);

    if (j == 0)
    {
        return;
    }

//    static gsl_vector * ftmp;
//    static gsl_vector * ttmp;
//    static int init_tmp=1;
//    if (init_tmp==1)
//    {
//        ftmp = gsl_vector_calloc (3);
//        ttmp = gsl_vector_calloc (3);
//        init_tmp=0;
//    }
    static gsl_matrix * vert;
    static gsl_vector * v;
    static int init_tmp=1;
    if (init_tmp==1)
    {
        vert = gsl_matrix_calloc (3, 8);
        v = gsl_vector_calloc (3);
        init_tmp=0;
    }


    double res;

    gsl_vector * ftmp = gsl_vector_calloc (3);
    gsl_vector * ttmp = gsl_vector_calloc (3);

    gsl_vector_set_zero(ftmp);
    gsl_vector_set_zero(ttmp);
    if (uLINK[j].isPolygon==1)
    {
        //int Kf = 1000000;        //1.0E+4 Rigidite (N/m),           //40000
        //int Df = 1000;        //1.0E+3 viscosite (N/(m/s)) du sol    //
//        int Kf = 1000000;        //1.0E+4 Rigidite (N/m),           //40000
//        int Df = 1000;        //1.0E+3 viscosite (N/(m/s)) du sol    //
//        int Kf = 100000000;        //1.0E+4 Rigidite (N/m),           //sherpa
//        int Df = 20000;        //1.0E+3 viscosite (N/(m/s)) du sol    //
        int Kf = 5000000;        //1.0E+4 Rigidite (N/m),           //20000000
        int Df = 0.5/Dtime;        //1.0E+3 viscosite (N/(m/s)) du sol    //5000

        int i,k;
        double val;
//        gsl_matrix * vert = gsl_matrix_calloc (3, 8);
//        gsl_vector * v = gsl_vector_calloc (3);
        gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, uLINK[j].R, uLINK[j].vert, 0.0, vert);
        for (i = 0; i < 3; ++i)
        {
            for (k = 0; k < 8; ++k)
            {
                val=gsl_matrix_get (vert,i,k)+gsl_vector_get (uLINK[j].p,i);
                gsl_matrix_set (vert,i,k,val);
            }
        }

        if (j==Status->right_foot_ID)
        {
            //right foot
            gsl_vector_set (Status->FootCenter_R,0,(gsl_matrix_get(vert,0,0)+gsl_matrix_get(vert,0,2))/2);
            gsl_vector_set (Status->FootCenter_R,1,(gsl_matrix_get(vert,1,0)+gsl_matrix_get(vert,1,2))/2);
            gsl_vector_set (Status->FootCenter_R,2,(gsl_matrix_get(vert,2,0)+gsl_matrix_get(vert,2,2))/2);
            gsl_vector_set_zero(Status->forCoP_R);
            gsl_vector_set_zero(Status->posCoP_R);
            Status->right_scale=0.0;
            Status->support &= (~0x01); //foot right support=0
        }

        if (j==Status->left_foot_ID)
        {
            //left foot
            gsl_vector_set (Status->FootCenter_L,0,(gsl_matrix_get(vert,0,0)+gsl_matrix_get(vert,0,2))/2);
            gsl_vector_set (Status->FootCenter_L,1,(gsl_matrix_get(vert,1,0)+gsl_matrix_get(vert,1,2))/2);
            gsl_vector_set (Status->FootCenter_L,2,(gsl_matrix_get(vert,2,0)+gsl_matrix_get(vert,2,2))/2);
            gsl_vector_set_zero(Status->forCoP_L);
            gsl_vector_set_zero(Status->posCoP_L);
            Status->left_scale=0.0;
            Status->support &= (~0x02); //foot left support=0
        }

        for (i = 0; i < 8; ++i)
        {
            if (gsl_matrix_get(vert,2,i)<uLINK[j].supportHeight)
            {
                gsl_matrix_get_col(v, vert, i);
                Cross(uLINK[j].w,v,1);
                gsl_vector_add (v, uLINK[j].vo);
                gsl_vector_set (ftmp,0,Df*gsl_vector_get(v,0));
                gsl_vector_set (ftmp,1,Df*gsl_vector_get(v,1));
                gsl_vector_set (ftmp,2,Kf*(gsl_matrix_get(vert,2,i)-uLINK[j].supportHeight)+Df*gsl_vector_get(v,2));
                if (gsl_vector_get (ftmp,2)>0)
                {
                    gsl_vector_set(ftmp,2,0);
                }
                    gsl_vector_add(f, ftmp);
                    gsl_vector_memcpy (ttmp, ftmp);
                    gsl_matrix_get_col(v, vert, i);
                    //gsl_vector_sub(v,uLINK[j].p);
                    Cross(v,ttmp,1);
                    gsl_vector_add(t, ttmp);
                    //gsl_vector_add(v,uLINK[j].p);

                    //gsl_vector_scale (ftmp, -0.01);
                    //DrawForceMarker(v,ftmp);
                    gsl_vector_set(uLINK[j].isContact,i,1);
                    gsl_matrix_set_col (uLINK[j].posContact, i, v);
                    gsl_matrix_set_col (uLINK[j].forContact, i, ftmp);

                    if (j==Status->right_foot_ID)
                    {
                        gsl_vector_set (v, 2, 0);
                        gsl_vector_scale (v, gsl_vector_get (ftmp,2));
                        Status->right_scale=Status->right_scale+gsl_vector_get (ftmp,2);
                        //gsl_vector_scale (ftmp, -0.0025);
                        gsl_vector_add(Status->forCoP_R,ftmp);
                        gsl_vector_add(Status->posCoP_R,v);
                        Status->support |= 0x01;
                    }

                    if (j==Status->left_foot_ID)
                    {
                        gsl_vector_set (v, 2, 0);
                        gsl_vector_scale (v, gsl_vector_get (ftmp,2));
                        Status->left_scale=Status->left_scale+gsl_vector_get (ftmp,2);
                        //gsl_vector_scale (ftmp, -0.0025);
                        gsl_vector_add(Status->forCoP_L,ftmp);
                        gsl_vector_add(Status->posCoP_L,v);
                        Status->support |= 0x02;
                    }

//                }
//                else
//                {
//                    gsl_vector_set(uLINK[j].isContact,i,0);
//                }
            }
            else
            {
                gsl_vector_set(uLINK[j].isContact,i,0);
            }
        }

//        gsl_matrix_free(vert);
//        gsl_vector_free(v);
    }



    ExternalForces(uLINK,Status,uLINK[j].child,ftmp,ttmp);

    gsl_vector_add (f, ftmp);
    gsl_vector_add (t, ttmp);



    if (j != 1)
    {
        gsl_blas_ddot (uLINK[j].hv, f, &res);
        uLINK[j].uef = res;
        gsl_blas_ddot (uLINK[j].hw, t, &res);
        uLINK[j].uef = res + uLINK[j].uef ;
    }
    /* //if j ~= 1
     * //    uLINK(j).u = uLINK(j).hv' * f + uLINK(j).hw' * t;  % joint driving force
     * //end
     */


    /* //% return force to mother, with sisters
     *     //gsl_vector_set_zero(ftmp);
     *     //gsl_vector_set_zero(ttmp);
     */
    ExternalForces(uLINK,Status,uLINK[j].sister,ftmp,ttmp);

    gsl_vector_add (f, ftmp);
    gsl_vector_add (t, ttmp);


    gsl_vector_free(ftmp);
    gsl_vector_free(ttmp);

}




