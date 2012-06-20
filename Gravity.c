#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_math.h>
#include "uLINK.h"
#include "Cross.h"
#include "CalcCoM.h"
#include "TotalMass.h"
#include "Gravity.h"

#include "Setup.h"


void Gravity(SuLINK uLINK[],State *Status,int j,gsl_vector * f,gsl_vector * t)
{

    static gsl_vector * w_c;
    static int init_tmp=1;
    if (init_tmp==1)
    {
        w_c = gsl_vector_calloc (3);
        init_tmp=0;
    }

    gsl_vector_set_zero(f);
    gsl_vector_set_zero(t);

    if (j == 0)
    {
        return;
    }

    double res;

    gsl_vector * ftmp = gsl_vector_calloc (3);
    gsl_vector * ttmp = gsl_vector_calloc (3);


    gsl_vector_set_zero(ftmp);
    gsl_vector_set_zero(ttmp);
//    gsl_vector * w_c = gsl_vector_calloc (3);
    gsl_blas_dgemv(CblasNoTrans, 1.0, uLINK[j].R, uLINK[j].c, 0.0, w_c);
    gsl_vector_add (w_c, uLINK[j].p);

    gsl_vector_set (ftmp, 2, 9.81*uLINK[j].m);
    /* ////w_c = uLINK(j).R * uLINK(j).c + uLINK(j).p; % Centre de masse
     * ////f3 = -[0 0 -uLINK(j).m * G]'; % Gravite
     */
    gsl_vector_memcpy (ttmp, ftmp);
    Cross(w_c,ttmp,1);
    /*////t3 = cross(w_c, f3);*/
    gsl_vector_add (f, ftmp);
    gsl_vector_add (t, ttmp);
//    gsl_vector_free(w_c);
    /* ////f = f + f3;
     * ////t = t + t3;
     * ////
     */

    /* //% from child link
     *     //gsl_vector_set_zero(ftmp);
     *     //gsl_vector_set_zero(ttmp);
     */
    Gravity(uLINK,Status,uLINK[j].child,ftmp,ttmp);
    /* //[f1,t1] = InverseDynamics(uLINK(j).child);
     * //
     */
    gsl_vector_add (f, ftmp);
    gsl_vector_add (t, ttmp);
    /* //f = f + f1;
     * //t = t + t1;
     */


    if (j != 1)
    {
        gsl_blas_ddot (uLINK[j].hv, f, &res);
        uLINK[j].ug = res;
        gsl_blas_ddot (uLINK[j].hw, t, &res);
        uLINK[j].ug = res + uLINK[j].ug ;
    }
    /* //if j ~= 1
     * //    uLINK(j).u = uLINK(j).hv' * f + uLINK(j).hw' * t;  % joint driving force
     * //end
     */


    /* //% return force to mother, with sisters
     *     //gsl_vector_set_zero(ftmp);
     *     //gsl_vector_set_zero(ttmp);
     */
    Gravity(uLINK,Status,uLINK[j].sister,ftmp,ttmp);
    /*//[f2,t2] = InverseDynamics(uLINK(j).sister);*/
    gsl_vector_add (f, ftmp);
    gsl_vector_add (t, ttmp);
    /* //f = f + f2;
     * //t = t + t2;
     */

    gsl_vector_free(ftmp);
    gsl_vector_free(ttmp);

}




