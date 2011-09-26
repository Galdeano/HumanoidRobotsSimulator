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

    double res;

    gsl_vector * ftmp = gsl_vector_calloc (3);
    gsl_vector * ttmp = gsl_vector_calloc (3);



    gsl_vector_set_zero(ftmp);
    gsl_vector_set_zero(ttmp);
    if (uLINK[j].isPolygon==1 && !Suspendu)
    {
        gsl_vector * com = gsl_vector_calloc (3);
        CalcCoM(uLINK,com);
        if (j==Status->right_foot_ID)
        {
/*            //right foot*/
            gsl_vector_set (ftmp, 2, 9.81*TotalMass(uLINK,1));
            gsl_vector_scale(ftmp,-1*(1.0-Status->distribution_y));
/*             //gsl_vector_scale(ftmp,-1*(Status->distribution_y));
 *             //gsl_vector_scale(ftmp,0.5);
 */

            gsl_vector_add (f, ftmp);
            gsl_vector_memcpy (ttmp, Status->FootCenter_R);
            gsl_vector_set (ttmp,0,gsl_vector_get (com,0));
            Cross(ttmp,ftmp,0);
            gsl_vector_add (t, ttmp);

/* //            if (Status->right_scale!=0.0)
 * //            {
 * //                gsl_vector_memcpy (pos,Status->posCoP_R);
 * //                gsl_vector_scale (pos, 1/Status->right_scale);
 * //            }
 * //            gsl_vector_memcpy (Visu,Status->forCoP_R);
 * //            gsl_vector_add (f, Visu);
 * //            Cross(pos,Visu,1);
 * //            gsl_vector_add (t, Visu);
 */
        }

        if (j==Status->left_foot_ID)
        {
/*            //left foot*/

            gsl_vector_set (ftmp, 2, 9.81*TotalMass(uLINK,1));
            gsl_vector_scale(ftmp,-1*(Status->distribution_y));
/*             //gsl_vector_scale(ftmp,-1*(1.0-Status->distribution_y));
 *             //gsl_vector_scale(ftmp,0.5);
 */

            gsl_vector_add (f, ftmp);
            gsl_vector_memcpy (ttmp, Status->FootCenter_L);
            gsl_vector_set (ttmp,0,gsl_vector_get (com,0));
            Cross(ttmp,ftmp,0);
            gsl_vector_add (t, ttmp);

/*             //gsl_vector_add (f, ftmp);
 *             //Cross(Status->FootCenter_L,ftmp,1);
 *             //gsl_vector_add (t, ftmp);
 * //            if (Status->left_scale!=0.0)
 * //            {
 * //                gsl_vector_memcpy (pos,Status->posCoP_L);
 * //                gsl_vector_scale (pos, 1/Status->left_scale);
 * //            }
 * //            gsl_vector_memcpy (Visu,Status->forCoP_L);
 * //            gsl_vector_add (f, Visu);
 * //            Cross(pos,Visu,1);
 * //            gsl_vector_add (t, Visu);
 */

        }
        gsl_vector_free(com);
    }



    ExternalForces(uLINK,Status,uLINK[j].child,ftmp,ttmp);

    gsl_vector_add (f, ftmp);
    gsl_vector_add (t, ttmp);



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
    ExternalForces(uLINK,Status,uLINK[j].sister,ftmp,ttmp);

    gsl_vector_add (f, ftmp);
    gsl_vector_add (t, ttmp);


    gsl_vector_free(ftmp);
    gsl_vector_free(ttmp);

}




