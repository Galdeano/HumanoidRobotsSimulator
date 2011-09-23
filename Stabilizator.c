#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_math.h>
#include "uLINK.h"
#include "CalcCoM.h"
#include "Stabilizator.h"


void Stabilizator(SuLINK uLINK[],State *Status,gsl_vector * stab,double Dtime,double t)
{

    if (Status->desired_support==0)
    {
        Status->integral_R=0.0;
        Status->integral_L=0.0;

        gsl_vector * com = gsl_vector_calloc (3);
        CalcCoM(uLINK,com);
        gsl_vector_memcpy (Status->com_old,com);
        gsl_vector_free(com);
    }


    if (Status->desired_support==3)
    {
//        double tmp;
//        tmp = gsl_vector_get (u,11);
//        if (Status->right_scale!=0.0)
//        {
//            gsl_vector_set ( u , 11 , (tmp + 5*Status->right_scale*(gsl_vector_get (Status->FootCenter_R,1) - (gsl_vector_get (Status->posCoP_R,1))/Status->right_scale) - 10000*Status->integral_R));
//            //Status->integral_R += 1*(gsl_vector_get (Status->FootCenter_R,1) - gsl_vector_get (Status->posCoP_R,1)*(1/Status->right_scale))*Dtime;
//        }
//
//
//        tmp = gsl_vector_get (u,17);
//        if (Status->left_scale!=0.0)
//        {
//            gsl_vector_set ( u , 17 , (tmp + 5*Status->left_scale*(gsl_vector_get (Status->FootCenter_L,1) - (gsl_vector_get (Status->posCoP_L,1))/Status->left_scale) - 10000*Status->integral_L));
//            //Status->integral_L += 1*(gsl_vector_get (Status->FootCenter_L,1) - gsl_vector_get (Status->posCoP_L,1)*(1/Status->left_scale))*Dtime;
//        }
        gsl_vector * com = gsl_vector_calloc (3);
        CalcCoM(uLINK,com);
        gsl_vector_memcpy (Status->com_old,com);
        gsl_vector_free(com);
   }




    if (Status->desired_support==1)
    {
        gsl_vector * com = gsl_vector_calloc (3);
        CalcCoM(uLINK,com);

//        gsl_vector_set (u,10,gsl_vector_get (u,10) - 1000*(gsl_vector_get (Status->FootCenter_R,0) - gsl_vector_get (com,0)));
//        gsl_vector_set (u,11,gsl_vector_get (u,11) + 2000*(gsl_vector_get (Status->FootCenter_R,1) - gsl_vector_get (com,1)));

        gsl_vector_set (stab,10, - 100*(gsl_vector_get (Status->FootCenter_R,0) - gsl_vector_get (com,0)) + 20*(gsl_vector_get(com,0)-gsl_vector_get(Status->com_old,0))/Dtime);
        gsl_vector_set (stab,11, + 200*(gsl_vector_get (Status->FootCenter_R,1) - gsl_vector_get (com,1)) - 20*(gsl_vector_get(com,1)-gsl_vector_get(Status->com_old,1))/Dtime);

        //uLINK[6].u_joint += -5000*(gsl_vector_get (comdesire,0)-gsl_vector_get (com,0));
        //uLINK[7].u_joint += 10000*(gsl_vector_get (comdesire,1)-gsl_vector_get (com,1));

        gsl_vector_memcpy (Status->com_old,com);
        gsl_vector_free(com);

        Status->integral_R=0.0;
        Status->integral_L=0.0;
    }

    if (Status->desired_support==2)
    {
        gsl_vector * com = gsl_vector_calloc (3);
        CalcCoM(uLINK,com);

//        gsl_vector_set (u,16,gsl_vector_get (u,16) - 1000*(gsl_vector_get (Status->FootCenter_L,0) - gsl_vector_get (com,0)));
//        gsl_vector_set (u,17,gsl_vector_get (u,17) + 2000*(gsl_vector_get (Status->FootCenter_L,1) - gsl_vector_get (com,1)));

        gsl_vector_set (stab,16, - 100*(gsl_vector_get (Status->FootCenter_L,0) - gsl_vector_get (com,0)) + 20*(gsl_vector_get(com,0)-gsl_vector_get(Status->com_old,0))/Dtime);
        gsl_vector_set (stab,17, + 200*(gsl_vector_get (Status->FootCenter_L,1) - gsl_vector_get (com,1)) - 20*(gsl_vector_get(com,1)-gsl_vector_get(Status->com_old,1))/Dtime);

        //uLINK[6].u_joint += -5000*(gsl_vector_get (comdesire,0)-gsl_vector_get (com,0));
        //uLINK[7].u_joint += 10000*(gsl_vector_get (comdesire,1)-gsl_vector_get (com,1));

        gsl_vector_memcpy (Status->com_old,com);
        gsl_vector_free(com);

        Status->integral_R=0.0;
        Status->integral_L=0.0;
    }


}


