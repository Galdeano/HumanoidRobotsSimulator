#include "uLINK_f.h"

#include "Mat.h"

#include "CalcCoM_f.h"
#include "Stabilizator_f.h"

#include "Setup.h"


void Stabilizator_f( Struct_uLINK uLINK[], Struct_State *Status, float *stab)
{
    float com[3];
    CalcCoM_f(uLINK,com);


    MatClrf( stab, (Status->ddl)-6, 1) ;

    if (Status->desired_support==0)
    {

    }


    if (Status->desired_support==3)
    {

    }

    if (Status->desired_support==1)
    {

//        gsl_vector_set (u,10,gsl_vector_get (u,10) - 1000*(gsl_vector_get (Status->FootCenter_R,0) - gsl_vector_get (com,0)));
//        gsl_vector_set (u,11,gsl_vector_get (u,11) + 2000*(gsl_vector_get (Status->FootCenter_R,1) - gsl_vector_get (com,1)));

        //gsl_vector_set (stab,10, - 100*(gsl_vector_get (Status->FootCenter_R,0) - gsl_vector_get (com,0)) + 20*(gsl_vector_get(com,0)-gsl_vector_get(Status->com_old,0))/Dtime);
        //gsl_vector_set (stab,11, + 200*(gsl_vector_get (Status->FootCenter_R,1) - gsl_vector_get (com,1)) - 20*(gsl_vector_get(com,1)-gsl_vector_get(Status->com_old,1))/Dtime);

        stab[4] = (float)(- 10.f*(Status->FootCenter_R[0] - com[0]) + 2.f*(com[0]-Status->com_old[0])/Te);
        stab[5] = (float)(  20.f*(Status->FootCenter_R[1] - com[1]) - 2.f*(com[1]-Status->com_old[1])/Te);
    }

    if (Status->desired_support==2)
    {

//        gsl_vector_set (u,16,gsl_vector_get (u,16) - 1000*(gsl_vector_get (Status->FootCenter_L,0) - gsl_vector_get (com,0)));
//        gsl_vector_set (u,17,gsl_vector_get (u,17) + 2000*(gsl_vector_get (Status->FootCenter_L,1) - gsl_vector_get (com,1)));

        //gsl_vector_set (stab,16, - 100*(gsl_vector_get (Status->FootCenter_L,0) - gsl_vector_get (com,0)) + 20*(gsl_vector_get(com,0)-gsl_vector_get(Status->com_old,0))/Dtime);
        //gsl_vector_set (stab,17, + 200*(gsl_vector_get (Status->FootCenter_L,1) - gsl_vector_get (com,1)) - 20*(gsl_vector_get(com,1)-gsl_vector_get(Status->com_old,1))/Dtime);

        stab[10] = (float)(- 10.f*(Status->FootCenter_L[0] - com[0]) + 2.f*(com[0]-Status->com_old[0])/Te);
        stab[11] = (float)(  20.f*(Status->FootCenter_L[1] - com[1]) - 2.f*(com[1]-Status->com_old[1])/Te);


    }

    MatCpyf( Status->com_old, com, 3, 1) ;

}


