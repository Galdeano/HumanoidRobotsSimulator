#include "uLINK_f.h"
#include "Rodrigues_f.h"

#include "Mat.h"

#include "ForwardKinematics_f.h"

void ForwardKinematics_f(Struct_uLINK uLINK[],int j)
{

    if (j == 0)
    {
        return;
    }

    if (j != 1)
    {
        float rot[9];
        /*        //position and orientation*/
        MatMulf( uLINK[j].p, uLINK[uLINK[j].mother].R, uLINK[j].b, 3, 3, 1) ;
        MatAddf( uLINK[j].p, uLINK[j].p, uLINK[uLINK[j].mother].p, 3, 1) ;
        /*        //uLINK[j].p = uLINK[uLINK[j].mother].R * uLINK[j].b + uLINK[uLINK[j].mother].p;*/

        Rodrigues_f(rot,uLINK[j].a,uLINK[j].q);
        MatMulf( uLINK[j].R, uLINK[uLINK[j].mother].R, rot, 3, 3, 3) ;
        /*        //uLINK[j].R = uLINK[uLINK[j].mother].R * Rodrigues(uLINK[j].a, uLINK[j].q);*/

        /*        //spatial velocity*/
        MatMulf( uLINK[j].hw, uLINK[uLINK[j].mother].R, uLINK[j].a, 3, 3, 1) ;
        MatProVectorielf( uLINK[j].hv, uLINK[j].p, uLINK[j].hw) ;
        /* //        hw = uLINK(mom).R * uLINK(j).a;  //axis vector in world frame
         * //        hv = cross(uLINK(j).p, hw);      //p_i x axis
         * //        uLINK(j).hw = hw;                //store h1 and h2 for future use
         * //        uLINK(j).hv = hv;
         */

    }

    ForwardKinematics_f(uLINK,uLINK[j].sister);
    ForwardKinematics_f(uLINK,uLINK[j].child);


}
