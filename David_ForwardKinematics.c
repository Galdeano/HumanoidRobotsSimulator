#include "David_uLINK.h"
#include "David_Rodrigues.h"

#include "d:\Mb\StLib\Mat.h"

#include "David_ForwardKinematics.h"

void David_ForwardKinematics(Struct_uLINK uLINK[],int j)
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

        David_Rodrigues(rot,uLINK[j].a,uLINK[j].q);
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

    David_ForwardKinematics(uLINK,uLINK[j].sister);
    David_ForwardKinematics(uLINK,uLINK[j].child);


}
