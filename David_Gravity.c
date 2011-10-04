#include "David_uLINK.h"
#include "David_CalcCoM.h"
#include "David_TotalMass.h"

#include "d:\Mb\StLib\Mat.h"

#include "David_Gravity.h"
#include "David_Cross.h"
#include "Setup.h"

#include <stdio.h>


void David_Gravity(Struct_uLINK uLINK[],Struct_State *Status, int j, float *f, float *t)
{

    float ftmp[3], ttmp[3], wc[3];

    MatClrf( f, 3, 1) ;
    MatClrf( t, 3, 1) ;

    if (j == 0)
    {
        return;
    }


    MatClrf( ftmp, 3, 1) ;
    MatClrf( ttmp, 3, 1) ;


    MatMulf( wc, uLINK[j].R, uLINK[j].c, 3, 3, 1) ;
    MatAddf( wc, wc, uLINK[j].p, 3 , 1) ;
    /*//w_c = uLINK(j).R * uLINK(j).c + uLINK(j).p; % Centre de masse*/
    ftmp[2]=9.81f*uLINK[j].m;
    /*//f3 = -[0 0 -uLINK(j).m * G]'; % Gravite*/
    MatProVectorielf( ttmp, wc, ftmp) ;
    /*//t3 = cross(w_c, f3);*/
    MatAddf( f, f, ftmp, 3, 1) ;
    /*//////f = f + f3;*/
    MatAddf( t, t, ttmp, 3, 1) ;
    /*//////t = t + t3;*/




    MatClrf( ftmp, 3, 1) ;
    MatClrf( ttmp, 3, 1) ;

    if (((j==Status->right_foot_ID)||(j==Status->left_foot_ID)) && !Suspendu)
    {
        float com[3];
        David_CalcCoM(uLINK,com);


        if (j==Status->right_foot_ID)
        {
            /*            //right foot*/
            ftmp[2]=-9.81f*David_TotalMass(uLINK,1)*(1.f-Status->distribution_y);
            MatAddf( f, f, ftmp, 3, 1) ;

            MatCpyf( ttmp, Status->FootCenter_R, 3, 1);
            ttmp[0]=com[0];
            David_Cross(ttmp,ftmp,0);
            MatAddf( t, t, ttmp, 3, 1) ;

        }

        if (j==Status->left_foot_ID)
        {
            /*            //left foot*/
            ftmp[2]=-9.81f*David_TotalMass(uLINK,1)*(Status->distribution_y);
            MatAddf( f, f, ftmp, 3, 1) ;

            MatCpyf( ttmp, Status->FootCenter_L, 3, 1);
            ttmp[0]=com[0];
            David_Cross(ttmp,ftmp,0);
            MatAddf( t, t, ttmp, 3, 1) ;
        }
    }

    /*//% from child link*/
    David_Gravity(uLINK,Status,uLINK[j].child,ftmp,ttmp);
    /*//[f1,t1] = InverseDynamics(uLINK(j).child);*/

    MatAddf( f, f, ftmp, 3, 1) ;
    MatAddf( t, t, ttmp, 3, 1) ;
    /* //f = f + f1;
     * //t = t + t1;
     */





    if (j != 1)
    {
        uLINK[j].ug = MatProScalf( uLINK[j].hv, f, 3) + MatProScalf( uLINK[j].hw, t, 3) ;
    }
    /* //if j ~= 1
     * //    uLINK(j).u = uLINK(j).hv' * f + uLINK(j).hw' * t;  % joint driving force
     * //end
     * //
     */


    /*//% return force to mother, with sisters*/
    David_Gravity(uLINK,Status,uLINK[j].sister,ftmp,ttmp);

    MatAddf( f, f, ftmp, 3, 1) ;
    MatAddf( t, t, ttmp, 3, 1) ;
    /* //f = f + f2;
     * //t = t + t2;
     */


}



