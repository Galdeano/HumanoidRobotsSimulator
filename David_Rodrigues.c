#include <math.h>

#include "d:\Mb\StLib\Mat.h"

#include "David_Rodrigues.h"


void David_Rodrigues(float *rot,float *w,float dt)
{
    float d,th;
    float wn[3], w_wedge[9], tmp[9];

    d=MatNormf( w,3);
    th = d*dt;

    MatMulScalf( wn, 1/d, w , 3, 1); // normarized vector

    MatTilde( w_wedge, wn);

    MatI( rot, 3); //R = eye(3) + w_wedge * sin(th) + w_wedge^2 * (1-cos(th));

    MatMulScalf( tmp , (float)sin(th), w_wedge , 3, 3);
    MatAddf( rot , rot , tmp , 3 , 3 ) ;


    MatMulf( tmp, w_wedge, w_wedge, 3, 3, 3) ;
    MatMulScalf( tmp, 1.f-(float)cos(th), tmp, 3, 3);
    MatAddf( rot , rot , tmp , 3 , 3 ) ;

}



