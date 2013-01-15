#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_math.h>
#include "uLink.h"
#include "DrawAllJoints.h"

#include <GL/gl.h>
#include <GL/glu.h>
#include "DrawPolygon.h"
#include "DrawCylinder.h"
#include "Connect3D.h"
#include "Setup.h"
#include "ObjLoader.h"

void DrawAllJoints(SuLINK uLINK[],int j)
{

    if (j != 0)
    {
#if LoadObj
        int i,k;
        glPushMatrix();
        GLdouble rotgl[16];


        for (i = 0; i < 3; ++i)
        {
            for (k = 0; k < 3; ++k)
            {
                rotgl[i*4+k] = gsl_matrix_get (uLINK[j].R, k,i);
            }
            rotgl[i*4+3]=0.0;
            //rotgl[i+12]=gsl_vector_get(uLINK[j].p,i);
        }
        for (i = 0; i < 3; ++i)
        {
            rotgl[i+12]=gsl_vector_get(uLINK[j].p,i);//+gsl_vector_get(uLINK[j].obj_offset,i);
        }
        rotgl[15]=1.0;
//0.035000 0.000000 0.125000

        glMultMatrixd(rotgl);

        for (i = 0; i < 3; ++i)
        {
            for (k = 0; k < 3; ++k)
            {
                rotgl[i*4+k] = 0;
            }
            rotgl[i*4+3]=0.0;

        }
        for (i = 0; i < 3; ++i)
        {
            rotgl[i+12]=gsl_vector_get(uLINK[j].obj_offset,i);
        }
        rotgl[0]=1.0;
        rotgl[5]=1.0;
        rotgl[10]=1.0;
        rotgl[15]=1.0;
        glMultMatrixd(rotgl);
        draw_model(&(uLINK[j].Mesh_obj));

        //DrawOBJ(uLINK[j].obj);
        glPopMatrix();
#else

        if (uLINK[j].isPolygon==1)
        {
            DrawPolygon(uLINK,j);
        }

        if (uLINK[j].mother != 0)
        {
            Connect3D(uLINK,j);
        }


        if (j != 1)
        {
            DrawCylinder(uLINK,j);
        }

#endif

        DrawAllJoints(uLINK,uLINK[j].sister);
        DrawAllJoints(uLINK,uLINK[j].child);
    }

}
