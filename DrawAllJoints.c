#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_math.h>
#include "uLINK.h"
#include "DrawAllJoints.h"

#include <GL/gl.h>
#include <GL/glu.h>
#include "DrawPolygon.h"
#include "DrawCylinder.h"
#include "Connect3D.h"
#include "Setup.h"

void DrawAllJoints(SuLINK uLINK[],int j)
{

    if (j != 0)
    {
#if LoadObj
        int i,k;
        glPushMatrix();
        GLdouble rotgl[16];
        glGetDoublev(GL_MODELVIEW_MATRIX, rotgl);//charge avec identitee
        for (i = 0; i < 3; ++i)
        {
            for (k = 0; k < 3; ++k)
            {
                rotgl[i*4+k] = gsl_matrix_get (uLINK[j].R, k,i);
            }
            rotgl[i+12]=gsl_vector_get(uLINK[j].p,i);
        }
        glMultMatrixd(rotgl);
        DrawOBJ(uLINK[j].obj);
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
#endif

        if (j != 1)
        {
            DrawCylinder(uLINK,j);
        }

        DrawAllJoints(uLINK,uLINK[j].sister);
        DrawAllJoints(uLINK,uLINK[j].child);
    }

}
