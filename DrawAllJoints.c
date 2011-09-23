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

void DrawAllJoints(SuLINK uLINK[],int j)
{

    if (j != 0)
    {

        if (uLINK[j].isPolygon==1)
        {
            DrawPolygon(uLINK,j);
        }

        if (uLINK[j].mother != 0)
        {
            Connect3D(uLINK,j);
        }

        DrawCylinder(uLINK,j);

        DrawAllJoints(uLINK,uLINK[j].sister);
        DrawAllJoints(uLINK,uLINK[j].child);
    }

}
