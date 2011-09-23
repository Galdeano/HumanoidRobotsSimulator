#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_math.h>
#include "uLINK.h"
#include "Connect3D.h"

#include <GL/gl.h>
#include <GL/glu.h>

void Connect3D(SuLINK uLINK[],int j)
{
    glLineWidth( 5.0f );
    glBegin(GL_LINES);

    glColor3ub(0,0,0);
    glVertex3d(gsl_vector_get (uLINK[j].p,0),gsl_vector_get (uLINK[j].p,1),gsl_vector_get (uLINK[j].p,2));
    glVertex3d(gsl_vector_get (uLINK[uLINK[j].mother].p,0),gsl_vector_get (uLINK[uLINK[j].mother].p,1),gsl_vector_get (uLINK[uLINK[j].mother].p,2));
    glEnd();
}
