#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_math.h>
#include "DrawForceMarker.h"

#include <GL/gl.h>
#include <GL/glu.h>

void DrawForceMarker(gsl_vector * ori,gsl_vector * vec)
{

    glLineWidth( 1.0f );
    glBegin(GL_LINES);

    //glColor3ub(0,0,255);
    glVertex3d(gsl_vector_get (ori,0),gsl_vector_get (ori,1),gsl_vector_get (ori,2));
    glVertex3d(gsl_vector_get (ori,0)+gsl_vector_get (vec,0),gsl_vector_get (ori,1)+gsl_vector_get (vec,1),gsl_vector_get (ori,2)+gsl_vector_get (vec,2));
    glEnd();

}
