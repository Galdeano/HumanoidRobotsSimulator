#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_math.h>
#include "DrawMarker.h"

#include <GL/gl.h>
#include <GL/glu.h>



void DrawMarker(gsl_vector * com)
{
    double d = 0.1;

    glLineWidth( 1.0f );
    glBegin(GL_LINES);
    glVertex3d(gsl_vector_get (com,0)-d,gsl_vector_get (com,1),gsl_vector_get (com,2));
    glVertex3d(gsl_vector_get (com,0)+d,gsl_vector_get (com,1),gsl_vector_get (com,2));
    glEnd();

    glLineWidth( 1.0f );
    glBegin(GL_LINES);
    glVertex3d(gsl_vector_get (com,0),gsl_vector_get (com,1)-d,gsl_vector_get (com,2));
    glVertex3d(gsl_vector_get (com,0),gsl_vector_get (com,1)+d,gsl_vector_get (com,2));
    glEnd();

    glLineWidth( 1.0f );
    glBegin(GL_LINES);
    glVertex3d(gsl_vector_get (com,0),gsl_vector_get (com,1),gsl_vector_get (com,2)-d);
    glVertex3d(gsl_vector_get (com,0),gsl_vector_get (com,1),gsl_vector_get (com,2)+d);
    glEnd();

}

