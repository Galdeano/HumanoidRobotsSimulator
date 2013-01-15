#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_math.h>
#include "uLink.h"
#include "Connect3D.h"
#include "Setup.h"
#include <GL/gl.h>
#include <GL/glu.h>
#include "DrawLight.h"

void Connect3D(SuLINK uLINK[],int j)
{

    glLineWidth( 5.0f );
#if colorsGL
    glColor3ub(0,0,0);
#endif
#if materials
    set_material(&black_rubber);
#endif

    glBegin(GL_LINES);
    glVertex3d(gsl_vector_get (uLINK[j].p,0),gsl_vector_get (uLINK[j].p,1),gsl_vector_get (uLINK[j].p,2));
    glVertex3d(gsl_vector_get (uLINK[uLINK[j].mother].p,0),gsl_vector_get (uLINK[uLINK[j].mother].p,1),gsl_vector_get (uLINK[uLINK[j].mother].p,2));
    glEnd();

}
