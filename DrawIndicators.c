#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_math.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include "uLINK.h"
#include "DrawForceMarker.h"
#include "CalcCoM.h"
#include "CalcCoP.h"
#include "DrawMarker.h"
#include "DrawIndicators.h"
#include "setup.h"


void DrawIndicators(SuLINK uLINK[],State *Status,gsl_vector * com,gsl_vector * CoP,int ground)
{
    int i,j,k;
    float RadToDeg = 180/M_PI;

    CalcCoM(uLINK,com);
    glColor3ub(0,0,255);
    if (!ground)
    {
        gsl_vector_set (com, 2, 0);
    }
    DrawMarker(com);

    double f=0.0;
    gsl_vector_set_zero(CoP);
    f=CalcCoP(uLINK,CoP,1);
    if (f!=0.0)
    {
        glColor3ub(255,0,0);
        gsl_vector_scale (CoP, 1/f);
        gsl_vector_set (CoP, 2, 0);
        DrawMarker(CoP);
    }


    glColor3ub(0,255,0);
    if (Status->desired_support==1)
    {
        DrawMarker(Status->FootCenter_R);
    }
    if (Status->desired_support==2)
    {
        DrawMarker(Status->FootCenter_L);
    }
    if (Status->desired_support==3)
    {
        DrawMarker(Status->FootCenter_R);
        DrawMarker(Status->FootCenter_L);
    }

    gsl_vector * pos = gsl_vector_calloc (3);
    gsl_vector * Visu = gsl_vector_calloc (3);

    glColor3ub(0,255,255);
    if (Status->right_scale!=0.0)
    {
        gsl_vector_memcpy (pos,Status->posCoP_R);
        gsl_vector_scale (pos, 1/Status->right_scale);
    }
    gsl_vector_memcpy (Visu,Status->forCoP_R);
    gsl_vector_scale (Visu, -0.0025);
    DrawForceMarker(pos,Visu);

    if (Status->left_scale!=0.0)
    {
        gsl_vector_memcpy (pos,Status->posCoP_L);
        gsl_vector_scale (pos, 1/Status->left_scale);
    }
    gsl_vector_memcpy (Visu,Status->forCoP_L);
    gsl_vector_scale (Visu, -0.0025);
    DrawForceMarker(pos,Visu);


    #if VisuArticularsLimits

    GLUquadric* params = gluNewQuadric();
    gluQuadricDrawStyle(params,GLU_LINE);
    GLdouble rotgl[16];
    glLineWidth( 1.0f );
    glColor3ub(0,0,255);

    for (j=0; j<(Status->ddl-6); j++)
    {
        glPushMatrix();

        glGetDoublev(GL_MODELVIEW_MATRIX, rotgl);//charge avec identitee
        for (i = 0; i < 3; ++i)
            for (k = 0; k < 3; ++k)
                rotgl[i*4+k] = gsl_matrix_get (uLINK[j+1].R, k,i);

        glTranslated(gsl_vector_get (uLINK[j+2].p,0),gsl_vector_get (uLINK[j+2].p,1),gsl_vector_get (uLINK[j+2].p,2));
        glMultMatrixd(rotgl);
        if (gsl_vector_get (uLINK[j+2].a,0)==1)
            glRotated(90,0,1,0);
        if (gsl_vector_get (uLINK[j+2].a,1)==1)
            glRotated(90,1,0,0);
        gluPartialDisk(params,0.04,0.12,20,1,180+(uLINK[j+2].qmin)*RadToDeg,-1*(-uLINK[j+2].qmax+uLINK[j+2].qmin)*RadToDeg);
        glPopMatrix();
    }

    gluDeleteQuadric(params);

    #endif

    gsl_vector_free(pos);
    gsl_vector_free(Visu);

}


