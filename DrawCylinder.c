#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_math.h>
#include "uLINK.h"
#include "DrawCylinder.h"
#include "Setup.h"

#include <GL/gl.h>
#include <GL/glu.h>

void   DrawCylinder(SuLINK uLINK[],int j)
{

    double radius    = 0.015;
    double len       = 0.045;
//    double radius    = 0.02;
//    double len       = 0.06;
    int i,k;
    int faces =12;
    double RadToDeg = 180/M_PI;

    glPushMatrix();
    //glLoadIdentity();
    GLUquadric* params = gluNewQuadric();
    glLineWidth( 1.0f );

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


//DrawMarker(uLINK[j].p);
    //glTranslated(gsl_vector_get(uLINK[j].p,0),gsl_vector_get(uLINK[j].p,1),gsl_vector_get(uLINK[j].p,2));


    glMultMatrixd(rotgl);
    if (gsl_vector_get (uLINK[j].a,0)==1)
        glRotated(90,0,1,0);
    if (gsl_vector_get (uLINK[j].a,1)==1)
        glRotated(90,1,0,0);
    glTranslated(0,0,-len/2);

    gluQuadricDrawStyle(params,GLU_FILL);
    //GLU_LINE
#if !VisuTorquesColor
    if (uLINK[j].color==0)//red
        glColor3ub(255,0,0);
    if (uLINK[j].color==1)//green
        glColor3ub(0,255,0);
    if (uLINK[j].color==2)//blue
        glColor3ub(0,0,255);
    if (uLINK[j].color==3)//grey
        glColor3ub(124,124,124);
    if (uLINK[j].color==4)//yellow
        glColor3ub(255,255,0);
    if (uLINK[j].color==5)//cyan
        glColor3ub(0,255,255);
    if (uLINK[j].color==6)//magenta
        glColor3ub(255,0,255);
#endif
#if VisuTorquesColor
    int colorT;
    if (uLINK[j].u_joint>0)
    {
        colorT=(int)((uLINK[j].u_joint/uLINK[j].umax)*255);
        glColor3ub(colorT,255-colorT,0);
    }
    else
    {
        colorT=(int)((uLINK[j].u_joint/uLINK[j].umin)*255);
        glColor3ub(colorT,255-colorT,colorT);
    }
#endif
    gluCylinder(params,radius,radius,len,faces,1);

    glRotated(180,0,1,0);
    gluDisk(params,0,radius,faces,1);
    glRotated(-180,0,1,0);
    glTranslated(0,0,len);
    gluDisk(params,0,radius,faces,1);
    glTranslated(0,0,-len);
    gluQuadricDrawStyle(params,GLU_LINE);
    glColor3ub(0,0,0);
    gluCylinder(params,radius,radius,len,faces,1);
    gluDeleteQuadric(params);
#if VisuArticularsLimits
    gluPartialDisk(params,0.04,0.12,20,1,180+(uLINK[j+2].qmin)*RadToDeg,-1*(-uLINK[j+2].qmax+uLINK[j+2].qmin)*RadToDeg);
#endif
    glPopMatrix();

}

