#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_math.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include "uLink.h"
#include "DrawForceMarker.h"
#include "CalcCoM.h"
#include "CalcCoP.h"
#include "DrawMarker.h"
#include "DrawIndicators.h"
#include "Setup.h"
#include "Hoap_calc_zmp.h"
#include "DrawLight.h"

void DrawIndicators(SuLINK uLINK[],State *Status,gsl_vector * com,gsl_vector * CoP,int ground)
{
    int i,j,k;

    static double RadToDeg;
    static gsl_vector * pos;
    static gsl_vector * Visu;
    static int init_tmp=1;
    if (init_tmp==1)
    {
        RadToDeg = 180/M_PI;
        pos = gsl_vector_calloc (3);
        Visu = gsl_vector_calloc (3);
        init_tmp=0;
    }

    CalcCoM(uLINK,com);
#if colorsGL
    glColor3ub(0,0,255);
#endif
#if materials
    set_material(&turquoise);
#endif
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
#if colorsGL
        glColor3ub(255,0,0);
#endif
        gsl_vector_scale (CoP, 1/f);
        gsl_vector_set (CoP, 2, 0);
        DrawMarker(CoP);
    }

#if colorsGL
    glColor3ub(0,255,0);
#endif
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

//    gsl_vector * pos = gsl_vector_calloc (3);
//    gsl_vector * Visu = gsl_vector_calloc (3);

#if colorsGL
    glColor3ub(0,255,255);
#endif
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
#if colorsGL
    glColor3ub(0,0,255);
#endif;
#if materials
    set_material(&turquoise);
#endif
    for (j=0; j<(Status->ddl-6); j++)
    {
        glPushMatrix();

//        glGetDoublev(GL_MODELVIEW_MATRIX, rotgl);//charge avec identitee
//        for (i = 0; i < 3; ++i)
//            for (k = 0; k < 3; ++k)
//                rotgl[i*4+k] = gsl_matrix_get (uLINK[j+1].R, k,i);
//
//        glTranslated(gsl_vector_get (uLINK[j+2].p,0),gsl_vector_get (uLINK[j+2].p,1),gsl_vector_get (uLINK[j+2].p,2));


        glGetDoublev(GL_MODELVIEW_MATRIX, rotgl);//charge avec identitee

        for (i = 0; i < 3; ++i)
        {
            for (k = 0; k < 3; ++k)
            {
                rotgl[i*4+k] = gsl_matrix_get (uLINK[j+1].R, k,i);
            }
            rotgl[i+12]=gsl_vector_get(uLINK[j+2].p,i);
        }

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

//    gsl_vector_free(pos);
//    gsl_vector_free(Visu);

}


void Hoap_calc_zmp_visu(SuLINK uLINK[],State *Status,zmp_calc* zmp)
{
    int i,k;
    GLdouble rotgl[16];
    glLineWidth( 10.0f );


    glPushMatrix();
    glGetDoublev(GL_MODELVIEW_MATRIX, rotgl);//charge avec identitee
    for (i = 0; i < 3; ++i)
    {
        for (k = 0; k < 3; ++k)
        {
            rotgl[i*4+k] = gsl_matrix_get (uLINK[Status->right_foot_ID].R, k,i);
        }
        rotgl[i+12]=gsl_vector_get(uLINK[Status->right_foot_ID].p,i);
    }
    rotgl[14]+=-0.04;
    glMultMatrixd(rotgl);

    if (zmp->zmp_right.W>10)
    {
#if colorsGL
        glColor3ub(0,0,255);
#endif
#if materials
        set_material(&turquoise);
#endif
    }
    else
    {
#if colorsGL
        glColor3ub(255,0,0);
#endif
#if materials
        set_material(&ruby);
#endif
    }

    glBegin(GL_LINES);
    //glColor3ub(0,0,255);
    glVertex3d(zmp->zmp_right.x/1000,zmp->zmp_right.y/1000,0);
    glVertex3d(zmp->zmp_right.x/1000,zmp->zmp_right.y/1000,zmp->zmp_right.W/100);
    glEnd();
    glPopMatrix();


    glPushMatrix();
    glGetDoublev(GL_MODELVIEW_MATRIX, rotgl);//charge avec identitee
    for (i = 0; i < 3; ++i)
    {
        for (k = 0; k < 3; ++k)
        {
            rotgl[i*4+k] = gsl_matrix_get (uLINK[Status->left_foot_ID].R, k,i);
        }
        rotgl[i+12]=gsl_vector_get(uLINK[Status->left_foot_ID].p,i);
    }
    rotgl[14]+=-0.04;
    glMultMatrixd(rotgl);

    if (zmp->zmp_left.W>10)
    {
#if colorsGL
        glColor3ub(0,0,255);
#endif
#if materials
        set_material(&turquoise);
#endif
    }
    else
    {
#if colorsGL
        glColor3ub(255,0,0);
#endif
#if materials
        set_material(&ruby);
#endif
    }
    glBegin(GL_LINES);
    //glColor3ub(0,0,255);
    glVertex3d(zmp->zmp_left.x/1000,zmp->zmp_left.y/1000,0);
    glVertex3d(zmp->zmp_left.x/1000,zmp->zmp_left.y/1000,zmp->zmp_left.W/100);
    glEnd();
    glPopMatrix();
}
