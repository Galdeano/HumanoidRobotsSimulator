#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_math.h>
#include "uLINK.h"
#include "DrawPolygon.h"
#include "DrawForceMarker.h"
#include "Cross.h"

#include <GL/gl.h>
#include <GL/glu.h>

#include "Setup.h"

void DrawPolygon(SuLINK uLINK[],int j)
{
    int i,k;

    double tmp;

    static gsl_vector * v;
    static gsl_vector * ftmp;
    static gsl_matrix * vert;
    //static gsl_vector * norm;
    static gsl_matrix * norm;
    static gsl_vector * pos;
    static int init_tmp=1;
    if (init_tmp==1)
    {
        v = gsl_vector_calloc (3);
        ftmp = gsl_vector_calloc (3);
        vert = gsl_matrix_calloc (3, 8);
        norm = gsl_matrix_calloc (3, 6);
        //norm = gsl_vector_alloc (3);
        pos = gsl_vector_alloc (3);
        init_tmp=0;
    }


    gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, uLINK[j].R, uLINK[j].vert, 0.0, vert);
    gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, uLINK[j].R, uLINK[j].normalface, 0.0, norm);


    for (i = 0; i < 3; ++i)
    {
        for (k = 0; k < 8; ++k)
        {
            tmp=gsl_matrix_get(vert,i,k)+gsl_vector_get (uLINK[j].p,i);
            gsl_matrix_set (vert,i,k,tmp);

        }
    }

    for (i = 0; i < 6; ++i)
    {
        glNormal3d(gsl_matrix_get(norm,0,i), gsl_matrix_get(norm,1,i), gsl_matrix_get(norm,2,i));
//        gsl_matrix_get_col (ftmp, norm, i);
//        gsl_matrix_get_col (v, vert, gsl_matrix_get(uLINK[j].face,0,i));
//        gsl_matrix_get_col (pos, vert, gsl_matrix_get(uLINK[j].face,2,i));
//        gsl_vector_add(v,pos);
//        gsl_vector_scale(v,0.5);
//        DrawForceMarker(v,ftmp);
        glBegin(GL_QUADS);

        glColor3f(0.9f,0.9f,0.9f);
        for (k = 0; k < 4; ++k)
        {
            glVertex3d(gsl_matrix_get(vert,0,gsl_matrix_get(uLINK[j].face,k,i)),gsl_matrix_get(vert,1,gsl_matrix_get(uLINK[j].face,k,i)),gsl_matrix_get(vert,2,gsl_matrix_get(uLINK[j].face,k,i)));
        }
        glEnd();
        //gsl_vector_scale(norm,10);

//DrawForceMarker(pos,norm);

        glLineWidth( 2.0f );
        glBegin(GL_LINE_LOOP);
        glColor3ub(0,0,0);
        for (k = 0; k < 4; ++k)
        {
            glVertex3d(gsl_matrix_get(vert,0,gsl_matrix_get(uLINK[j].face,k,i)),gsl_matrix_get(vert,1,gsl_matrix_get(uLINK[j].face,k,i)),gsl_matrix_get(vert,2,gsl_matrix_get(uLINK[j].face,k,i)));
        }
        glEnd();

//        glLineWidth( 5.0f );
//        glBegin(GL_LINES);
//        glColor3ub(0,0,0);
//        glVertex3d(gsl_vector_get (uLINK[j].p,0),gsl_vector_get (uLINK[j].p,1),gsl_vector_get (uLINK[j].p,2));
//        //com*R
//        glVertex3d(gsl_vector_get (uLINK[j].p,0)+uLINK[j].com[0],gsl_vector_get (uLINK[j].p,1)+uLINK[j].com[1],gsl_vector_get (uLINK[j].p,2)+uLINK[j].com[2]);
//        glEnd();

    }


    glLineWidth( 5.0f );
    glBegin(GL_LINES);
    glColor3ub(0,0,0);
    glVertex3d(gsl_vector_get (uLINK[j].p,0),gsl_vector_get (uLINK[j].p,1),gsl_vector_get (uLINK[j].p,2));

    glVertex3d((gsl_matrix_get(vert,0,0)+gsl_matrix_get(vert,0,6))/2,
               (gsl_matrix_get(vert,1,0)+gsl_matrix_get(vert,1,6))/2,
               (gsl_matrix_get(vert,2,0)+gsl_matrix_get(vert,2,6))/2);
    glEnd();


    glColor3ub(0,0,255);
    for (i = 0; i < 8; ++i)///////////////////////////////////////////////////////////
    {
        if (gsl_vector_get(uLINK[j].isContact,i)==1)
        {
//            gsl_vector * v = gsl_vector_calloc (3);
//            gsl_vector * ftmp = gsl_vector_calloc (3);
            gsl_matrix_get_col (v, uLINK[j].posContact, i);
            gsl_matrix_get_col (ftmp, uLINK[j].forContact, i);
            gsl_vector_scale (ftmp, -0.01);
            DrawForceMarker(v,ftmp);
//            gsl_vector_free(ftmp);
//            gsl_vector_free(v);
        }
    }

    for (i = 0; i < xContact*yContact; ++i)///////////////////////////////////////////////////////////
    {
        if (gsl_vector_get(uLINK[j].iscontact,i)==1)
        {
            gsl_matrix_get_col(v, uLINK[j].pcontact, i);
//        DrawMarker(v);
            gsl_matrix_get_col (v, uLINK[j].pcontact, i);
            gsl_matrix_get_col (ftmp, uLINK[j].fcontact, i);
            gsl_vector_scale (ftmp, -0.01);
            DrawForceMarker(v,ftmp);
        }
    }

//    gsl_matrix_free(vert);
}

