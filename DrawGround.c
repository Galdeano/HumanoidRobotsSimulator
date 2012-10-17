#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_math.h>
#include "DrawGround.h"
#include "Setup.h"
#include <GL/gl.h>
#include <GL/glu.h>
#include "DrawLight.h"


void DrawGround(double x,double y,double z,double dx,double dy,double dz)
{

//    float color=0.75f;
//
//    GLfloat VertexArray[24] =
//    {
//        x-dx, y+dy, z-dz,
//        x-dx, y-dy, z-dz,
//        x-dx, y+dy, z+dz,
//        x-dx, y-dy, z+dz,
//        x+dx, y+dy, z+dz,
//        x+dx, y-dy, z+dz,
//        x+dx, y+dy, z-dz,
//        x+dx, y-dy, z-dz
//    };
//
//
//    GLfloat ColorArray[24] =
//    {
//        color, color, color,
//        color, color, color,
//        color, color, color,
//        color, color, color,
//        color, color, color,
//        color, color, color,
//        color, color, color,
//        color, color, color
//    };
//
//    GLuint IndiceArray[36] =
//    {
//        0,1,2,2,1,3,
//        4,5,6,6,5,7,
//        3,1,5,5,1,7,
//        0,2,6,6,2,4,
//        6,7,0,0,7,1,
//        2,3,4,4,3,5
//    };
//
//    glEnableClientState( GL_VERTEX_ARRAY );
//    glEnableClientState( GL_COLOR_ARRAY );
//
//    glVertexPointer( 3, GL_FLOAT, 0, VertexArray );
//    glColorPointer( 3, GL_FLOAT, 0, ColorArray );
//
//    glDrawElements( GL_TRIANGLES, 36, GL_UNSIGNED_INT, IndiceArray );
//
//    glDisableClientState( GL_COLOR_ARRAY );
//    glDisableClientState( GL_VERTEX_ARRAY );
//
//
//    glLineWidth( 2.0f );
//    glBegin(GL_LINE_LOOP);
//    glColor3ub(0,0,0);
//    glVertex3d(x-dx,y-dy,z+dz);
//    glVertex3d(x+dx,y-dy,z+dz);
//    glVertex3d(x+dx,y+dy,z+dz);
//    glVertex3d(x-dx,y+dy,z+dz);
//    glEnd();




    static int i,k,l;
    static char Masque[3][8]= {{0,0,1,1,0,0,1,1},{0,1,1,0,0,1,1,0},{0,0,0,0,1,1,1,1}};
    static gsl_matrix * vert;
    static gsl_matrix * face;
    static gsl_matrix * normalface;
    static double shape[3];
    static double com[3];

    static int init_tmp=1;
    if (init_tmp==1)
    {
        shape[0]=dx;
        shape[1]=dy;
        shape[2]=dz;
        com[0]=x;
        com[1]=y;
        com[2]=z;
        vert = gsl_matrix_calloc (3, 8);
        face = gsl_matrix_calloc (4, 6);
        normalface = gsl_matrix_calloc (3,6);
        for(i=0; i<3; i++)
        {
            for(k=0; k<8; k++)
            {
                gsl_matrix_set ( vert,i,k,Masque[i][k]*shape[i]-shape[i]/2+com[i]);
            }
        }


        gsl_matrix_set (face,0,0,0);
        gsl_matrix_set (face,1,0,1);
        gsl_matrix_set (face,2,0,2);
        gsl_matrix_set (face,3,0,3);

        gsl_matrix_set (face,0,1,1);
        gsl_matrix_set (face,1,1,5);
        gsl_matrix_set (face,2,1,6);
        gsl_matrix_set (face,3,1,2);

        gsl_matrix_set (face,0,2,3);
        gsl_matrix_set (face,1,2,2);
        gsl_matrix_set (face,2,2,6);
        gsl_matrix_set (face,3,2,7);

        gsl_matrix_set (face,0,3,0);// 0473
        gsl_matrix_set (face,1,3,3);
        gsl_matrix_set (face,2,3,7);
        gsl_matrix_set (face,3,3,4);

        gsl_matrix_set (face,0,4,0);// 0154
        gsl_matrix_set (face,1,4,4);
        gsl_matrix_set (face,2,4,5);
        gsl_matrix_set (face,3,4,1);

        gsl_matrix_set (face,0,5,4);// 4567
        gsl_matrix_set (face,1,5,7);
        gsl_matrix_set (face,2,5,6);
        gsl_matrix_set (face,3,5,5);


        gsl_matrix_set ( normalface,0,0,0);
        gsl_matrix_set ( normalface,0,1,0);
        gsl_matrix_set ( normalface,0,2,1);
        gsl_matrix_set ( normalface,0,3,0);
        gsl_matrix_set ( normalface,0,4,-1);
        gsl_matrix_set ( normalface,0,5,0);

        gsl_matrix_set ( normalface,1,0,0);
        gsl_matrix_set ( normalface,1,1,1);
        gsl_matrix_set ( normalface,1,2,0);
        gsl_matrix_set ( normalface,1,3,-1);
        gsl_matrix_set ( normalface,1,4,0);
        gsl_matrix_set ( normalface,1,5,0);

        gsl_matrix_set ( normalface,2,0,-1);
        gsl_matrix_set ( normalface,2,1,0);
        gsl_matrix_set ( normalface,2,2,0);
        gsl_matrix_set ( normalface,2,3,0);
        gsl_matrix_set ( normalface,2,4,0);
        gsl_matrix_set ( normalface,2,5,1);


        init_tmp=0;
    }



    for (i = 0; i < 6; ++i)
    {
        glNormal3d(gsl_matrix_get(normalface,0,i), gsl_matrix_get(normalface,1,i), gsl_matrix_get(normalface,2,i));
//        gsl_matrix_get_col (ftmp, norm, i);
//        gsl_matrix_get_col (v, vert, gsl_matrix_get(uLINK[j].face,0,i));
//        gsl_matrix_get_col (pos, vert, gsl_matrix_get(uLINK[j].face,2,i));
//        gsl_vector_add(v,pos);
//        gsl_vector_scale(v,0.5);
//        DrawForceMarker(v,ftmp);
#if colors
        glColor3f(0.8f,0.8f,0.8f);
#endif
#if materials
        set_material(&chrome);
        //set_material(&white_shiny_plastic);
#endif
        glBegin(GL_QUADS);
        for (k = 0; k < 4; ++k)
        {
            glVertex3d(gsl_matrix_get(vert,0,gsl_matrix_get(face,k,i)),gsl_matrix_get(vert,1,gsl_matrix_get(face,k,i)),gsl_matrix_get(vert,2,gsl_matrix_get(face,k,i)));
        }
        glEnd();
        //gsl_vector_scale(norm,10);

//DrawForceMarker(pos,norm);

        glLineWidth( 2.0f );
        glBegin(GL_LINE_LOOP);
#if colors
        glColor3ub(0,0,0);
#endif
#if materials
        set_material(&black_rubber);
#endif
        for (k = 0; k < 4; ++k)
        {
            glVertex3d(gsl_matrix_get(vert,0,gsl_matrix_get(face,k,i)),gsl_matrix_get(vert,1,gsl_matrix_get(face,k,i)),gsl_matrix_get(vert,2,gsl_matrix_get(face,k,i)));
        }
        glEnd();

    }

}







