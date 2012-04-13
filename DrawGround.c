#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_math.h>
#include "DrawGround.h"

#include <GL/gl.h>
#include <GL/glu.h>



void DrawGround(float x,float y,float z,float dx,float dy,float dz)
{

float color=0.5f;

GLfloat VertexArray[24] = {
	x-dx, y+dy, z-dz,
	x-dx, y-dy, z-dz,
	x-dx, y+dy, z+dz,
	x-dx, y-dy, z+dz,
	x+dx, y+dy, z+dz,
	x+dx, y-dy, z+dz,
	x+dx, y+dy, z-dz,
	x+dx, y-dy, z-dz
};


GLfloat ColorArray[24] = {
	color, color, color,
	color, color, color,
	color, color, color,
	color, color, color,
	color, color, color,
	color, color, color,
	color, color, color,
	color, color, color
};

GLuint IndiceArray[36] = {
	0,1,2,2,1,3,
	4,5,6,6,5,7,
	3,1,5,5,1,7,
	0,2,6,6,2,4,
	6,7,0,0,7,1,
	2,3,4,4,3,5
};

glEnableClientState( GL_VERTEX_ARRAY );
glEnableClientState( GL_COLOR_ARRAY );

glVertexPointer( 3, GL_FLOAT, 0, VertexArray );
glColorPointer( 3, GL_FLOAT, 0, ColorArray );

glDrawElements( GL_TRIANGLES, 36, GL_UNSIGNED_INT, IndiceArray );

glDisableClientState( GL_COLOR_ARRAY );
glDisableClientState( GL_VERTEX_ARRAY );


glLineWidth( 2.0f );
glBegin(GL_LINE_LOOP);
glColor3ub(0,0,0);
glVertex3d(x-dx,y-dy,z+dz);
glVertex3d(x+dx,y-dy,z+dz);
glVertex3d(x+dx,y+dy,z+dz);
glVertex3d(x-dx,y+dy,z+dz);
glEnd();

//    int i,k;
//    char Masque[3][8]= {{0,0,1,1,0,0,1,1},{0,1,1,0,0,1,1,0},{0,0,0,0,1,1,1,1}};
//    gsl_matrix * vert = gsl_matrix_calloc (3, 8);
//    gsl_matrix * face = gsl_matrix_calloc (4, 6);
//    float pos[3];
//    float shape[3];
//
//    pos[0]=x;
//    pos[1]=y;
//    pos[2]=z;
//
//    shape[0]=dx;
//    shape[1]=dy;
//    shape[2]=dz;
//
//    for(i=0; i<3; i++)
//    {
//        for(k=0; k<8; k++)
//        {
//            gsl_matrix_set (vert,i,k,Masque[i][k]*shape[i]-shape[i]/2+pos[i]);
//        }
//    }
//
//
//    gsl_matrix_set (face,0,0,0);
//    gsl_matrix_set (face,0,1,1);
//    gsl_matrix_set (face,0,2,3);
//    gsl_matrix_set (face,0,3,0);
//    gsl_matrix_set (face,0,4,0);
//    gsl_matrix_set (face,0,5,4);
//
//    gsl_matrix_set (face,1,0,1);
//    gsl_matrix_set (face,1,1,5);
//    gsl_matrix_set (face,1,2,2);
//    gsl_matrix_set (uface,1,3,4);
//    gsl_matrix_set (face,1,4,1);
//    gsl_matrix_set (face,1,5,5);
//
//    gsl_matrix_set (face,2,0,2);
//    gsl_matrix_set (face,2,1,6);
//    gsl_matrix_set (face,2,2,6);
//    gsl_matrix_set (face,2,3,7);
//    gsl_matrix_set (uLINK[j].face,2,4,5);
//    gsl_matrix_set (uLINK[j].face,2,5,6);
//
//    gsl_matrix_set (uLINK[j].face,3,0,3);
//    gsl_matrix_set (uLINK[j].face,3,1,2);
//    gsl_matrix_set (uLINK[j].face,3,2,7);
//    gsl_matrix_set (uLINK[j].face,3,3,3);
//    gsl_matrix_set (uLINK[j].face,3,4,4);
//    gsl_matrix_set (uLINK[j].face,3,5,7);
//
//
//
//    double tmp;
//
//    gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, uLINK[j].R, uLINK[j].vert, 0.0, vert);
//    for (i = 0; i < 3; ++i)
//    {
//        for (k = 0; k < 8; ++k)
//        {
//            tmp=gsl_matrix_get (vert,i,k)+gsl_vector_get (uLINK[j].p,i);
//            gsl_matrix_set (vert,i,k,tmp);
//        }
//    }
//
//    for (i = 0; i < 6; ++i)
//    {
//        glBegin(GL_QUADS);
//        glColor3ub(124,124,124);
//        for (k = 0; k < 4; ++k)
//        {
//            glVertex3d(gsl_matrix_get(vert,0,gsl_matrix_get(uLINK[j].face,k,i)),gsl_matrix_get(vert,1,gsl_matrix_get(uLINK[j].face,k,i)),gsl_matrix_get(vert,2,gsl_matrix_get(uLINK[j].face,k,i)));
//        }
//        glEnd();
//
//        glLineWidth( 2.0f );
//        glBegin(GL_LINE_LOOP);
//        glColor3ub(0,0,0);
//        for (k = 0; k < 4; ++k)
//        {
//            glVertex3d(gsl_matrix_get(vert,0,gsl_matrix_get(uLINK[j].face,k,i)),gsl_matrix_get(vert,1,gsl_matrix_get(uLINK[j].face,k,i)),gsl_matrix_get(vert,2,gsl_matrix_get(uLINK[j].face,k,i)));
//        }
//        glEnd();
//
//
//    gsl_matrix_free(vert);
}







