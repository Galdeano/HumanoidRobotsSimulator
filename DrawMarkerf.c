
#include "DrawMarkerf.h"

#include <GL/gl.h>
#include <GL/glu.h>



void DrawMarkerf(double *com)
{
    double d = 0.1;

    glLineWidth( 1.0f );
    glBegin(GL_LINES);
    glVertex3d(com[0]-d,com[1],com[2]);
    glVertex3d(com[0]+d,com[1],com[2]);
    glEnd();

    glLineWidth( 1.0f );
    glBegin(GL_LINES);
    glVertex3d(com[0],com[1]-d,com[2]);
    glVertex3d(com[0],com[1]+d,com[2]);
    glEnd();

    glLineWidth( 1.0f );
    glBegin(GL_LINES);
    glVertex3d(com[0],com[1],com[2]-d);
    glVertex3d(com[0],com[1],com[2]+d);
    glEnd();

}
