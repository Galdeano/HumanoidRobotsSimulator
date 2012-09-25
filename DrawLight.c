#include <GL/gl.h>
#include <GL/glu.h>
#include "DrawLight.h"


void Init_light()
{
    //différents paramètres
    GLfloat ambient[] = {0.25f,0.25f,0.25f,1.0f};
    GLfloat diffuse[] = {0.8f,0.8f,0.8f,1.0f};
    GLfloat light0_position [] = {3.0f, -3.0f, 3.0f, 1.0f};
    GLfloat specular_reflexion[] = {0.8f,0.8f,0.8f,1.0f};
    //GLdouble specular_reflexion[] = {1.0f,1.0f,1.0f,1.0f};
    GLubyte shiny_obj = 100;

    //positionnement de la lumière avec les différents paramètres
    glEnable(GL_LIGHTING);
    glLightfv(GL_LIGHT0,GL_AMBIENT,ambient);
    glLightfv(GL_LIGHT0,GL_DIFFUSE,diffuse);
    glLightfv(GL_LIGHT0,GL_POSITION,light0_position);
    glEnable(GL_LIGHT0);

    //spécification de la réflexion sur les matériaux
    glEnable(GL_COLOR_MATERIAL);
//    glColorMaterial(GL_FRONT,GL_AMBIENT_AND_DIFFUSE);
//    glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,ambient);
//    glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,diffuse);

        glColorMaterial(GL_FRONT,GL_AMBIENT_AND_DIFFUSE);
    glMaterialfv(GL_FRONT,GL_AMBIENT,ambient);
    glMaterialfv(GL_FRONT,GL_DIFFUSE,diffuse);

//    double black[4]={0,0,0,0};
//glMaterialfv(GL_FRONT,GL_AMBIENT,black);
//glMaterialfv(GL_FRONT,GL_SPECULAR,black);

    glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,specular_reflexion);
    glMateriali(GL_FRONT_AND_BACK,GL_SHININESS,shiny_obj);

}


