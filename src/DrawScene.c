
#include <GL/gl.h>
#include <GL/glu.h>
#include <SDL2/SDL.h>
#include "Setup.h"
extern SDL_Window* window;
#include "uLink.h"
#include "CalcCoM.h"

#include <dirent.h>
#include "DrawCamera.h"
#include "DrawLight.h"
#include "DrawGround.h"
#include "DrawMarker.h"
#include "DrawAllJoints.h"
#include "bmp.h"
#include "DrawScene.h"

void DrawScene(SuLINK uLINK[],State *Status,CamParam_s *CamParam)
{

    static int init_tmp=1;
    static gsl_vector * com;
    static gsl_vector * offset;
    if (init_tmp==1)
    {
//        char buf[256];
//        DIR *d;
//        struct dirent *dir;
//        d = opendir("./Robots/");
//        d = opendir("./../Simu_images/");
//
//        while(dir = readdir(d))
//        {
//            //printf("%s\n",dir->d_name);
//            sprintf(buf, "%s/%s", "./../../Simu_images/", dir->d_name);
//            remove(buf);
//        }
        offset = gsl_vector_calloc (3);
        com = gsl_vector_calloc (3);
        init_tmp=0;
    }
    //gsl_vector * com = gsl_vector_calloc (3);

#if !Light

    //printf(titre,"Visualisation t= %3.3f", t2);
    //SDL_WM_SetCaption(titre, NULL);

    glClearStencil(0);
//        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT |GL_STENCIL_BUFFER_BIT);
    //glCullFace(GL_BACK);
    //glCullFace(GL_FRONT);



    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity();
    Camlook(Status,CamParam);

    //glRotated(angular_z,0,0,1);


#if shadow

    GLfloat light0_position [] = {2.0f, -2.0f, 2.0f, 1.0f};
    GLfloat shadowPlane [] = {0.0f, 0.0f, 1.0f, 0.0f};
    //GLfloat shadowPlane [] = {0.0f, 0.0f, 1.0f, 0.0f};
    buildShadowMatrix( g_shadowMatrix, light0_position , shadowPlane );


    //
    // Render the floor to the stencil buffer so we can use it later to trim
    // the shadow at the floor's edge...
    //
    glEnable( GL_STENCIL_TEST );
    glStencilFunc( GL_ALWAYS, 1, 0xFFFFFFFF );         // Write a 1 to the stencil buffer everywhere we are about to draw
    glStencilOp( GL_REPLACE, GL_REPLACE, GL_REPLACE ); // If a 1 is written to the stencil buffer - simply replace the current value stored there with the 1.

    glColorMask( GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE ); // Disable writing to the color buffer
    glDepthMask( GL_FALSE );                               // Disable writing to the depth buffer

    //glDisable(GL_COLOR_MATERIAL);
    DrawGround(2.0,0.0,-0.05,6.0,6.0,0.1);
    //glEnable(GL_COLOR_MATERIAL);

    glColorMask( GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE ); // Re-enable writing to the color buffer
    glDepthMask( GL_TRUE );                            // Re-enable writing to the depth buffer
    glDisable( GL_STENCIL_TEST );

    //
    // Render the floor...
    //

    DrawGround(2.0,0.0,-0.05,6.0,6.0,0.1);


    //
    // Render the shadow...
    //

    // Use our stencil to keep the shadow from running off the floor.
    glEnable( GL_STENCIL_TEST );
    glStencilFunc( GL_EQUAL, 1, 0xFFFFFFFF ); // Only write to areas where the stencil buffer has a 1.
    glStencilOp( GL_KEEP, GL_KEEP, GL_KEEP ); // Don't modify the contents of the stencil buffer

//    glStencilOp(GL_KEEP,    // stencil fail
//            GL_DECR,    // stencil pass, depth fail
//            GL_INCR);   // stencil pass, depth pass
//glStencilMask(0xFFFFFFFF );

    glDisable( GL_LIGHTING );
    glDisable( GL_DEPTH_TEST );
    glEnable( GL_BLEND );
    glDisable(GL_COLOR_MATERIAL);

    glPushMatrix();
    {
        // Load the teapot's shadow matrix
        glMultMatrixf( g_shadowMatrix );

        glColor3f(0.1f, 0.1f, 0.1f); // shadow color

        {
            DrawAllJoints(uLINK,1);
        }
    }
    glPopMatrix();

    glEnable( GL_DEPTH_TEST );
    glDisable( GL_BLEND );
    glEnable( GL_LIGHTING );
    glDisable( GL_STENCIL_TEST );
    glEnable(GL_COLOR_MATERIAL);


//#if colorsGL
//    glColor3ub(255,0,0);
//#endif //colorsGL
//#if materials
//    set_material(&ruby);
//#endif //materials
//gsl_vector_set(offset,0,0.015);
//gsl_vector_set(offset,1,0.0);
//gsl_vector_set(offset,2,-0.039);
//gsl_vector_add(offset,uLINK[7].p);
//DrawMarker(offset);
//
//gsl_vector_set(offset,0,0.015);
//gsl_vector_set(offset,1,0.0);
//gsl_vector_set(offset,2,-0.039);
//gsl_vector_add(offset,uLINK[13].p);
//DrawMarker(offset);
//
//#if colorsGL
//    glColor3ub(0,0,255);
//#endif //colorsGL
//#if materials
//    set_material(&turquoise);
//#endif //materials
//
//gsl_vector_set(offset,0,0.048);
//gsl_vector_set(offset,1,-0.01);
//gsl_vector_set(offset,2,-0.039);
//gsl_vector_add(offset,uLINK[7].p);
//DrawMarker(offset);
//
//gsl_vector_set(offset,0,0.033);
//gsl_vector_set(offset,1,0.015);
//gsl_vector_set(offset,2,-0.039);
//gsl_vector_add(offset,uLINK[13].p);
//DrawMarker(offset);


// active la transparence, penser a changer material.kd[3]
//glEnable(GL_BLEND);
//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


    DrawAllJoints(uLINK,1);



//glEnable (GL_BLEND);
//glBlendFunc (GL_ONE, GL_ONE);

#endif //shadow


#if !shadow
    DrawGround(2.0,0.0,-0.05,6.0,6.0,0.1);
    DrawAllJoints(uLINK,1);
#endif //!shadow

//    glDisable(GL_DEPTH_TEST);
//    glDisable(GL_LIGHTING);
//    glColor3f(0.0f, 0.0f, 0.0f); // Shadow's color
//    glPushMatrix();
//    {
//        glMultMatrixf((GLfloat *)g_shadowMatrix);
//        DrawAllJoints(uLINK,1);
//    }
//    glPopMatrix();
//    glEnable(GL_DEPTH_TEST);
//    glEnable(GL_LIGHTING);

//#if zmp_filtering
//    DrawMarker(zmp_right);
//    DrawMarker(zmp_left);
//    DrawMarker(zmp_moy);
//#endif //zmp_filtering

    CalcCoM(uLINK,com);
#if colorsGL
    glColor3ub(0,0,255);
#endif //colorsGL
#if materials
    set_material(&turquoise);
#endif //materials
    gsl_vector_set (com, 2, 0);
    //DrawMarker(com);

#if colorsGL
    glColor3ub(255,0,0);
#endif //colorsGL
#if materials
    set_material(&ruby);
#endif //materials
    //Hoap_calc_zmp_visu(uLINK,&Status,&zmp_c);


//gsl_vector_set(offset,0,0.015);
//gsl_vector_set(offset,1,0.0);
//gsl_vector_set(offset,2,-0.039);
//gsl_vector_add(offset,uLINK[7].p);
//DrawMarker(offset);
//
//
//gsl_vector_set(offset,0,0.015);
//gsl_vector_set(offset,1,0.0);
//gsl_vector_set(offset,2,-0.039);
//gsl_vector_add(offset,uLINK[13].p);
//DrawMarker(offset);
//
//#if colorsGL
//    glColor3ub(0,0,255);
//#endif //colorsGL
//#if materials
//    set_material(&turquoise);
//#endif //materials
//
//gsl_vector_set(offset,0,0.035);
//gsl_vector_set(offset,1,-0.01);
//gsl_vector_set(offset,2,-0.039);
//gsl_vector_add(offset,uLINK[7].p);
//DrawMarker(offset);
//
//gsl_vector_set(offset,0,0.030);
//gsl_vector_set(offset,1,0.015);
//gsl_vector_set(offset,2,-0.039);
//gsl_vector_add(offset,uLINK[13].p);
//DrawMarker(offset);


//draw_model(&obj);
//DrawOBJ("./cube2.obj");

    Init_light();


    //glFlush();
    SDL_GL_SwapWindow(window);
    //SDL_Delay(5);

//    static int i=0;
//    char files[40];
//    static char pixel_data[3*1024*768];
//    sprintf(files,"./../Simu_images/Test%.4d.bmp", i);
//    //glReadBuffer(GL_FRONT);
//    glReadPixels(0, 0, 1024, 768, GL_RGB, GL_UNSIGNED_BYTE, pixel_data);//GL_BGR
//    write_bmp(files, 1024, 768, pixel_data);
//    i++;

#endif //!Light


}
