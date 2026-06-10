#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_math.h>
#include "OpenGLHeaders.h"
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
//    int i,j,k;

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
    // GLU quadrics and disk drawing are removed in modern OpenGL Core Profile
#endif

//    gsl_vector_free(pos);
//    gsl_vector_free(Visu);

}


void Hoap_calc_zmp_visu(SuLINK uLINK[],State *Status,zmp_calc* zmp)
{
    // Right foot ZMP visualization
    glm::mat4 M_right(1.0f);
    for (int i = 0; i < 3; ++i) {
        for (int k = 0; k < 3; ++k) {
            M_right[i][k] = (float)gsl_matrix_get(uLINK[Status->right_foot_ID].R, k, i);
        }
        M_right[3][i] = (float)gsl_vector_get(uLINK[Status->right_foot_ID].p, i);
    }
    M_right = glm::translate(M_right, glm::vec3(0.0f, 0.0f, -0.04f));

    if (zmp->zmp_right.W > 10) {
        set_material(&turquoise);
    } else {
        set_material(&ruby);
    }

    glm::vec3 local_start_r(zmp->zmp_right.x / 1000.0f, zmp->zmp_right.y / 1000.0f, 0.0f);
    glm::vec3 local_end_r(zmp->zmp_right.x / 1000.0f, zmp->zmp_right.y / 1000.0f, zmp->zmp_right.W / 100.0f);
    
    glm::vec3 global_start_r = glm::vec3(M_right * glm::vec4(local_start_r, 1.0f));
    glm::vec3 global_end_r = glm::vec3(M_right * glm::vec4(local_end_r, 1.0f));

    draw_line(global_start_r, global_end_r, activeColor, 10.0f);

    // Left foot ZMP visualization
    glm::mat4 M_left(1.0f);
    for (int i = 0; i < 3; ++i) {
        for (int k = 0; k < 3; ++k) {
            M_left[i][k] = (float)gsl_matrix_get(uLINK[Status->left_foot_ID].R, k, i);
        }
        M_left[3][i] = (float)gsl_vector_get(uLINK[Status->left_foot_ID].p, i);
    }
    M_left = glm::translate(M_left, glm::vec3(0.0f, 0.0f, -0.04f));

    if (zmp->zmp_left.W > 10) {
        set_material(&turquoise);
    } else {
        set_material(&ruby);
    }

    glm::vec3 local_start_l(zmp->zmp_left.x / 1000.0f, zmp->zmp_left.y / 1000.0f, 0.0f);
    glm::vec3 local_end_l(zmp->zmp_left.x / 1000.0f, zmp->zmp_left.y / 1000.0f, zmp->zmp_left.W / 100.0f);
    
    glm::vec3 global_start_l = glm::vec3(M_left * glm::vec4(local_start_l, 1.0f));
    glm::vec3 global_end_l = glm::vec3(M_left * glm::vec4(local_end_l, 1.0f));

    draw_line(global_start_l, global_end_l, activeColor, 10.0f);
}
