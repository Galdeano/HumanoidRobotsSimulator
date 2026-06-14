#include <Eigen/Dense>
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

void draw_line(const glm::vec3 &start, const glm::vec3 &end, const glm::vec3 &color, float width);
extern glm::vec3 activeColor;

void DrawIndicators(SuLINK uLINK[], State *Status, Eigen::Vector3d & com, Eigen::Vector3d & CoP, int ground)
{
    static double RadToDeg;
    static Eigen::Vector3d pos;
    static Eigen::Vector3d Visu;
    static int init_tmp = 1;
    if (init_tmp == 1)
    {
        RadToDeg = 180.0 / M_PI;
        pos.setZero();
        Visu.setZero();
        init_tmp = 0;
    }

    CalcCoM(uLINK, com);
#if colorsGL
    glColor3ub(0, 0, 255);
#endif
#if materials
    set_material(&turquoise);
#endif
    if (!ground)
    {
        com(2) = 0.0;
    }
    DrawMarker(com);

    double f = 0.0;
    CoP.setZero();
    f = CalcCoP(uLINK, CoP, 1);
    if (f != 0.0)
    {
#if colorsGL
        glColor3ub(255, 0, 0);
#endif
        CoP /= f;
        CoP(2) = 0.0;
        DrawMarker(CoP);
    }

#if colorsGL
    glColor3ub(0, 255, 0);
#endif
    if (Status->desired_support == 1)
    {
        DrawMarker(Status->FootCenter_R);
    }
    if (Status->desired_support == 2)
    {
        DrawMarker(Status->FootCenter_L);
    }
    if (Status->desired_support == 3)
    {
        DrawMarker(Status->FootCenter_R);
        DrawMarker(Status->FootCenter_L);
    }

#if colorsGL
    glColor3ub(0, 255, 255);
#endif
    if (Status->right_scale != 0.0)
    {
        pos = Status->posCoP_R / Status->right_scale;
    }
    Visu = Status->forCoP_R * -0.0025;
    DrawForceMarker(pos, Visu);

    if (Status->left_scale != 0.0)
    {
        pos = Status->posCoP_L / Status->left_scale;
    }
    Visu = Status->forCoP_L * -0.0025;
    DrawForceMarker(pos, Visu);
}

void Hoap_calc_zmp_visu(SuLINK uLINK[], State *Status, zmp_calc* zmp)
{
    // Right foot ZMP visualization
    glm::mat4 M_right(1.0f);
    for (int i = 0; i < 3; ++i) {
        for (int k = 0; k < 3; ++k) {
            M_right[i][k] = (float)uLINK[Status->right_foot_ID].R(k, i);
        }
        M_right[3][i] = (float)uLINK[Status->right_foot_ID].p(i);
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
            M_left[i][k] = (float)uLINK[Status->left_foot_ID].R(k, i);
        }
        M_left[3][i] = (float)uLINK[Status->left_foot_ID].p(i);
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


