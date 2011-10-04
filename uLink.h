#ifndef H_uLink
#define H_uLink

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

/** \file uLink.h
 * This describe link properties.
 * Every bodies is represent by a link whith many properties (like mass, position, inertia,...) in this formulation.
 */

/**
 * \struct SuLINK uLink.h
 * \brief Object Link.
 *
 * SuLINK is a descriptor of a physical link in the multibody system
 * used for the simulation
 */
typedef struct
{
    char name[8]; /*!< Name of the Link */
    unsigned char sister,child,mother,color,isPolygon;
    double m,q,dq,ddq,Ir,gr,u,ug,uef,u_joint,supportHeight;
    gsl_vector * a;
    gsl_vector * b;
    gsl_vector * c;
    gsl_vector * p;
    gsl_vector * v;
    gsl_vector * vo;
    gsl_vector * w;
    gsl_vector * dvo;
    gsl_vector * dw;
    gsl_vector * hw;
    gsl_vector * hv;
    gsl_vector * isContact;
    float com[3];
    float shape[3];
    gsl_matrix * R;
    gsl_matrix * I;
    gsl_matrix * vert;
    gsl_matrix * face;
    gsl_matrix * posContact;
    gsl_matrix * forContact;
} SuLINK ;

/**
 * \struct State uLink.h
 * \brief Object State.
 *
 * SuLINK  contain some information about the robot wich are not local
 */
typedef struct
{
    int desired_support;
    unsigned char ddl,support,right_foot_ID,left_foot_ID; /*0:none,1:right,2:left,3:both*/
    float right_scale,left_scale,integral_R,integral_L,distribution_y,distribution_x;
    gsl_vector * com_old;
    gsl_vector * posCoP_R;
    gsl_vector * forCoP_R;
    gsl_vector * posCoP_L;
    gsl_vector * forCoP_L;
    gsl_vector * FootCenter_R;
    gsl_vector * FootCenter_L;
} State ;

/*
//    uLINK(n).dq     = 0;            % joint velocity [rad/s]
//    uLINK(n).ddq    = 0;            % joint acceleration [rad/s^2]
//    %uLINK(n).c      = [0 0 0]';     % center of gravity [m]
//    uLINK(n).c      = uLINK(n).b/2;     % center of gravity [m]
//    uLINK(n).I      = zeros(3,3);   % around the center of gravity inertia tensor [kg.m^2]
//    uLINK(n).Ir     = 0.0;          % child electric motor moment of inertia [kg.m^2]
//    uLINK(n).gr     = 0.0;          % reduction ratio of the motor [-]
//    uLINK(n).u      = 0.0;          % joint torque [Nm]
*/

/**
 * \fn void SetupRobot(SuLINK uLINK[],State *Status)
 * \brief Fonction de création d'une nouvelle instance d'un objet Str_t.
 *
 * \param SuLINK uLINK[] Chaîne à stocker dans l'objet Str_t, ne peut être NULL.
 * \param State *Status Chaîne  dans l'objet Str_t, ne peut être NULL.
 * \return Instance nouvellement allouée d'un objet de type Str_t ou NULL.
 */

void SetupRobot(SuLINK uLINK[],State *Status);


#endif




