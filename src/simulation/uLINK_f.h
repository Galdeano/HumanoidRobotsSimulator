#ifndef H_uLink_f
#define H_uLink_f

/** \file uLink_f.h
 *  \brief Link initialisation and declarations (used in control loop)
 *  \author    David Galdeano
 *  \date      10/2011
 *
 * This file describe link properties for control loop.
 */


/**
 * \struct Struct_uLINK uLink.h
 * \brief Struct_uLINK is a descriptor of a physical link in the multibody system used for the simulation (used in control loop). See \ref SuLINK.
 */
struct SStruct_uLINK
{
    char name[8];
    unsigned char sister,child,mother;
    double m,q,dq,ddq,u,ug,uef,u_joint;
    double a[3];
    double b[3];
    double c[3];
    double p[3];
    double hw[3];
    double hv[3];
    double R[9];
};

typedef struct  SStruct_uLINK  Struct_uLINK;

/**
 * \struct Struct_State uLink.h
 * \brief Struct_State contain some global information about the robot state (used in control loop). See \ref State.
 */
typedef struct
{
    int desired_support;
    unsigned char ddl,support,right_foot_ID,left_foot_ID; /*  support: 0:none,1:right,2:left,3:both*/
    double distribution_y;
    double com_old[3];
    double FootCenter_R[3];
    double FootCenter_L[3];
} Struct_State ;

/* //    uLINK(n).dq     = 0;            % joint velocity [rad/s]
 * //    uLINK(n).ddq    = 0;            % joint acceleration [rad/s^2]
 * //    %uLINK(n).c      = [0 0 0]';     % center of gravity [m]
 * //    uLINK(n).c      = uLINK(n).b/2;     % center of gravity [m]
 * //    uLINK(n).I      = zeros(3,3);   % around the center of gravity inertia tensor [kg.m^2]
 * //    uLINK(n).Ir     = 0.0;          % child electric motor moment of inertia [kg.m^2]
 * //    uLINK(n).gr     = 0.0;          % reduction ratio of the motor [-]
 * //    uLINK(n).u      = 0.0;          % joint torque [Nm]
 */

/**
 * \fn void SetupRobot_f(Struct_uLINK uLINK[],Struct_State *Status)
 * \brief Initialize the link structure with the model value (used in control loop). See \ref SetupRobot.
 *
 * \param uLINK[] Structure wich describe the robot link by link
 * \param Status Structure wich describe the state of the robot
 */
void SetupRobot_f(Struct_uLINK uLINK[],Struct_State *Status);


#endif


