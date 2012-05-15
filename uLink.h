#ifndef H_uLink
#define H_uLink

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

/** \file uLink.h
 *  \brief Link initialisation and declarations
 *  \author    David Galdeano
 *  \date      10/2011
 *
 * This file describe link properties.
 * Every bodies is represent by a link whith many properties (like mass, position, inertia,...) in this formulation.
 */

/**
 * \struct SuLINK uLink.h
 * \brief SuLINK is a descriptor of a physical link in the multibody system used for the simulation
 */
struct StuLINK
{
    char name[8]; /*!< Name of the Link */
    int sister;/*!< Sister ID */
    int child;/*!< Child ID */
    int mother;/*!< Mother ID */
    int upper;/*!< Upper ID */
    int color;/*!< Color representation of the joint */
    int isPolygon;/*!< is the link capable of contact */
    double m;/*!< Joint mass [kg] */
    double q;/*!< Joint position [rad] */
    double dq;/*!< Joint velocity [rad/s] */
    double ddq;/*!< Joint acceleration [rad/s^2] */
    double Ir;/*!< Child electric motor moment of inertia [kg.m^2] */
    double gr;/*!< Reduction ratio of the motor [-] */
    double u;/*!< Joint torque [Nm] */
    double ug;/*!< Gravity generated torque [Nm] */
    double uef;/*!< Externals forces generated torque [Nm] */
    double u_joint;/*!< Command torque [Nm] */
    double umin;/*!< Command torque limit [Nm] */
    double umax;/*!< Command torque limit [Nm] */
    double supportHeight;/*!< Ground height [m] */
    double qmin;/*!< Joint low limit [rad] */
    double qmax;/*!< Joint high limit [rad] */
    double qmoy;/*!< Joint natural value [rad] */
    gsl_vector * a;/*!< Axe of rotation 3*1 [-] */
    gsl_vector * b;/*!< Link dimensions 3*1 [m] */
    gsl_vector * c;/*!< Center of gravity 3*1 [m] */
    gsl_vector * p;/*!< Absolute link position 3*1 [m]  */
    gsl_vector * v;/*!< Absolute linear velocity 3*1 [m/s] */
    gsl_vector * vo;/*!< Main body linear velocity 3*1 [m/s] (only used for main body) */
    gsl_vector * w;/*!< Absolute orientation velocity 3*1 [rad/s] */
    gsl_vector * dvo;/*!< Main body linear acceleration 3*1 [m/s^2] (only used for main body) */
    gsl_vector * dw;/*!< Absolute orientation acceleration 3*1 [rad/s^2] */
    gsl_vector * hw;/*!< Axe of rotation in world reference 3*1 [-] */
    gsl_vector * hv;/*!< Axe of rotation in world reference cross by position of the link 3*1 */
    gsl_vector * isContact;/*!< is the link corner in contact */
    float com[3];
    float shape[3];
    gsl_matrix * R;/*!< Absolute orientation 3*3 [-] */
    gsl_matrix * I;/*!< around the center of gravity inertia tensor 3*3 [kg.m^2] */
    gsl_matrix * vert;/*!< Shape of the link */
    gsl_matrix * face;/*!< Shape of the link */
    gsl_matrix * posContact;
    gsl_matrix * forContact;
};

typedef struct  StuLINK  SuLINK ;


/**
 * \struct State uLink.h
 * \brief State contain some global information about the robot state
 */
typedef struct
{
    char RobotFile[255]; /*!< Path of XML file for robot description */
    int desired_support;/*!< Desired foot of support: 0:none,1:right,2:left,3:both */
    int ddl;/*!< Number of Degrees of freedom */
    int support;/*!< Foot of support: 0:none,1:right,2:left,3:both */
    int right_foot_ID;/*!< Right foot ID */
    int left_foot_ID;/*!< Right foot ID */
    float right_scale;/*!< External forces drawing */
    float left_scale;/*!< External forces drawing */
    float integral_R;/*!< Stabilisator command */
    float integral_L;/*!< Stabilisator command */
    float distribution_y;/*!< External forces distribution betwenn the two legs on y axis [-] */
    float distribution_x;/*!< External forces distribution betwenn the two legs on x axis [-] */
    gsl_vector * com_old;/*!< Position of the Center of Mass at previous iteration [m] */
    gsl_vector * posCoP_R;/*!< Right foot external forces position 3*1 [m] */
    gsl_vector * forCoP_R;/*!< Right foot external forces amplitude 3*1 [N] */
    gsl_vector * posCoP_L;/*!< Left foot external forces position 3*1 [m] */
    gsl_vector * forCoP_L;/*!< Left foot external forces amplitude 3*1 [N] */
    gsl_vector * FootCenter_R;/*!< Right foot position 3*1 [m] */
    gsl_vector * FootCenter_L;/*!< Left foot position 3*1 [m] */
} State ;


/**
 * \fn void SetupRobot(SuLINK uLINK[],State *Status)
 * \brief Initialize the link structure with the model value
 *
 * \param uLINK[] Structure wich describe the robot link by link
 * \param Status Structure wich describe the state of the robot
 */
void SetupRobot(SuLINK uLINK[],State *Status);


#endif




