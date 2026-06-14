#ifndef H_uLink
#define H_uLink

#include <Eigen/Dense>
#include "Setup.h"

#if LoadObj
#include "ObjLoader.h"
#endif
#include "butterworth.h"

#include <utility>
#include <cstring>
#include <string>

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
    int fixed;/*!< Is the link a dof */
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
    
    Eigen::Vector3d a;/*!< Axe of rotation 3*1 [-] */
    Eigen::Vector3d b;/*!< Link dimensions 3*1 [m] */
    Eigen::Vector3d c;/*!< Center of gravity 3*1 [m] */
    Eigen::Vector3d p;/*!< Absolute link position 3*1 [m]  */
    Eigen::Vector3d v;/*!< Absolute linear velocity 3*1 [m/s] */
    Eigen::Vector3d vo;/*!< Main body linear velocity 3*1 [m/s] (only used for main body) */
    Eigen::Vector3d w;/*!< Absolute orientation velocity 3*1 [rad/s] */
    Eigen::Vector3d dvo;/*!< Main body linear acceleration 3*1 [m/s^2] (only used for main body) */
    Eigen::Vector3d dw;/*!< Absolute orientation acceleration 3*1 [rad/s^2] */
    Eigen::Vector3d hw;/*!< Axe of rotation in world reference 3*1 [-] */
    Eigen::Vector3d hv;/*!< Axe of rotation in world reference cross by position of the link 3*1 */
    Eigen::VectorXd isContact;/*!< is the link corner in contact */
    double com[3] = {0.0, 0.0, 0.0};
    double shape[3] = {0.0, 0.0, 0.0};
    Eigen::Matrix3d R;/*!< Absolute orientation 3*3 [-] */
    Eigen::Matrix3d I;/*!< around the center of gravity inertia tensor 3*3 [kg.m^2] */
    
    Eigen::MatrixXd vert;/*!< Shape of the link */
    Eigen::MatrixXd face;/*!< Shape of the link */
    Eigen::MatrixXd normalface;/*!< Shape of the link */
    Eigen::MatrixXd contact;/*!< relative position of contacts */
    Eigen::MatrixXd pcontact;/*!< absolute position of contacts */
    Eigen::MatrixXd fcontact;/*!< force of contacts */
    Eigen::VectorXd iscontact;/*!< is the link in contact */
    Eigen::MatrixXd posContact;/*!< absolute position of contacts */
    Eigen::MatrixXd forContact;/*!< force of contacts */
    char obj[50] = {0}; /*!< obj path */
    #if LoadObj
    MeshObj Mesh_obj;
    Eigen::Vector3d obj_offset;
    #endif
    ButterworthData filter;

    StuLINK() {
        memset(name, 0, sizeof(name));
        fixed = 0;
        sister = 0;
        child = 0;
        mother = 0;
        upper = 0;
        color = 0;
        isPolygon = 0;
        m = 0.0;
        q = 0.0;
        dq = 0.0;
        ddq = 0.0;
        Ir = 0.0;
        gr = 0.0;
        u = 0.0;
        ug = 0.0;
        uef = 0.0;
        u_joint = 0.0;
        umin = 0.0;
        umax = 0.0;
        supportHeight = 0.0;
        qmin = 0.0;
        qmax = 0.0;
        qmoy = 0.0;

        a.setZero();
        b.setZero();
        c.setZero();
        p.setZero();
        v.setZero();
        vo.setZero();
        w.setZero();
        dvo.setZero();
        dw.setZero();
        hw.setZero();
        hv.setZero();
        isContact = Eigen::VectorXd::Zero(8);
        R.setIdentity();
        I.setZero();
        #if LoadObj
        obj_offset.setZero();
        #endif
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef struct  StuLINK  SuLINK ;


/**
 * \struct State uLink.h
 * \brief State contain some global information about the robot state
 */
struct State
{
    char RobotFile[255]; /*!< Path of XML file for robot description */
    int desired_support;/*!< Desired foot of support: 0:none,1:right,2:left,3:both */
    int ddl;/*!< Number of Degrees of freedom */
    int support;/*!< Foot of support: 0:none,1:right,2:left,3:both */
    int right_foot_ID;/*!< Right foot ID */
    int left_foot_ID;/*!< Right foot ID */
    int nb_limb;
    double right_scale;/*!< External forces drawing */
    double left_scale;/*!< External forces drawing */
    double integral_R;/*!< Stabilisator command */
    double integral_L;/*!< Stabilisator command */
    double distribution_y;/*!< External forces distribution betwenn the two legs on y axis [-] */
    double distribution_x;/*!< External forces distribution betwenn the two legs on x axis [-] */
    Eigen::Vector3d com_old;/*!< Position of the Center of Mass at previous iteration [m] */
    Eigen::Vector3d posCoP_R;/*!< Right foot external forces position 3*1 [m] */
    Eigen::Vector3d forCoP_R;/*!< Right foot external forces amplitude 3*1 [N] */
    Eigen::Vector3d posCoP_L;/*!< Left foot external forces position 3*1 [m] */
    Eigen::Vector3d forCoP_L;/*!< Left foot external forces amplitude 3*1 [N] */
    Eigen::Vector3d FootCenter_R;/*!< Right foot position 3*1 [m] */
    Eigen::Vector3d FootCenter_L;/*!< Left foot position 3*1 [m] */
    Eigen::VectorXd limbID;

    State() {
        memset(RobotFile, 0, sizeof(RobotFile));
        desired_support = 0;
        ddl = 0;
        support = 0;
        right_foot_ID = 0;
        left_foot_ID = 0;
        nb_limb = 0;
        right_scale = 0;
        left_scale = 0;
        integral_R = 0;
        integral_L = 0;
        distribution_y = 0;
        distribution_x = 0;

        com_old.setZero();
        posCoP_R.setZero();
        forCoP_R.setZero();
        posCoP_L.setZero();
        forCoP_L.setZero();
        FootCenter_R.setZero();
        FootCenter_L.setZero();
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * \fn void SetupRobot(SuLINK uLINK[],State *Status)
 * \brief Initialize the link structure with the model value
 *
 * \param uLINK[] Structure wich describe the robot link by link
 * \param Status Structure wich describe the state of the robot
 */
void SetupRobot(SuLINK uLINK[],State *Status);


#endif
