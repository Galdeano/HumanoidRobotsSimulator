#ifndef H_uLink
#define H_uLink

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include "Setup.h"

#if LoadObj
#include "ObjLoader.h"
#endif
#include "butterworth.h"

#include <utility>
#include <cstring>

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
    double com[3];
    double shape[3];
    gsl_matrix * R;/*!< Absolute orientation 3*3 [-] */
    gsl_matrix * I;/*!< around the center of gravity inertia tensor 3*3 [kg.m^2] */
    gsl_matrix * vert;/*!< Shape of the link */
    gsl_matrix * face;/*!< Shape of the link */
    gsl_matrix * normalface;/*!< Shape of the link */
    gsl_matrix * contact;/*!< relative position of contacts */
    gsl_matrix * pcontact;/*!< absolute position of contacts */
    gsl_matrix * fcontact;/*!< force of contacts */
    gsl_vector * iscontact;/*!< is the link in contact */
    gsl_matrix * posContact;/*!< absolute position of contacts */
    gsl_matrix * forContact;/*!< force of contacts */
    char obj[50]; /*!< obj path */
    #if LoadObj
    MeshObj Mesh_obj;
    gsl_vector * obj_offset;
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
        a = nullptr;
        b = nullptr;
        c = nullptr;
        p = nullptr;
        v = nullptr;
        vo = nullptr;
        w = nullptr;
        dvo = nullptr;
        dw = nullptr;
        hw = nullptr;
        hv = nullptr;
        isContact = nullptr;
        memset(com, 0, sizeof(com));
        memset(shape, 0, sizeof(shape));
        R = nullptr;
        I = nullptr;
        vert = nullptr;
        face = nullptr;
        normalface = nullptr;
        contact = nullptr;
        pcontact = nullptr;
        fcontact = nullptr;
        iscontact = nullptr;
        posContact = nullptr;
        forContact = nullptr;
        memset(obj, 0, sizeof(obj));
        #if LoadObj
        obj_offset = nullptr;
        #endif
    }

    ~StuLINK() {
        if (a) gsl_vector_free(a);
        if (b) gsl_vector_free(b);
        if (c) gsl_vector_free(c);
        if (p) gsl_vector_free(p);
        if (v) gsl_vector_free(v);
        if (vo) gsl_vector_free(vo);
        if (w) gsl_vector_free(w);
        if (dvo) gsl_vector_free(dvo);
        if (dw) gsl_vector_free(dw);
        if (hw) gsl_vector_free(hw);
        if (hv) gsl_vector_free(hv);
        if (isContact) gsl_vector_free(isContact);
        if (R) gsl_matrix_free(R);
        if (I) gsl_matrix_free(I);
        if (vert) gsl_matrix_free(vert);
        if (face) gsl_matrix_free(face);
        if (normalface) gsl_matrix_free(normalface);
        if (contact) gsl_matrix_free(contact);
        if (pcontact) gsl_matrix_free(pcontact);
        if (fcontact) gsl_matrix_free(fcontact);
        if (iscontact) gsl_vector_free(iscontact);
        if (posContact) gsl_matrix_free(posContact);
        if (forContact) gsl_matrix_free(forContact);
        #if LoadObj
        if (obj_offset) gsl_vector_free(obj_offset);
        #endif
    }

    // Disable copy semantics
    StuLINK(const StuLINK&) = delete;
    StuLINK& operator=(const StuLINK&) = delete;

    // Enable move semantics
    StuLINK(StuLINK&& other) noexcept {
        *this = std::move(other);
    }
    StuLINK& operator=(StuLINK&& other) noexcept {
        if (this != &other) {
            this->~StuLINK();

            memcpy(name, other.name, sizeof(name));
            fixed = other.fixed;
            sister = other.sister;
            child = other.child;
            mother = other.mother;
            upper = other.upper;
            color = other.color;
            isPolygon = other.isPolygon;
            m = other.m;
            q = other.q;
            dq = other.dq;
            ddq = other.ddq;
            Ir = other.Ir;
            gr = other.gr;
            u = other.u;
            ug = other.ug;
            uef = other.uef;
            u_joint = other.u_joint;
            umin = other.umin;
            umax = other.umax;
            supportHeight = other.supportHeight;
            qmin = other.qmin;
            qmax = other.qmax;
            qmoy = other.qmoy;
            memcpy(com, other.com, sizeof(com));
            memcpy(shape, other.shape, sizeof(shape));
            memcpy(obj, other.obj, sizeof(obj));
            #if LoadObj
            Mesh_obj = std::move(other.Mesh_obj);
            #endif
            filter = other.filter;

            a = other.a; other.a = nullptr;
            b = other.b; other.b = nullptr;
            c = other.c; other.c = nullptr;
            p = other.p; other.p = nullptr;
            v = other.v; other.v = nullptr;
            vo = other.vo; other.vo = nullptr;
            w = other.w; other.w = nullptr;
            dvo = other.dvo; other.dvo = nullptr;
            dw = other.dw; other.dw = nullptr;
            hw = other.hw; other.hw = nullptr;
            hv = other.hv; other.hv = nullptr;
            isContact = other.isContact; other.isContact = nullptr;
            R = other.R; other.R = nullptr;
            I = other.I; other.I = nullptr;
            vert = other.vert; other.vert = nullptr;
            face = other.face; other.face = nullptr;
            normalface = other.normalface; other.normalface = nullptr;
            contact = other.contact; other.contact = nullptr;
            pcontact = other.pcontact; other.pcontact = nullptr;
            fcontact = other.fcontact; other.fcontact = nullptr;
            iscontact = other.iscontact; other.iscontact = nullptr;
            posContact = other.posContact; other.posContact = nullptr;
            forContact = other.forContact; other.forContact = nullptr;
            #if LoadObj
            obj_offset = other.obj_offset; other.obj_offset = nullptr;
            #endif
        }
        return *this;
    }
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
    gsl_vector * com_old;/*!< Position of the Center of Mass at previous iteration [m] */
    gsl_vector * posCoP_R;/*!< Right foot external forces position 3*1 [m] */
    gsl_vector * forCoP_R;/*!< Right foot external forces amplitude 3*1 [N] */
    gsl_vector * posCoP_L;/*!< Left foot external forces position 3*1 [m] */
    gsl_vector * forCoP_L;/*!< Left foot external forces amplitude 3*1 [N] */
    gsl_vector * FootCenter_R;/*!< Right foot position 3*1 [m] */
    gsl_vector * FootCenter_L;/*!< Left foot position 3*1 [m] */
    gsl_vector * limbID;

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
        com_old = nullptr;
        posCoP_R = nullptr;
        forCoP_R = nullptr;
        posCoP_L = nullptr;
        forCoP_L = nullptr;
        FootCenter_R = nullptr;
        FootCenter_L = nullptr;
        limbID = nullptr;
    }

    ~State() {
        if (com_old) gsl_vector_free(com_old);
        if (posCoP_R) gsl_vector_free(posCoP_R);
        if (forCoP_R) gsl_vector_free(forCoP_R);
        if (posCoP_L) gsl_vector_free(posCoP_L);
        if (forCoP_L) gsl_vector_free(forCoP_L);
        if (FootCenter_R) gsl_vector_free(FootCenter_R);
        if (FootCenter_L) gsl_vector_free(FootCenter_L);
        if (limbID) gsl_vector_free(limbID);
    }

    // Disable copy semantics
    State(const State&) = delete;
    State& operator=(const State&) = delete;

    // Enable move semantics
    State(State&& other) noexcept {
        *this = std::move(other);
    }
    State& operator=(State&& other) noexcept {
        if (this != &other) {
            this->~State();
            memcpy(RobotFile, other.RobotFile, sizeof(RobotFile));
            desired_support = other.desired_support;
            ddl = other.ddl;
            support = other.support;
            right_foot_ID = other.right_foot_ID;
            left_foot_ID = other.left_foot_ID;
            nb_limb = other.nb_limb;
            right_scale = other.right_scale;
            left_scale = other.left_scale;
            integral_R = other.integral_R;
            integral_L = other.integral_L;
            distribution_y = other.distribution_y;
            distribution_x = other.distribution_x;

            com_old = other.com_old; other.com_old = nullptr;
            posCoP_R = other.posCoP_R; other.posCoP_R = nullptr;
            forCoP_R = other.forCoP_R; other.forCoP_R = nullptr;
            posCoP_L = other.posCoP_L; other.posCoP_L = nullptr;
            forCoP_L = other.forCoP_L; other.forCoP_L = nullptr;
            FootCenter_R = other.FootCenter_R; other.FootCenter_R = nullptr;
            FootCenter_L = other.FootCenter_L; other.FootCenter_L = nullptr;
            limbID = other.limbID; other.limbID = nullptr;
        }
        return *this;
    }
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




