#ifndef H_PrintState
#define H_PrintState

/** \file PrintState.h
 *  \brief Save and load states
 *  \author    David Galdeano
 *  \date      10/2011
 *
 * These files allow saving and loading some state of the robot
 */

/**
 * \struct SaveLINK PrintState.h
 * \brief SaveLINK is a copy of Sulink with no pointer to allow a binary save of every variables in the robot state
 */
typedef struct
{
    double q;/*!< Joint position [rad] */
    double dq;/*!< Joint velocity [rad/s] */
    double ddq;/*!< Joint acceleration [rad/s^2] */
    double u;/*!< Joint torque [Nm] */
    double ug;/*!< Gravity generated torque [Nm] */
    double uef;/*!< Externals forces generated torque [Nm] */
    double u_joint;/*!< Command torque [Nm] */
    double p[3];/*!< Absolute link position 3*1 [m]  */
    double v[3];/*!< Absolute linear velocity 3*1 [m/s] */
    double vo[3];/*!< Main body linear velocity 3*1 [m/s] (only used for main body) */
    double w[3];/*!< Absolute orientation velocity 3*1 [rad/s] */
    double dvo[3];/*!< Main body linear acceleration 3*1 [m/s^2] */
    double dw[3];/*!< Absolute orientation acceleration 3*1 [rad/s^2] */
    double hw[3];/*!< Axis of rotation in world reference 3*1 [-] */
    double hv[3];/*!< Axis of rotation in world reference cross by position of the link 3*1 */
    double R[9];/*!< Absolute orientation 3*3 [-] */
} SaveLINK ;


/**
 * \fn void SaveStateXML(SuLINK uLINK[],double t)
 * \brief Save the state of the robot in a XML file
 *
 * \param uLINK[] Structure wich describe the robot link by link
 * \param t Current time
 */
void SaveStateXML(SuLINK uLINK[],State *Status,double t);

/**
 * \fn void SaveRobotXML(SuLINK uLINK[],double t)
 * \brief Save the structure of the robot in a XML file
 *
 * \param uLINK[] Structure wich describe the robot link by link
 */
void SaveRobotXML(SuLINK uLINK[],State *Status);

/**
 * \fn void SaveState(SuLINK uLINK[],long *t)
 * \brief Save the state of the robot in a binary file
 * \param uLINK[] Structure wich describe the robot link by link
 * \param t Current time
 */
void SaveState(SuLINK uLINK[],State *Status,long *t);

/**
 * \fn void SaveStuctLink(SuLINK uLINK[],SaveLINK CopyuLINK[])
 * \brief Transfert the state of the robot to a structure without pointer
 * \param uLINK[] Structure wich describe the robot link by link
 * \param CopyuLINK[] Structure wich describe the robot witout pointers
 */
void SaveStuctLink(SuLINK uLINK[],SaveLINK CopyuLINK[],State *Status);


/**
 * \fn void LoadState(SuLINK uLINK[],long *t)
 * \brief Load the state of the robot from a binary file
 * \param uLINK[] Structure wich describe the robot link by link
 * \param t Current time
 */
void LoadState(SuLINK uLINK[],State *Status,long *t);

/**
 * \fn void LoadStuctLink(SuLINK uLINK[],SaveLINK CopyuLINK[])
 * \brief Transfert the saved structure without pointer to the actual state of the robot
 * \param uLINK[] Structure wich describe the robot link by link
 * \param CopyuLINK[] Structure wich describe the robot witout pointers
 */
void LoadStuctLink(SuLINK uLINK[],SaveLINK CopyuLINK[],State *Status);

#endif
