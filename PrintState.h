#ifndef H_PrintState
#define H_PrintState

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>


/**
 * \struct SaveLINK CopyuLink.h
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
    double dvo[3];/*!< Main body linear acceleration 3*1 [m/s^2] (only used for main body) */
    double dw[3];/*!< Absolute orientation acceleration 3*1 [rad/s^2] */
    double hw[3];/*!< Axe of rotation in world reference 3*1 [-] */
    double hv[3];/*!< Axe of rotation in world reference cross by position of the link 3*1 */
    double R[9];/*!< Absolute orientation 3*3 [-] */
} SaveLINK ;


void SaveStateXML(SuLINK uLINK[],float t);

void SaveState(SuLINK uLINK[],long *t);
void SaveStuctLink(SuLINK uLINK[],SaveLINK CopyuLINK[]);

void LoadState(SuLINK uLINK[],long *t);
void LoadStuctLink(SuLINK uLINK[],SaveLINK CopyuLINK[]);

#endif
