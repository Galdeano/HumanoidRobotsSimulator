#ifndef H_ForwardDynamicsWorkspace
#define H_ForwardDynamicsWorkspace

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_permutation.h>
#include "uLink.h"
#include "uLINK_f.h"

typedef struct {
    // Outer initialization block (init == 1)
    Struct_uLINK *uLINKc;
    gsl_matrix * Inertia_Motor;
    gsl_matrix *A;
    gsl_vector *b;
    gsl_vector *tmp;
    gsl_vector *g;
    gsl_vector *ef;
    gsl_vector *stab;
    gsl_vector *u;
    gsl_vector * ddq;
    gsl_permutation * p;
    gsl_matrix * A2;
    Struct_State Statusc;

    // Gravity locals (f and tau formerly leaked)
    gsl_vector *f;
    gsl_vector *tau;

    // init_com block (init_com == 1)
    double *uPD;
    double *qd;
    double *dqd;
    double com[3];
    double FootR[3];
    double FootL[3];

    // init_G block (init_G == 1)
    double *uG;
    double *fG;
    double *tG;
    double *uStab;

    // init_task block (init_task == 1)
    gsl_vector * idx1;
    gsl_vector * idx2;
    gsl_matrix * J1;
    gsl_matrix * J2;
    gsl_matrix * JCoMR;
    gsl_matrix * JCoML;
    gsl_matrix * Jtilde;
    gsl_vector * task1;
    gsl_vector * task2;
    gsl_vector * taskCoMR;
    gsl_vector * taskCoML;
    gsl_vector * CoM;
    gsl_vector * vec3;
    gsl_vector * vec3_2;
    gsl_vector * p_task;  // Renamed from 'p' to prevent naming conflicts
    gsl_matrix * R;
    gsl_matrix * P1;
    gsl_matrix * P2;
    gsl_matrix * PCoMR;
    gsl_matrix * PCoML;
    gsl_matrix * Ptilde;
    gsl_matrix * Ptmp;
    gsl_matrix * invJ;
    gsl_matrix * invJCoM;
    gsl_vector * dq;
    gsl_vector * dqtmp;
    gsl_vector * dqtmp2;
    gsl_vector * dq_old;
    gsl_vector * ddq_task;  // Renamed from 'ddq' to prevent naming conflicts
    double *opd;
    gsl_vector * trace;
    gsl_vector * adphi;
    double *qdev;
    gsl_vector * CoP;
    gsl_vector * zmp;
    gsl_vector * dzmp;
    gsl_vector * q_pd;

    double *Jf;
    double *pf;
    double *Rf;
    double *taskf;
    double *invf;
    double *dqf;

    // Initialization tracking flags
    int init;
    int init_com;
    int init_G;
    int init_task;

    // Additional tracking state
    double wO;
    double amp;

} ForwardDynamicsWorkspace;

ForwardDynamicsWorkspace* ForwardDynamicsWorkspace_alloc(int nDoF);
void ForwardDynamicsWorkspace_free(ForwardDynamicsWorkspace* ws);

#endif
