#include <stdlib.h>
#include "ForwardDynamicsWorkspace.h"

ForwardDynamicsWorkspace* ForwardDynamicsWorkspace_alloc(int nDoF) {
    ForwardDynamicsWorkspace* ws = calloc(1, sizeof(ForwardDynamicsWorkspace));
    if (!ws) return NULL;

    ws->init = 1;
    ws->init_com = 1;
    ws->init_G = 1;
    ws->init_task = 1;

    ws->wO = 5.0;
    ws->amp = 0.043;

    ws->FootR[0] = 0.0853; ws->FootR[1] = 0.0; ws->FootR[2] = -0.11;
    ws->FootL[0] = 0.0853; ws->FootL[1] = 0.0; ws->FootL[2] = -0.11;

    ws->uLINKc = calloc(nDoF + 2 - 6, sizeof(Struct_uLINK));
    ws->Inertia_Motor = gsl_matrix_calloc(nDoF, nDoF);
    ws->A = gsl_matrix_calloc(nDoF, nDoF);
    ws->b = gsl_vector_calloc(nDoF);
    ws->tmp = gsl_vector_calloc(nDoF);
    ws->g = gsl_vector_calloc(nDoF);
    ws->ef = gsl_vector_calloc(nDoF);
    ws->stab = gsl_vector_calloc(nDoF);
    ws->u = gsl_vector_calloc(nDoF);
    ws->ddq = gsl_vector_alloc(nDoF);
    ws->p = gsl_permutation_alloc(nDoF);
    ws->A2 = gsl_matrix_calloc(nDoF, nDoF);

    ws->f = gsl_vector_calloc(3);
    ws->tau = gsl_vector_calloc(3);

    ws->uPD = calloc(nDoF - 6, sizeof(double));
    ws->qd = calloc(nDoF - 6, sizeof(double));
    ws->dqd = calloc(nDoF - 6, sizeof(double));

    ws->uG = calloc(nDoF - 6, sizeof(double));
    ws->fG = calloc(nDoF - 6, sizeof(double));
    ws->tG = calloc(nDoF - 6, sizeof(double));
    ws->uStab = calloc(nDoF - 6, sizeof(double));

    ws->idx1 = gsl_vector_calloc(8);
    ws->idx2 = gsl_vector_calloc(14);
    ws->J1 = gsl_matrix_calloc(6, nDoF - 6);
    ws->J2 = gsl_matrix_calloc(6, nDoF - 6);
    ws->JCoMR = gsl_matrix_calloc(3, nDoF - 6);
    ws->JCoML = gsl_matrix_calloc(3, nDoF - 6);
    ws->Jtilde = gsl_matrix_calloc(3, nDoF - 6);
    ws->task1 = gsl_vector_calloc(6);
    ws->task2 = gsl_vector_calloc(6);
    ws->taskCoMR = gsl_vector_calloc(3);
    ws->taskCoML = gsl_vector_calloc(3);
    ws->CoM = gsl_vector_calloc(3);
    ws->vec3 = gsl_vector_calloc(3);
    ws->vec3_2 = gsl_vector_calloc(3);
    ws->p_task = gsl_vector_calloc(3);
    ws->R = gsl_matrix_calloc(3, 3);
    ws->P1 = gsl_matrix_calloc(nDoF - 6, nDoF - 6);
    ws->P2 = gsl_matrix_calloc(nDoF - 6, nDoF - 6);
    ws->PCoMR = gsl_matrix_calloc(nDoF - 6, nDoF - 6);
    ws->PCoML = gsl_matrix_calloc(nDoF - 6, nDoF - 6);
    ws->Ptilde = gsl_matrix_calloc(nDoF - 6, nDoF - 6);
    ws->Ptmp = gsl_matrix_calloc(nDoF - 6, nDoF - 6);
    ws->invJ = gsl_matrix_calloc(nDoF - 6, 6);
    ws->invJCoM = gsl_matrix_calloc(nDoF - 6, 3);
    ws->dq = gsl_vector_calloc(nDoF - 6);
    ws->dqtmp = gsl_vector_calloc(nDoF - 6);
    ws->dqtmp2 = gsl_vector_calloc(nDoF - 6);
    ws->dq_old = gsl_vector_calloc(nDoF - 6);
    ws->ddq_task = gsl_vector_calloc(nDoF - 6);
    ws->opd = calloc(9, sizeof(double));
    ws->trace = gsl_vector_calloc(3);
    ws->adphi = gsl_vector_calloc(nDoF - 6);
    ws->qdev = calloc(nDoF - 6, sizeof(double));
    ws->CoP = gsl_vector_calloc(3);
    ws->zmp = gsl_vector_calloc(3);
    ws->dzmp = gsl_vector_calloc(3);
    ws->q_pd = gsl_vector_calloc(nDoF - 6);

    ws->Jf = calloc(6 * (nDoF - 6), sizeof(double));
    ws->pf = calloc(3, sizeof(double));
    ws->Rf = calloc(9, sizeof(double));
    ws->taskf = calloc(6, sizeof(double));
    ws->invf = calloc((nDoF - 6) * 6, sizeof(double));
    ws->dqf = calloc(nDoF - 6, sizeof(double));

    return ws;
}

void ForwardDynamicsWorkspace_free(ForwardDynamicsWorkspace* ws) {
    if (!ws) return;

    if (ws->uLINKc) free(ws->uLINKc);
    if (ws->Inertia_Motor) gsl_matrix_free(ws->Inertia_Motor);
    if (ws->A) gsl_matrix_free(ws->A);
    if (ws->b) gsl_vector_free(ws->b);
    if (ws->tmp) gsl_vector_free(ws->tmp);
    if (ws->g) gsl_vector_free(ws->g);
    if (ws->ef) gsl_vector_free(ws->ef);
    if (ws->stab) gsl_vector_free(ws->stab);
    if (ws->u) gsl_vector_free(ws->u);
    if (ws->ddq) gsl_vector_free(ws->ddq);
    if (ws->p) gsl_permutation_free(ws->p);
    if (ws->A2) gsl_matrix_free(ws->A2);

    if (ws->f) gsl_vector_free(ws->f);
    if (ws->tau) gsl_vector_free(ws->tau);

    if (ws->uPD) free(ws->uPD);
    if (ws->qd) free(ws->qd);
    if (ws->dqd) free(ws->dqd);

    if (ws->uG) free(ws->uG);
    if (ws->fG) free(ws->fG);
    if (ws->tG) free(ws->tG);
    if (ws->uStab) free(ws->uStab);

    if (ws->idx1) gsl_vector_free(ws->idx1);
    if (ws->idx2) gsl_vector_free(ws->idx2);
    if (ws->J1) gsl_matrix_free(ws->J1);
    if (ws->J2) gsl_matrix_free(ws->J2);
    if (ws->JCoMR) gsl_matrix_free(ws->JCoMR);
    if (ws->JCoML) gsl_matrix_free(ws->JCoML);
    if (ws->Jtilde) gsl_matrix_free(ws->Jtilde);
    if (ws->task1) gsl_vector_free(ws->task1);
    if (ws->task2) gsl_vector_free(ws->task2);
    if (ws->taskCoMR) gsl_vector_free(ws->taskCoMR);
    if (ws->taskCoML) gsl_vector_free(ws->taskCoML);
    if (ws->CoM) gsl_vector_free(ws->CoM);
    if (ws->vec3) gsl_vector_free(ws->vec3);
    if (ws->vec3_2) gsl_vector_free(ws->vec3_2);
    if (ws->p_task) gsl_vector_free(ws->p_task);
    if (ws->R) gsl_matrix_free(ws->R);
    if (ws->P1) gsl_matrix_free(ws->P1);
    if (ws->P2) gsl_matrix_free(ws->P2);
    if (ws->PCoMR) gsl_matrix_free(ws->PCoMR);
    if (ws->PCoML) gsl_matrix_free(ws->PCoML);
    if (ws->Ptilde) gsl_matrix_free(ws->Ptilde);
    if (ws->Ptmp) gsl_matrix_free(ws->Ptmp);
    if (ws->invJ) gsl_matrix_free(ws->invJ);
    if (ws->invJCoM) gsl_matrix_free(ws->invJCoM);
    if (ws->dq) gsl_vector_free(ws->dq);
    if (ws->dqtmp) gsl_vector_free(ws->dqtmp);
    if (ws->dqtmp2) gsl_vector_free(ws->dqtmp2);
    if (ws->dq_old) gsl_vector_free(ws->dq_old);
    if (ws->ddq_task) gsl_vector_free(ws->ddq_task);
    if (ws->opd) free(ws->opd);
    if (ws->trace) gsl_vector_free(ws->trace);
    if (ws->adphi) gsl_vector_free(ws->adphi);
    if (ws->qdev) free(ws->qdev);
    if (ws->CoP) gsl_vector_free(ws->CoP);
    if (ws->zmp) gsl_vector_free(ws->zmp);
    if (ws->dzmp) gsl_vector_free(ws->dzmp);
    if (ws->q_pd) gsl_vector_free(ws->q_pd);

    if (ws->Jf) free(ws->Jf);
    if (ws->pf) free(ws->pf);
    if (ws->Rf) free(ws->Rf);
    if (ws->taskf) free(ws->taskf);
    if (ws->invf) free(ws->invf);
    if (ws->dqf) free(ws->dqf);

    free(ws);
}
