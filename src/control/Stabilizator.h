#ifndef H_Stabilizator
#define H_Stabilizator

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include "uLink.h"

void Stabilizator(SuLINK uLINK[], State *Status, gsl_vector *stab, double t);

#endif


