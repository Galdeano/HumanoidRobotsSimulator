#ifndef H_DrawIndicators
#define H_DrawIndicators

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

void DrawIndicators(SuLINK uLINK[],State *Status,gsl_vector * com,gsl_vector * CoP,gsl_vector * Visu,int ground);


#endif
