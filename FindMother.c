#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include "uLINK.h"
#include "FindMother.h"

void FindMother(SuLINK uLINK[],int j)
{
    if (j != 0)
    {
        if (j == 1)
        {
            uLINK[j].mother = 0;
        }
        if (uLINK[j].child != 0)
        {
            uLINK[uLINK[j].child].mother = j;
            uLINK[uLINK[j].child].upper = j;
            FindMother(uLINK,uLINK[j].child);
        }

        if (uLINK[j].sister != 0)
        {
            uLINK[uLINK[j].sister].mother = uLINK[j].mother;
            uLINK[uLINK[j].sister].upper = j;
            FindMother(uLINK,uLINK[j].sister);
        }
    }
    //printf("%d \n",uLINK[j].mother);
}
