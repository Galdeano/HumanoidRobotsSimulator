#include "uLink.h"
#include "FindMother.h"

void FindMother(SuLINK uLINK[], State *Status, int j)
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
            FindMother(uLINK, Status, uLINK[j].child);
        }

        if (uLINK[j].sister != 0)
        {
            uLINK[uLINK[j].sister].mother = uLINK[j].mother;
            uLINK[uLINK[j].sister].upper = j;
            FindMother(uLINK, Status, uLINK[j].sister);
        }
        if ((uLINK[j].sister == 0) && (uLINK[j].child == 0))
        {
            Status->limbID(Status->nb_limb) = j;
            Status->nb_limb = Status->nb_limb + 1;
        }
    }
}
