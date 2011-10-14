#include "uLINK_f.h"
#include "FindMother_f.h"

void FindMother_f(Struct_uLINK uLINK[],int j)
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
            FindMother_f(uLINK,uLINK[j].child);
        }

        if (uLINK[j].sister != 0)
        {
            uLINK[uLINK[j].sister].mother = uLINK[j].mother;
            FindMother_f(uLINK,uLINK[j].sister);
        }
    }
}
