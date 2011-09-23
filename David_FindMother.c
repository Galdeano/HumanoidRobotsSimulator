#include "David_uLINK.h"
#include "David_FindMother.h"

void David_FindMother(Struct_uLINK uLINK[],int j)
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
            David_FindMother(uLINK,uLINK[j].child);
        }

        if (uLINK[j].sister != 0)
        {
            uLINK[uLINK[j].sister].mother = uLINK[j].mother;
            David_FindMother(uLINK,uLINK[j].sister);
        }
    }
}
