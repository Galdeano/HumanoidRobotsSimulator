#include "uLink.h"
#include "FindRoute.h"

int FindRoute(SuLINK uLINK[], int *path, int to)
{
    int nbre = 0;
    FindRouteReq(uLINK, path, &nbre, to);
    nbre++;
    path[nbre - 1] = 1;
    return nbre;
}

void FindRouteReq(SuLINK uLINK[], int *path, int *nbre_elements, int to)
{
    int i = uLINK[to].mother;

    if (i == 1)
    {
        path[*nbre_elements] = to;
        (*nbre_elements)++;
    }
    else
    {
        path[*nbre_elements] = to;
        (*nbre_elements)++;
        FindRouteReq(uLINK, path, nbre_elements, i);
    }
}
