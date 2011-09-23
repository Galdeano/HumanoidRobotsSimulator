#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include "uLINK.h"
#include "SetupRigidBody.h"



void SetupRigidBody(SuLINK uLINK[],int j)
{

    int i,k;
    char Masque[3][8]= {{0,0,1,1,0,0,1,1},{0,1,1,0,0,1,1,0},{0,0,0,0,1,1,1,1}};
    uLINK[j].vert = gsl_matrix_calloc (3, 8);
    uLINK[j].face = gsl_matrix_calloc (4, 6);
    uLINK[j].isContact = gsl_vector_calloc (8);
    uLINK[j].posContact = gsl_matrix_calloc (3, 8);
    uLINK[j].forContact = gsl_matrix_calloc (3, 8);

    for(i=0; i<3; i++)
    {
        for(k=0; k<8; k++)
        {
            gsl_matrix_set (uLINK[j].vert,i,k,Masque[i][k]*uLINK[j].shape[i]-uLINK[j].shape[i]/2+uLINK[j].com[i]);
        }
    }


    gsl_matrix_set (uLINK[j].face,0,0,0);
    gsl_matrix_set (uLINK[j].face,0,1,1);
    gsl_matrix_set (uLINK[j].face,0,2,3);
    gsl_matrix_set (uLINK[j].face,0,3,0);
    gsl_matrix_set (uLINK[j].face,0,4,0);
    gsl_matrix_set (uLINK[j].face,0,5,4);

    gsl_matrix_set (uLINK[j].face,1,0,1);
    gsl_matrix_set (uLINK[j].face,1,1,5);
    gsl_matrix_set (uLINK[j].face,1,2,2);
    gsl_matrix_set (uLINK[j].face,1,3,4);
    gsl_matrix_set (uLINK[j].face,1,4,1);
    gsl_matrix_set (uLINK[j].face,1,5,5);

    gsl_matrix_set (uLINK[j].face,2,0,2);
    gsl_matrix_set (uLINK[j].face,2,1,6);
    gsl_matrix_set (uLINK[j].face,2,2,6);
    gsl_matrix_set (uLINK[j].face,2,3,7);
    gsl_matrix_set (uLINK[j].face,2,4,5);
    gsl_matrix_set (uLINK[j].face,2,5,6);

    gsl_matrix_set (uLINK[j].face,3,0,3);
    gsl_matrix_set (uLINK[j].face,3,1,2);
    gsl_matrix_set (uLINK[j].face,3,2,7);
    gsl_matrix_set (uLINK[j].face,3,3,3);
    gsl_matrix_set (uLINK[j].face,3,4,4);
    gsl_matrix_set (uLINK[j].face,3,5,7);


    uLINK[j].isPolygon=1;

}




