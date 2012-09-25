#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include "uLINK.h"
#include "SetupRigidBody.h"
#include "Setup.h"


void SetupRigidBody(SuLINK uLINK[],int j)
{

    int i,k,l;
    char Masque[3][8]= {{0,0,1,1,0,0,1,1},{0,1,1,0,0,1,1,0},{0,0,0,0,1,1,1,1}};
    uLINK[j].vert = gsl_matrix_calloc (3, 8);
    uLINK[j].face = gsl_matrix_calloc (4, 6);
    uLINK[j].normalface = gsl_matrix_calloc (3,6);
    uLINK[j].isContact = gsl_vector_calloc (8);
    uLINK[j].posContact = gsl_matrix_calloc (3, 8);
    uLINK[j].forContact = gsl_matrix_calloc (3, 8);
    uLINK[j].contact = gsl_matrix_calloc (3, xContact*yContact);
    uLINK[j].pcontact = gsl_matrix_calloc (3, xContact*yContact);
    uLINK[j].fcontact = gsl_matrix_calloc (3, xContact*yContact);
    uLINK[j].iscontact = gsl_vector_calloc (xContact*yContact);


    for(i=0; i<3; i++)
    {
        for(k=0; k<8; k++)
        {
            gsl_matrix_set (uLINK[j].vert,i,k,Masque[i][k]*uLINK[j].shape[i]-uLINK[j].shape[i]/2+uLINK[j].com[i]);
        }
    }


    for(k=0; k<xContact; k++)
    {
        for(l=0; l<yContact; l++)
        {
            gsl_matrix_set (uLINK[j].contact,0,k*yContact+l, (double)k/(double)(xContact-1)*uLINK[j].shape[0]-uLINK[j].shape[0]/2+uLINK[j].com[0]);
            gsl_matrix_set (uLINK[j].contact,1,k*yContact+l, (double)l/(double)(yContact-1)*uLINK[j].shape[1]-uLINK[j].shape[1]/2+uLINK[j].com[1]);
            gsl_matrix_set (uLINK[j].contact,2,k*yContact+l, -uLINK[j].shape[2]/2+uLINK[j].com[2]);
        }
    }

    //PrintGSLMatrixTranspose(uLINK[j].contact);


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


    gsl_matrix_set (uLINK[j].normalface,0,0,0);
    gsl_matrix_set (uLINK[j].normalface,0,1,0);
    gsl_matrix_set (uLINK[j].normalface,0,2,1);
    gsl_matrix_set (uLINK[j].normalface,0,3,0);
    gsl_matrix_set (uLINK[j].normalface,0,4,-1);
    gsl_matrix_set (uLINK[j].normalface,0,5,0);

    gsl_matrix_set (uLINK[j].normalface,1,0,0);
    gsl_matrix_set (uLINK[j].normalface,1,1,1);
    gsl_matrix_set (uLINK[j].normalface,1,2,0);
    gsl_matrix_set (uLINK[j].normalface,1,3,-1);
    gsl_matrix_set (uLINK[j].normalface,1,4,0);
    gsl_matrix_set (uLINK[j].normalface,1,5,0);

    gsl_matrix_set (uLINK[j].normalface,2,0,-1);
    gsl_matrix_set (uLINK[j].normalface,2,1,0);
    gsl_matrix_set (uLINK[j].normalface,2,2,0);
    gsl_matrix_set (uLINK[j].normalface,2,3,0);
    gsl_matrix_set (uLINK[j].normalface,2,4,0);
    gsl_matrix_set (uLINK[j].normalface,2,5,1);

    uLINK[j].isPolygon=1;

}




