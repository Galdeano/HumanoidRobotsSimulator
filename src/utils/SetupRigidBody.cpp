#include <Eigen/Dense>
#include "uLink.h"
#include "SetupRigidBody.h"
#include "Setup.h"

void SetupRigidBody(SuLINK uLINK[], int j)
{
    int i, k, l;
    char Masque[3][8] = {{0,0,1,1,0,0,1,1},{0,1,1,0,0,1,1,0},{0,0,0,0,1,1,1,1}};
    
    uLINK[j].vert = Eigen::MatrixXd::Zero(3, 8);
    uLINK[j].face = Eigen::MatrixXd::Zero(4, 6);
    uLINK[j].normalface = Eigen::MatrixXd::Zero(3, 6);
    uLINK[j].isContact = Eigen::VectorXd::Zero(8);
    uLINK[j].posContact = Eigen::MatrixXd::Zero(3, 8);
    uLINK[j].forContact = Eigen::MatrixXd::Zero(3, 8);
    uLINK[j].contact = Eigen::MatrixXd::Zero(3, xContact*yContact);
    uLINK[j].pcontact = Eigen::MatrixXd::Zero(3, xContact*yContact);
    uLINK[j].fcontact = Eigen::MatrixXd::Zero(3, xContact*yContact);
    uLINK[j].iscontact = Eigen::VectorXd::Zero(xContact*yContact);

    for(i = 0; i < 3; i++)
    {
        for(k = 0; k < 8; k++)
        {
            uLINK[j].vert(i, k) = Masque[i][k] * uLINK[j].shape[i] - uLINK[j].shape[i]/2.0 + uLINK[j].com[i];
        }
    }

    for(k = 0; k < xContact; k++)
    {
        for(l = 0; l < yContact; l++)
        {
            uLINK[j].contact(0, k*yContact+l) = (double)k / (double)(xContact-1) * uLINK[j].shape[0] - uLINK[j].shape[0]/2.0 + uLINK[j].com[0];
            uLINK[j].contact(1, k*yContact+l) = (double)l / (double)(yContact-1) * uLINK[j].shape[1] - uLINK[j].shape[1]/2.0 + uLINK[j].com[1];
            uLINK[j].contact(2, k*yContact+l) = -uLINK[j].shape[2]/2.0 + uLINK[j].com[2];
        }
    }

    uLINK[j].face(0, 0) = 0;
    uLINK[j].face(1, 0) = 1;
    uLINK[j].face(2, 0) = 2;
    uLINK[j].face(3, 0) = 3;

    uLINK[j].face(0, 1) = 1;
    uLINK[j].face(1, 1) = 5;
    uLINK[j].face(2, 1) = 6;
    uLINK[j].face(3, 1) = 2;

    uLINK[j].face(0, 2) = 3;
    uLINK[j].face(1, 2) = 2;
    uLINK[j].face(2, 2) = 6;
    uLINK[j].face(3, 2) = 7;

    uLINK[j].face(0, 3) = 0;
    uLINK[j].face(1, 3) = 3;
    uLINK[j].face(2, 3) = 7;
    uLINK[j].face(3, 3) = 4;

    uLINK[j].face(0, 4) = 0;
    uLINK[j].face(1, 4) = 4;
    uLINK[j].face(2, 4) = 5;
    uLINK[j].face(3, 4) = 1;

    uLINK[j].face(0, 5) = 4;
    uLINK[j].face(1, 5) = 7;
    uLINK[j].face(2, 5) = 6;
    uLINK[j].face(3, 5) = 5;

    uLINK[j].normalface(0, 0) = 0;
    uLINK[j].normalface(0, 1) = 0;
    uLINK[j].normalface(0, 2) = 1;
    uLINK[j].normalface(0, 3) = 0;
    uLINK[j].normalface(0, 4) = -1;
    uLINK[j].normalface(0, 5) = 0;

    uLINK[j].normalface(1, 0) = 0;
    uLINK[j].normalface(1, 1) = 1;
    uLINK[j].normalface(1, 2) = 0;
    uLINK[j].normalface(1, 3) = -1;
    uLINK[j].normalface(1, 4) = 0;
    uLINK[j].normalface(1, 5) = 0;

    uLINK[j].normalface(2, 0) = -1;
    uLINK[j].normalface(2, 1) = 0;
    uLINK[j].normalface(2, 2) = 0;
    uLINK[j].normalface(2, 3) = 0;
    uLINK[j].normalface(2, 4) = 0;
    uLINK[j].normalface(2, 5) = 1;

    uLINK[j].isPolygon = 1;
}
