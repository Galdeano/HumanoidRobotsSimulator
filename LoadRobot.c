#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <stdio.h>
#include <string.h>

#include "uLink.h"
#include "uLINK_f.h"
#include "SetupRigidBody.h"
#include "FindMother.h"
#include "ForwardKinematics.h"
#include "ForwardVelocity.h"
#include "CalcCoM.h"
#include "Setup.h"
#include "d:\Mb\StLib\Mat.h"
#include "FindMother_f.h"
#include "ForwardKinematics_f.h"
#include "LoadRobot.h"


void LoadRobotXML(SuLINK uLINK[],State *Status,char* RobotFile)
{

    int i,j,k;

    FILE *f=fopen(RobotFile,"r");
    char tmp_s [100];
    int dof,tmp_i;
    float tmp_f;
    float tmp_tab3[3], tmp_tab9[9];
    if (f == NULL) perror ("Error opening robot description file");
    fscanf (f, "%s", tmp_s);
    fscanf (f, "%s", tmp_s);
    fscanf (f, "%i", &dof);
//printf("DoF: %i \n",dof);


    for(i=1; i<(dof+2); i++)
    {
        uLINK[i].q = 0.0;
        uLINK[i].dq = 0.0;
        uLINK[i].ddq = 0.0;
        uLINK[i].Ir = 0.0;
        uLINK[i].gr = 0.0;
        uLINK[i].u = 0.0;
        uLINK[i].ug = 0.0;
        uLINK[i].uef = 0.0;
        uLINK[i].u_joint = 0.0;
        uLINK[i].isPolygon=0;
        uLINK[i].a = gsl_vector_calloc (3);
        uLINK[i].b = gsl_vector_calloc (3);
        uLINK[i].p = gsl_vector_calloc (3);
        uLINK[i].c = gsl_vector_calloc (3);
        uLINK[i].v = gsl_vector_calloc (3);
        uLINK[i].vo = gsl_vector_calloc (3);
        uLINK[i].w = gsl_vector_calloc (3);
        uLINK[i].dvo = gsl_vector_calloc (3);
        uLINK[i].dw = gsl_vector_calloc (3);
        uLINK[i].hw = gsl_vector_calloc (3);
        uLINK[i].hv = gsl_vector_calloc (3);
        uLINK[i].R = gsl_matrix_calloc (3, 3);
        uLINK[i].I = gsl_matrix_calloc (3, 3);
    }



    Status->ddl=dof+6;
    Status->support=0; //0:none,1:right,2:left,3:both
    Status->desired_support=0;
    Status->distribution_y=0.5;

    for(j=0;j<6;j++)
    {fscanf (f, "%s", tmp_s);}
    fscanf (f, "%i", &tmp_i);
    Status->right_foot_ID=tmp_i;
//printf("right: %i \n",tmp_i);
    for(j=0;j<2;j++)
    {fscanf (f, "%s", tmp_s);}
    fscanf (f, "%i", &tmp_i);
    Status->left_foot_ID=tmp_i;
//printf("left: %i \n",tmp_i);
    Status->integral_R=0;
    Status->integral_L=0;
    Status->com_old=gsl_vector_calloc (3);
    Status->forCoP_R=gsl_vector_calloc (3);
    Status->posCoP_R=gsl_vector_calloc (3);
    Status->forCoP_L=gsl_vector_calloc (3);
    Status->posCoP_L=gsl_vector_calloc (3);
    Status->FootCenter_R=gsl_vector_calloc (3);
    Status->FootCenter_L=gsl_vector_calloc (3);

        for(j=0;j<2;j++)
        {fscanf (f, "%s", tmp_s);}


    for(i=1; i<(dof+2); i++)
    {
        for(j=0;j<3;j++)
        {fscanf (f, "%s", tmp_s);}
        fscanf (f, "%s", tmp_s);
        strcpy(uLINK[i].name, tmp_s);
//printf("name: %s \n",tmp_s);

        for(j=0;j<2;j++)
        {fscanf (f, "%s", tmp_s);}
        fscanf (f, "%f", &tmp_f);
        uLINK[i].m  = tmp_f;
//printf("poids: %f \n",tmp_f);

        for(j=0;j<2;j++)
        {fscanf (f, "%s", tmp_s);}
        fscanf (f, "%d", &tmp_i);
        uLINK[i].color  = tmp_i;
//printf("color: %d \n",tmp_i);

        for(j=0;j<2;j++)
        {fscanf (f, "%s", tmp_s);}
        fscanf (f, "%d", &tmp_i);
        uLINK[i].sister  = tmp_i;
//printf("sister: %d \n",tmp_i);

        for(j=0;j<2;j++)
        {fscanf (f, "%s", tmp_s);}
        fscanf (f, "%d", &tmp_i);
        uLINK[i].child  = tmp_i;
//printf("child: %d \n",tmp_i);


        for(j=0;j<2;j++)
        {fscanf (f, "%s", tmp_s);}
        fscanf (f, "%f %f %f ", &tmp_tab3[0], &tmp_tab3[1], &tmp_tab3[2]);
        for(j=0;j<3;j++)
        {gsl_vector_set (uLINK[i].a, j, tmp_tab3[j]);}
//printf("a: %f %f %f \n",tmp_tab3[0],tmp_tab3[1],tmp_tab3[2]);

        for(j=0;j<2;j++)
        {fscanf (f, "%s", tmp_s);}
        fscanf (f, "%f %f %f ", &tmp_tab3[0], &tmp_tab3[1], &tmp_tab3[2]);
        for(j=0;j<3;j++)
        {gsl_vector_set (uLINK[i].b, j, tmp_tab3[j]);}
//printf("b: %f %f %f \n",tmp_tab3[0],tmp_tab3[1],tmp_tab3[2]);

        for(j=0;j<2;j++)
        {fscanf (f, "%s", tmp_s);}
        fscanf (f, "%f %f %f ", &tmp_tab3[0], &tmp_tab3[1], &tmp_tab3[2]);
        for(j=0;j<3;j++)
        {gsl_vector_set (uLINK[i].p, j, tmp_tab3[j]);}
//printf("p: %f %f %f \n",tmp_tab3[0],tmp_tab3[1],tmp_tab3[2]);

        for(j=0;j<2;j++)
        {fscanf (f, "%s", tmp_s);}
        fscanf (f, "%f %f %f ", &tmp_tab3[0], &tmp_tab3[1], &tmp_tab3[2]);
        for(j=0;j<3;j++)
        {gsl_vector_set (uLINK[i].c, j, tmp_tab3[j]);}
//printf("c: %f %f %f \n",tmp_tab3[0],tmp_tab3[1],tmp_tab3[2]);


        for(j=0;j<2;j++)
        {fscanf (f, "%s", tmp_s);}
        fscanf (f, "%f %f %f %f %f %f %f %f %f ", &tmp_tab9[0], &tmp_tab9[1], &tmp_tab9[2], &tmp_tab9[3], &tmp_tab9[4], &tmp_tab9[5], &tmp_tab9[6], &tmp_tab9[7], &tmp_tab9[8]);
        for(j=0;j<3;j++)
        {for(k=0;k<3;k++)
         {gsl_matrix_set(uLINK[i].R, j, k, tmp_tab9[3*j+k]);
         }
        }
//printf("R: \n %f %f %f \n %f %f %f \n %f %f %f \n",tmp_tab9[0],tmp_tab9[1],tmp_tab9[2],tmp_tab9[3],tmp_tab9[4],tmp_tab9[5],tmp_tab9[6],tmp_tab9[7],tmp_tab9[8]);


        for(j=0;j<2;j++)
        {fscanf (f, "%s", tmp_s);}
        fscanf (f, "%f %f %f %f %f %f %f %f %f ", &tmp_tab9[0], &tmp_tab9[1], &tmp_tab9[2], &tmp_tab9[3], &tmp_tab9[4], &tmp_tab9[5], &tmp_tab9[6], &tmp_tab9[7], &tmp_tab9[8]);
        for(j=0;j<3;j++)
        {for(k=0;k<3;k++)
         {gsl_matrix_set(uLINK[i].I, j, k, tmp_tab9[3*j+k]);
         }
        }
//printf("I: \n %f %f %f \n %f %f %f \n %f %f %f \n",tmp_tab9[0],tmp_tab9[1],tmp_tab9[2],tmp_tab9[3],tmp_tab9[4],tmp_tab9[5],tmp_tab9[6],tmp_tab9[7],tmp_tab9[8]);



        for(j=0;j<2;j++)
        {fscanf (f, "%s", tmp_s);}

    }

     int solid, body;

        for(j=0;j<2;j++)
        {fscanf (f, "%s", tmp_s);}
        fscanf (f, "%i", &solid);
//printf("solide: %i \n",solid);
        fscanf (f, "%s", tmp_s);



    for(i=0; i<solid; i++)
    {
        for(j=0;j<3;j++)
        {fscanf (f, "%s", tmp_s);}
        fscanf (f, "%i", &tmp_i);
        body = tmp_i;
//printf("solid: %i \n",tmp_i);

        for(j=0;j<2;j++)
        {fscanf (f, "%s", tmp_s);}
        fscanf (f, "%f", &tmp_f);
        uLINK[body].supportHeight  = tmp_f;
//printf("supportHeight: %f \n",tmp_f);

        for(j=0;j<2;j++)
        {fscanf (f, "%s", tmp_s);}
        fscanf (f, "%f %f %f ", &(uLINK[body].shape[0]), &(uLINK[body].shape[1]), &(uLINK[body].shape[2]));
//printf("shape: %f %f %f \n",uLINK[body].shape[0],uLINK[body].shape[1],uLINK[body].shape[2]);

        for(j=0;j<2;j++)
        {fscanf (f, "%s", tmp_s);}
        fscanf (f, "%f %f %f ", &(uLINK[body].com[0]), &(uLINK[body].com[1]), &(uLINK[body].com[2]));
//printf("com: %f %f %f \n",uLINK[body].com[0],uLINK[body].com[1],uLINK[body].com[2]);

        for(j=0;j<2;j++)
        {fscanf (f, "%s", tmp_s);}

        SetupRigidBody(uLINK,body);

    }


    FindMother(uLINK,1);

    ForwardKinematics(uLINK,1);

    ForwardVelocity(uLINK,1);

    gsl_vector * com = gsl_vector_calloc (3);
    CalcCoM(uLINK,com);
    gsl_vector_memcpy (Status->com_old,com);
    gsl_vector_free(com);




    fclose(f);

}



void LoadRobotXML_f(Struct_uLINK uLINK[],Struct_State *Status,char* RobotFile)
{

    int i,j;

    FILE *f=fopen(RobotFile,"r");
    char tmp_s [100];
    int dof,tmp_i;
    float tmp_f;
    float tmp_tab9[9];
    if (f == NULL) perror ("Error opening robot description file");
    fscanf (f, "%s", tmp_s);
    fscanf (f, "%s", tmp_s);
    fscanf (f, "%i", &dof);
//printf("DoF: %i \n",dof);


    for(i=1; i<(dof+2); i++)
    {
        uLINK[i].q = 0.0;
        uLINK[i].dq = 0.0;
        uLINK[i].ddq = 0.0;
        uLINK[i].u = 0.0;
        uLINK[i].ug = 0.0;
        uLINK[i].uef = 0.0;
        uLINK[i].u_joint = 0.0;
        MatClrf(uLINK[i].a,3,1);
        MatClrf(uLINK[i].b,3,1);
        MatClrf(uLINK[i].c,3,1);
        MatClrf(uLINK[i].p,3,1);
        MatClrf(uLINK[i].hw,3,1);
        MatClrf(uLINK[i].hv,3,1);

    }



    Status->ddl=dof+6;
    Status->support=0; //0:none,1:right,2:left,3:both
    Status->desired_support=0;
    Status->distribution_y=0.5;

    for(j=0;j<6;j++)
    {fscanf (f, "%s", tmp_s);}
    fscanf (f, "%i", &tmp_i);
    Status->right_foot_ID=tmp_i;
//printf("right: %i \n",tmp_i);
    for(j=0;j<2;j++)
    {fscanf (f, "%s", tmp_s);}
    fscanf (f, "%i", &tmp_i);
    Status->left_foot_ID=tmp_i;
//printf("left: %i \n",tmp_i);


        for(j=0;j<2;j++)
        {fscanf (f, "%s", tmp_s);}

    for(i=1; i<(dof+2); i++)
    {
        for(j=0;j<3;j++)
        {fscanf (f, "%s", tmp_s);}
        fscanf (f, "%s", tmp_s);
        strcpy(uLINK[i].name, tmp_s);
//printf("name: %s \n",tmp_s);

        for(j=0;j<2;j++)
        {fscanf (f, "%s", tmp_s);}
        fscanf (f, "%f", &tmp_f);
        uLINK[i].m  = tmp_f;
//printf("poids: %f \n",tmp_f);

        for(j=0;j<2;j++)
        {fscanf (f, "%s", tmp_s);}
        fscanf (f, "%d", &tmp_i);
        //uLINK[i].color  = tmp_i;
//printf("color: %d \n",tmp_i);

        for(j=0;j<2;j++)
        {fscanf (f, "%s", tmp_s);}
        fscanf (f, "%d", &tmp_i);
        uLINK[i].sister  = tmp_i;
//printf("sister: %d \n",tmp_i);

        for(j=0;j<2;j++)
        {fscanf (f, "%s", tmp_s);}
        fscanf (f, "%d", &tmp_i);
        uLINK[i].child  = tmp_i;
//printf("child: %d \n",tmp_i);


        for(j=0;j<2;j++)
        {fscanf (f, "%s", tmp_s);}
        fscanf (f, "%f %f %f ", &uLINK[i].a[0], &uLINK[i].a[1], &uLINK[i].a[2]);
//        for(j=0;j<3;j++)
//        {gsl_vector_set (uLINK[i].a, j, tmp_tab3[j]);}
//printf("a: %f %f %f \n",tmp_tab3[0],tmp_tab3[1],tmp_tab3[2]);

        for(j=0;j<2;j++)
        {fscanf (f, "%s", tmp_s);}
        fscanf (f, "%f %f %f ", &uLINK[i].b[0], &uLINK[i].b[1], &uLINK[i].b[2]);
//        for(j=0;j<3;j++)
//        {gsl_vector_set (uLINK[i].b, j, tmp_tab3[j]);}
//printf("b: %f %f %f \n",tmp_tab3[0],tmp_tab3[1],tmp_tab3[2]);

        for(j=0;j<2;j++)
        {fscanf (f, "%s", tmp_s);}
        fscanf (f, "%f %f %f ", &uLINK[i].p[0], &uLINK[i].p[1], &uLINK[i].p[2]);
//        for(j=0;j<3;j++)
//        {gsl_vector_set (uLINK[i].p, j, tmp_tab3[j]);}
//printf("p: %f %f %f \n",tmp_tab3[0],tmp_tab3[1],tmp_tab3[2]);

        for(j=0;j<2;j++)
        {fscanf (f, "%s", tmp_s);}
        fscanf (f, "%f %f %f ", &uLINK[i].c[0], &uLINK[i].c[1], &uLINK[i].c[2]);
//        for(j=0;j<3;j++)
//        {gsl_vector_set (uLINK[i].c, j, tmp_tab3[j]);}
//printf("c: %f %f %f \n",tmp_tab3[0],tmp_tab3[1],tmp_tab3[2]);


        for(j=0;j<2;j++)
        {fscanf (f, "%s", tmp_s);}
        fscanf (f, "%f %f %f %f %f %f %f %f %f ", &tmp_tab9[0], &tmp_tab9[1], &tmp_tab9[2], &tmp_tab9[3], &tmp_tab9[4], &tmp_tab9[5], &tmp_tab9[6], &tmp_tab9[7], &tmp_tab9[8]);
//        for(j=0;j<3;j++)
//        {for(k=0;k<3;k++)
//         {gsl_matrix_set(uLINK[i].R, j, k, tmp_tab9[3*j+k]);
//         }
//        }
//printf("R: \n %f %f %f \n %f %f %f \n %f %f %f \n",tmp_tab9[0],tmp_tab9[1],tmp_tab9[2],tmp_tab9[3],tmp_tab9[4],tmp_tab9[5],tmp_tab9[6],tmp_tab9[7],tmp_tab9[8]);


        for(j=0;j<2;j++)
        {fscanf (f, "%s", tmp_s);}
        fscanf (f, "%f %f %f %f %f %f %f %f %f ", &tmp_tab9[0], &tmp_tab9[1], &tmp_tab9[2], &tmp_tab9[3], &tmp_tab9[4], &tmp_tab9[5], &tmp_tab9[6], &tmp_tab9[7], &tmp_tab9[8]);
//        for(j=0;j<3;j++)
//        {for(k=0;k<3;k++)
//         {gsl_matrix_set(uLINK[i].I, j, k, tmp_tab9[3*j+k]);
//         }
//        }
//printf("I: \n %f %f %f \n %f %f %f \n %f %f %f \n",tmp_tab9[0],tmp_tab9[1],tmp_tab9[2],tmp_tab9[3],tmp_tab9[4],tmp_tab9[5],tmp_tab9[6],tmp_tab9[7],tmp_tab9[8]);



        for(j=0;j<2;j++)
        {fscanf (f, "%s", tmp_s);}

    }



    FindMother_f(uLINK,1);

    ForwardKinematics_f(uLINK,1);


    fclose(f);

}

































