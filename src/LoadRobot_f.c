
#include <stdio.h>
#include <string.h>


#include "uLINK_f.h"
#include "Mat.h"
#include "FindMother_f.h"
#include "ForwardKinematics_f.h"
#include "LoadRobot_f.h"

#include "./ezxml/ezxml.h"



void LoadRobotParserXML_f(Struct_uLINK uLINK[],Struct_State *Status,char* RobotFile)
{

    int i;
    int dof,numlink;

    ezxml_t robot = ezxml_parse_file(RobotFile), State, Link;

    sscanf (ezxml_child(robot, "DoF")->txt,"%d",&dof);
    //printf("dof: %d \n", dof);

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

    State = ezxml_child(robot, "State");
    int temp_right_foot_ID, temp_left_foot_ID;
    sscanf (ezxml_child(State, "right_foot_ID")->txt,"%d",&temp_right_foot_ID);
    Status->right_foot_ID = (unsigned char)temp_right_foot_ID;
    sscanf (ezxml_child(State, "left_foot_ID")->txt,"%d",&temp_left_foot_ID);
    Status->left_foot_ID = (unsigned char)temp_left_foot_ID;

    for (Link = ezxml_child(robot, "Link"); Link; Link = Link->next)
    {
        numlink = atoi(ezxml_attr(Link, "no"));
        sscanf (ezxml_child(Link, "Name")->txt,"%s", &uLINK[numlink].name);
        sscanf (ezxml_child(Link, "m")->txt,"%lf", &uLINK[numlink].m);
        int temp_sister, temp_child;
        sscanf (ezxml_child(Link, "sister")->txt,"%d", &temp_sister);
        uLINK[numlink].sister = (unsigned char)temp_sister;
        sscanf (ezxml_child(Link, "child")->txt,"%d", &temp_child);
        uLINK[numlink].child = (unsigned char)temp_child;
        sscanf (ezxml_child(Link, "a")->txt,"%lf %lf %lf", &uLINK[numlink].a[0], &uLINK[numlink].a[1], &uLINK[numlink].a[2]);
        sscanf (ezxml_child(Link, "b")->txt,"%lf %lf %lf", &uLINK[numlink].b[0], &uLINK[numlink].b[1], &uLINK[numlink].b[2]);
        sscanf (ezxml_child(Link, "p")->txt,"%lf %lf %lf", &uLINK[numlink].p[0], &uLINK[numlink].p[1], &uLINK[numlink].p[2]);
        sscanf (ezxml_child(Link, "c")->txt,"%lf %lf %lf", &uLINK[numlink].c[0], &uLINK[numlink].c[1], &uLINK[numlink].c[2]);
    }

    FindMother_f(uLINK,1);

    ForwardKinematics_f(uLINK,1);
}




