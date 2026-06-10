
#include <stdio.h>
#include <string.h>


#include "uLINK_f.h"
#include "Mat.h"
#include "FindMother_f.h"
#include "ForwardKinematics_f.h"
#include "LoadRobot_f.h"

#include <pugixml.hpp>



void LoadRobotParserXML_f(Struct_uLINK uLINK[],Struct_State *Status,char* RobotFile)
{
    int i;
    int dof,numlink;

    pugi::xml_document doc;
    if (!doc.load_file(RobotFile))
    {
        perror("Error opening robot description file");
        return;
    }
    pugi::xml_node robot = doc.child("Robot");

    dof = robot.child("DoF").text().as_int();

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

    pugi::xml_node State = robot.child("State");
    int temp_right_foot_ID = State.child("right_foot_ID").text().as_int();
    Status->right_foot_ID = (unsigned char)temp_right_foot_ID;
    int temp_left_foot_ID = State.child("left_foot_ID").text().as_int();
    Status->left_foot_ID = (unsigned char)temp_left_foot_ID;

    for (pugi::xml_node Link : robot.children("Link"))
    {
        numlink = Link.attribute("no").as_int();
        strcpy(uLINK[numlink].name, Link.child("Name").text().as_string());
        uLINK[numlink].m = Link.child("m").text().as_double();
        int temp_sister = Link.child("sister").text().as_int();
        uLINK[numlink].sister = (unsigned char)temp_sister;
        int temp_child = Link.child("child").text().as_int();
        uLINK[numlink].child = (unsigned char)temp_child;
        sscanf (Link.child("a").text().as_string(), "%lf %lf %lf", &uLINK[numlink].a[0], &uLINK[numlink].a[1], &uLINK[numlink].a[2]);
        sscanf (Link.child("b").text().as_string(), "%lf %lf %lf", &uLINK[numlink].b[0], &uLINK[numlink].b[1], &uLINK[numlink].b[2]);
        sscanf (Link.child("p").text().as_string(), "%lf %lf %lf", &uLINK[numlink].p[0], &uLINK[numlink].p[1], &uLINK[numlink].p[2]);
        sscanf (Link.child("c").text().as_string(), "%lf %lf %lf", &uLINK[numlink].c[0], &uLINK[numlink].c[1], &uLINK[numlink].c[2]);
    }

    FindMother_f(uLINK,1);

    ForwardKinematics_f(uLINK,1);
}




