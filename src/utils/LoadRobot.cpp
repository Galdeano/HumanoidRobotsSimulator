#include <Eigen/Dense>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <cmath>

#include "uLink.h"
#include "SetupRigidBody.h"
#include "FindMother.h"
#include "ForwardKinematics.h"
#include "ForwardVelocity.h"
#include "CalcCoM.h"
#include "Setup.h"
#include "LoadRobot.h"
#include "PrintGSLMatrix.h"

#include <pugixml.hpp>

void LoadRobotXML(SuLINK uLINK[], State *Status, char* RobotFile)
{
    int i, j, k;

    FILE *f = fopen(RobotFile, "r");
    char tmp_s[100];
    int dof, tmp_i;
    float tmp_f;
    float tmp_tab3[3], tmp_tab9[9];
    if (f == NULL) {
        perror("Error opening robot description file");
        return;
    }
    
    if (fscanf(f, "%s", tmp_s) != 1) return;
    if (fscanf(f, "%s", tmp_s) != 1) return;
    if (fscanf(f, "%i", &dof) != 1) return;

    for (i = 1; i < (dof + 2); i++)
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
        uLINK[i].isPolygon = 0;
        uLINK[i].a.setZero();
        uLINK[i].b.setZero();
        uLINK[i].p.setZero();
        uLINK[i].c.setZero();
        uLINK[i].v.setZero();
        uLINK[i].vo.setZero();
        uLINK[i].w.setZero();
        uLINK[i].dvo.setZero();
        uLINK[i].dw.setZero();
        uLINK[i].hw.setZero();
        uLINK[i].hv.setZero();
        uLINK[i].R.setIdentity();
        uLINK[i].I.setZero();
    }

    Status->dof = dof + 6;
    Status->support = 0; // 0:none,1:right,2:left,3:both
    Status->desired_support = 0;
    Status->distribution_y = 0.5;

    for (j = 0; j < 6; j++)
    {
        if (fscanf(f, "%s", tmp_s) != 1) return;
    }
    if (fscanf(f, "%i", &tmp_i) != 1) return;
    Status->right_foot_ID = tmp_i;

    for (j = 0; j < 2; j++)
    {
        if (fscanf(f, "%s", tmp_s) != 1) return;
    }
    if (fscanf(f, "%i", &tmp_i) != 1) return;
    Status->left_foot_ID = tmp_i;
    Status->integral_R = 0;
    Status->integral_L = 0;
    Status->com_old.setZero();
    Status->forCoP_R.setZero();
    Status->posCoP_R.setZero();
    Status->forCoP_L.setZero();
    Status->posCoP_L.setZero();
    Status->FootCenter_R.setZero();
    Status->FootCenter_L.setZero();

    for (j = 0; j < 2; j++)
    {
        if (fscanf(f, "%s", tmp_s) != 1) return;
    }

    for (i = 1; i < (dof + 2); i++)
    {
        for (j = 0; j < 3; j++)
        {
            if (fscanf(f, "%s", tmp_s) != 1) return;
        }
        if (fscanf(f, "%s", tmp_s) != 1) return;
        strcpy(uLINK[i].name, tmp_s);

        for (j = 0; j < 2; j++)
        {
            if (fscanf(f, "%s", tmp_s) != 1) return;
        }
        if (fscanf(f, "%f", &tmp_f) != 1) return;
        uLINK[i].m = tmp_f;

        for (j = 0; j < 2; j++)
        {
            if (fscanf(f, "%s", tmp_s) != 1) return;
        }
        if (fscanf(f, "%d", &tmp_i) != 1) return;
        uLINK[i].color = tmp_i;

        for (j = 0; j < 2; j++)
        {
            if (fscanf(f, "%s", tmp_s) != 1) return;
        }
        if (fscanf(f, "%d", &tmp_i) != 1) return;
        uLINK[i].sister = tmp_i;

        for (j = 0; j < 2; j++)
        {
            if (fscanf(f, "%s", tmp_s) != 1) return;
        }
        if (fscanf(f, "%d", &tmp_i) != 1) return;
        uLINK[i].child = tmp_i;

        for (j = 0; j < 2; j++)
        {
            if (fscanf(f, "%s", tmp_s) != 1) return;
        }
        if (fscanf(f, "%f %f %f ", &tmp_tab3[0], &tmp_tab3[1], &tmp_tab3[2]) != 3) return;
        uLINK[i].a << tmp_tab3[0], tmp_tab3[1], tmp_tab3[2];

        for (j = 0; j < 2; j++)
        {
            if (fscanf(f, "%s", tmp_s) != 1) return;
        }
        if (fscanf(f, "%f %f %f ", &tmp_tab3[0], &tmp_tab3[1], &tmp_tab3[2]) != 3) return;
        uLINK[i].b << tmp_tab3[0], tmp_tab3[1], tmp_tab3[2];

        for (j = 0; j < 2; j++)
        {
            if (fscanf(f, "%s", tmp_s) != 1) return;
        }
        if (fscanf(f, "%f %f %f ", &tmp_tab3[0], &tmp_tab3[1], &tmp_tab3[2]) != 3) return;
        uLINK[i].p << tmp_tab3[0], tmp_tab3[1], tmp_tab3[2];

        for (j = 0; j < 2; j++)
        {
            if (fscanf(f, "%s", tmp_s) != 1) return;
        }
        if (fscanf(f, "%f %f %f ", &tmp_tab3[0], &tmp_tab3[1], &tmp_tab3[2]) != 3) return;
        uLINK[i].c << tmp_tab3[0], tmp_tab3[1], tmp_tab3[2];

        for (j = 0; j < 2; j++)
        {
            if (fscanf(f, "%s", tmp_s) != 1) return;
        }
        if (fscanf(f, "%f %f %f %f %f %f %f %f %f ", &tmp_tab9[0], &tmp_tab9[1], &tmp_tab9[2], &tmp_tab9[3], &tmp_tab9[4], &tmp_tab9[5], &tmp_tab9[6], &tmp_tab9[7], &tmp_tab9[8]) != 9) return;
        uLINK[i].R << tmp_tab9[0], tmp_tab9[3], tmp_tab9[6],
                      tmp_tab9[1], tmp_tab9[4], tmp_tab9[7],
                      tmp_tab9[2], tmp_tab9[5], tmp_tab9[8];

        for (j = 0; j < 2; j++)
        {
            if (fscanf(f, "%s", tmp_s) != 1) return;
        }
        if (fscanf(f, "%f %f %f %f %f %f %f %f %f ", &tmp_tab9[0], &tmp_tab9[1], &tmp_tab9[2], &tmp_tab9[3], &tmp_tab9[4], &tmp_tab9[5], &tmp_tab9[6], &tmp_tab9[7], &tmp_tab9[8]) != 9) return;
        uLINK[i].I << tmp_tab9[0], tmp_tab9[3], tmp_tab9[6],
                      tmp_tab9[1], tmp_tab9[4], tmp_tab9[7],
                      tmp_tab9[2], tmp_tab9[5], tmp_tab9[8];

        for (j = 0; j < 2; j++)
        {
            if (fscanf(f, "%s", tmp_s) != 1) return;
        }
    }

    int solid, body;
    for (j = 0; j < 2; j++)
    {
        if (fscanf(f, "%s", tmp_s) != 1) return;
    }
    if (fscanf(f, "%i", &solid) != 1) return;
    if (fscanf(f, "%s", tmp_s) != 1) return;

    for (i = 0; i < solid; i++)
    {
        for (j = 0; j < 3; j++)
        {
            if (fscanf(f, "%s", tmp_s) != 1) return;
        }
        if (fscanf(f, "%i", &tmp_i) != 1) return;
        body = tmp_i;

        for (j = 0; j < 2; j++)
        {
            if (fscanf(f, "%s", tmp_s) != 1) return;
        }
        if (fscanf(f, "%f", &tmp_f) != 1) return;
        uLINK[body].supportHeight = tmp_f;

        for (j = 0; j < 2; j++)
        {
            if (fscanf(f, "%s", tmp_s) != 1) return;
        }
        if (fscanf(f, "%lf %lf %lf ", &(uLINK[body].shape[0]), &(uLINK[body].shape[1]), &(uLINK[body].shape[2])) != 3) return;

        for (j = 0; j < 2; j++)
        {
            if (fscanf(f, "%s", tmp_s) != 1) return;
        }
        if (fscanf(f, "%lf %lf %lf ", &(uLINK[body].com[0]), &(uLINK[body].com[1]), &(uLINK[body].com[2])) != 3) return;

        for (j = 0; j < 2; j++)
        {
            if (fscanf(f, "%s", tmp_s) != 1) return;
        }

        SetupRigidBody(uLINK, body);
    }

    FindMother(uLINK, Status, 1);
    ForwardKinematics(uLINK, 1);
    ForwardVelocity(uLINK, 1);

    Eigen::Vector3d com;
    CalcCoM(uLINK, com);
    Status->com_old = com;

    fclose(f);
}

static void trim_copy(char* dest, const char* src, size_t dest_size) {
    if (dest_size == 0) return;
    while (*src && isspace((unsigned char)*src)) {
        src++;
    }
    size_t len = strlen(src);
    while (len > 0 && isspace((unsigned char)src[len - 1])) {
        len--;
    }
    size_t copy_len = len < dest_size - 1 ? len : dest_size - 1;
    strncpy(dest, src, copy_len);
    dest[copy_len] = '\0';
}

void LoadRobotParserXML(SuLINK uLINK[], State *Status, char* RobotFile)
{
    int i;
    int dof, numlink, body;
    double DegToRad = M_PI / 180.0;

    pugi::xml_document doc;
    if (!doc.load_file(RobotFile))
    {
        perror("Error opening robot description file");
        return;
    }
    pugi::xml_node robot = doc.child("Robot");

    dof = robot.child("DoF").text().as_int();

    for (i = 1; i < (dof + 2); i++)
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
        uLINK[i].umin = -300;
        uLINK[i].umax = 300;
        uLINK[i].isPolygon = 0;
        uLINK[i].qmin = -3.14;
        uLINK[i].qmax = 3.14;
        uLINK[i].qmoy = 0;
        uLINK[i].a.setZero();
        uLINK[i].b.setZero();
        uLINK[i].p.setZero();
        uLINK[i].c.setZero();
        uLINK[i].v.setZero();
        uLINK[i].vo.setZero();
        uLINK[i].w.setZero();
        uLINK[i].dvo.setZero();
        uLINK[i].dw.setZero();
        uLINK[i].hw.setZero();
        uLINK[i].hv.setZero();
        uLINK[i].R.setIdentity();
        uLINK[i].I.setZero();
    }

    Status->dof = dof + 6;
    Status->support = 0; // 0:none,1:right,2:left,3:both
    Status->desired_support = 0;
    Status->distribution_y = 0.5;
    Status->integral_R = 0;
    Status->integral_L = 0;
    Status->com_old.setZero();
    Status->forCoP_R.setZero();
    Status->posCoP_R.setZero();
    Status->forCoP_L.setZero();
    Status->posCoP_L.setZero();
    Status->FootCenter_R.setZero();
    Status->FootCenter_L.setZero();

    pugi::xml_node StateNode = robot.child("State");

    Status->right_foot_ID = StateNode.child("right_foot_ID").text().as_int();
    Status->left_foot_ID = StateNode.child("left_foot_ID").text().as_int();

    for (pugi::xml_node Link : robot.children("Link"))
    {
        numlink = Link.attribute("no").as_int();
        trim_copy(uLINK[numlink].name, Link.child("Name").text().as_string(), sizeof(uLINK[numlink].name));

        uLINK[numlink].m = Link.child("m").text().as_double();
        if (Link.child("fixed"))
        {
            uLINK[numlink].fixed = Link.child("fixed").text().as_int();
        }
        uLINK[numlink].color = Link.child("color").text().as_int();
        uLINK[numlink].sister = Link.child("sister").text().as_int();
        uLINK[numlink].child = Link.child("child").text().as_int();
        
        if (Link.child("qmin"))
        {
            uLINK[numlink].qmin = Link.child("qmin").text().as_double();
            uLINK[numlink].qmin *= DegToRad;
        }
        else
        {
            uLINK[numlink].qmin = -M_PI;
        }
        
        if (Link.child("qmax"))
        {
            uLINK[numlink].qmax = Link.child("qmax").text().as_double();
            uLINK[numlink].qmax *= DegToRad;
        }
        else
        {
            uLINK[numlink].qmax = M_PI;
        }
        
        if (Link.child("qmoy"))
        {
            uLINK[numlink].qmoy = Link.child("qmoy").text().as_double();
            uLINK[numlink].qmoy *= DegToRad;
            uLINK[numlink].q = uLINK[numlink].qmoy;
        }
        else
        {
            uLINK[numlink].qmoy = 0;
        }
        
        if (Link.child("umin"))
        {
            uLINK[numlink].umin = Link.child("umin").text().as_double();
        }
        else
        {
            uLINK[numlink].umin = -100;
        }
        
        if (Link.child("umax"))
        {
            uLINK[numlink].umax = Link.child("umax").text().as_double();
        }
        else
        {
            uLINK[numlink].umax = 100;
        }
        
        double ax, ay, az;
        sscanf(Link.child("a").text().as_string(), "%lf %lf %lf", &ax, &ay, &az);
        uLINK[numlink].a << ax, ay, az;
        
        double bx, by, bz;
        sscanf(Link.child("b").text().as_string(), "%lf %lf %lf", &bx, &by, &bz);
        uLINK[numlink].b << bx, by, bz;
        
        double px, py, pz;
        sscanf(Link.child("p").text().as_string(), "%lf %lf %lf", &px, &py, &pz);
        uLINK[numlink].p << px, py, pz;
        
        double cx, cy, cz;
        sscanf(Link.child("c").text().as_string(), "%lf %lf %lf", &cx, &cy, &cz);
        uLINK[numlink].c << cx, cy, cz;

        double r[9];
        sscanf(Link.child("R").text().as_string(), "%lf %lf %lf %lf %lf %lf %lf %lf %lf",
                &r[0], &r[1], &r[2],
                &r[3], &r[4], &r[5],
                &r[6], &r[7], &r[8]);
        uLINK[numlink].R << r[0], r[3], r[6],
                            r[1], r[4], r[7],
                            r[2], r[5], r[8];

        double inertia[9];
        sscanf(Link.child("I").text().as_string(), "%lf %lf %lf %lf %lf %lf %lf %lf %lf",
                &inertia[0], &inertia[1], &inertia[2],
                &inertia[3], &inertia[4], &inertia[5],
                &inertia[6], &inertia[7], &inertia[8]);
        uLINK[numlink].I << inertia[0], inertia[3], inertia[6],
                            inertia[1], inertia[4], inertia[7],
                            inertia[2], inertia[5], inertia[8];

#if LoadObj
        if (Link.child("obj"))
        {
            trim_copy(uLINK[numlink].obj, Link.child("obj").text().as_string(), sizeof(uLINK[numlink].obj));
            load_obj(uLINK[numlink].obj, &(uLINK[numlink].Mesh_obj));
        }
        uLINK[numlink].obj_offset.setZero();
        if (Link.child("obj_offset"))
        {
            double ox, oy, oz;
            sscanf(Link.child("obj_offset").text().as_string(), "%lf %lf %lf", &ox, &oy, &oz);
            uLINK[numlink].obj_offset << ox, oy, oz;
        }
#endif
    }

    pugi::xml_node Contacts = robot.child("Contacts");

    for (pugi::xml_node Contact : Contacts.children("Contact"))
    {
        body = Contact.child("body").text().as_int();
        uLINK[body].supportHeight = Contact.child("supportHeight").text().as_double();
        sscanf(Contact.child("shape").text().as_string(), "%lf %lf %lf", &(uLINK[body].shape[0]), &(uLINK[body].shape[1]), &(uLINK[body].shape[2]));
        sscanf(Contact.child("com").text().as_string(), "%lf %lf %lf", &(uLINK[body].com[0]), &(uLINK[body].com[1]), &(uLINK[body].com[2]));

        SetupRigidBody(uLINK, body);
    }

    Status->nb_limb = 0;
    Status->limbID = Eigen::VectorXd::Zero(5);
    FindMother(uLINK, Status, 1);
    ForwardKinematics(uLINK, 1);
    ForwardVelocity(uLINK, 1);

    Eigen::Vector3d com;
    CalcCoM(uLINK, com);
    Status->com_old = com;
}
