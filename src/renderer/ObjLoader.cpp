//#include <stdlib.h>
//#include <stdio.h>
//#include <iostream.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "OpenGLHeaders.h"
#include "uLink.h"
#include "Setup.h"
#include "DrawLight.h"

#include "Cross_f.h"
#include "ObjLoader.h"
#include "ping.h"

#define No_opt 0
#define Use_displayList 0//low perf graphic card
#define Use_vertexArrays 0
#define Use_vertexArraysHeavy 0//classic
#define Use_vertexBufferObjects 0
#define Use_vertexBufferObjectsHeavy 1//high perf graphic card

void DrawOBJ(char *fName)
{

    Vertex v[50000];
    Triangle t[50000];
    Normals vn[50000];


    int i;
    int vertexCount = 0;
    int triangleCount = 0;
    int normalsCount = 0;

    double tmp[3];
    double tmp2[3];

    char line[100];

    FILE *fp = fopen(fName,"r");

    if (fp != NULL)
    {
        while (fgets(line, 99, fp))
        {
            if (line[0] == 'v')
            {
                if(line[1]==' ')
                {
                    sscanf(line, "%*c %f %f %f", &v[vertexCount].x, &v[vertexCount].y, &v[vertexCount].z);
                    //printf("%f %f %f\n", v[vertexCount].x, v[vertexCount].y, v[vertexCount].z);
                    vertexCount++;
                }
                if(line[1]=='n')
                {
                    sscanf(line, "%*c %f %f %f", &vn[normalsCount].vn1, &vn[normalsCount].vn2, &vn[normalsCount].vn3);
                    //printf("%f %f %f\n", v[vertexCount].x, v[vertexCount].y, v[vertexCount].z);
                    normalsCount++;
                }
            }
            else if (line[0] == 'f')
            {
                if(normalsCount==0)
                {
                    sscanf(line, "%*c %d %d %d", &t[triangleCount].v1, &t[triangleCount].v2, &t[triangleCount].v3);
                    printf("%d %d %d\n", t[triangleCount].v1, t[triangleCount].v2, t[triangleCount].v3);
                    triangleCount++;
                }
                else
                {
                    sscanf(line, "%*c %d//%*d %d//%*d %d//%*d", &t[triangleCount].v1, &t[triangleCount].v2, &t[triangleCount].v3);
                    printf("%d %d %d\n", t[triangleCount].v1, t[triangleCount].v2, t[triangleCount].v3);
                    triangleCount++;
                }
            }
        }
    }
    else
    {
        printf("Cant load: %s\n",fName);
    }

    fclose(fp);

    glColor3ub(200,200,200);
    glBegin(GL_TRIANGLES);


    for (i=0; i<triangleCount; i++)
    {

        if(normalsCount==0)
        {
            tmp[0]=v[t[i].v2-1].x-v[t[i].v1-1].x;
            tmp[1]=v[t[i].v2-1].y-v[t[i].v1-1].y;
            tmp[2]=v[t[i].v2-1].z-v[t[i].v1-1].z;
            tmp2[0]=v[t[i].v1-1].x-v[t[i].v3-1].x;
            tmp2[1]=v[t[i].v1-1].y-v[t[i].v3-1].y;
            tmp2[2]=v[t[i].v1-1].z-v[t[i].v3-1].z;
            Cross_f( tmp, tmp2, 0);
            glNormal3f(tmp[0], tmp[1], tmp[2]);
            //glNormal3f(vn.vn1, vn.vn2, vn.vn3);
        }
        else
        {
            glNormal3f(vn[i].vn1, vn[i].vn2, vn[i].vn3);
        }

        glVertex3f(v[t[i].v1-1].x , v[t[i].v1-1].y , v[t[i].v1-1].z );
        glVertex3f(v[t[i].v2-1].x , v[t[i].v2-1].y , v[t[i].v2-1].z );
        glVertex3f(v[t[i].v3-1].x , v[t[i].v3-1].y , v[t[i].v3-1].z );
//        glVertex3f(v[t[i].v1-1].x * 0.25, v[t[i].v1-1].y * 0.25, v[t[i].v1-1].z * 0.25);
//        glVertex3f(v[t[i].v2-1].x * 0.25, v[t[i].v2-1].y * 0.25, v[t[i].v2-1].z * 0.25);
//        glVertex3f(v[t[i].v3-1].x * 0.25, v[t[i].v3-1].y * 0.25, v[t[i].v3-1].z * 0.25);
    }

    glEnd();

    //glFlush();
}





void load_obj(char *fName,MeshObj *obj)
{
    int i;
    int obj_size=500000;

    double tmp[3];
    double tmp2[3];

    char line[100];


    obj->v= (Vertex*) calloc(obj_size,sizeof(Vertex));
    obj->t= (Triangle*) calloc(obj_size,sizeof(Triangle));
    obj->vn= (Normals*) calloc(obj_size,sizeof(Normals));

    Normals * vn_temp;
    vn_temp= (Normals*) calloc(obj_size,sizeof(Normals));

    obj->gl_v= (GLfloat *)calloc(3*obj_size,sizeof(GLfloat));
    obj->gl_t= (unsigned int *)calloc(3*obj_size,sizeof(unsigned int));
    obj->gl_vn= (GLfloat *)calloc(3*obj_size,sizeof(GLfloat));


    unsigned int *gl_tn;
    gl_tn= (unsigned int *)calloc(3*obj_size,sizeof(unsigned int));

    GLfloat *gl_vn_count;
    gl_vn_count= (GLfloat *)calloc(obj_size,sizeof(GLfloat));

    obj->vertexCount = 0;
    obj->triangleCount = 0;
    obj->normalsCount = 0;


    FILE *fp = fopen(fName,"r");


    if (fp != NULL)
    {
        while (fgets(line, 99, fp))
        {
            if (line[0] == 'v')
            {
                if(line[1]==' ')
                {
                    sscanf(line, "%*s %f %f %f", &obj->v[obj->vertexCount].x, &obj->v[obj->vertexCount].y, &obj->v[obj->vertexCount].z);
                    //printf("%d %f %f %f\n",obj->vertexCount , obj->v[obj->vertexCount].x, obj->v[obj->vertexCount].y, obj->v[obj->vertexCount].z);
                    obj->vertexCount++;
                }
                if(line[1]=='n')
                {
                    sscanf(line, "%*s %f %f %f", &vn_temp[obj->normalsCount].vn1, &vn_temp[obj->normalsCount].vn2, &vn_temp[obj->normalsCount].vn3);
                    //printf("%f %f %f\n", vn_temp[obj->normalsCount].vn1, vn_temp[obj->normalsCount].vn2, vn_temp[obj->normalsCount].vn3);
                    obj->normalsCount++;
                }
            }
            else if (line[0] == 'f')
            {
                if(obj->normalsCount==0)
                {
                    sscanf(line, "%*s %d %d %d", &obj->t[obj->triangleCount].v2, &obj->t[obj->triangleCount].v1, &obj->t[obj->triangleCount].v3);
                    obj->t[obj->triangleCount].v1-=1;
                    obj->t[obj->triangleCount].v2-=1;
                    obj->t[obj->triangleCount].v3-=1;
                    //printf("%d %d %d %d\n",obj->triangleCount , obj->t[obj->triangleCount].v1, obj->t[obj->triangleCount].v2, obj->t[obj->triangleCount].v3);
                    obj->triangleCount++;
                }
                else
                {
                    sscanf(line, "%*s %d//%d %d//%*d %d//%*d", &obj->t[obj->triangleCount].v1, &gl_tn[obj->triangleCount], &obj->t[obj->triangleCount].v2, &obj->t[obj->triangleCount].v3);
                    obj->t[obj->triangleCount].v1-=1;
                    obj->t[obj->triangleCount].v2-=1;
                    obj->t[obj->triangleCount].v3-=1;
                    gl_tn[obj->triangleCount]-=1;
                    //printf("%d %d %d %d\n",obj->triangleCount , obj->t[obj->triangleCount].v1, obj->t[obj->triangleCount].v2, obj->t[obj->triangleCount].v3);
                    obj->triangleCount++;
                }
            }
        }
    }
    else
    {
        printf("Cant load: %s\n",fName);
    }

    fclose(fp);


    if(obj->normalsCount==0)
    {
        for (i=0; i<obj->triangleCount; i++)
        {
            tmp[0]=obj->v[obj->t[i].v2].x-obj->v[obj->t[i].v1].x;
            tmp[1]=obj->v[obj->t[i].v2].y-obj->v[obj->t[i].v1].y;
            tmp[2]=obj->v[obj->t[i].v2].z-obj->v[obj->t[i].v1].z;
            tmp2[0]=obj->v[obj->t[i].v1].x-obj->v[obj->t[i].v3].x;
            tmp2[1]=obj->v[obj->t[i].v1].y-obj->v[obj->t[i].v3].y;
            tmp2[2]=obj->v[obj->t[i].v1].z-obj->v[obj->t[i].v3].z;
            Cross_f( tmp, tmp2, 0);

            obj->vn[i].vn1=-tmp[0];
            obj->vn[i].vn2=-tmp[1];
            obj->vn[i].vn3=-tmp[2];
            obj->normalsCount++;
            //printf("%d %f %f %f \n",obj->triangleCount,obj->v[obj->t[i].v1].x , obj->v[obj->t[i].v1].y , obj->v[obj->t[i].v1].z);
        }
    }
    else
    {
        for (i=0; i<obj->triangleCount; i++)
        {
            obj->vn[i].vn1=vn_temp[gl_tn[i]].vn1;
            obj->vn[i].vn2=vn_temp[gl_tn[i]].vn2;
            obj->vn[i].vn3=vn_temp[gl_tn[i]].vn3;
        }
        obj->normalsCount=obj->triangleCount;
    }

#if (Use_vertexArrays+Use_vertexBufferObjects)==1
    for (i=0; i<obj->vertexCount; i++)
    {
        obj->gl_v[i*3  ]= obj->v[i].x;
        obj->gl_v[i*3+1]= obj->v[i].y;
        obj->gl_v[i*3+2]= obj->v[i].z;
    }
    for (i=0; i<obj->normalsCount; i++)
    {
//        obj->gl_vn[i*3  ]= obj->vn[i].vn1;
//        obj->gl_vn[i*3+1]= obj->vn[i].vn2;
//        obj->gl_vn[i*3+2]= obj->vn[i].vn3;
        gl_vn_count[obj->t[i].v1]++;
        gl_vn_count[obj->t[i].v2]++;
        gl_vn_count[obj->t[i].v3]++;

        obj->gl_vn[obj->t[i].v1*3  ]= ((gl_vn_count[obj->t[i].v1]-1)*obj->gl_vn[obj->t[i].v1*3  ]+obj->vn[i].vn1)/gl_vn_count[obj->t[i].v1];
        obj->gl_vn[obj->t[i].v1*3+1]= ((gl_vn_count[obj->t[i].v1]-1)*obj->gl_vn[obj->t[i].v1*3+1]+obj->vn[i].vn2)/gl_vn_count[obj->t[i].v1];
        obj->gl_vn[obj->t[i].v1*3+2]= ((gl_vn_count[obj->t[i].v1]-1)*obj->gl_vn[obj->t[i].v1*3+2]+obj->vn[i].vn3)/gl_vn_count[obj->t[i].v1];

        obj->gl_vn[obj->t[i].v2*3  ]= ((gl_vn_count[obj->t[i].v2]-1)*obj->gl_vn[obj->t[i].v2*3  ]+obj->vn[i].vn1)/gl_vn_count[obj->t[i].v2];
        obj->gl_vn[obj->t[i].v2*3+1]= ((gl_vn_count[obj->t[i].v2]-1)*obj->gl_vn[obj->t[i].v2*3+1]+obj->vn[i].vn2)/gl_vn_count[obj->t[i].v2];
        obj->gl_vn[obj->t[i].v2*3+2]= ((gl_vn_count[obj->t[i].v2]-1)*obj->gl_vn[obj->t[i].v2*3+2]+obj->vn[i].vn3)/gl_vn_count[obj->t[i].v2];

        obj->gl_vn[obj->t[i].v3*3  ]= ((gl_vn_count[obj->t[i].v3]-1)*obj->gl_vn[obj->t[i].v3*3  ]+obj->vn[i].vn1)/gl_vn_count[obj->t[i].v3];
        obj->gl_vn[obj->t[i].v3*3+1]= ((gl_vn_count[obj->t[i].v3]-1)*obj->gl_vn[obj->t[i].v3*3+1]+obj->vn[i].vn2)/gl_vn_count[obj->t[i].v3];
        obj->gl_vn[obj->t[i].v3*3+2]= ((gl_vn_count[obj->t[i].v3]-1)*obj->gl_vn[obj->t[i].v3*3+2]+obj->vn[i].vn3)/gl_vn_count[obj->t[i].v3];
//        obj->gl_vn[obj->t[i].v1*3  ]= ((gl_vn_count[obj->t[i].v1]-1)*obj->gl_vn[obj->t[i].v1*3  ]+obj->vn[i].vn1)/gl_vn_count[obj->t[i].v1];
//        obj->gl_vn[obj->t[i].v1*3+1]= obj->vn[i].vn2;
//        obj->gl_vn[obj->t[i].v1*3+2]= obj->vn[i].vn3;
//
//        obj->gl_vn[obj->t[i].v2*3  ]= obj->vn[i].vn1;
//        obj->gl_vn[obj->t[i].v2*3+1]= obj->vn[i].vn2;
//        obj->gl_vn[obj->t[i].v2*3+2]= obj->vn[i].vn3;
//
//        obj->gl_vn[obj->t[i].v3*3  ]= obj->vn[i].vn1;
//        obj->gl_vn[obj->t[i].v3*3+1]= obj->vn[i].vn2;
//        obj->gl_vn[obj->t[i].v3*3+2]= obj->vn[i].vn3;

//        obj->gl_vn[i*3  ]= obj->vn[obj->t[i].v1].vn1;
//        obj->gl_vn[i*3+1]= obj->vn[obj->t[i].v1].vn2;
//        obj->gl_vn[i*3+2]= obj->vn[obj->t[i].v1].vn3;
    }
    for (i=0; i<obj->triangleCount; i++)
    {
        obj->gl_t[i*3  ]= (unsigned int)obj->t[i].v1;
        obj->gl_t[i*3+1]= (unsigned int)obj->t[i].v2;
        obj->gl_t[i*3+2]= (unsigned int)obj->t[i].v3;
    }
#endif
#if (Use_vertexArraysHeavy+Use_vertexBufferObjectsHeavy)==1

    for (i=0; i<obj->triangleCount; i++)
    {
//        obj->gl_t[i*3  ]= (unsigned int)obj->t[i].v1;
//        obj->gl_t[i*3+1]= (unsigned int)obj->t[i].v2;
//        obj->gl_t[i*3+2]= (unsigned int)obj->t[i].v3;

        obj->gl_t[i*3  ]= (unsigned int)i*3;
        obj->gl_t[i*3+1]= (unsigned int)i*3+1;
        obj->gl_t[i*3+2]= (unsigned int)i*3+2;

        obj->gl_v[i*9  ]= obj->v[obj->t[i].v1].x;
        obj->gl_v[i*9+1]= obj->v[obj->t[i].v1].y;
        obj->gl_v[i*9+2]= obj->v[obj->t[i].v1].z;
        obj->gl_v[i*9+3]= obj->v[obj->t[i].v2].x;
        obj->gl_v[i*9+4]= obj->v[obj->t[i].v2].y;
        obj->gl_v[i*9+5]= obj->v[obj->t[i].v2].z;
        obj->gl_v[i*9+6]= obj->v[obj->t[i].v3].x;
        obj->gl_v[i*9+7]= obj->v[obj->t[i].v3].y;
        obj->gl_v[i*9+8]= obj->v[obj->t[i].v3].z;

        obj->gl_vn[i*9  ]= obj->vn[i].vn1;
        obj->gl_vn[i*9+1]= obj->vn[i].vn2;
        obj->gl_vn[i*9+2]= obj->vn[i].vn3;
        obj->gl_vn[i*9+3]= obj->vn[i].vn1;
        obj->gl_vn[i*9+4]= obj->vn[i].vn2;
        obj->gl_vn[i*9+5]= obj->vn[i].vn3;
        obj->gl_vn[i*9+6]= obj->vn[i].vn1;
        obj->gl_vn[i*9+7]= obj->vn[i].vn2;
        obj->gl_vn[i*9+8]= obj->vn[i].vn3;

    }
#endif

    obj->firstTime = 1;
    obj->has_buffers = 0;
    obj->vao = 0;
    obj->vbo = 0;
    obj->vbo_normal = 0;
    obj->ebo = 0;
    obj->index_count = 0;
}


void load_mtl(char *fName)
{

}


void buffer_model(MeshObj *obj)
{
    if (obj->has_buffers) return;

    glGenVertexArrays(1, &obj->vao);
    glGenBuffers(1, &obj->vbo);
    glGenBuffers(1, &obj->vbo_normal);
    glGenBuffers(1, &obj->ebo);

    glBindVertexArray(obj->vao);

    // Position VBO
    glBindBuffer(GL_ARRAY_BUFFER, obj->vbo);
    glBufferData(GL_ARRAY_BUFFER, 9 * obj->triangleCount * sizeof(float), obj->gl_v, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Normal VBO
    glBindBuffer(GL_ARRAY_BUFFER, obj->vbo_normal);
    glBufferData(GL_ARRAY_BUFFER, 9 * obj->triangleCount * sizeof(float), obj->gl_vn, GL_STATIC_DRAW);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);

    // EBO
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, obj->ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, 3 * obj->triangleCount * sizeof(unsigned int), obj->gl_t, GL_STATIC_DRAW);

    glBindVertexArray(0);

    obj->index_count = 3 * obj->triangleCount;
    obj->has_buffers = 1;
}

extern Shader defaultShader;
extern glm::mat4 ProjectionMatrix;
extern glm::mat4 ViewMatrix;
extern glm::vec3 CameraPosition;

void draw_model(MeshObj *obj, const glm::mat4 &Model)
{
    if (!obj->has_buffers) return;

    defaultShader.use();

    // Set matrices
    glm::mat4 MVP = ProjectionMatrix * ViewMatrix * Model;
    glm::mat3 NormalMatrix = glm::transpose(glm::inverse(glm::mat3(Model)));

    defaultShader.setMat4("MVP", MVP);
    defaultShader.setMat4("Model", Model);
    defaultShader.setMat3("NormalMatrix", NormalMatrix);

    // Set lighting & camera uniforms
    defaultShader.setVec3("view_Pos", CameraPosition);
    defaultShader.setVec3("light_Pos", glm::vec3(2.0f, -2.0f, 2.0f)); // Light position
    defaultShader.setVec3("light_Color", glm::vec3(1.0f, 1.0f, 1.0f));
    defaultShader.setVec3("ambient_Color", glm::vec3(1.0f, 1.0f, 1.0f));

    // Set default materials or flat shadow color
    if (shadowPassActive) {
        defaultShader.setBool("use_Flat_Color", true);
        defaultShader.setVec3("base_Color", glm::vec3(0.1f, 0.1f, 0.1f));
    } else {
        defaultShader.setBool("use_Flat_Color", false);
        defaultShader.setVec3("material_Ambient", glm::vec3(0.19225f, 0.19225f, 0.19225f));
        defaultShader.setVec3("material_Diffuse", glm::vec3(0.50754f, 0.50754f, 0.50754f));
        defaultShader.setVec3("material_Specular", glm::vec3(0.508273f, 0.508273f, 0.508273f));
        defaultShader.setFloat("material_Shininess", 51.2f);
        defaultShader.setBool("use_Base_Color", false);
    }

    glBindVertexArray(obj->vao);
    glDrawElements(GL_TRIANGLES, obj->index_count, GL_UNSIGNED_INT, (void*)0);
    glBindVertexArray(0);
}

void setMaterialsAndTex(Material mats, GLuint tex)
{

}


