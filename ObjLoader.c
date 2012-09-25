//#include <stdlib.h>
//#include <stdio.h>
//#include <iostream.h>

#include <stdio.h>
#include <string.h>

#include <GL/gl.h>
#include <GL/glu.h>

#include "Cross_f.h"
#include "ObjLoader.h"


void DrawOBJ(char *fName)
{

    Vertex v[50000];
    Triangle t[50000];
    Normals vn[50000];

    int i;
    int vertexCount = 0;
    int triangleCount = 0;
    int NormalsCount = 0;

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
            }
            else if (line[0] == 'f')
            {
                sscanf(line, "%*c %d %d %d", &t[triangleCount].v1, &t[triangleCount].v2, &t[triangleCount].v3);
                //printf("%d %d %d\n", t[triangleCount].v1, t[triangleCount].v2, t[triangleCount].v3);
                triangleCount++;
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

     if(NormalsCount==0)
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
