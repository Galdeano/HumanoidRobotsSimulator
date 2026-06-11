#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_math.h>
#include "uLink.h"
#include "DrawAllJoints.h"

#include "OpenGLHeaders.h"
#include "DrawPolygon.h"
#include "DrawCylinder.h"
#include "Connect3D.h"
#include "Setup.h"
#include "ObjLoader.h"

void DrawAllJoints(SuLINK uLINK[],int j)
{

    if (j != 0)
    {
#if LoadObj
        glm::mat4 Mj(1.0f);
        for (int i = 0; i < 3; i++) {
            for (int k = 0; k < 3; k++) {
                Mj[i][k] = (float)gsl_matrix_get(uLINK[j].R, k, i);
            }
            Mj[3][i] = (float)gsl_vector_get(uLINK[j].p, i);
        }

        glm::vec3 offset(
            (float)gsl_vector_get(uLINK[j].obj_offset, 0),
            (float)gsl_vector_get(uLINK[j].obj_offset, 1),
            (float)gsl_vector_get(uLINK[j].obj_offset, 2)
        );
        glm::mat4 Model = glm::translate(Mj, offset);

        if (shadowPassActive) {
            extern float g_shadowMatrix[16];
            glm::mat4 shadowMatrix = glm::make_mat4(g_shadowMatrix);
            Model = shadowMatrix * Model;
        }

        draw_model(&(uLINK[j].Mesh_obj), Model);
#else

        if (uLINK[j].isPolygon==1)
        {
            DrawPolygon(uLINK,j);
        }

        if (uLINK[j].mother != 0)
        {
            Connect3D(uLINK,j);
        }


        if (j != 1)
        {
            DrawCylinder(uLINK,j);
        }

#endif

        DrawAllJoints(uLINK,uLINK[j].sister);
        DrawAllJoints(uLINK,uLINK[j].child);
    }

}
