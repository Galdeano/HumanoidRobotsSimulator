#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_math.h>
#include <vector>
#include "uLink.h"
#include "DrawCylinder.h"
#include "Setup.h"
#include "OpenGLHeaders.h"
#include "Shader.h"

static unsigned int cylVAO = 0;
static unsigned int cylVBO = 0;
static unsigned int cylEBO = 0;
static int cyl_index_count = 0;
static int cyl_initialized = 0;

extern Shader defaultShader;
extern glm::mat4 ProjectionMatrix;
extern glm::mat4 ViewMatrix;

static void init_cylinder() {
    const int faces = 12;
    std::vector<float> vertices;
    std::vector<unsigned int> indices;

    for (int i = 0; i <= faces; i++) {
        float angle = (float)i * 2.0f * M_PI / faces;
        float c = cos(angle);
        float s = sin(angle);

        // bottom ring
        vertices.push_back(c); // pos x
        vertices.push_back(s); // pos y
        vertices.push_back(-0.5f); // pos z
        vertices.push_back(c); // normal x
        vertices.push_back(s); // normal y
        vertices.push_back(0.0f); // normal z

        // top ring
        vertices.push_back(c); // pos x
        vertices.push_back(s); // pos y
        vertices.push_back(0.5f); // pos z
        vertices.push_back(c); // normal x
        vertices.push_back(s); // normal y
        vertices.push_back(0.0f); // normal z
    }

    for (int i = 0; i < faces; i++) {
        int b0 = i * 2;
        int t0 = i * 2 + 1;
        int b1 = (i + 1) * 2;
        int t1 = (i + 1) * 2 + 1;

        indices.push_back(b0);
        indices.push_back(t0);
        indices.push_back(b1);

        indices.push_back(b1);
        indices.push_back(t0);
        indices.push_back(t1);
    }

    glGenVertexArrays(1, &cylVAO);
    glGenBuffers(1, &cylVBO);
    glGenBuffers(1, &cylEBO);

    glBindVertexArray(cylVAO);
    glBindBuffer(GL_ARRAY_BUFFER, cylVBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), &vertices[0], GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cylEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);

    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Normal attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);

    cyl_index_count = indices.size();
    cyl_initialized = 1;
}

void DrawCylinder(SuLINK uLINK[], int j) {
    if (!cyl_initialized) {
        init_cylinder();
    }

    double radius = 0.015;
    double len = 0.045;

    glm::mat4 Mj(1.0f);
    for (int i = 0; i < 3; i++) {
        for (int k = 0; k < 3; k++) {
            Mj[i][k] = (float)gsl_matrix_get(uLINK[j].R, k, i);
        }
        Mj[3][i] = (float)gsl_vector_get(uLINK[j].p, i);
    }

    glm::mat4 R(1.0f);
    if (gsl_vector_get(uLINK[j].a, 0) == 1)
        R = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    else if (gsl_vector_get(uLINK[j].a, 1) == 1)
        R = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f));

    glm::mat4 S = glm::scale(glm::mat4(1.0f), glm::vec3((float)radius, (float)radius, (float)len));

    extern float g_shadowMatrix[16];
    glm::mat4 Model = Mj * R * S;
    if (shadowPassActive) {
        glm::mat4 shadowMatrix = glm::make_mat4(g_shadowMatrix);
        Model = shadowMatrix * Model;
    }
    glm::mat4 MVP = ProjectionMatrix * ViewMatrix * Model;
    glm::mat3 NormalMatrix = glm::transpose(glm::inverse(glm::mat3(Model)));

    defaultShader.use();
    defaultShader.setMat4("MVP", MVP);
    defaultShader.setMat4("Model", Model);
    defaultShader.setMat3("NormalMatrix", NormalMatrix);

    if (shadowPassActive) {
        defaultShader.setBool("use_Flat_Color", true);
        defaultShader.setVec3("base_Color", glm::vec3(0.1f, 0.1f, 0.1f));
    } else {
        defaultShader.setBool("use_Flat_Color", false);
        defaultShader.setBool("use_Base_Color", true);
        defaultShader.setVec3("base_Color", glm::vec3(0.5f, 0.5f, 0.5f));
    }

    glBindVertexArray(cylVAO);
    glDrawElements(GL_TRIANGLES, cyl_index_count, GL_UNSIGNED_INT, (void*)0);
    glBindVertexArray(0);
}
