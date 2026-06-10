#include "DrawGround.h"
#include "OpenGLHeaders.h"
#include "Shader.h"

static unsigned int groundVAO = 0;
static unsigned int groundVBO = 0;
static int ground_initialized = 0;

extern Shader defaultShader;
extern glm::mat4 ProjectionMatrix;
extern glm::mat4 ViewMatrix;

void DrawGround(double x, double y, double z, double dx, double dy, double dz)
{
    if (!ground_initialized)
    {
        // Define vertices for a flat quad representing the floor
        // Positions (x, y, z) and Normals (nx, ny, nz)
        float vertices[] = {
            // position          // normals
            -15.0f, -15.0f, 0.0f,  0.0f, 0.0f, 1.0f,
             15.0f, -15.0f, 0.0f,  0.0f, 0.0f, 1.0f,
             15.0f,  15.0f, 0.0f,  0.0f, 0.0f, 1.0f,

            -15.0f, -15.0f, 0.0f,  0.0f, 0.0f, 1.0f,
             15.0f,  15.0f, 0.0f,  0.0f, 0.0f, 1.0f,
            -15.0f,  15.0f, 0.0f,  0.0f, 0.0f, 1.0f
        };

        glGenVertexArrays(1, &groundVAO);
        glGenBuffers(1, &groundVBO);

        glBindVertexArray(groundVAO);
        glBindBuffer(GL_ARRAY_BUFFER, groundVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

        // Position attribute
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        // Normal attribute
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);

        glBindVertexArray(0);
        ground_initialized = 1;
    }

    defaultShader.use();

    // Model matrix is identity translated to (x, y, z)
    glm::mat4 Model = glm::translate(glm::mat4(1.0f), glm::vec3((float)x, (float)y, (float)z));
    glm::mat4 MVP = ProjectionMatrix * ViewMatrix * Model;
    glm::mat3 NormalMatrix = glm::transpose(glm::inverse(glm::mat3(Model)));

    defaultShader.setMat4("MVP", MVP);
    defaultShader.setMat4("Model", Model);
    defaultShader.setMat3("NormalMatrix", NormalMatrix);

    // Set ground color (modern neutral light gray)
    defaultShader.setBool("use_Base_Color", true);
    defaultShader.setVec3("base_Color", glm::vec3(0.9f, 0.9f, 0.9f));

    glBindVertexArray(groundVAO);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);
}
