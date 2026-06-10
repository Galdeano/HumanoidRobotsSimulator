#include "DrawGround.h"
#include "OpenGLHeaders.h"
#include "Shader.h"
#include "DrawLight.h"

static unsigned int groundVAO = 0;
static unsigned int groundVBO = 0;
static unsigned int groundEBO = 0;
static int ground_initialized = 0;

extern Shader defaultShader;
extern glm::mat4 ProjectionMatrix;
extern glm::mat4 ViewMatrix;

void DrawGround(double x, double y, double z, double dx, double dy, double dz)
{
    if (!ground_initialized)
    {
        // Define vertices for a unit cube centered at (0, 0, 0)
        // Positions (x, y, z) and Normals (nx, ny, nz)
        float vertices[] = {
            // Bottom face (z = -0.5, normal = 0, 0, -1)
            -0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
             0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
             0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
            -0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,

            // Top face (z = 0.5, normal = 0, 0, 1)
            -0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
             0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
             0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
            -0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,

            // Front face (y = -0.5, normal = 0, -1, 0)
            -0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,
             0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,
             0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,
            -0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,

            // Back face (y = 0.5, normal = 0, 1, 0)
            -0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,
             0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,
             0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
            -0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,

            // Left face (x = -0.5, normal = -1, 0, 0)
            -0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,
            -0.5f,  0.5f, -0.5f, -1.0f,  0.0f,  0.0f,
            -0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,
            -0.5f, -0.5f,  0.5f, -1.0f,  0.0f,  0.0f,

            // Right face (x = 0.5, normal = 1, 0, 0)
             0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,
             0.5f,  0.5f, -0.5f,  1.0f,  0.0f,  0.0f,
             0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,
             0.5f, -0.5f,  0.5f,  1.0f,  0.0f,  0.0f
        };

        unsigned int indices[] = {
            0, 1, 2,   2, 3, 0,       // Bottom
            4, 5, 6,   6, 7, 4,       // Top
            8, 9, 10,  10, 11, 8,     // Front
            12, 13, 14, 14, 15, 12,   // Back
            16, 17, 18, 18, 19, 16,   // Left
            20, 21, 22, 22, 23, 20    // Right
        };

        glGenVertexArrays(1, &groundVAO);
        glGenBuffers(1, &groundVBO);
        glGenBuffers(1, &groundEBO);

        glBindVertexArray(groundVAO);
        glBindBuffer(GL_ARRAY_BUFFER, groundVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, groundEBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

        // Position attribute
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        // Normal attribute
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);

        glBindVertexArray(0);
        ground_initialized = 1;
    }

    // Model matrix translated to (x, y, z) and scaled by (dx, dy, dz)
    glm::mat4 Model = glm::translate(glm::mat4(1.0f), glm::vec3((float)x, (float)y, (float)z));
    Model = glm::scale(Model, glm::vec3((float)dx, (float)dy, (float)dz));
    glm::mat4 MVP = ProjectionMatrix * ViewMatrix * Model;
    glm::mat3 NormalMatrix = glm::transpose(glm::inverse(glm::mat3(Model)));

    defaultShader.use();
    defaultShader.setMat4("MVP", MVP);
    defaultShader.setMat4("Model", Model);
    defaultShader.setMat3("NormalMatrix", NormalMatrix);

    // Set ground material (using the legacy "test" material)
    extern materialStruct test;
    set_material(&test);

    glBindVertexArray(groundVAO);
    glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, (void*)0);
    glBindVertexArray(0);

    // Draw outlines for the 12 edges (equivalent to line loops around faces)
    extern materialStruct black_rubber;
    glm::vec3 outlineColor(black_rubber.Kd[0], black_rubber.Kd[1], black_rubber.Kd[2]);

    float x_min = (float)(x - dx/2);
    float x_max = (float)(x + dx/2);
    float y_min = (float)(y - dy/2);
    float y_max = (float)(y + dy/2);
    float z_min = (float)(z - dz/2);
    float z_max = (float)(z + dz/2);

    glm::vec3 c0(x_min, y_min, z_min);
    glm::vec3 c1(x_max, y_min, z_min);
    glm::vec3 c2(x_max, y_max, z_min);
    glm::vec3 c3(x_min, y_max, z_min);
    glm::vec3 c4(x_min, y_min, z_max);
    glm::vec3 c5(x_max, y_min, z_max);
    glm::vec3 c6(x_max, y_max, z_max);
    glm::vec3 c7(x_min, y_max, z_max);

    // Bottom loop
    draw_line(c0, c1, outlineColor, 2.0f);
    draw_line(c1, c2, outlineColor, 2.0f);
    draw_line(c2, c3, outlineColor, 2.0f);
    draw_line(c3, c0, outlineColor, 2.0f);

    // Top loop
    draw_line(c4, c5, outlineColor, 2.0f);
    draw_line(c5, c6, outlineColor, 2.0f);
    draw_line(c6, c7, outlineColor, 2.0f);
    draw_line(c7, c4, outlineColor, 2.0f);

    // Verticals
    draw_line(c0, c4, outlineColor, 2.0f);
    draw_line(c1, c5, outlineColor, 2.0f);
    draw_line(c2, c6, outlineColor, 2.0f);
    draw_line(c3, c7, outlineColor, 2.0f);
}
