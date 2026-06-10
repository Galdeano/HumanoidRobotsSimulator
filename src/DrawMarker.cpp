#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_math.h>
#include "DrawMarker.h"
#include "OpenGLHeaders.h"
#include "Shader.h"

extern Shader defaultShader;
extern glm::mat4 ProjectionMatrix;
extern glm::mat4 ViewMatrix;

void draw_line(const glm::vec3 &start, const glm::vec3 &end, const glm::vec3 &color, float width) {
    static unsigned int lineVAO = 0;
    static unsigned int lineVBO = 0;
    if (lineVAO == 0) {
        glGenVertexArrays(1, &lineVAO);
        glGenBuffers(1, &lineVBO);
    }
    
    float vertices[] = {
        start.x, start.y, start.z,
        end.x, end.y, end.z
    };
    
    glBindVertexArray(lineVAO);
    glBindBuffer(GL_ARRAY_BUFFER, lineVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);
    
    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    
    defaultShader.use();
    glm::mat4 Model(1.0f);
    glm::mat4 MVP = ProjectionMatrix * ViewMatrix * Model;
    
    defaultShader.setMat4("MVP", MVP);
    defaultShader.setMat4("Model", Model);
    defaultShader.setBool("use_Flat_Color", true);
    defaultShader.setVec3("base_Color", color);
    
    glLineWidth(width);
    glDrawArrays(GL_LINES, 0, 2);
    glBindVertexArray(0);
}

void DrawMarker(gsl_vector * com)
{
    double d = 0.05;
    glm::vec3 center(
        (float)gsl_vector_get(com, 0),
        (float)gsl_vector_get(com, 1),
        (float)gsl_vector_get(com, 2)
    );
    draw_line(center - glm::vec3(d, 0.0f, 0.0f), center + glm::vec3(d, 0.0f, 0.0f), activeColor, 5.0f);
    draw_line(center - glm::vec3(0.0f, d, 0.0f), center + glm::vec3(0.0f, d, 0.0f), activeColor, 5.0f);
    draw_line(center - glm::vec3(0.0f, 0.0f, d), center + glm::vec3(0.0f, 0.0f, d), activeColor, 5.0f);
}
