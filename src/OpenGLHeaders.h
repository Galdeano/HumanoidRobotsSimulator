#ifndef OPENGL_HEADERS_H
#define OPENGL_HEADERS_H

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

extern glm::vec3 activeColor;
extern bool shadowPassActive;
void draw_line(const glm::vec3 &start, const glm::vec3 &end, const glm::vec3 &color, float width = 1.0f);

inline void glColor3ub(unsigned char r, unsigned char g, unsigned char b) {
    activeColor = glm::vec3(r / 255.0f, g / 255.0f, b / 255.0f);
}

inline void glColor3f(float r, float g, float b) {
    activeColor = glm::vec3(r, g, b);
}

#endif // OPENGL_HEADERS_H
