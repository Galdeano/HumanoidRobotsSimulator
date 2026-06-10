#include "DrawMarkerf.h"
#include "OpenGLHeaders.h"

void DrawMarkerf(double *com)
{
    double d = 0.1;
    glm::vec3 center((float)com[0], (float)com[1], (float)com[2]);
    draw_line(center - glm::vec3(d, 0.0f, 0.0f), center + glm::vec3(d, 0.0f, 0.0f), activeColor, 1.0f);
    draw_line(center - glm::vec3(0.0f, d, 0.0f), center + glm::vec3(0.0f, d, 0.0f), activeColor, 1.0f);
    draw_line(center - glm::vec3(0.0f, 0.0f, d), center + glm::vec3(0.0f, 0.0f, d), activeColor, 1.0f);
}
