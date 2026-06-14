#include <Eigen/Dense>
#include "DrawForceMarker.h"
#include "OpenGLHeaders.h"

extern glm::vec3 activeColor;
void draw_line(const glm::vec3 &start, const glm::vec3 &end, const glm::vec3 &color, float width);

void DrawForceMarker(const Eigen::Vector3d & ori, const Eigen::Vector3d & vec)
{
    glm::vec3 start(
        (float)ori(0),
        (float)ori(1),
        (float)ori(2)
    );
    glm::vec3 end = start + glm::vec3(
        (float)vec(0),
        (float)vec(1),
        (float)vec(2)
    );
    draw_line(start, end, activeColor, 1.0f);
}
