#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_math.h>
#include "DrawForceMarker.h"
#include "OpenGLHeaders.h"

void DrawForceMarker(gsl_vector * ori, gsl_vector * vec)
{
    glm::vec3 start(
        (float)gsl_vector_get(ori, 0),
        (float)gsl_vector_get(ori, 1),
        (float)gsl_vector_get(ori, 2)
    );
    glm::vec3 end = start + glm::vec3(
        (float)gsl_vector_get(vec, 0),
        (float)gsl_vector_get(vec, 1),
        (float)gsl_vector_get(vec, 2)
    );
    draw_line(start, end, activeColor, 1.0f);
}
