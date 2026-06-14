
#include "OpenGLHeaders.h"
#include <SDL2/SDL.h>

#include "uLink.h"
#include "DrawCamera.h"


void CamInit(CamParam_s *CamParam)
{
    CamParam->holdl = 0; // initially assume the button is not pressed
    CamParam->holdw = 0; // initially assume the button is not pressed
    CamParam->angleY = 23;
    CamParam->angleZ = 94;
    CamParam->distance = 0.1; // initial distance of the camera from the scene center
    CamParam->motionSensivity = 0.3;
    CamParam->scrollSensivity = 0.1;
    CamParam->strafeSensivity = 0.002;
    CamParam->X = -0.8;
    CamParam->Y = -0.32;
    CamParam->Z = -0.23;

}

//void CamInit(CamParam_s *CamParam)
//{
//    CamParam->holdl = 0; // initially assume the button is not pressed
//    CamParam->holdw = 0; // initially assume the button is not pressed
//    CamParam->angleY = 30;
//    CamParam->angleZ = 45;
//    CamParam->distance = 1.3; // initial distance of the camera from the scene center
//    CamParam->motionSensivity = 0.3;
//    CamParam->scrollSensivity = 0.1;
//    CamParam->strafeSensivity = 0.002;
//    CamParam->X = 0.0;
//    CamParam->Y = 0.0;
//    CamParam->Z = 0.0;
//
//}

void OnMouseMotion(CamParam_s *CamParam, SDL_MouseMotionEvent event)
{
    if (CamParam->holdl) // if we are holding the left button down
    {
        CamParam->angleZ += event.xrel*CamParam->motionSensivity; // mouse motion on X -> change horizontal rotation
        CamParam->angleY += event.yrel*CamParam->motionSensivity; // mouse motion on Y -> change vertical rotation
        // to avoid some issues, we limit the vertical rotation to angles between -90 and 90 degrees
        if (CamParam->angleY > 90)
            CamParam->angleY = 90;
        else if (CamParam->angleY < -90)
            CamParam->angleY = -90;
    }
    if (CamParam->holdw) // if we are holding the middle button down
    {
        CamParam->Y += event.xrel*CamParam->strafeSensivity; // mouse motion on X -> change horizontal translation
        CamParam->Z -= event.yrel*CamParam->strafeSensivity; // mouse motion on Y -> change vertical translation
        // to avoid some issues, we limit vertical rotation to angles between -90 and 90 degrees
    }
}

void OnMouseButton(CamParam_s *CamParam, SDL_MouseButtonEvent event)
{
    if (event.button == SDL_BUTTON_LEFT) // event concerns the left button
    {
        if ((CamParam->holdl)&&(event.type == SDL_MOUSEBUTTONUP)) // release while pressed
        {
            CamParam->holdl = 0; // mouse movement will no longer move the scene
            //SDL_SetCursor(_hand1); // set normal cursor
        }
        else if ((!CamParam->holdl)&&(event.type == SDL_MOUSEBUTTONDOWN)) // press while released
        {
            CamParam->holdl = 1; // mouse movement will move the scene
            //SDL_SetCursor(_hand2); // set special cursor
        }
    }
    else if ((event.button == SDL_BUTTON_WHEELUP)&&(event.type == SDL_MOUSEBUTTONDOWN)) // scroll wheel up
    {
        CamParam->distance -= CamParam->scrollSensivity; // zoom in, bring camera closer to center
        if (CamParam->distance < 0.1) // minimum distance, can be changed if needed (e.g. with an attribute)
            CamParam->distance = 0.1;
    }
    else if ((event.button == SDL_BUTTON_WHEELDOWN)&&(event.type == SDL_MOUSEBUTTONDOWN)) // scroll wheel down
    {
        CamParam->distance += CamParam->scrollSensivity; // zoom out, move camera away
    }
    else if (event.button == SDL_BUTTON_MIDDLE) // event concerns the middle button/wheel
    {
        if ((CamParam->holdw)&&(event.type == SDL_MOUSEBUTTONUP)) // release while pressed
        {
            CamParam->holdw = 0; // mouse movement will no longer move the scene
            //SDL_SetCursor(_hand1); // set normal cursor
        }
        else if ((!CamParam->holdw)&&(event.type == SDL_MOUSEBUTTONDOWN)) // press while released
        {
            CamParam->holdw = 1; // mouse movement will move the scene
            //SDL_SetCursor(_hand2); // set special cursor
        }
    }
}

glm::mat4 ProjectionMatrix(1.0f);
glm::mat4 ViewMatrix(1.0f);
glm::vec3 CameraPosition(0.0f);

extern double angular_z;

void Camlook(State *Status, CamParam_s *CamParam)
{
    // Projection matrix (70 degrees FOV, aspect ratio 1024/768, near 0.001, far 1000)
    ProjectionMatrix = glm::perspective(glm::radians(70.0f), 1024.0f / 768.0f, 0.001f, 1000.0f);

    // Initial View Matrix from gluLookAt
    glm::vec3 eye(CamParam->distance, 0.0f, 0.0f);
    glm::vec3 center(0.0f, 0.0f, 0.0f);
    glm::vec3 up(0.0f, 0.0f, 1.0f);
    glm::mat4 V = glm::lookAt(eye, center, up);

    // Combine with translation and rotations
    glm::mat4 T = glm::translate(glm::mat4(1.0f), glm::vec3(CamParam->X, CamParam->Y, CamParam->Z));
    glm::mat4 Ry = glm::rotate(glm::mat4(1.0f), glm::radians((float)CamParam->angleY), glm::vec3(0.0f, 1.0f, 0.0f));
    glm::mat4 Rz = glm::rotate(glm::mat4(1.0f), glm::radians((float)CamParam->angleZ) + (float)angular_z, glm::vec3(0.0f, 0.0f, 1.0f));

    ViewMatrix = V * T * Ry * Rz;

    // Camera position in world space is the translation part of inv(ViewMatrix)
    CameraPosition = glm::vec3(glm::inverse(ViewMatrix) * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));
}








