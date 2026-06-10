#ifndef H_DrawCamera
#define H_DrawCamera

#include <SDL2/SDL.h>

#ifndef SDL_BUTTON_WHEELUP
#define SDL_BUTTON_WHEELUP 4
#endif
#ifndef SDL_BUTTON_WHEELDOWN
#define SDL_BUTTON_WHEELDOWN 5
#endif

typedef struct
{
    double motionSensivity;
    double scrollSensivity;
    double strafeSensivity;
    char holdl;
    char holdw;
    double distance;
    double angleY;
    double angleZ;
    double X;
    double Y;
    double Z;
} CamParam_s ;

void CamInit(CamParam_s *CamParam);

void OnMouseMotion(CamParam_s *CamParam, SDL_MouseMotionEvent event);

void OnMouseButton(CamParam_s *CamParam, SDL_MouseButtonEvent event);

void Camlook(State *Status, CamParam_s *CamParam);

#endif
