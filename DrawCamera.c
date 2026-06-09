
#include <GL/gl.h>
#include <GL/glu.h>
#include <SDL/SDL.h>

#include "uLink.h"
#include "DrawCamera.h"


void CamInit(CamParam_s *CamParam)
{
    CamParam->holdl = 0; //au départ on part du principe que le bouton n'est pas maintenu
    CamParam->holdw = 0; //au départ on part du principe que le bouton n'est pas maintenu
    CamParam->angleY = 23;
    CamParam->angleZ = 94;
    CamParam->distance = 0.1; //distance initiale de la caméra avec le centre de la scène
    CamParam->motionSensivity = 0.3;
    CamParam->scrollSensivity = 0.1;
    CamParam->strafeSensivity = 0.002;
    CamParam->X = -0.8;
    CamParam->Y = -0.32;
    CamParam->Z = -0.23;

}

//void CamInit(CamParam_s *CamParam)
//{
//    CamParam->holdl = 0; //au départ on part du principe que le bouton n'est pas maintenu
//    CamParam->holdw = 0; //au départ on part du principe que le bouton n'est pas maintenu
//    CamParam->angleY = 30;
//    CamParam->angleZ = 45;
//    CamParam->distance = 1.3; //distance initiale de la caméra avec le centre de la scène
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
    if (CamParam->holdl) //si nous maintenons le bouton gauche enfoncé
    {
        CamParam->angleZ += event.xrel*CamParam->motionSensivity; //mouvement sur X de la souris -> changement de la rotation horizontale
        CamParam->angleY += event.yrel*CamParam->motionSensivity; //mouvement sur Y de la souris -> changement de la rotation verticale
        //pour éviter certains problèmes, on limite la rotation verticale à des angles entre -90° et 90°
        if (CamParam->angleY > 90)
            CamParam->angleY = 90;
        else if (CamParam->angleY < -90)
            CamParam->angleY = -90;
    }
    if (CamParam->holdw) //si nous maintenons le bouton millieu enfoncé
    {
        CamParam->Y += event.xrel*CamParam->strafeSensivity; //mouvement sur X de la souris -> changement de la rotation horizontale
        CamParam->Z -= event.yrel*CamParam->strafeSensivity; //mouvement sur Y de la souris -> changement de la rotation verticale
        //pour éviter certains problèmes, on limite la rotation verticale à des angles entre -90° et 90°
    }
}

void OnMouseButton(CamParam_s *CamParam, SDL_MouseButtonEvent event)
{
    if (event.button == SDL_BUTTON_LEFT) //l'événement concerne le bouton gauche
    {
        if ((CamParam->holdl)&&(event.type == SDL_MOUSEBUTTONUP)) //relâchement alors qu'on était enfoncé
        {
            CamParam->holdl = 0; //falsele mouvement de la souris ne fera plus bouger la scène
            //SDL_SetCursor(_hand1); //on met le curseur normal
        }
        else if ((!CamParam->holdl)&&(event.type == SDL_MOUSEBUTTONDOWN)) //appui alors qu'on était relâché
        {
            CamParam->holdl = 1; //true le mouvement de la souris fera bouger la scène
            //SDL_SetCursor(_hand2); //on met le curseur spécial
        }
    }
    else if ((event.button == SDL_BUTTON_WHEELUP)&&(event.type == SDL_MOUSEBUTTONDOWN)) //coup de molette vers le haut
    {
        CamParam->distance -= CamParam->scrollSensivity; //on zoome, donc rapproche la caméra du centre
        if (CamParam->distance < 0.1) //distance minimale, à changer si besoin (avec un attribut par exemple)
            CamParam->distance = 0.1;
    }
    else if ((event.button == SDL_BUTTON_WHEELDOWN)&&(event.type == SDL_MOUSEBUTTONDOWN)) //coup de molette vers le bas
    {
        CamParam->distance += CamParam->scrollSensivity; //on dézoome donc éloigne la caméra
    }
    else if (event.button == SDL_BUTTON_MIDDLE) //l'événement concerne le molette
    {
        if ((CamParam->holdw)&&(event.type == SDL_MOUSEBUTTONUP)) //relâchement alors qu'on était enfoncé
        {
            CamParam->holdw = 0; //falsele mouvement de la souris ne fera plus bouger la scène
            //SDL_SetCursor(_hand1); //on met le curseur normal
        }
        else if ((!CamParam->holdw)&&(event.type == SDL_MOUSEBUTTONDOWN)) //appui alors qu'on était relâché
        {
            CamParam->holdw = 1; //true le mouvement de la souris fera bouger la scène
            //SDL_SetCursor(_hand2); //on met le curseur spécial
        }
    }
}

void Camlook(State *Status, CamParam_s *CamParam)
{
    gluLookAt(CamParam->distance, 0, 0, 0, 0, 0, 0, 0, 1);
//    gluLookAt(CamParam->distance,0,0,
//              0,0,0,
//              0,0,1); // la caméra regarde le centre (0,0,0) et est sur l'axe X à une certaine distance du centre donc (_distance,0,0)
    glTranslated(CamParam->X, CamParam->Y, CamParam->Z);
    glRotated(CamParam->angleY,0,1,0); //la scène est tournée autour de l'axe Y
    glRotated(CamParam->angleZ,0,0,1); //la scène est tournée autour de l'axe Z

//printf("cam: %f %f %f %f %f %f \n",CamParam->angleY,CamParam->angleZ,CamParam->distance,CamParam->X,CamParam->Y,CamParam->Z);

}








