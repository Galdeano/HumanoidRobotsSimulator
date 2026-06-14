#ifndef H_DrawPolygon
#define H_DrawPolygon

/** \file DrawPolygon.h
 *  \brief Draw polygon faces, vectices and local contact forces
 *  \author    David Galdeano
 *  \date      10/2011
 *
 */


/**
 * \fn void DrawPolygon(SuLINK uLINK[],int j)
 * \brief Draw polygon faces, vectices and local contact forces
 *
 *
 * \param uLINK[] Structure wich describe the robot link by link
 * \param j ID of the link
 */
void DrawPolygon(SuLINK uLINK[],int j);

#endif
