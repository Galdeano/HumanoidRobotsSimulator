#ifndef H_FindMother_f
#define H_FindMother_f

/** \file FindMother_f.h
 *  \brief Recursive mother ID search for control loop
 *  \author    David Galdeano
 *  \date      10/2011
 *
 */


/**
 * \fn void FindMother_f(Struct_uLINK uLINK[],int j)
 * \brief Recursively find the ID ok the mother link for control loop
 *
 * Must be used as follow:
 * \code FindMother(uLINK,1); \endcode
 *
 *
 * \param uLINK[] Structure wich describe the robot link by link in control loop
 * \param j ID of the link
 */
void FindMother_f(Struct_uLINK uLINK[],int j);


#endif
