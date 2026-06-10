/* Mat.h  Module Matrice. Opérateurs de calcul matriciel et vectoriel.

11:20 22/10/10 MB

Les matrices sont passées en argument par un pointeur de Reel (ou tableau de Reel) et
 des int donnant leurs dimensions.

Si l'appelant utilise des tableaux bidimensionnels, lors de l'appel des fonctions de ce module,
il lui faudra faire des 'cast' (Reel *).

Pour éviter ces 'cast' il suffit de déclarer les matrices comme des tableaux à une dimension :

Reel mat[6 * 6] ,
     matrice[] =
     {
         6.83 , -3.07 ,
         5.90 , -0.44
     }
*/


#include "Reel.h" // définition du type Reel : double ou double


//----------------------------------------------------------
void MatTrans( Reel *_trans , const Reel *_mat , int _m , int _n ) ;
void MatTransf( double *_trans , const double *_mat , int _m , int _n ) ;
/*
Fonction : transpose une matrice.

Entrées : _mat matrice à transposer, de _m lignes et _n colonnes (m et n >= 0)

Sortie  : _trans matrice transposée (de _n lignes et _m colonnes)

NB : dans le cas d'un vecteur un MatCpy serait plus rapide (_m ou _n = 1)
     mais en fait on ne fait jamais de transposition d'un vecteur.
*/


//----------------------------------------------------------
void MatMul( Reel *_resul , const Reel *_mat1 , const Reel *_mat2 , int _l , int _m , int _n ) ;
void MatMulf( double *_resul , const double *_mat1 , const double *_mat2 , int _l , int _m , int _n ) ;
/*
Fonction : multiplie deux matrices.

Sortie   : _resul(l,n) = _mat1(l,m) * _mat2(m,n) ; avec l, m et n >= 0
           _resul peut être dans _mat1 ou _mat2 si l * n <= 12 * 12
*/


//----------------------------------------------------------
void MatAdd( Reel *_resul , const Reel *_mat1 , const Reel *_mat2 , int _m , int _n ) ;
void MatAddf( double *_resul , const double *_mat1 , const double *_mat2 , int _m , int _n ) ;
/*
Fonction : additionne deux matrices.

Sortie   : _resul(m,n) = _mat1(m,n) + _mat2(m,n) ; _resul peut être dans _mat1 ou _mat2
           avec _m lignes , _n colonnes (m et n >= 0)
*/


//----------------------------------------------------------
void MatSub( Reel *_resul , const Reel *_mat1 , const Reel *_mat2 , int _m , int _n ) ;
void MatSubf( double *_resul , const double *_mat1 , const double *_mat2 , int _m , int _n ) ;
/*
Fonction : soustraie deux matrices.

Sortie   : _resul(m,n) = _mat1(m,n) - _mat2(m,n) ; _resul peut être dans _mat1 ou _mat2
           avec _m lignes , _n colonnes (m et n >= 0)
*/


//----------------------------------------------------------
void MatMulScal( Reel *_resul , Reel _scal , const Reel *_mat , int _m , int _n ) ;
void MatMulScalf( double *_resul , double _scal , const double *_mat , int _m , int _n ) ;
/*
Fonction : multiplie une matrice par un scalaire.

Sortie   : _resul(m,n) = _scal * _mat(m,n) ; _resul peut être dans _mat
           avec _m lignes , _n colonnes (m et n >= 0)
*/


//----------------------------------------------------------
void MatCpy( Reel *_resul , const Reel *_mat , int _m , int _n ) ;
void MatCpyf( double *_resul , const double *_mat , int _m , int _n ) ;
/*
Fonction : copie une matrice.

Sortie   : _resul(m,n) = _mat(m,n) ;
           avec _m lignes , _n colonnes (m et n >= 0)
*/


//----------------------------------------------------------
void MatClr( Reel *_mat , int _m , int _n ) ;
void MatClrf( double *_mat , int _m , int _n ) ;
/*
Fonction :  Clear,initialise une matrice avec des 0

           avec _m lignes , _n colonnes (m et n >= 0)
*/


//----------------------------------------------------------
void MatI( Reel *_mat , int _n ) ;
void MatIf( double *_mat , int _n ) ;
/*
Fonction : initialise une matrice unitée de dimension n

           (n >= 0)
*/


//----------------------------------------------------------
void MatSet( Reel *_mat , int _m , int _n , Reel _scal ) ;
void MatSetf( double *_mat , int _m , int _n , double _scal ) ;
/*
Fonction : initialise une matrice avec un scalaire.

           avec _m lignes , _n colonnes (m et n >= 0)
*/

//----------------------------------------------------------
void MatPrint( const char *_titre , const char *_format , const Reel *_mat , int _m , int _n ) ;
void MatPrintf( const char *_titre , const char *_format , const double *_mat , int _m , int _n ) ;
/*
Fonction : affiche une matrice avec un format f.

Entrées  : _titre  ligne de texte précédant l'affichage de _mat
                   MatPrint ajoute 2 sauts de ligne ('\n\n') avant le _titre
           _format d'affichage de chaque élément de _mat

           avec _m lignes , _n colonnes (m et n >= 0)
*/


//----------------------------------------------------------
int  MatPseudoInv( Reel *_inv , const Reel *_mat , int _m , int _n ) ;
int  MatPseudoInvf( double *_inv , const double *_mat , int _m , int _n ) ;
/*
Fonction : calcule la pseudo inverse d'une matrice.

Sortie   : _inv(n,m) = _mat(m,n)* ;

           avec _m lignes , _n colonnes (18 >= m et n >= 0)

Retour   : 0 si Ok

Méthode : algorithme de Greville
*/


//----------------------------------------------------------
void MatTilde( Reel *_resul , const Reel *_vect ) ;
void MatTildef( double *_resul , const double *_vect ) ;
/*
Fonction : à partir d'un vecteur u de dimension 3 calcule une matrice 3x3 définie par

         [    0  - uz    uy ]
_resul = [   uz     0  - ux ]
         [ - uy    ux     0 ]
*/

//----------------------------------------------------------
void MatProVectoriel( Reel *_resul , const Reel *_vect1 , const Reel *_vect2 ) ;
void MatProVectorielf( double *_resul , const double *_vect1 , const double *_vect2 ) ;
/*
Fonction : produit vectoriel de 2 vecteurs de dimension 3.

Sortie   : _resul(3) = _vect1(3) ^ _vect2(3) ;
*/


//----------------------------------------------------------
Reel MatProScal( const Reel *_vect1 , const Reel *_vect2 , int _n ) ;
double MatProScalf( const double *_vect1 , const double *_vect2 , int _n ) ;
/*
Fonction : produit scalaire de 2 vecteurs de dimension _n.

Retour   : produit scalaire _vect1(n) * _vect2(n)

           avec _n >=0  (_n = 0 -> retour = 0 , _n = 1 -> retour = produit algébrique)
*/


//----------------------------------------------------------
Reel MatNorm( const Reel *_vect , int _n ) ;
double MatNormf( const double *_vect , int _n ) ;
/*
Fonction : norme d'un vecteur de dimension _n.

Retour   : norme de _vect(n)

           avec _n = 0 -> retour = 0  (_n = 1 -> retour = le nombre)
*/


//----------------------------------------------------------
//Reel MatInvNorm( const Reel *_vect , int _n ) ;
/*
Fonction : inverse de la norme d'un vecteurs de dimension _n.

Retour   : inverse de la norme de _vect(n)

           avec _n > 0  (_n = 1 -> retour = inverse d'un nombre)
*/


//----------------------------------------------------------
int MatTest() ;
/*
Fonction : teste toutes les fonctions du module.
*/
