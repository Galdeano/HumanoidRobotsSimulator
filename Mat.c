/* Mat.c  Module Matrice. Opérateurs de calcul matriciel.

16:39 18/10/07 MB V

Dans ce module on utilise une règle du C ANSI :
 les matrices sont mémorisées par rangées (ligne par ligne)
c'est le dernier indice qui varie le plus vite; (K&R p 219)

Un vecteur ligne est mémorisé de la même façon qu'un vecteur colonne,
autrement dit transposer un vecteur en mêmoire ne le modifie pas.
*/


#include "Mat.h"

#include <stdio.h>

#include <math.h> // pour sqrt en .cpp

int MatTest()
{
    {
        // Test de MatCpy 0 ou 1
        Reel mat[] =
        {
            6.83737f , -3.07323f ,
            5.90153f , -0.449802f
        } ,
        mat2[2 * 2] ;
//------------------
        MatClr( mat2 , 2 , 2 ) ;
        MatCpy( mat2 , mat , 1 , 1 ) ;
        MatPrint( "Copie matrice 1,1" , " %6.2f" , mat2 , 2 , 2 ) ;

        MatClr( mat2 , 2 , 2 ) ;
        MatCpy( mat2 , mat , 1 , 0 ) ;
        MatPrint( "Copie matrice 1,0" , " %6.2f" , mat2 , 2 , 2 ) ;
    } // Test de MatCpy 0 ou 1


    {
        // Test inversion pour BJ
#define Ordre2 2

        Reel mat[] =
        {
            6.83737f , -3.07323f , // ex BJ qui ne marcherai pas ! 19/09/04
            5.90153f , -0.449802f
        } ,
        matInv[Ordre2 * Ordre2] ,
        matUni[Ordre2 * Ordre2] ;
//------------------
        MatPseudoInv( matInv , mat , Ordre2 , Ordre2 ) ;
        MatPrint( "M(-1) Matrice inverse" , " %9.6f" , matInv , Ordre2 , Ordre2 ) ;

        MatMul( matUni , matInv , mat , Ordre2 , Ordre2 , Ordre2 ) ;
        MatPrint( "I Matrice unitaire = M(-1) * M" , " %6.2f" , matUni , Ordre2 , Ordre2 ) ;

        MatMul( matUni , mat , matInv , Ordre2 , Ordre2 , Ordre2 ) ;
        MatPrint( "I Matrice unitaire = M * M(-1)" , " %6.2f" , matUni , Ordre2 , Ordre2 ) ;
    } // Test inversion pour BJ


    {
        // Test inversion [1,1]
#define Ordre1 1

        Reel mat[] =
        {
            2.
        } ,
        matInv[Ordre1 * Ordre1] ,
        matUni[Ordre1 * Ordre1] ;
//------------------
        MatPseudoInv( matInv , mat , Ordre1 , Ordre1 ) ;
        MatPrint( "M(-1) Matrice inverse" , " %9.6f" , matInv , Ordre1 , Ordre1 ) ;

        MatMul( matUni , matInv , mat , Ordre1 , Ordre1 , Ordre1 ) ;
        MatPrint( "I Matrice unitaire = M(-1) * M" , " %6.2f" , matUni , Ordre1 , Ordre1 ) ;

        MatMul( matUni , mat , matInv , Ordre1 , Ordre1 , Ordre1 ) ;
        MatPrint( "I Matrice unitaire = M * M(-1)" , " %6.2f" , matUni , Ordre1 , Ordre1 ) ;
    } // Test inversion [1,1]


    {
        // Test inversion pour jeu GB
        /*
        Problème aboutissant à un système de 7 équations à 7 inconnues (6 équations indépendantes)

        On à 7 clés à 7 positions codées de [-3 à +3]. Vecteur colonne Position vectP

              6 1
             5 0 2
              4 3

        A partir d'une position initiale on veut mettre toutes les clés à 0. Vecteur colonne Résultat vectR
        Mais quand on tourne une clé les autres tournent aussi selon la formule:
        mat77 * vectC + vectP = vectR (Vecteur colonne Commande vectC)
        On cherche vectC pour avoir vectR = [0]
        */
#define Ordre6 6
        Reel mat77[] = // matrice symètrique complète mais 6 équations indépendantes, on en tire la 6,6 qui suit
        {
            //0   1   2   3   4   5   6   // la diagonale à 1 traduit le mouvement propre de chaque clé
            /*0*/ 1., 1., 1., 1., 1., 1., 1., // cette ligne traduit que chaque clé fait tourner la clé 0 de la même valeur
            /*1*/ 1., 1., 1., 2., 3., 2., 1.,
            /*2*/ 1., 1., 1., 1., 2., 3., 2., // ensuite pour 1 clé :
            /*3*/ 1., 2., 1., 1., 1., 2., 3., //     les 2 clés adjacentes tournent de la même valeur
            /*4*/ 1., 3., 2., 1., 1., 1., 2., //     la clé opposée 3 fois plus
            /*5*/ 1., 2., 3., 2., 1., 1., 1., //     les 2 clés intermédiaires 2 fois plus
            /*6*/ 1., 1., 2., 3., 2., 1., 1.
        } , // la colonne 0 traduit que la clé 0 fait tourner toutes les clés de la même valeur

        mat[] = // 6,6 en suppriment la dernière ligne et dernère colonne de la 7,7
        {
            1., 1., 1., 1., 1., 1.,
            1., 1., 1., 2., 3., 2.,
            1., 1., 1., 1., 2., 3.,
            1., 2., 1., 1., 1., 2.,
            1., 3., 2., 1., 1., 1.,
            1., 2., 3., 2., 1., 1.
        } ,

        matInv[Ordre6 * Ordre6] ,
        matUni[Ordre6 * Ordre6] ,
        vectP[7] = {  0. ,  0. , -2. , -1. , -2. , -3. , 0. } , // position initiale réelle, vecteurs colonne
                   vectC[7] ,
                   vectR[7] ;
//------------------
        MatPrint( "P Vecteur position initiale" , " %6.2f" , vectP , 1 , 7 ) ;
        printf("\n\nSomme de P1+P3+P5-(P2+P4+P6) doit être nulle = %2d", // pourquoi ?
               (int) ( vectP[1] - vectP[2] + vectP[3] - vectP[4] + vectP[5] - vectP[6] ) ) ;

        MatPseudoInv( matInv , mat , Ordre6 , Ordre6 ) ;
        MatPrint( "M(-1) Matrice inverse" , " %9.6f" , matInv , Ordre6 , Ordre6 ) ;

        MatMul( matUni , matInv , mat , Ordre6 , Ordre6 , Ordre6 ) ;
        MatPrint( "I Matrice unitaire = M(-1) * M" , " %6.2f" , matUni , Ordre6 , Ordre6 ) ;

        MatMul( matUni , mat , matInv , Ordre6 , Ordre6 , Ordre6 ) ;
        MatPrint( "I Matrice unitaire = M * M(-1)" , " %6.2f" , matUni , Ordre6 , Ordre6 ) ;

// C = - M(-1) * P
        MatMul( vectC , matInv , vectP , Ordre6 , Ordre6 , 1 ) ;
        MatMulScal( vectC , (Reel) -1. , vectC , Ordre6 , 1 ) ;
        vectC[6] = (Reel) 0. ; // lié, on ne tourne pas la clé 6
        MatPrint( "C Vecteur commande" , " %6.2f" , vectC , 1 , 7 ) ;
// vectC[7] = { -2. , 0. , 2. , -1. , 0. , 1. , 0. } ; // résultat attendu

// Vérification M * C + P = vecteur 0
        MatMul( vectR , mat77 , vectC , 7 , 7 , 1 ) ;
// vectC = { -2. , 0. , 2. , -1. , 0. , 1. , 0. }
        MatAdd( vectR , vectR , vectP , 7 , 1 ) ;
        MatPrint( "Vecteur résultat doit être nul" , " %6.2f" , vectR , 1 , 7 ) ;
    } // Test inversion pour jeu GB

    {
        // Test inversion pour Sherpa
        Reel mat0[] =
        {
            -1. ,  1. ,
            1. ,  1.
        } ,

        mat2[] =
        {
            1. ,  1. ,
            -1. ,  1.
        } ,

        mat4[] =
        {
            -1. , -1. ,
            -1. ,  1.
        } ,

        mat[] =
        {
            -1. ,  1. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,
            1. ,  1. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,
            0. ,  0. ,  1. ,  1. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,
            0. ,  0. , -1. ,  1. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,
            0. ,  0. ,  0. ,  0. , -1. , -1. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,
            0. ,  0. ,  0. ,  0. , -1. ,  1. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,
            0. ,  0. ,  0. ,  0. ,  0. ,  0. , -1. ,  1. ,  0. ,  0. ,  0. ,  0. ,
            0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  1. ,  1. ,  0. ,  0. ,  0. ,  0. ,
            0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  1. ,  1. ,  0. ,  0. ,
            0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. , -1. ,  1. ,  0. ,  0. ,
            0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. , -1. , -1. ,
            0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. ,  0. , -1. ,  1.
        } ,

        matInv[12*12] ;
//------------------
        MatPseudoInv( matInv , mat0 , 2 , 2 ) ;
        MatPrint( "M0(-1) Matrice inverse" , " %9.6f" , matInv , 2 , 2 ) ;

        MatPseudoInv( matInv , mat2 , 2 , 2 ) ;
        MatPrint( "M2(-1) Matrice inverse" , " %9.6f" , matInv , 2 , 2 ) ;

        MatPseudoInv( matInv , mat4 , 2 , 2 ) ;
        MatPrint( "M4(-1) Matrice inverse" , " %9.6f" , matInv , 2 , 2 ) ;

        MatPseudoInv( matInv , mat , 12 , 12 ) ;
        MatPrint( "M(-1) Matrice inverse" , " %9.6f" , matInv , 12 , 12 ) ;
    } // Test inversion pour Sherpa


    {
        // Test pseudo inversion générée sous Matlab par M=rand(10,4) et Pinv(M)
#define L 10
#define C  4

        Reel mat[] =
        {
            0.9501f ,    0.6154f ,    0.0579f ,    0.0153f ,
            0.2311f ,    0.7919f ,    0.3529f ,    0.7468f ,
            0.6068f ,    0.9218f ,    0.8132f ,    0.4451f ,
            0.4860f ,    0.7382f ,    0.0099f ,    0.9318f ,
            0.8913f ,    0.1763f ,    0.1389f ,    0.4660f ,
            0.7621f ,    0.4057f ,    0.2028f ,    0.4186f ,
            0.4565f ,    0.9355f ,    0.1987f ,    0.8462f ,
            0.0185f ,    0.9169f ,    0.6038f ,    0.5252f ,
            0.8214f ,    0.4103f ,    0.2722f ,    0.2026f ,
            0.4447f ,    0.8936f ,    0.1988f ,    0.6721f
        } ,
        /*
        inv[C*L] =
        {
            0.2866   -0.1416    0.0474   -0.0163    0.4094    0.2663   -0.0862   -0.2758    0.3039   -0.0753
            0.9830   -0.0593   -0.1445   -0.0167   -0.9076   -0.3528    0.2854    0.2626   -0.1173    0.4821
           -0.7886    0.1529    0.9193   -0.4797    0.4668    0.2071   -0.3883    0.4095    0.2235   -0.4467
           -0.9693    0.3171   -0.1287    0.4963    0.6585    0.2364    0.1466   -0.1164   -0.1219   -0.1061
        }
        */
        matInv[12*12] ,
        matUni[12*12] ;
//------------------
        MatPseudoInv( matInv , mat , L , C ) ;
        MatPrint( "mat(-1) Matrice inverse" , " %9.6f" , matInv , C , L ) ;

        MatMul( matUni , matInv , mat , C , L , C ) ;
        MatPrint( "I" , "%9.6f" , matUni , C , C ) ;
    } // Test pseudo inversion générée sous Matlab par M=rand(10,4) et Pinv(M)


    {
        // Test pseudo inversion de Julien A. robot Omni
#define LJ 3
#define CJ 6

        Reel mat[] = // ne marche pas en double, passer en double
        {
            0.2459f ,    0.f     ,    0.1224f ,    9.9998f ,   10.f     ,    9.9998f ,
            19.9998f ,   -0.123f  ,    0.f     ,   -0.06f   ,   19.9998f ,   20.f     ,
            -0.954f  ,   -1.92f   ,   -1.88f   ,    3.75f   ,    5.4533f ,   -7.5f
        } ,
        /*
        inv[] =
        {
            0.0003    0.0197    0.0614
           -0.0001    0.0136   -0.0618
            0.0001    0.0167    0.0004
            0.0334   -0.0012   -0.0186
            0.0334   -0.0011   -0.0182
            0.0332    0.0016    0.0353
        }
        */
        matInv[12*12] ,
        matUni[12*12] ;
//------------------
        MatPseudoInv( matInv , mat , LJ , CJ ) ;
        MatPrint( "mat(-1) Matrice inverse Julien A." , " %9.6f" , matInv , CJ , LJ ) ;

        MatMul( matUni , matInv , mat , CJ , LJ , CJ ) ;
        MatPrint( "I" , "%9.6f" , matUni , CJ , CJ ) ;
    } // Test pseudo inversion de Julien A. robot Omni



    return 0 ;
} // ModuleTest

//#define MMax 18 // M Max lignes
//#define NMax 18 // N Max colonnes

#define MMax 30 // M Max lignes
#define NMax 30 // N Max colonnes

//----------------------------------------------------------
void MatTrans( Reel *_trans , const Reel *_mat, int _m , int _n )
{
    /*
    Méthode : _mat est balayée ligne par ligne, _trans colonne par colonne.

    Abandonné la possibilité d'utiliser _mat pour y transférer la transposée.
    En plus c'est une n,m
    */
    Reel *p ; // pointe un élément de _trans
    int j ,
    i ;
//------------------
    for ( j = _m ; j-- ; _trans++ )                                // _m colonnes de _trans
        for ( p = _trans , i = _n ; i-- ; p += _m ) *p = *_mat++ ; // _n lignes de _trans
} // MatTrans


//----------------------------------------------------------
void MatTransf( double *_trans , const double *_mat, int _m , int _n )
{
    double *p ; // pointe un élément de _trans
    int j ,
    i ;
//------------------
    for ( j = _m ; j-- ; _trans++ )                                // _m colonnes de _trans
        for ( p = _trans , i = _n ; i-- ; p += _m ) *p = *_mat++ ; // _n lignes de _trans
} // MatTransf


//----------------------------------------------------------
void MatMul( Reel *_resul , const Reel *_mat1 , const Reel *_mat2 , int _l , int _m , int _n )
{
// !!! voir en récursif ???
    const Reel *pCol2 , // pointe une colonne de _mat2
    *p1 ,    // pointe un élément de _mat1
    *p2 ;    // pointe un élément de _mat2
    int i ,
    j ,
    k ;
    Reel aux ,
    *pResul ,
    temp[NMax * MMax] ;
//------------------
    if ( ( _resul == _mat2 ) || ( _resul == _mat1 ) )
    {
        // _resul est dans _mat1 ou _mat2
        pResul = _resul ;
        _resul = temp ;
    }
    else pResul = NULL ;

    for ( i = _l ; i-- ; _mat1 += _m )     // _l lignes de _mat1 et de _resul dans l'ordre
    {
        for ( pCol2 = _mat2 , j = _n ; j-- ; pCol2++ ) // _n colonnes de _mat2 et de _resul dans l'ordre
        {
            for ( p1 = _mat1 , p2 = pCol2 , aux = (Reel) 0.f , k = _m ; k-- ; p2 += _n ) aux += *p1++ * *p2 ;
            *_resul++ = aux ;
        }
    } // for i

    if ( pResul ) MatCpy( pResul , temp , _m , _n ) ;
} // MatMul


//----------------------------------------------------------
void MatMulf( double *_resul , const double *_mat1 , const double *_mat2 , int _l , int _m , int _n )
{
    const double *pCol2 , // pointe une colonne de _mat2
    *p1 ,    // pointe un élément de _mat1
    *p2 ;    // pointe un élément de _mat2
    int i ,
    j ,
    k ;
    double aux ,
    *pResul ,
    temp[NMax * MMax] ;
//------------------
    if ( ( _resul == _mat2 ) || ( _resul == _mat1 ) )
    {
        // _resul est dans _mat1 ou _mat2
        pResul = _resul ;
        _resul = temp ;
    }
    else pResul = NULL ;

    for ( i = _l ; i-- ; _mat1 += _m )     // _l lignes de _mat1 et de _resul dans l'ordre
    {
        for ( pCol2 = _mat2 , j = _n ; j-- ; pCol2++ ) // _n colonnes de _mat2 et de _resul dans l'ordre
        {
            for ( p1 = _mat1 , p2 = pCol2 , aux = 0.f , k = _m ; k-- ; p2 += _n ) aux += *p1++ * *p2 ;
            *_resul++ = aux ;
        }
    } // for i

    if ( pResul ) MatCpyf( pResul , temp , _m , _n ) ;
} // MatMulf


//----------------------------------------------------------
void MatAdd( Reel *_resul , const Reel *_mat1 , const Reel *_mat2 , int _m , int _n )
// voir direct _m * _n = _taille en paramètre !!!! idem Sub Scal Cpy Clr Set
{
    int i ;
//------------------
    for ( i = _m * _n ; i-- ; ) *_resul++ = *_mat1++ + *_mat2++ ; // _m * _n éléments
}


//----------------------------------------------------------
void MatAddf( double *_resul , const double *_mat1 , const double *_mat2 , int _m , int _n )
// voir direct _m * _n = _taille en paramètre !!!! idem Sub Scal Cpy Clr Set
{
    int i ;
//------------------
    for ( i = _m * _n ; i-- ; ) *_resul++ = *_mat1++ + *_mat2++ ; // _m * _n éléments
}


//----------------------------------------------------------
void MatSub( Reel *_resul , const Reel *_mat1 , const Reel *_mat2 , int _m , int _n )
{
    int i ;
//------------------
    for ( i = _m * _n ; i-- ; ) *_resul++ = *_mat1++ - *_mat2++ ; // _m * _n éléments
}


//----------------------------------------------------------
void MatSubf( double *_resul , const double *_mat1 , const double *_mat2 , int _m , int _n )
{
    int i ;
//------------------
    for ( i = _m * _n ; i-- ; ) *_resul++ = *_mat1++ - *_mat2++ ; // _m * _n éléments
}


//----------------------------------------------------------
void MatMulScal( Reel *_resul , Reel _scal , const Reel *_mat , int _m , int _n )
{
    int i ;
//------------------
    for ( i = _m * _n ; i-- ; ) *_resul++ = _scal * *_mat++ ; // _m * _n éléments
}


//----------------------------------------------------------
void MatMulScalf( double *_resul , double _scal , const double *_mat , int _m , int _n )
{
    int i ;
//------------------
    for ( i = _m * _n ; i-- ; ) *_resul++ = _scal * *_mat++ ; // _m * _n éléments
}


#include <string.h> // pour memcpy en .cpp


//----------------------------------------------------------
void MatCpy( Reel *_resul , const Reel *_mat , int _m , int _n ) // nom comme memcpy ou strcpy
{
    memcpy( _resul , _mat , _m * _n * sizeof( Reel ) ) ;    // 0 ou 1 marchent, sont testés
}


//----------------------------------------------------------
void MatCpyf( double *_resul , const double *_mat , int _m , int _n ) // nom comme memcpy ou strcpy
{
    memcpy( _resul , _mat , _m * _n * sizeof( double ) ) ;    // 0 ou 1 marchent, sont testés
}


//----------------------------------------------------------
void MatClr( Reel *_mat , int _m , int _n )
{
    int i ;
//------------------
    for ( i = _m * _n ; i-- ; ) *_mat++ = (Reel) 0.f ; // _m * _n éléments
}


//----------------------------------------------------------
void MatClrf( double *_mat , int _m , int _n )
{
    int i ;
//------------------
    for ( i = _m * _n ; i-- ; ) *_mat++ = 0.f ; // _m * _n éléments
}


//----------------------------------------------------------
void MatI( Reel *_mat , int _n )
{
    MatClr( _mat , _n , _n ) ;
    {
        int i ;
//------------------
        for ( i = _n * _n - 1 ; i >= 0 ; i -= _n + 1 ) _mat[i] = (Reel) 1.f ; // LOURD !!!
    }
} // MatI


//----------------------------------------------------------
void MatIf( double *_mat , int _n )
{
    MatClrf( _mat , _n , _n ) ;
    {
        int i ;
//------------------
        for ( i = _n * _n - 1 ; i >= 0 ; i -= _n + 1 ) _mat[i] = 1.f ; // LOURD !!!
    }
} // MatIf


//----------------------------------------------------------
void MatSet( Reel *_mat , int _m , int _n , Reel _scal )
{
    int i ;
//------------------
    for ( i = _m * _n ; i-- ; ) *_mat++ = _scal ; // _m * _n éléments
}


//----------------------------------------------------------
void MatSetf( double *_mat , int _m , int _n , double _scal )
{
    int i ;
//------------------
    for ( i = _m * _n ; i-- ; ) *_mat++ = _scal ; // _m * _n éléments
}


//----------------------------------------------------------
void MatPrint( const char *_titre , const char *_format , const Reel *_mat , int _m , int _n )
{
    int i ,
    j ;
//------------------
    printf( "\n\n%s" , _titre ) ;
    for ( i = _m ; i-- ; )                                                           // _m lignes
        for ( printf( "\n" ) , j = _n ; j-- ; ) printf( _format , (Reel) *_mat++ ) ; // _n colonnes
} // MatPrint


//----------------------------------------------------------
void MatPrintf( const char *_titre , const char *_format , const double *_mat , int _m , int _n )
{
    int i ,
    j ;
//------------------
    printf( "\n\n%s" , _titre ) ;
    for ( i = _m ; i-- ; )                                                            // _m lignes
        for ( printf( "\n" ) , j = _n ; j-- ; ) printf( _format , (double) *_mat++ ) ; // _n colonnes
} // MatPrintf


//---------------------------------------------------------- privé
static Reel NormeCarre( const Reel *_vect , int _n )
{
    /*
    Fonction : Carré de la norme d'un vecteur

    //MatMul( &retour , _vect , _vect , 1 , _n , 1) ; // normeCarre = t_vect * _vect serait plus long
    //retour = MatProScal( _vect , _vect , _n ) serait plus long
    */
    int  i ;
    Reel retour = 0.f ;
//------------------
    for ( i = _n ; i-- ; )  retour += _vect[i] * _vect[i] ;
    return retour ;
} // NormeCarre


//---------------------------------------------------------- privé
static double NormeCarref( const double *_vect , int _n )
{
    int  i ;
    double retour = 0.f ;
//------------------
    for ( i = _n ; i-- ; )  retour += _vect[i] * _vect[i] ;
    return retour ;
} // NormeCarref


//----------------------------------------------------------
int MatPseudoInv( Reel *_inv , const Reel *_mat , int _m , int _n )
{
    /*
    Les noms des vecteurs de l'algorithme de Greville (d,b,c) ont été conservé
    Biblio : Théorie Calcul Matriciel p 129 (bibli Lirmm n° 7068)

    _inv est calculé ligne par ligne

    indice k = [0 à n[

    A   : matrice [m,n]
    A+  : pseudo [n,m]
    ak  : colonne k de A [m,1]
    akT : transposée de ak ou ligne k de AT [1,m]
    Ak  : matrice formée des k premières colonnes de A [m,k-1]
    Ak+ : matrice formée des k premières lignes de A+ [k-1,m]

     !!! faire + rapide
     !!! si _m < _n utiliser (A+)t = (At)+ si + rapide à mesurer sous RTX
     !!! voir en récursif Non car le pas est complexe par rapport à l'ensemble de l'algorithme
                              même si le test d'arrêt et le premier pas sont simples.
     !!! voir pour k = 0 si on l'intégre dans la boucle


    Pour mémoire :
    - Pseudo inverse à droite : Pinv(M)=Mt*(M*Mt)^-1
    - Pseudo inverse à gauche : Pinv(M)=(Mt*M)^-1*Mt
    */

#define Pr 0 // print dans la boucle de calcul

#define EPS (Reel) 1e-16 // -12 chez Xavier

    int  k ;
    Reel tmat[NMax * MMax] , // transposée de _mat(_n,_m)
    *pat ,              // pointe la ligne k de tmat (pour la colonne k de _mat)
    xnorm ,
    xnul ,  // groupé avec xmax
    d[NMax-1] ,    // vecteur colonne de taille k variable, maximum _n-1, utilisé aussi pour td (transposée de d)
    x ,
    matAux[( NMax - 1 ) * MMax] ,
    *ctPuisb ; // pointeur sur un vecteur ligne ct puis b , pointe la ligne k de _inv

//------------------
    if ( ( _m > MMax ) || ( _n > NMax ) )
    {
        printf("\nMatPseudoInv: erreur matrice %2d x %2d maximum.", MMax , NMax ) ; // !!! message err
        return 1 ;
    }

    MatTrans( tmat , _mat , _m , _n ) ; // tmat(_n,_m) = transposée de _mat l'accé aux lignes y est plus pratique

    for ( xnul = (Reel) 0.f , k = _n ; k-- ; ) // sur les _n vecteurs lignes de tmat
    {
        xnorm = NormeCarre( tmat + k * _m , _m ) ; // ligne k de tmat, tmat à _m colonnes
        if ( xnorm > xnul ) xnul = xnorm ; // garder la plus grande
    }
// Le dernier xnorm calculé est celui de la ligne 0 de tmat, soit la colonne 0 de _mat

    if ( xnul > (Reel) 1e32 ) return 2 ; // erreur, n'est pas dans Xavier
    xnul *= EPS ;

    pat     = tmat ; // on utilise les lignes de tmat plus facile d'accès que les colonnes de _mat
    ctPuisb = _inv ; // on utilise la première ligne non encore calculée de _inv pour ranger ck puis bk


//si a0 = 0 alors A1+ = 0                    // ligne 0 seule de la speudo [1,m]
//          sinon A1+ = (a0T * a0)^-1 * a0T  // ( 1 / ( [1,m] * [m,1] ) ) * [1,m] = [1,m]

// Si la première colonne de _mat n'est pas nulle
    if ( xnorm > xnul ) MatMulScal( ctPuisb , (Reel) 1.f / xnorm , pat , 1 , _m ) ; // init 1ère ligne de _inv k = 0
    else                MatClr( ctPuisb , 1 , _m ) ; // clear 1ère ligne de _inv si la norme est nulle

#if Pr
    MatPrint( "_inv" , " %10.6f" , _inv , 1 , _m ) ; // k = 0
#endif

    for ( k = 1 ; k < _n ; k++ )
    {
        pat     += _m ; // passe à la ligne suivante, pointe sur la ligne k de tmat (0 à n-1) colonne k de _mat
        ctPuisb += _m ; // passe à la ligne suivante, pointe sur la ligne k de _inv

        // d = Ak+ * ak     // [k,m] * [m,1] = [k,1] ce vecteur auxiliaire peut être écrasé à chaque boucle donc pas d'indice pour d
        MatMul( d , _inv , pat , k , _m , 1 ) ; // at est identique à a
        // ck = ak - Ak * d // [m,1] - [m,k] * [k,1] = [m,1] sera rangé provisoirement dans la ligne k de la pseudo, elle est libre
        // Ak étant difficilement accessible on utilise la relation : _mat * d = t(td * tmat) ; [m,k] * [k,1] = t([1,k] * [k,m]) = [m,1]
        MatMul( ctPuisb , d , tmat , 1 , k , _m ) ;  // utilise d pour transposé de d
        MatSub( ctPuisb , pat , ctPuisb , 1 , _m ) ; // ligne k de tmat et on utilise la place de _inv

        //si ck = 0 alors bk = (1 + dT * d)^-1 * dT * Ak+ // scalaire * [1,k] * [k,m] = [1,m]
        //          sinon bk = (ckT * ck)^-1 * ckT  // ( 1 / ( [1,m] * [m,1] ) ) * [1,m] = [1,m]

        xnorm = NormeCarre( ctPuisb , _m ) ;
        if ( xnorm > xnul ) MatMulScal( ctPuisb , (Reel) 1.f / xnorm , ctPuisb , 1 , _m ) ; // c'est bk
        else             // Si cette norme est nulle
        {
            x = (Reel) 1.f / ( (Reel) 1.f + NormeCarre( d , k ) ) ;
            MatMul( ctPuisb , d , _inv , 1 , k , _m ) ;    // cPuisb = td * _inv (les k 1ères lignes)
            MatMulScal( ctPuisb , x , ctPuisb , 1 , _m ) ; // c'esr bk
        }
        // le résultat du if/else est dans bk, c'est sur la ligne k de _inv, est déjà en place

        // Ak+ = Ak+ - d * bk // [k,m] - [k,1] * [1,m] = [k,m]
        MatMul( matAux , d , ctPuisb , k , 1 , _m ) ; // matAux(k,m) = d(k) * cPuisb(_m)
        MatSub( _inv , _inv , matAux , k , _m ) ;  // et bk est déjà en place sur la ligne suivante de _inv

#if Pr
        MatPrint( "_inv" , " %10.6f" , _inv , k + 1 , _m ) ;
#endif

    } // for k

    return 0 ;
} // MatPseudoInv


//----------------------------------------------------------
int MatPseudoInvf( double *_inv , const double *_mat , int _m , int _n )
{

#define EPSf (double) 1e-12 // -12 chez Xavier

    int  k ;
    double tmat[NMax * MMax] , // transposée de _mat(_n,_m)
    *pat ,              // pointe la ligne k de tmat (pour la colonne k de _mat)
    xnorm ,
    xnul ,  // groupé avec xmax
    d[NMax-1] ,    // vecteur colonne de taille k variable, maximum _n-1, utilisé aussi pour td (transposée de d)
    x ,
    matAux[( NMax - 1 ) * MMax] ,
    *ctPuisb ; // pointeur sur un vecteur ligne ct puis b , pointe la ligne k de _inv

//------------------
    if ( ( _m > MMax ) || ( _n > NMax ) )
    {
        printf("\nMatPseudoInv: erreur matrice %2d x %2d maximum.", MMax , NMax ) ; // !!! message err
        return 1 ;
    }

    MatTransf( tmat , _mat , _m , _n ) ; // tmat(_n,_m) = transposée de _mat l'accé aux lignes y est plus pratique

    for ( xnul = 0.f , k = _n ; k-- ; ) // sur les _n vecteurs lignes de tmat
    {
        xnorm = NormeCarref( tmat + k * _m , _m ) ; // ligne k de tmat, tmat à _m colonnes
        if ( xnorm > xnul ) xnul = xnorm ; // garder la plus grande
    }
// Le dernier xnorm calculé est celui de la ligne 0 de tmat, soit la colonne 0 de _mat

    if ( xnul > (double) 1e32 ) return 2 ; // erreur, n'est pas dans Xavier
    xnul *= EPSf ;

    pat     = tmat ; // on utilise les lignes de tmat plus facile d'accès que les colonnes de _mat
    ctPuisb = _inv ; // on utilise la première ligne non encore calculée de _inv pour ranger ck puis bk


//si a0 = 0 alors A1+ = 0                    // ligne 0 seule de la speudo [1,m]
//          sinon A1+ = (a0T * a0)^-1 * a0T  // ( 1 / ( [1,m] * [m,1] ) ) * [1,m] = [1,m]

// Si la première colonne de _mat n'est pas nulle
    if ( xnorm > xnul ) MatMulScalf( ctPuisb , 1.f / xnorm , pat , 1 , _m ) ; // init 1ère ligne de _inv k = 0
    else                MatClrf( ctPuisb , 1 , _m ) ; // clear 1ère ligne de _inv si la norme est nulle

#if Pr
    MatPrintf( "_inv" , " %10.6f" , _inv , 1 , _m ) ; // k = 0
#endif

    for ( k = 1 ; k < _n ; k++ )
    {
        pat     += _m ; // passe à la ligne suivante, pointe sur la ligne k de tmat (0 à n-1) colonne k de _mat
        ctPuisb += _m ; // passe à la ligne suivante, pointe sur la ligne k de _inv

        // d = Ak+ * ak     // [k,m] * [m,1] = [k,1] ce vecteur auxiliaire peut être écrasé à chaque boucle donc pas d'indice pour d
        MatMulf( d , _inv , pat , k , _m , 1 ) ; // at est identique à a
        // ck = ak - Ak * d // [m,1] - [m,k] * [k,1] = [m,1] sera rangé provisoirement dans la ligne k de la pseudo, elle est libre
        // Ak étant difficilement accessible on utilise la relation : _mat * d = t(td * tmat) ; [m,k] * [k,1] = t([1,k] * [k,m]) = [m,1]
        MatMulf( ctPuisb , d , tmat , 1 , k , _m ) ;  // utilise d pour transposé de d
        MatSubf( ctPuisb , pat , ctPuisb , 1 , _m ) ; // ligne k de tmat et on utilise la place de _inv

        //si ck = 0 alors bk = (1 + dT * d)^-1 * dT * Ak+ // scalaire * [1,k] * [k,m] = [1,m]
        //          sinon bk = (ckT * ck)^-1 * ckT  // ( 1 / ( [1,m] * [m,1] ) ) * [1,m] = [1,m]

        xnorm = NormeCarref( ctPuisb , _m ) ;
        if ( xnorm > xnul ) MatMulScalf( ctPuisb , 1.f / xnorm , ctPuisb , 1 , _m ) ; // c'est bk
        else             // Si cette norme est nulle
        {
            x = 1.f / ( 1.f + NormeCarref( d , k ) ) ;
            MatMulf( ctPuisb , d , _inv , 1 , k , _m ) ;    // cPuisb = td * _inv (les k 1ères lignes)
            MatMulScalf( ctPuisb , x , ctPuisb , 1 , _m ) ; // c'esr bk
        }
        // le résultat du if/else est dans bk, c'est sur la ligne k de _inv, est déjà en place

        // Ak+ = Ak+ - d * bk // [k,m] - [k,1] * [1,m] = [k,m]
        MatMulf( matAux , d , ctPuisb , k , 1 , _m ) ; // matAux(k,m) = d(k) * cPuisb(_m)
        MatSubf( _inv , _inv , matAux , k , _m ) ;  // et bk est déjà en place sur la ligne suivante de _inv

#if Pr
        MatPrintf( "_inv" , " %10.6f" , _inv , k + 1 , _m ) ;
#endif

    } // for k

    return 0 ;
} // MatPseudoInvf


//----------------------------------------------------------
void MatTilde( Reel *_resul , const Reel *_vect )
{
    Reel ux , uy , uz ;
//------------------
    ux = *_vect++ ;
    uy = *_vect++ ;
    uz = *_vect ;

    *_resul++ = (Reel) 0.f ;
    *_resul++ =       - uz ;
    *_resul++ =         uy ;
    *_resul++ =         uz ;
    *_resul++ = (Reel) 0.f ;
    *_resul++ =       - ux ;
    *_resul++ =       - uy ;
    *_resul++ =         ux ;
    *_resul   = (Reel) 0.f ;
} // MatTilde


//----------------------------------------------------------
void MatTildef( double *_resul , const double *_vect )
{
    double ux , uy , uz ;
//------------------
    ux = *_vect++ ;
    uy = *_vect++ ;
    uz = *_vect ;

    *_resul++ =  0.f ;
    *_resul++ = - uz ;
    *_resul++ =   uy ;
    *_resul++ =   uz ;
    *_resul++ =  0.f ;
    *_resul++ = - ux ;
    *_resul++ = - uy ;
    *_resul++ =   ux ;
    *_resul   =  0.f ;
} // MatTildef


//----------------------------------------------------------
void MatProVectoriel( Reel *_resul , const Reel *_vect1 , const Reel *_vect2 )
{
#if 1

    int i , j , k ;
//------------------
    for ( i = 0 ; i < 3 ; i ++ )
    {
        j = ( i + 1 ) % 3 ;
        k = ( i + 2 ) % 3 ;
        *_resul++ = *( _vect1 + j ) * *( _vect2 + k )  -  *( _vect1 + k ) * *( _vect2 + j ) ;
    } // for i

#else // Méthode avec MatTilde

    Reel tilde[3*3] ;
//------------------
    MatTilde( tilde , _vect1 ) ;
    MatMul( _resul , tilde , _vect2 , 3 , 3 , 1 ) ;

#endif
} // MatProVectoriel


//----------------------------------------------------------
void MatProVectorielf( double *_resul , const double *_vect1 , const double *_vect2 )
{
#if 1

    int i , j , k ;
//------------------
    for ( i = 0 ; i < 3 ; i ++ )
    {
        j = ( i + 1 ) % 3 ;
        k = ( i + 2 ) % 3 ;
        *_resul++ = *( _vect1 + j ) * *( _vect2 + k )  -  *( _vect1 + k ) * *( _vect2 + j ) ;
    } // for i

#else // Méthode avec MatTilde

    double tilde[3*3] ;
//------------------
    MatTildef( tilde , _vect1 ) ;
    MatMulf( _resul , tilde , _vect2 , 3 , 3 , 1 ) ;

#endif
} // MatProVectorielf


//----------------------------------------------------------
Reel MatProScal( const Reel *_vect1 , const Reel *_vect2 , int _n )
{
    int  i ;
    Reel retour = 0.f ;
//------------------
    for ( i = _n ; i-- ; )  retour += _vect1[i] * _vect2[i] ;
//MatMul( &retour , _vect1 , _vect2 , 1 , _n , 1) ; // ProScal = t_vect1 * _vect2 serait plus long
    return retour ;
} // MatProScal


//----------------------------------------------------------
double MatProScalf( const double *_vect1 , const double *_vect2 , int _n )
{
    int  i ;
    double retour = 0.f ;
//------------------
    for ( i = _n ; i-- ; )  retour += _vect1[i] * _vect2[i] ;
//MatMulf( &retour , _vect1 , _vect2 , 1 , _n , 1) ; // ProScal = t_vect1 * _vect2 serait plus long
    return retour ;
} // MatProScalf




//----------------------------------------------------------
Reel MatNorm( const Reel *_vect , int _n )
{
    return (Reel) sqrt( (double) NormeCarre( _vect , _n ) ) ;
}


//----------------------------------------------------------
double MatNormf( const double *_vect , int _n )
{
    return (double) sqrt( (double) NormeCarref( _vect , _n ) ) ;
}


//----------------------------------------------------------
//Reel MatInvNorm( const Reel *_vect , int _n ) { return (Reel) ( 1. / sqrt( (double) NormeCarre( _vect , _n ) ) ) ; }
// SI _vect = 0 on a une / 0 !!!

#if 0
// Corbeille ********************************************

Reel MatNormeInfini( const Reel * _v , int _n )
{
// élément le plus grand en valeur absolue d'un vecteur
    int i ;
    Reel resultat	= (Reel) 0.f ;
//------------------
    for ( i = n ; i-- ; )
    {
        if ( fabs( _v[i]) > resultat ) resultat = (Reel) fabs( _v[i] ) ;
        ne pas recalculer fabs!!!
    }
    return resultat;
}
#endif
