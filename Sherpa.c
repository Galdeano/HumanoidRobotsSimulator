/* Sherpa.c

pourquoi 2 qc et err ???

10:33 05/10/10 MB

sur mon PC le temps est trop lent 22/11/10
 x 8 environ avec tache périodique

Période de l'asservissement : 5 ms

Codeurs absolus standards :
numérotation de haut en bas, jambe D en 1er. (0 est le plus haut de la jambe D)
sens standard repère xyz, (imposés dans AbsEncGetAngle)
mesure en rad,

Moteurs :
numérotation robot vu de face de G à D et de haut en bas, jambe D en 1er.
sens + = sens trigo vu du coté opposé au tube guide
Consigne des variateurs +/- 10 V (strap R10)

Couplages:
Chaque axe (codeur absolu) est piloté par 2 moteurs, les sens correspondants sont indiqués dans la fonction QToPoulies.

Réduction moteur vers poulie Rr = 8 * Pi
Moteur    Kci = 0,17 Nm/A
Variateur Kcv = Imax/Vmax = 5,9 A / 10 V
Rendement = 1

Variateurs :
Enable si 24 V sur Enable des var J2-1.
Reset J2-2
*/

// Paramètres de compilation.
/* Configurations
TacheAsserv     = 1 Asserv est lancé comme tâche périodique de RTX
                  0 Asserv est appelé dans la boucle à 5 ms de la tâche de fond (pseudo périodique) ATTENTION EST à 1 ms
CodeursCNA      = 1 Utilise les codeurs et les CNA ; = 0 les valeurs q 'lues' sont les qc de la période précédente
SecuChienEnable = 1 Utilise le chien et la sortie EnablePC ; = 0 n'utilise pas.
BoutonArret     = 1 Utilise le bouton d'arret logiciel ; = 0 n'utilise pas.
Observateur     = 1 Enregistre les variables de la table variables. ; = 0 n'enregistre rien.
TempsReel       = 0 un t fictif est incrémenté de Te à chaque période Marche avec TacheAsserv = 0
RTX             = 1 utilise des fonctions RTX ; = 0 N'utilise pas RTX donc pas d'E/S et avec TacheAsserv = 0
*/

#define NAMax 12 // nombre d'axes maxi laisser 12 dans le fichier même si une seule jambe

#if 1 // si 1 alors 2 jambes
#define NA    12 // nombre d'axes asservis de 0 à NA-1
#define NATI   2 // nombre de capteur d'effort ATI 
#else // jambe droite seule
#define NA     6 // nombre d'axes asservis de 0 à NA-1
#define NATI   1 // nombre de capteur d'effort ATI 
#endif

#define AsservQ 1 // Asserv en Q sinon en Poulie (ou m)

#define NumAxe 5         // axe ou moteur
#define MCReglageGainsSin 0 // réglage des gains des PID moteurs avec un sin
#define MCReglageGainsEch 0 // réglage des gains des PID moteurs avec un echelon de position
#define MCSin          0 // sin sur les articulations
#define MCTrajectoire  1 // trajectoire SLF, lecture des fichiers de points,
#define MCPosition     0 // trajectoire linéaire de la position initiale mesurée vers la position finale indiquée,

#if 0 // = 1 Robot réel ; = 0 Sans E/S   

#define TacheAsserv 1
#define CodeursCNA  1
#define ATI         0
#define SecuChienEnable 0
#define BoutonArret 1
#define Observateur 1
#define TempsReel   1 // pas fini
#define RTX         1 // pas fini
#define Avatar      1

#else // Sans E/S (pseudo simulation)

#define TacheAsserv 1
#define CodeursCNA  0
#define ATI         0
#define SecuChienEnable 0
#define BoutonArret 0
#define Observateur 1
#define TempsReel   1 // pas fini
#define RTX         1 // pas fini
#define Avatar      0

#endif
 
#define Te 0.005 // période d'échantillonage en s et en double

#include <stdio.h>
#include <math.h>

#include "..\OsLib\McrTimer.h"

#include "..\StLib\PortSTIBoard.h"
#include "..\StLib\Temps.h"
#include "..\StLib\McrES.h"
#include "..\StLib\Mat.h"
#if 1 // Observateur laisser le include Obsrv.h car MatLabRd est dans Obsrv pour l'instant
#include "..\StLib\Obsrv.h"
#endif

#if Avatar
#include "d:\Mb\OsLib\SMem.h"

#include "d:\Mb\StLib\Fifo.h"
#include "d:\Mb\StLib\Quat.h"
#endif

#include "..\CaptLib\AbsEnc.h"
#include "..\CaptLib\ATI.h"


//----------------------------------------------------------
void QToPoulies( float _poulie[NA] , const float _q[NA] )
/*
Fonction : passage des positions des NA axes q aux positions des NA poulies.

Entrées  : _q      : position des axes (q ou qc)

Sorties  : _poulie : position des poulies

Méthode : ne pas utiliser une matrice 12x12 qui aurait alors 10 zéro par ligne et
          ne marcherait pas si on ne commande que la jambe droite (NA=6).
*/
{                               // inverse 1/2
*_poulie++ = - _q[0]  + _q[1] ; // - +
*_poulie++ =   _q[0]  + _q[1] ; // + +

*_poulie++ =   _q[2]  + _q[3] ; // + -
*_poulie++ = - _q[2]  + _q[3] ; // + +

*_poulie++ = - _q[4]  - _q[5] ; // - -
*_poulie++ = - _q[4]  + _q[5] ; // - +

#if NA == 12
*_poulie++ = - _q[6]  + _q[7] ;
*_poulie++ =   _q[6]  + _q[7] ;

*_poulie++ =   _q[8]  + _q[9] ;
*_poulie++ = - _q[8]  + _q[9] ;

*_poulie++ = - _q[10] - _q[11] ;
*_poulie   = - _q[10] + _q[11] ;
#endif
} // QToPoulies


/*
Fonctionnement de la sécurité :

- sortie TTL bit 0 Enable/ = 1 (ou chien = 1 ) => entrée Enable des var =  0V => var disables (car circuit inverseur).

- sortie TTL       Enable/ = 0 (et chien = 0 ) => entrée Enable des var = 24V => var enables mais après un Reset

- sortie TTL Bit 1 Reset   = 0 pendant >= 5ms  => entrée Reset des var  = 24V => Reset (car circuit inverseur).

A la mise sous tension :
- Le chien, qui ne dépend pas du bus PCI, est à 1 (au pire à 0 pendant 6 ms) => var disables.

Au run de l'exécutable .rtss :
- la sortie TTL Enable/ est mise à 1 (confirme l'action du chien pour var disables)
- la sortie TTL Reset   est mise à 1 (Reset inactif)
- les CNA sont mis à 0V.

Avant la mise périodique de la tâche Asserv :
- la sortie TTL Enable/ est mise à 0 et la sortie TTL Reset reste à 1
- lance le chien pour 6 ms pour avoir l'entrée Enable des var à 24V
- la sortie TTL Reset passe à 0 pour avoir l'entrée Reset à 24V
- maintient du Reset 5 ms
- la sortie TTL Reset repasse à 1 pour avoir Enable des var
- relance le chien pour 6 ms en attendant la 1er exécution de la tâche Asserv (dans 5ms) qui le maintiendra à 0
Maintenant les var sont prêt et les CNA sont à 0V.

Arrêt matériel :

- si le chien n'est pas relancé au moins toute les 6ms, (exception, boucle trop longue, etc...)
  1 - les variateurs sont disables par le matériel.
  2 - si le programme arrive à reprendre son exécution des erreurs seront vite détectées par le logiciel (de poursuite, ...) ,
      ce qui permettra de terminer proprement et de sauver les enregistrements. 

Arrêt logiciel :

- un grand nombre de tests sont effectués dans la tâche Asserv, si une ereur est détectée (dont le bouton poussoir)
   - la sortie TTL Enable est mise à 1 => var disables et la sortie TTL Reset est laissée à 1
   - les CNA sont mis à 0V.
   - le programme est terminé proprement et les enregistrements sont sauvés.

*/


//----------------------------------------------------------
static void ArretVar()
/*
Fonction : Arrêt des variateurs : disable les variateurs et RAZ des CNA (consignes variateurs)
*/
{
#if SecuChienEnable
McrPutTTL( 0x3 ) ; // Disable var Carte sécurité. (Enable var à 0V (bit 0 = 1) et Reset var reste à 0V (bit 1 = 1) 
#endif

#if CodeursCNA
{ // Consigne des variateurs à 0.
static const float val[NAMax] = { 0.f , 0.f , 0.f , 0.f , 0.f , 0.f ,
                                  0.f , 0.f , 0.f , 0.f , 0.f , 0.f } ;
//------------------
McrCNASherpa( val , NA ) ;
} // Consigne des variateurs à 0.
#endif CodeursCNA
} // ArretVar


static int _NbErreur = 0 ; // nombre d'erreurs à l'exécution

// Seuils d'erreur : 
#define TimeOutPeriode     0.0001  // double s 2% erreur sur la période de la tâche Asserv
#define TimeOutTache       0.00015 // double s le temps prévu pour l'exécution de la tâche Asserv est dépassé

// Codes d'erreur : 
#define NoError                0 // pas d'erreur, RAZ du fichier code
#define ErrArretPoussoir       1 // arrêt par bouton poussoir
#define ErrArretFinTrajectoire 2 // arrêt par fin de trajectoire RESERVé
#define ErrTimeOutPeriode      3 // erreur sur la période de la tâche Asserv
#define ErrTimeOutTache        4 // le temps prévu pour l'exécution de la tâche Asserv est dépassé
#define ErrTimeOutTrajectoire  5 // le temps prévu pour la trajectoire est dépassé
#define ErrPoursuite     100 // + i  erreur de poursuite sur mi
#define ErrButeeQiPlus   200 // + i  erreur de butée qi +
#define ErrButeeQiMoins  300 // + i  erreur de butée qi -
#define ErrButeeMiPlus   400 // + i  erreur de butée mi +
#define ErrButeeMiMoins  500 // + i  erreur de butée mi -


//----------------------------------------------------------
static void Erreur( int _num )
/*
Fonction : Arrêt des variateurs (au 1er appel),
           incrémente _NbErreur, enregistre dans code l'erreur _num.

Entrée   : _num  numéro de l'erreur.
*/
{
//if ( _num == ErrTimeOutTrajectoire )
{
if ( !(_NbErreur++) ) ArretVar() ;

#if Observateur
ObsrvFloat( 0 , (float) _num ) ; // dans code
#endif
}
} // Erreur


#if !CodeursCNA
static float *_Qcp ; // pointeur sur un tableau des NA consignes de q (utilisé en 'simulation' sans codeurs)
#endif


//----------------------------------------------------------
float *Mesures()
/*
Fonction : lecture des codeurs des axes q et passage à la position des poulies m
           en mode sans E/S retourne les consignes de l'échantillon précédent. 

Retour   : pointeur sur la position des moteurs q ou des m (poulies) selon AsservQ
           ATTENTION les valeurs sont dans la fonction en static. 
*/
{
static float m[NA] , // rad positions moteurs (poulies)
             q[NA] ; // rad position des axes

int i ;
//------------------
#if CodeursCNA
AbsEncGetAngle( q , NA ) ;    // lecture codeurs
#else                         // en 'simulation'
MatCpyf( q , _Qcp , NA , 1 ) ; // qc du coup précédent car q est arrivé en qc
//Si MCReglageGains copier directement mc => m !!!!!!!!!!!!!!!
#endif

{ // Test butées q. 0,02 rad = 1,15 degré qqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqq
static const float buteePlus[NAMax]  = {  1.07f ,  0.30f ,  1.02f ,  1.85f ,  1.06f ,  1.06f , // rad
                                          1.16f ,  0.29f ,  1.02f ,  1.85f ,  1.09f ,  1.07f } ; // 8&9 copy de 2&3
static const float buteeMoins[NAMax] = { -1.06f , -1.91f , -1.01f , -0.10f , -1.06f , -1.03f , // rad
                                         -1.04f , -1.70f , -1.01f , -0.10f , -1.03f , -1.08f } ; // 8&9 copy de 2&3
//------------------
for ( i = NA ; i-- ; )
{ 
    if      ( q[i] > buteePlus[i]  ) Erreur( ErrButeeQiPlus  + i ) ;
    else if ( q[i] < buteeMoins[i] ) Erreur( ErrButeeQiMoins + i ) ;
}
} // Test butées q.

QToPoulies( m , q ) ; // Passage en position des poulies

{ // Test butées m.                   mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
static const float buteePlus[NAMax]  = {  1.14f ,  0.71f ,  2.76f ,  2.76f ,  1.68f ,  1.72f , // rad
                                          0.73f ,  1.08f ,  2.76f ,  2.76f ,  1.66f ,  1.74f } ; // 8&9 copy de 2&3
static const float buteeMoins[NAMax] = { -1.89f , -2.08f , -0.67f , -0.72f , -1.67f , -1.72f , // rad
                                         -2.22f , -2.26f , -0.67f , -0.72f , -1.78f , -1.49f } ; // 8&9 copy de 2&3
//------------------
for ( i = NA ; i-- ; )
{ 
    if      ( m[i] > buteePlus[i]  ) Erreur( ErrButeeMiPlus  + i ) ;
    else if ( m[i] < buteeMoins[i] ) Erreur( ErrButeeMiMoins + i ) ;
} // for i
} // Test butées m.

#if Observateur
for ( i = NA ; i-- ; )
{
   ObsrvFloat( 26 + i , q[i] ) ;
   ObsrvFloat( 38 + i , m[i] ) ;
}
#endif // Observateur

#if AsservQ 
return q ; // on a quand même calculé m pour le test des butées m
#else
return m ;
#endif
} // Mesures


//----------------------------------------------------------
// float *MC( int  )
/*
Pour MC générique :

Fonction : génére les consignes 'moteurs' (poulies) mc pour asserv.
           en interne génére les consignes articulaires qc
           et en mode sans E/S celles ci seront utilisées comme mesures lors du prochain échantillonnage 

Entrée   : numéro de l'axe utilisé ou bidon.

Retour   : mc   pointeur sur le tableau des consignes moteur.
           NULL fin de trajectoire où il y reste asservi ??? !!!
*/


#if MCPosition
#define TimeOutTrajectoire 5.1 // double s le temps prévu pour la trajectoire est dépassé (pour MCPosition)
//----------------------------------------------------------
float *MC( int _nonUtilise )
/*
Fonction : génére une trajectoire linéaire de la position initiale vers une position finale.
*/
{
static const double dureeTraj = 5. ; // s durée de la trajectoire

#if 1
static const float qcf[NAMax] = { 0.f , 0.f , 0.f , 0.f , 0.f , 0.f ,   // position finale à zéro
                                  0.f , 0.f , 0.f , 0.f , 0.f , 0.f } ;
#else
static const float qcf[NAMax] = { 0.f , 0.f , 0.f , 0.f , 0.f , 0.f ,   // rad position finale
                                  0.f , 0.f , 0.f , 0.f , 0.f , 0.f } ;
#endif

static float mc[NA]  , // rad
             qc0[NA] , // rad position initiale des axes à l'instant 0 (ou q0)
             pente[NA] , // pente
             qc[NA]  ; // rad

static double t0 , // s temps à l'instant 0
              tf ; // fin

static int initFait = 0 ;
//------------------
if ( !initFait)
{ // Appel une seule fois dans le main
    printf( "\nRetour au zéro ou autre position initiale %f s\n" , TimeOutTrajectoire ) ; 
    t0 = TempsTickToS( McrTimerGetCount( NULL ) ) ; // Lecture du temps t0

    #if CodeursCNA
    AbsEncGetAngle( qc0 , NA ) ;  // lecture des positions initiales des axes
    #else
    MatSetf( qc0 , NA , 1 , 0.25f ) ; // rad init de q0 par exemple
    MatCpyf( qc , qc0 , NA , 1 ) ;
    _Qcp = qc ;
    #endif

    tf = t0 + dureeTraj ;

    MatSubf( pente , qcf , qc0 , NA , 1 ) ;    // pente = ( qf - q0 )
    MatMulScalf( pente , (Reel) (1. / dureeTraj) , pente , NA , 1 ) ; // pente = ( qf - q0 ) / dureeTraj

    initFait = 1 ;
} // if ( !initFait)

else

{ // Ne passe pas au 1er appel qui est fait dans le main mais on est déjà en q0
    double t ; // s temps de l'échantillon
    //------------------
    t = TempsTickToS( McrTimerGetCount( NULL ) ) ; // Lecture du temps

    if ( tf - t <= Te ) MatCpyf( qc , qcf , NA , 1 ) ; // se répète pour asservir à qcf
    else
    {
        // Génération de la trajectoire linéaire sur dureeTraj s.
        // qc = ( (qcf - qc0) / dureeTraj ) * (t-t0) + qc0 = a * (t-t0) + qc0 
        float temps ;
        //------------------
        temps = (float) ( t - t0 ) ;
        MatMulScalf( qc , temps , pente , NA , 1 ) ;
        MatAddf( qc , qc , qc0 , NA , 1 ) ;
    }

    #if !AsservQ 
    QToPoulies( mc , qc ) ;
    #endif

    #if Observateur
    {
    int i ;
    //------------------
    for ( i = NA ; i-- ; )
    {
        ObsrvFloat(  2 + i , qc[i] ) ;
        #if !AsservQ 
        ObsrvFloat( 14 + i , mc[i] ) ;
        #endif
    } // for i
    }
    #endif Observateur

} // else de if ( ! initFait )

// return inutilisé au 1er appel dans le main.
#if AsservQ 
return qc ;
#else
return mc ;
#endif
} // MC
#endif // MCPosition


#if MCReglageGainsSin
/*
% Tracé mc.m

close all
clear all

RadToDeg = 180. / pi

load t

load mc0
load mc1
load mc2
load mc3
load mc4
load mc5

load m0
load m1
load m2
load m3
load m4
load m5
hold
plot( t , mc0 * RadToDeg , 'r' ) 
plot( t , mc1 * RadToDeg , 'r' ) 
plot( t , mc2 * RadToDeg , 'r' ) 
plot( t , mc3 * RadToDeg , 'r' ) 
plot( t , mc4 * RadToDeg , 'r' ) 
plot( t , mc5 * RadToDeg , 'r' )

plot( t , m0 * RadToDeg , 'k' ) 
plot( t , m1 * RadToDeg , 'k' ) 
plot( t , m2 * RadToDeg , 'k' ) 
plot( t , m3 * RadToDeg , 'k' ) 
plot( t , m4 * RadToDeg , 'k' ) 
plot( t , m5 * RadToDeg , 'k' )
% c cyan ; m magenta 
legend( 'mc0' , 'mc1' , 'mc2' , 'mc3' , 'mc4' , 'mc5' )
set(gcf , 'name' , 'mc' )
title( 'mc' )
xlabel( 't (s)' )
ylabel( 'mc (°)' )

figure

load mc6
load mc7
load mc8
load mc9
load mc10
load mc11
hold
plot( t , mc6 * RadToDeg , 'r' ) 
plot( t , mc7 * RadToDeg , 'r' ) 
plot( t , mc8 * RadToDeg , 'r' ) 
plot( t , mc9 * RadToDeg , 'r' ) 
plot( t , mc10 * RadToDeg , 'r' ) 
plot( t , mc11 * RadToDeg , 'r' )
% c cyan ; m magenta 
legend( 'mc6' , 'mc7' , 'mc8' , 'mc9' , 'mc10' , 'mc11' )
set(gcf , 'name' , 'mc' )
title( 'mc' )
xlabel( 't (s)' )
ylabel( 'mc (°)' )

load code
code

*/
#define AsservQ 0 // imposé
#define CodeursCNA 1 // imposé
#define TimeOutTrajectoire 15. // double s le temps prévu pour la trajectoire est dépassé
//----------------------------------------------------------
float *MC( int _numMoteur )
/*
Fonction : génére une trajectoire sinusoidale, autour de la position initiale, pour un seul moteur ou poulie.

Entrée   : _numMoteur = numéro du moteur.

Sortie   : pointeur sur le tableau des consignes moteurs.
           Les consignes des moteurs autres que _numMoteur restent à leur valeur initiale.
*/

{
static float mc[NA] ,      // rad
             m0NumMoteur ; // rad position poulie à l'instant 0

static double  t0 ; // s temps à l'instant 0

static int initFait = 0 ;
//------------------
if ( !initFait)
{ // Appel une seule fois dans le main
    printf( "\nReglage des gains avec sin %f s\n" , TimeOutTrajectoire ) ; 
    t0 = TempsTickToS( McrTimerGetCount( NULL ) ) ; // Lecture du temps t0

    {
    float q0[NA] ;
    //------------------
    AbsEncGetAngle( q0 , NA ) ;  // lecture des positions initiales des axes
    QToPoulies( mc , q0 ) ;
    }

    m0NumMoteur = mc[_numMoteur] ;

    initFait = 1 ;
}

else

{ // Ne passe pas au 1er appel fait dans le main mais on est déjà en m0
    static const float a = 0.3f ; // rad amplitude SK

    #define Pi 3.1415926535

    static const double deuxPi      = 2. * Pi ,
                        deuxPiSurTi = 2. * Pi / 5. ; // Ti période du sin en s

    double t ; // s temps de l'échantillon
    //------------------
    t = TempsTickToS( McrTimerGetCount( NULL ) ) ; // Lecture du temps
    mc[_numMoteur] = m0NumMoteur + a * (float) sin( fmod( ( t - t0 ) * deuxPiSurTi , deuxPi ) ) ;

    #if Observateur
    {
    int i ;
    //------------------
    for ( i = NA ; i-- ; )
    {
//        ObsrvFloat(  2 + i , qc[i] ) ;
//        #if !AsservQ 
        ObsrvFloat( 14 + i , mc[i] ) ;
//        #endif
    } // for i
    }
    #endif Observateur

} // else de if ( ! initFait )

return mc ; // toujours mc
} // MC
#endif // MCReglageGainsSin


#if MCReglageGainsEch
#define AsservQ 0 // imposé
//#define CodeursCNA 1 // imposé
#define TimeOutTrajectoire 5. // double s le temps prévu pour la trajectoire est dépassé
//----------------------------------------------------------
float *MC( int _numMoteur )
/*
Fonction : génére un échelon, à partir de la position initiale, pour un seul moteur ou poulie.

Entrée   : _numMoteur = numéro du moteur.

Sortie   : pointeur sur le tableau des consignes moteurs.
           Les consignes des moteurs autres que _numMoteur restent à leur valeur initiale.
*/

{
static float mc[NA] ; // rad

static int initFait = 0 ;
//------------------
if ( !initFait)
{ // Appel une seule fois dans le main
    printf( "\nReglage des gains avec échelon %f s\n" , TimeOutTrajectoire ) ; 

    {
    float q0[NA] ;
    //------------------
    AbsEncGetAngle( q0 , NA ) ;  // lecture des positions initiales des axes
    QToPoulies( mc , q0 ) ;
    }

    {
    static const float a = 0.05f ; // rad amplitude SK        
    //------------------
    mc[_numMoteur] += a ;
    }

    initFait = 1 ;
}

else

{ // Ne passe pas au 1er appel fait dans le main mais on est déjà en m0


    #if Observateur
    {
    int i ;
    //------------------
    for ( i = NA ; i-- ; )
    {
//        ObsrvFloat(  2 + i , qc[i] ) ;
//        #if !AsservQ 
        ObsrvFloat( 14 + i , mc[i] ) ;
//        #endif
    } // for i
    }
    #endif Observateur

} // else de if ( ! initFait )

return mc ; // toujours mc
} // MC
#endif // MCReglageGainsEch


#if MCSin
#define TimeOutTrajectoire 15. // double s le temps prévu pour la trajectoire est dépassé
//----------------------------------------------------------
float *MC( int _nonUtilise )
/*
Fonction : génére, pour qc, une trajectoire sinusoidale, autour de la position initiale.
           Passage en consignes moteurs. 

Retour   : pointeur sur le tableau des consignes moteurs (mc).
*/
{
static float mc[NA] ,  // rad
             qc0[NA] , // rad position des axes à l'instant 0
             qc[NA] ;  // rad

static double  t0 ; // s temps à l'instant 0

static int initFait = 0 ;
//------------------
if ( !initFait)
{ // Appel une seule fois dans le main
    printf( "\nConsignes en sin %f s\n" , TimeOutTrajectoire ) ; 
    t0 = TempsTickToS( McrTimerGetCount( NULL ) ) ; // Lecture du temps t0

    #if CodeursCNA
    AbsEncGetAngle( qc0 , NA ) ;  // lecture des positions initiales des axes
    #else
    MatSetf( qc0 , NA , 1 , 0.f ) ; // rad init de qc0
    MatCpyf( qc , qc0 , NA , 1 ) ;
    _Qcp = qc ;
    #endif

    initFait = 1 ;
} // if ( !initFait)

else

{ // Ne passe pas au 1er appel qui est fait dans le main mais on est déjà en m0
    #if 0
    // Oscille plateau horizontal , part en avant
    static const float a[NAMax] = { 0.f , 0.05f , 0.f , 0.f , -0.05f , 0.f , // rad amplitude
                                    0.f , 0.05f , 0.f , 0.f , -0.05f , 0.f } ;
//    static const float a[NAMax] = { 0.f , 0.50f , 0.f , 0.f , -0.00f , 0.f , // rad amplitude
//                                    0.f , 0.00f , 0.f , 0.5f ,  0.00f , 0.f } ;
    #else

    // Flexions plateau horizontal
    static const float a[NAMax] = { 0.f , -0.045f , 0.f , 0.09f , -0.045f , 0.f , // rad amplitude
                                    0.f , -0.045f , 0.f , 0.09f , -0.055f , 0.f } ; 
    #endif

    #define Pi 3.1415926535

    static const double deuxPi      = 2. * Pi ,
                        deuxPiSurTi = 2. * Pi / 5. ; // Ti période du sin en s

    double t ; // s temps de l'échantillon

    float sinAlpha ;
    //------------------
    t = TempsTickToS( McrTimerGetCount( NULL ) ) ; // Lecture du temps

    sinAlpha = (float) sin( fmod( ( t - t0 ) * deuxPiSurTi , deuxPi ) ) ;
    MatMulScalf( qc , sinAlpha , a , NA , 1 ) ;
    MatAddf( qc , qc , qc0 , NA , 1 ) ;

    #if !AsservQ 
    QToPoulies( mc , qc ) ;
    #endif

    #if Observateur
    {
    int i ;
    //------------------
    for ( i = NA ; i-- ; )
    {
        ObsrvFloat(  2 + i , qc[i] ) ;
        #if !AsservQ 
        ObsrvFloat( 14 + i , mc[i] ) ;
        #endif
    } // for i
    }
    #endif Observateur

} // else de if ( ! initFait )

// return inutilisé au 1er appel dans le main.
#if AsservQ 
return qc ;
#else
return mc ;
#endif
} // MC
#endif // MCSin


#if MCTrajectoire
#define NbBoucles 1 // nb de boucles du fichier Qcycle
#define TimeOutTrajectoire 30. // 40.  // en double s le temps prévu pour la trajectoire est dépassé
// 5 ms + 5 s de zéro + 6.15 Init + 3 pas de 6 SLF = 29.155
// 5 ms + 5 s de zéro + 2.005 Init + 2 pas de 4.010 = 15.03
// Avec les fichiers Q*4.mat
// 5 ms + 5 s de zéro + 2.005 Init + n pas de 20.005 + 2.005 Arret= 15.03 
//----------------------------------------------------------
float *MC(  int _nonUtilise  )
/*
Fonction : calcul des NA consignes moteurs en fonction de la trajectoire.

Retour : pointeur sur le tableau des NA consignes moteurs. (sauf au premier appel dans le main)
*/
{
#define TailleMax 5000 // * 5ms soit 25s (ou 2000 pour 10s)
#define NAFichiers 12 // NA dans les fichiers de Sébastien le Floch (SLF)


static float tabInit[TailleMax*NAFichiers] ,
             tabCycle[TailleMax*NAFichiers] ,
             tabArret[TailleMax*NAFichiers] ,
             *qcp ; // pointeur sur un tableau des NA consignes de q

static int nb     ,  // nombre d'échantillons dans le fichier Qinit puis général
           nbCycle , // nombre d'échantillons dans le fichier Qcycle
           nbArret ; // nombre d'échantillons dans le fichier Qarret

static float mc[NA] ,
             qc0[NA] , // rad
             qc[NA] ;  // rad

static int phase = 0 ;
//------------------
switch ( phase )
{
    case 0 : // Lecture des fichiers de points. Appel une seule fois dans le main
        printf( "\nTaille max des phases %d, duree %f s" , TailleMax  , (float) TailleMax * 0.005f ) ; 
        nb      = MatLabRd( tabInit  , "d:\\Mb\\Sherpa" , "Qinit4"  ) / NAFichiers ;
        printf( "\nTaille de la phase Init  %d, duree %f s" , nb      , (float) nb      * 0.005f ) ; 
        nbCycle = MatLabRd( tabCycle , "d:\\Mb\\Sherpa" , "Qcycle4" ) / NAFichiers ;
        printf( "\nTaille de la phase Cycle %d, duree %f s" , nbCycle , (float) nbCycle * 0.005f ) ;
  
        nbArret = MatLabRd( tabArret , "d:\\Mb\\Sherpa" , "Qarret4" ) / NAFichiers ;
        printf( "\nTaille de la phase Arret %d, duree %f s" , nbArret , (float) nbArret * 0.005f ) ;
  
        printf( "\n" ) ;

// Affichage Matlab
// load Qcycle4
// plot (Qcycle(5:12:end)) pour qc4

        #if CodeursCNA
        AbsEncGetAngle( qc0 , NA ) ;  // lecture des positions initiales des axes
        #else
        MatSetf( qc0 , NA , 1 , 0.25f ) ; // rad init de q0 par exemple
        MatCpyf( qc , qc0 , NA , 1 ) ;
        #endif

        qcp  = qc ; // init

        // phase = 1 ; // indic mis à 1 plus bas
    break ;

    case 1 : // Retour à la position 0
    {
        static int ne = 1000 ; // 5 s nb d'échantillons pour le passage des qi au démarrage (qi0) aux qi = 0.f
        //------------------
        // On est déjà en q0 d'où le -- seulement après ne
        if ( ne-- ) MatMulScalf( qc , (float) ne * 0.001f , qc0 , NA , 1 ) ; // calcul avec de 999 à 0
        else
        { // Passer au 1er point du fichier Qinit
            qcp = tabInit ; // Suposé être aussi au zéro mais ne l'est pas dans le fichier $$
            phase = 2 ;     // phase suivante
        } // if ( ne-- )
    }
    break ;

    case 2 : // Initialisation, 1er pas. 
        // On a déjà fait le point zéro d'ou le -- avant nb
        if ( --nb ) qcp += NAFichiers ;
        else
        { // Passer au début du fichier Qcycle
            qcp = tabCycle ;
            nb  = nbCycle ;
            phase = 3 ;     // phase suivante
        } // if ( --nb )
    break ;

    case 3 : // Pas normal G puis D. 
        // On a déjà fait le point zéro d'ou le -- avant nb
        if ( --nb ) qcp += NAFichiers ; // on fait toujours le Cycle une fois !!!
        else
        {
            {
            static int nbBoucles = NbBoucles ;
            //------------------     
            if ( --nbBoucles ) // si le raccord est bon entre Qinit et Qarret on peut ne pas faire de Qcycle NbPas 0 PAS FAIT
            { // Repasser au début du fichier Qcycle
                qcp = tabCycle ;
                nb  = nbCycle ;
            }
            else
            { // Passer au début du fichier Qarret
                qcp = tabArret ;
                nb  = nbArret ;
                phase = 4 ;     // phase suivante
            } // if ( --nbBoucles )
            }
        } // if ( --nb )
    break ;

    case 4 : // Dernier pas d'arrêt. 
        // On a déjà fait le point zéro d'ou le -- avant nb
        if ( --nb ) qcp += NAFichiers ;
        else phase = 5 ;     // phase suivante
    //break ;

    //case 5 : // Asservi au repos sur le dernier point de Qarret. 

} // switch ( phase )

#if !CodeursCNA
_Qcp = qcp ; // pour simu
#endif

if ( !phase ) phase = 1 ;
else
{ // Appel dans Asserv seulement, mais pas dans l'appel du main
    #if !AsservQ
    QToPoulies( mc , qcp ) ;
    #endif

    #if Observateur
    {
    int i ;
    //------------------
    for ( i = NA ; i-- ; )
    {
        ObsrvFloat(  2 + i , qcp[i] ) ;
        #if !AsservQ 
        ObsrvFloat( 14 + i , mc[i] ) ;
        #endif
    } // for i
    }
    #endif Observateur
} // if ( !phase )

// return inutilisé au 1er appel dans le main.
#if AsservQ 
return qcp ;
#else
return mc ;
#endif
} // MC
#endif // MCTrajectoire





//----------------------------------------------------------
//----------------------------------------------------------
void Asserv()
// Tâche périodique d'asservissement des moteurs.
{
TimeTick t ; // temps au début de la période

float
#if AsservQ
      gammaQ[NA] ,
      gammaP[NA] ,
#endif
      *qmc ,   // rad consignes axes ou moteurs (poulies)
      *qm ,    // rad positions axes ou moteurs (poulies)

      c[NA] ; // +/- 10 V  commande

#if ATI
float ft[NATI * 6] ; // ATI
#endif
//------------------
#if SecuChienEnable
McrWatchDogRestart() ;   // relance le chien pour 6 ms, pour maintenir les vars enables
#endif

McrTimerGetCount( &t ) ; // Lecture du temps au début de la tâche Asserv

qm = Mesures() ; // lectures codeurs (ou poulies)

#if ATI
ATIGetFT( ft , NATI ) ; // lecture ATI
#endif

// Récup des consignes calculées pour cet échantillon (axes ou poulies).
qmc = MC( NumAxe ) ; // argument utilisé si un seul axe asservi 

{ // Asservissement des axes ou des poulies.
static float ePre[NA] ,  // erreur de position Precédente
             eInte[NA] ; // intégrale de l'erreur de position

static int initFait = 0 ;
//------------------
if ( !initFait)
{
    MatSubf( ePre , qmc , qm , NA , 1 ) ; // pour e - ePre = 0 au 1er coup
    MatClrf( eInte , NA , 1 ) ;           // pour eInte    = 0 au 1er coup
    initFait = 1 ;
}

// Il ne faut pas de else

{ // NA asservissements
#define SEP 0.3f // Seuil Erreur de poursuite
 
float e ; // erreur de position

int i ;
//------------------
for ( i = NA ; i-- ; )
{
    e = qmc[i] - qm[i] ; // non enregistré, sous Matlab faire plot( qmc - qm )

    { // Test erreur de poursuite. Modifier suivant que c'est q ou m !!!!!!!!!!!!!!!!!!!  
    static const float
    #if 1 /////////////////////
//    seuil[NAMax] = { 0.01f , 0.01f , 0.01f , 0.01f , 0.01f , 0.01f ,   // 0 à 5 rad
//                     0.01f , 0.01f , 0.01f , 0.01f , 0.01f , 0.01f } ; // 6 à 11
//    seuil[NAMax] = { 0.1f , 0.1f , 0.1f , 0.1f , 0.1f , 0.1f ,   // 0 à 5 rad
//                     0.1f , 0.1f , 0.1f , 0.1f , 0.1f , 0.1f } ; // 6 à 11
    seuil[NAMax] = { SEP , SEP , SEP , SEP , SEP , SEP ,   // 0 à 5 rad
                     SEP , SEP , SEP , SEP , SEP , SEP } ; // 6 à 11
    #else
    seuil[NAMax] = { 0.125f , 0.125f , 0.125f , 0.125f , 0.125f , 0.125f ,   // 0 à 5
                     0.125f , 0.125f , 0.125f , 0.125f , 0.125f , 0.125f } ; // 6 à 11
    #endif
    //------------------
    if ( fabs( (double) e ) > seuil[i] ) Erreur( ErrPoursuite + i ) ;
    } // Test erreur de poursuite.

    { // Correcteur PID
    #if AsservQ

    static const float // gains
    kp[NAMax] = { 150.f , 150.f , 100.f , 100.f , 150.f , 150.f ,   // 0 à 5  6 lignes Gains q REVOIR commande en couple q
                  150.f , 150.f , 100.f , 100.f , 150.f , 150.f } , // 6 à 11
    ki[NAMax] = {   1.f ,   1.f ,   1.f ,   1.f ,   1.f ,   1.f ,   // 0 à 5
                    1.f ,   1.f ,   1.f ,   1.f ,   1.f ,   1.f } , // 6 à 11
    kd[NAMax] = { 500.f , 500.f , 200.f , 200.f , 500.f , 500.f ,   // 0 à 5
                  500.f , 500.f , 200.f , 200.f , 500.f , 500.f } ; // 6 à 11
    //------------------
    gammaQ[i] = kp[i] * e  +  ki[i] * eInte[i]  +  kd[i] * ( e - ePre[i] ) ; // en rad et en Nm

    #else // Asservissement PID des poulies

    static const float // gains
    kp[NAMax] = { 150.f , 150.f , 100.f , 100.f , 150.f , 150.f ,   // 0 à 5  SK * 6 lignes Gains m commande en volt
                  150.f , 150.f , 100.f , 100.f , 150.f , 150.f } , // 6 à 11
    ki[NAMax] = {   1.f ,   1.f ,   1.f ,   1.f ,   1.f ,   1.f ,   // 0 à 5
                    1.f ,   1.f ,   1.f ,   1.f ,   1.f ,   1.f } , // 6 à 11
    kd[NAMax] = { 500.f , 500.f , 200.f , 200.f , 500.f , 500.f ,   // 0 à 5
                  500.f , 500.f , 200.f , 200.f , 500.f , 500.f } ; // 6 à 11
    //------------------
    c[i] = kp[i] * e  +  ki[i] * eInte[i]  +  kd[i] * ( e - ePre[i] ) ; // en rad et en V

    #endif
    } // Correcteur PID


#if AsservQ /// !!!!!!!!!!!!!!!!! est dans le for i !!!!!!!!!!!!! pas bon
// On a des commandes en couple sur q en Nm
{ // Saturation pour VB (mesures ATI dans ft[NATI * 6]
}

/* Passage en couple sur m en Nm
Je préfère qu'on dise gammaP pour poulie, sachant qu'avec cette convention gammaP=Rr*gammeM
Ce qui est sûr c'est que Couple Flexion = Couple poulie droite + Couple poulie gauche.
Donc pour moi l'expression suivante est juste :
gammaQ[0] = - gammaP[0] + gammaP[1] ;
gammaQ[1] =   gammaP[0] + gammaP[1] ; <-Couple Flexion = Couple poulie droite + Couple poulie gauche
*/
QToPoulies( gammaP , gammaQ ) ;
MatMulScalf( gammaP , 0.5f , gammaP , 1 , NA ) ;

{ // Passage en Volts
//Rendement = 1.f
#define Pif 3.1415926535f
#define Rr ( 8.f * Pif )    // rapport du réducteur moteur vers poulie
#define Kci 0.17f // Nm/A coeff de couple ( +/- 1.003 Nm sur l'axe moteur ou 25.2 Nm sur une poulie)
#define Imax 5.9f // A Courant max du variateur
#define Vmax 10.f // V Consigne = +/- 10 Volt  
#define Kcv ( Imax / Vmax ) // laisser les ( ) c'est mieux mais correct sans dans Cte =

const float Cte = 1.f / ( Rr * Kci * Kcv ) ; // 1. / (Nm/A) (A/V) = V/Nm

int i ;
//------------------
for ( i = NA ; i-- ; ) c[i] = gammaP[i] * Cte ;
//MatMulScalf( c , Cte , gammaP , 1 , NA ) ; // 10/09/10 MB faire fusion avec le 0.5f plus haut
} // Passage en V
#endif

    #if Observateur
    if ( i == NumAxe )
    {
    ObsrvFloat( 63 , eInte[i] ) ;
    ObsrvFloat( 64 , ( e - ePre[i] ) ) ; // plot( diff(mc - m) )
    } // if ( i == NumAxe )
    #endif //Observateur

    eInte[i] += e ; // Intégrale de l'erreur pour l'échantillon suivant

    { // Saturation de l'erreur Intégrale
    static const float eInteMax = 0.1f ; // rad saturation de l'erreur Intégrale REVOIR suivant que c'est q ou m !!!!!!!!!!!!!
    //------------------
    if (      eInte[i] >   eInteMax ) eInte[i] =   eInteMax ; // saturation de l'intégrale de l'erreur
    else if ( eInte[i] < - eInteMax ) eInte[i] = - eInteMax ;
    } // Saturation de l'erreur Intégrale

    ePre[i] = e ; // pour l"échantillon suivant

} // for ( i = NA ; i-- ; )

} // NA asservissements
} // Asservissement. 


#if CodeursCNA
if (!_NbErreur ) McrCNASherpa( c , NA ) ; // +/- 10 V Commande si pas d'erreurs.
#endif

#if !AsservQ
{ // Enregistrement des couples articulaires qui n'étaient pas calculés.
/* Fichier visu.m sous MatLab
% Tracé GammaQ.m

close all
clear all

load code
code

load t
load duree

hold

load gammaQ0
load gammaQ1
load gammaQ2
plot( t , gammaQ0 , 'k' )
plot( t , gammaQ1 , 'r' )
plot( t , gammaQ2 , 'b' )
title( 'Couples articulaires Droite' )
xlabel( 't (s)' )
ylabel( 'gammaQ (Nm)' )
set(gcf , 'name' , 't (s) , gammaQ0à2 (Nm)' )
legend( 'gammaQ0' , 'gammaQ1' , 'gammaQ2' )

figure
hold

load gammaQ3
load gammaQ4
load gammaQ5
plot( t , gammaQ3 , 'k' )
plot( t , gammaQ4 , 'r' )
plot( t , gammaQ5 , 'b' )
title( 'Couples articulaires Droite' )
xlabel( 't (s)' )
ylabel( 'gammaQ (Nm)' )
set(gcf , 'name' , 't (s) , gammaQ3à5 (Nm)' )
legend( 'gammaQ3' , 'gammaQ4' , 'gammaQ5' )

figure
hold on

load fx1
load fy1
load fz1
plot( t , fx1 , 'k' )
plot( t , fy1 , 'r' )
plot( t , fz1 , 'b' )
title( 'ATI F Gauche' )
xlabel( 't (s)' )
ylabel( 'F (N)' )
set(gcf , 'name' , 't (s) , F (N)' )
legend( 'fx1' , 'fy1' , 'fz1' )

figure
hold

load tx1
load ty1
load tz1
plot( t , tx1 , 'k' )
plot( t , ty1 , 'r' )
plot( t , tz1 , 'b' )
title( 'ATI T Gauche' )
xlabel( 't (s)' )
ylabel( 'T (Nm)' )
set(gcf , 'name' , 't (s) , T (Nm)' )
legend( 'tx1' , 'ty1' , 'tz1' )
*/

/* Explications de SK le 15/04/10
Q[0] = 0.5f * ( - P[0] + P[1] ) ;
Q[1] = 0.5f * (   P[0] + P[1] ) ;
Car inverser la matrice fait passer de P à Q.


Si j'appelle M la matrice [-1 1;1 1].
alors P= M*Q devient Q=M^-1*P

NB : ^T veut dire transposée.

Pour passer de P, Q à gP, gQ il faut écrire la conservation de la puissance des Q aux P :
P^T* gP = Q^T* gQ


En remplaçant P^T par Q^T*M^T j'obtiens :
Q^T*M^T*gP = Q^T* gQ
En simplifiant pat Q^T, on obtient alors :
gQ=M^T*gP
soit :


gammaQ[0] = - gammaP[0] + gammaP[1] ;
gammaQ[1] =   gammaP[0] + gammaP[1] ;


car M^T vaut M ici...
*/
//Rendement = 1.f
#define Pif 3.1415926535f
        #define Rr ( 8.f * Pif )    // rapport du réducteur moteur vers poulie
        #define Kci 0.17f // Nm/A coeff de couple
        #define Imax 5.9f // A Courant max du variateur
        #define Vmax 10.f // V Consigne = +/- 10 Volt  
        #define Kcv ( Imax / Vmax ) // laisser les ( ) c'est mieux mais correct sans dans Cte =

        const float Cte = Rr * Kci * Kcv ; // (Nm/A) (A/V) = Nm/V

float gammaP[NA] ,
      gammaQ[NA] ;

int i ;
//------------------
MatMulScalf( gammaP , Cte , c , 1 , NA ) ;

{ // Gamma Poulies to Q il ne faut pas le 1/2
gammaQ[0] = - gammaP[0] + gammaP[1] ;
gammaQ[1] =   gammaP[0] + gammaP[1] ;

gammaQ[2] =   gammaP[2] - gammaP[3] ;
gammaQ[3] =   gammaP[3] + gammaP[3] ; 

gammaQ[4] = - gammaP[4] - gammaP[5] ;
gammaQ[5] = - gammaP[4] + gammaP[5] ;


#if NA == 12
gammaQ[6]  = - gammaP[6] + gammaP[7] ;
gammaQ[7]  =   gammaP[6] + gammaP[7] ; 

gammaQ[8]  =   gammaP[8] - gammaP[9] ;
gammaQ[9]  =   gammaP[8] + gammaP[9] ; 

gammaQ[10] = - gammaP[10] - gammaP[11] ;
gammaQ[11] = - gammaP[10] + gammaP[11] ;
#endif
} // Gamma Poulies to Q

for (i = NA ; i-- ; ) ObsrvFloat( 79 + i , gammaQ[i] ) ;
} // Enregistrement des couples articulaires qui n'était pas calculés.
#endif


// Plusieurs tests d'arrêt après la commande.

#if BoutonArret // = 1 si push poussoir (sur le bit 1 carte 0)
// Si pas de poussoir il faut un strap ou mettre le define BoutonArret à 0.
if ( McrGetTTL() & 0x2 ) Erreur( ErrArretPoussoir ) ;
#endif

{ // Tests de la période et de la durée de la tâche.
static double tDeb = 0. ; // s temps au début de la tâche Asserv
//------------------

#if TempsReel
{ // Test de la période la tâche.
double tDebLocal ;
//------------------
tDebLocal = TempsTickToS( t ) ; // passage en s et en double
#if 1 ////////////////////
if ( tDeb ) // pas de test au 1er passage
    if ( fabs( tDebLocal - tDeb - Te ) >= TimeOutPeriode ) Erreur( ErrTimeOutPeriode ) ;
#endif
tDeb = tDebLocal ;
#if Observateur
ObsrvFloat(  1 , (float) tDeb ) ; // affichage de la période avec avec plot( diff(t) )
#endif
} // Test de la période la tâche.
#else // PAS FAIT
#endif // TempsReel

{ // Test de la durée la tâche. Valable en temps réel ou pas.
double dureeTache ; // duree de la tâche Asserv
//------------------
dureeTache = TempsTickToS( McrTimerGetCount( NULL ) ) - tDeb ; // duree de la tâche
if ( dureeTache > TimeOutTache ) Erreur( ErrTimeOutTache ) ;
#if Observateur
ObsrvFloat( 66 , (float) dureeTache ) ;
#endif
} // Test de la durée la tâche.

} // Tests de la période et de la durée de la tâche.

{ // Test de la durée de la trajectoire.
#if TempsReel
if ( TempsTickToS( McrTimerGetCount( NULL ) ) >= TimeOutTrajectoire ) Erreur( ErrTimeOutTrajectoire ) ; // si le temps indiqué est dépassé
#else
// if ( _Temps  faire un test d'arrêt sur le numEchant nombre de passages dans la boucle A REVOIR
#if 0
{voir si pas en temps réel
static int echant = 0 ;
//------------------
echant++ ;
ObsrvFloat( 1 , (float) echant * 0.005f ) ; // Te
}
#endif
#endif // TempsReel
} // Test de la durée de la trajectoire.


{ // Enregistrement des variables
#if Observateur
int i ;
//------------------
for ( i = NA ; i-- ; ) ObsrvFloat( 50 + i , c[i] ) ;
#if ATI
for ( i = NATI * 6 ; i-- ; ) ObsrvFloat( 67 + i , ft[i] ) ;
#endif

#endif // Observateur
} // Enregistrement des variables


#if Avatar
//----------------------------------------------------------

{ // Envoie dans une fifo en mémoire partagée
/*
Entrée : quat quaternion pour l'orientation
*/
#define FifoChannelTx 1
#define FifoTxNbElementMax 45 // suppose que l'affichage suit !!!

typedef struct
{
double temps ;       // en s ; résolution le tick
int status ;
float q[12] , // laisser 12 et non NA au cas ou une seule jambe est asservie
      position[3] , // m
      orientation[3] , // axe
      angle ; // rad
} ObjetVirtuelData ;

static int initFifoFait = 0 ;

//int saveFifoChannel ;

static ObjetVirtuelData data = { 0. , 0 ,
                                 0.f , 0.f , 0.f , 0.f , 0.f , 0.f ,       // q D
                                 0.f , 0.f , 0.f , 0.f , 0.f , 0.f ,       // q G
                                -0.5f , 0.f , 0.5f ,
                                 0.f , 0.f , 1.f , 0.8f } ; // Sherpa Pi/4 de 3/4
//                                 0.f , 0.f , 1.f , 1.57f } ; // Sherpa de profil
//                                 0.f , 0.f , 1.f , 0.f } ; // Sherpa de face
//------------------
        if ( !initFifoFait )
        { // Créer une fifo dans une mémoire partagée.
//            saveFifoChannel = FifoGetChannel() ; // save
            FifoSetChannel( FifoChannelTx ) ;
            FifoSetAddress(
                            SMemGetAddress( "ObjetVirtuelFifoTx" ,
                                            (unsigned long) FifoGetMemSize( sizeof (ObjetVirtuelData) ,
                                                                            FifoTxNbElementMax
                                                                          )
                                          ) 
                          ) ; // ObjetVirtuelData est considéré comme un type, donc il doit être entre ()
            FifoInit( sizeof (ObjetVirtuelData) , FifoTxNbElementMax ) ;
//            FifoSetChannel( saveFifoChannel ) ;  // restore

            initFifoFait = 1 ;
        } // if !initFifoFait

{ // compteur
static int cpt = 1 ; // affichage au premier coup
//------------------
if ( !--cpt )
{
    //saveFifoChannel = FifoGetChannel() ; // save
    //FifoSetChannel( FifoChannelTx ) ;

    //QuatToRot( data.orientation , &data.angle , quat ) ; // décompose le quaternion en axe et angle
    MatCpy( data.q , qm , 1 , NA ) ; // Ranger les q

    FifoPutElt( &data ) ; // range la structure

    //FifoSetChannel( saveFifoChannel ) ;  // restore
    cpt = 8 ; // puis affichage 1 sur 8 donc toutes les 40 ms = 25 images / s
} // if ( !--cpt )
} // compteur

} // Envoie dans une fifo en mémoire partagée
#endif //  Avatar


} // Asserv





//----------------------------------------------------------
//----------------------------------------------------------
int main()
/* Fonction : Tâche de fond.
   Priorité : 7 < à celle de la tâche Asserv
   Boucle avec un sleep pour libérer l'UC, sinon Timeout RTX réglé à 5 s dans RTX Properties, starvation (inanition)
*/

{
#if TacheAsserv
unsigned numTache ; // numéro de la tâche périodique Asserv
#endif
//------------------
ArretVar() ;

#if 0 // Test port sécu
McrPutTTL( 0x2 ) ; // Disable var Carte sécurité. (Enable var à 0V (bit 0 = 1) et Reset var reste à 0V (bit 1 = 1) 
printf( "\nTest Sherpa FIN" ) ;
exit(0) ;
#endif

printf( "\nTest Sherpa 02/03/11 MB" ) ;

#define Priorite 7 // doit être plus basse que celles les tâches Update des capteurs et de l'asservissement.
McrTimerSetCurrentPriority( Priorite ) ; // de la tâche de fond (passe aussi en Win32)

#if ATI
{
/* Lecture ATI
 avant ObsrvInit pour ne pas enregistrer l'offset
 avant Raz Temps car au 1er coup l'initialisation du driver prend du temps 7ms ??? !!! 
*/
float ft[NATI * 6] ; // ATI
//------------------
printf( "\nSherpa doit être déjà suspendu pour mesurer l'offset des capteurs d'effort" ) ;
printf( "\nEt il doit être déjà posé dans 5 s maxi" ) ;
ATIGetFT( ft , NATI ) ; // Init et offset
McrTimerSuspend( 5. ) ; // s ; Attendre pour poser le robot.
printf( "\nSherpa doit être déjà posé pour les mouvements" ) ;
}
#endif


#if Observateur
{ // ObsrvInit Réservation des tableaux en RAM pour les données enregistrées au format .mat (Level 4 MAT-File Format).
/* Fichier visu.m sous MatLab
% Tracé sherpa.m

close all
clear all

RadToDeg = 180. / pi

load code
code

load t

load q4
load q5
load qc4
load qc5
hold
plot( t , q4  * RadToDeg , 'k' ) 
plot( t , q5  * RadToDeg , 'b' ) 
plot( t , qc4 * RadToDeg , 'r' ) 
plot( t , qc5 * RadToDeg , 'g' ) 
legend( 'q4' , 'q5' , 'qc4' , 'qc5' )
set(gcf , 'name' , 'q' )
title( 'q' )
xlabel( 't (s)' )
ylabel( 'q (°)' )

figure

load m4
load m5
load mc4
load mc5
hold
plot( t , m4  * RadToDeg , 'k' ) 
plot( t , m5  * RadToDeg , 'b' ) 
plot( t , mc4 * RadToDeg , 'r' ) 
plot( t , mc5 * RadToDeg , 'g' ) 
legend( 'm4' , 'm5' , 'mc4' , 'mc5' )
set(gcf , 'name' , 'm' )
title( 'm' )
xlabel( 't (s)' )
ylabel( 'm (°)' )

figure

load c4
load c5
hold
plot( t , c4 , 'k' ) 
plot( t , c5 , 'r' ) 
legend( 'c4' , 'c5' )
set(gcf , 'name' , 'ci' )
title( 'ci' )
xlabel( 't (s)' )
ylabel( 'ci (V)' )

figure

load t
plot( diff(t) )
set(gcf , 'name' , 'période' )
title( 'période' )
xlabel( 'ech' )
ylabel( 'période (s)' )

figure

plot( diff( diff(t) ) )
set(gcf , 'name' , 'delta période' )
title( 'delta période' )
xlabel( 'ech' )
ylabel( 'delta période (s)' )



hold

load t
load ref
plot( t   , 'k' )
plot( ref , 'r' )
legend( 't' , 'ref' )

set(gcf , 'name' , 'temps' )
title( 'temps' )
xlabel( 'ech' )
ylabel( 't (s)' )

figure
hold

load low
plot( low )

set(gcf , 'name' , 'low' )
title( 'low' )
xlabel( 'ech' )
ylabel( 'low' )

figure
hold

plot( diff(low) * 0.838095345 )

set(gcf , 'name' , 'diff low' )
title( 'difflow' )
xlabel( 'ech' )
ylabel( 'diff low (µs)' )

load code
code ( 1:end )
*/
/*
% Tracé qc.m

close all
clear all

RadToDeg = 180. / pi

load t

load qc0
load qc1
load qc2
load qc3
load qc4
load qc5

load q0
load q1
load q2
load q3
load q4
load q5
hold
plot( t , qc0 * RadToDeg , 'k' ) 
plot( t , qc1 * RadToDeg , 'r' ) 
plot( t , qc2 * RadToDeg , 'y' ) 
plot( t , qc3 * RadToDeg , 'g' ) 
plot( t , qc4 * RadToDeg , 'b' ) 
plot( t , qc5 * RadToDeg , 'c' )

plot( t , q0 * RadToDeg , 'k' ) 
plot( t , q1 * RadToDeg , 'r' ) 
plot( t , q2 * RadToDeg , 'y' ) 
plot( t , q3 * RadToDeg , 'g' ) 
plot( t , q4 * RadToDeg , 'b' ) 
plot( t , q5 * RadToDeg , 'c' )
% c cyan ; m magenta 
legend( 'qc0' , 'qc1' , 'qc2' , 'qc3' , 'qc4' , 'qc5' )
set(gcf , 'name' , 'qc' )
title( 'qc' )
xlabel( 't (s)' )
ylabel( 'qc (°)' )

figure

load qc6
load qc7
load qc8
load qc9
load qc10
load qc11
hold
plot( t , qc6 * RadToDeg , 'k' ) 
plot( t , qc7 * RadToDeg , 'r' ) 
plot( t , qc8 * RadToDeg , 'y' ) 
plot( t , qc9 * RadToDeg , 'g' ) 
plot( t , qc10 * RadToDeg , 'b' ) 
plot( t , qc11 * RadToDeg , 'c' )
% c cyan ; m magenta 
legend( 'qc6' , 'qc7' , 'qc8' , 'qc9' , 'qc10' , 'qc11' )
set(gcf , 'name' , 'qc' )
title( 'qc' )
xlabel( 't (s)' )
ylabel( 'qc (°)' )

load code
code

*/
#define NbEnr 12000    // nombre d'enregistrements 1 min à 5 ms

static ObsrvVar variables = // Liste des variables à enregistrer sur disque.
{
/* Les dérivées peuvent être calculées sous Matlab avec diff ex: pour la période faire plot(diff(t))
Pour chaque variable : "nom du fichier .mat" , nombre de points enregistrés, code d'enregistrement
code d'enregistrement : 0 = ne pas enregistrer ; 1 = enregistre et 'sature' ; 2 enregistrement 'circulaire'  
*/
"code"   , 100   , 2 , // n0 utilisé pour les codes d'erreur datés (t code t code ...)
"t"      , NbEnr , 2 , // n1 s temps de début de la période, date des enregistrements suivant

"qc0" , NbEnr , 2 , // n2 ; qc rad consigne de position des axes 
"qc1" , NbEnr , 2 ,
"qc2" , NbEnr , 2 ,
"qc3" , NbEnr , 2 ,
"qc4" , NbEnr , 2 ,
"qc5" , NbEnr , 2 ,
"qc6" , NbEnr , 2 ,
"qc7" , NbEnr , 2 ,
"qc8" , NbEnr , 2 ,
"qc9" , NbEnr , 2 ,
"qc10" , NbEnr , 2 ,
"qc11" , NbEnr , 2 ,

"mc0" , NbEnr , 2 , // n14 ; mc rad consigne de position des poulies
"mc1" , NbEnr , 2 ,
"mc2" , NbEnr , 2 ,
"mc3" , NbEnr , 2 ,
"mc4" , NbEnr , 2 ,
"mc5" , NbEnr , 2 ,
"mc6" , NbEnr , 2 ,
"mc7" , NbEnr , 2 ,
"mc8" , NbEnr , 2 ,
"mc9" , NbEnr , 2 ,
"mc10" , NbEnr , 2 ,
"mc11" , NbEnr , 2 ,

"q0" , NbEnr , 2 , // n26 ; q rad position des axes. Pour la vitesse en rad/période faire plot( diff(q0) )
"q1" , NbEnr , 2 ,
"q2" , NbEnr , 2 ,
"q3" , NbEnr , 2 ,
"q4" , NbEnr , 2 ,
"q5" , NbEnr , 2 ,
"q6" , NbEnr , 2 ,
"q7" , NbEnr , 2 ,
"q8" , NbEnr , 2 ,
"q9" , NbEnr , 2 ,
"q10" , NbEnr , 2 ,
"q11" , NbEnr , 2 ,

"m0" , NbEnr , 2 , // n38 ; m  rad position des poulies
"m1" , NbEnr , 2 ,
"m2" , NbEnr , 2 ,
"m3" , NbEnr , 2 ,
"m4" , NbEnr , 2 ,
"m5" , NbEnr , 2 ,
"m6" , NbEnr , 2 ,
"m7" , NbEnr , 2 ,
"m8" , NbEnr , 2 ,
"m9" , NbEnr , 2 ,
"m10" , NbEnr , 2 ,
"m11" , NbEnr , 2 ,

"c0" , NbEnr , 2 , // n50 ; commande V (Volts)
"c1" , NbEnr , 2 ,
"c2" , NbEnr , 2 ,
"c3" , NbEnr , 2 ,
"c4" , NbEnr , 2 ,
"c5" , NbEnr , 2 ,
"c6" , NbEnr , 2 ,
"c7" , NbEnr , 2 ,
"c8" , NbEnr , 2 ,
"c9" , NbEnr , 2 ,
"c10" , NbEnr , 2 ,
"c11" , NbEnr , 2 ,

"tmp0" , NbEnr , 2 , // n62 ; variables temporaires, pour enregistrer des variables intermédiares
"tmp1" , NbEnr , 2 ,
"tmp2" , NbEnr , 2 ,
"tmp3" , NbEnr , 2 ,

"duree" , NbEnr , 2 , // n66 ; s de la tâche Asserv

"fx0" , NbEnr , 2 , // n67
"fy0" , NbEnr , 2 ,
"fz0" , NbEnr , 2 ,
"tx0" , NbEnr , 2 ,
"ty0" , NbEnr , 2 ,
"tz0" , NbEnr , 2 ,
"fx1" , NbEnr , 2 ,
"fy1" , NbEnr , 2 ,
"fz1" , NbEnr , 2 ,
"tx1" , NbEnr , 2 ,
"ty1" , NbEnr , 2 ,
"tz1" , NbEnr , 2 , //

"gammaQ0" , NbEnr , 2 , // 79
"gammaQ1" , NbEnr , 2 ,
"gammaQ2" , NbEnr , 2 ,
"gammaQ3" , NbEnr , 2 ,
"gammaQ4" , NbEnr , 2 ,
"gammaQ5" , NbEnr , 2 ,
"gammaQ6" , NbEnr , 2 ,
"gammaQ7" , NbEnr , 2 ,
"gammaQ8" , NbEnr , 2 ,
"gammaQ9" , NbEnr , 2 ,
"gammaQ10" , NbEnr , 2 ,
"gammaQ11" , NbEnr , 2 ,

"v0" , NbEnr , 2 , // 91 ; ATI V (Volts)
"v1" , NbEnr , 2 ,
"v2" , NbEnr , 2 ,
"v3" , NbEnr , 2 ,
"v4" , NbEnr , 2 ,
"v5" , NbEnr , 2 ,
"v6" , NbEnr , 2 ,
"v7" , NbEnr , 2 ,
"v8" , NbEnr , 2 ,
"v9" , NbEnr , 2 ,
"v10" , NbEnr , 2 ,
"v11" , NbEnr , 2 ,

NULL // fin de tableau
} ;
//------------------
ObsrvInit( variables ) ; //  malloc
} // ObsrvInit

#endif

#if SecuChienEnable
McrPutTTL( 0x2 ) ;         // la sortie TTL Enable/ est mise à 0 et la sortie TTL Reset reste à 1
McrWatchDogRestart() ;     // lance le chien pour 6 ms pour avoir l'entrée Enable des var à 24V
McrPutTTL( 0x0 ) ;         // la sortie TTL Reset passe à 0 pour avoir l'entrée Reset à 24V
McrTimerSuspend( 0.005 ) ; // maintient du Reset 5 ms
McrPutTTL( 0x2 ) ;         // la sortie TTL Reset repasse à 1 pour avoir Enable des var
McrWatchDogRestart() ;     // relance le chien pour 6 ms en attendant la 1er exécution de la tâche Asserv (dans 5ms)
#endif                     //     qui le maintiendra à 0

TempsSetTemps( TempsModeZero ) ; // init temps t à 0 avant d'initialiser MC et de lancer la tâche Asserv

MC( NumAxe ) ; // Initialisation des consignes des 'moteurs' (poulies) : lecture des fichiers de points etc, (retour non utilisé ici)
               // Init de _Qcp en 'simulation'.
               // doit être après la RAZ temps pour le t0

//printf( "\nSTOP" ) ;
//return 0 ; // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#if TacheAsserv
// Lancer la tâche asservissement // Ne fera rien en Win32
numTache = McrTimerOrdonnance( Asserv , Te , Te , 20 ) ; // retard du 1er coup, attention au chien ; 20 = priorité
#endif

{  // Tâche de fond
do
{
#if !TacheAsserv
    Asserv() ;
#endif
#if TempsReel
McrTimerSuspend( 0.001 ) ; //5 ) ; // pour timeOut de Rtx 5 ms trop lent
#endif

#if 1
    { // Affichage du chronomètre à la s
    static float ts = 1.f ;
    float t ;
    //------------------
    t = (float) TempsTickToS( McrTimerGetCount( NULL ) ) ; // Lecture du temps
    if ( t > ts )
    {
        printf( " %3.0f" , t ) ;
        ts += 1.f ;
        { // compteur saut de ligne 
        static int cpt = 10 ;
        //------------------
        if ( !--cpt )
        {
            printf( "\n" ) ;
            cpt = 10 ;
        }
        } // compteur
    }
    } // Affichage du chronomètre à la s
#endif

} while ( !_NbErreur ) ; // sort si erreur dans la tâche Asserv
}  // Tâche de fond

#if TacheAsserv
// Arrêt
McrTimerDesordonnance( numTache ) ; // Ne fera rien en Win32
McrTimerSuspend( 0.05 ) ; // s attendre que la tâche Asserv soit désordonnancée.
#endif

Erreur( NoError ) ; // pour ArretVar() et écraser le fichier code précédent si pas d'erreur
//if ( !_NbErreur ) ArretVar() ; // c'est déjà fait si erreur (le seul arrêt normal c'est fin de trajectoire)

// Durée totale de la trajectoire. (Le temps est mis à 0 au début de la trajectoire).
#if TacheAsserv
printf( "\nDurée %8.4f s" , (float) TempsTickToS( McrTimerGetCount( NULL ) ) - 0.05f ) ; // - 0.05 s du McrTimerSuspend si TacheAsserv = 1
#else
printf( "\nDurée %8.4f s" , (float) TempsTickToS( McrTimerGetCount( NULL ) ) ) ;
#endif

#if Observateur
//ObsrvFloat( 0 , (float) NoError ) ; //  pour écraser le fichier code précédent si pas d'erreur
// Arrêt d'enregistrement et sauvegarde des données de la RAM vers des fichiers disques MATLAB
printf("\nSherpa : Sauvegarde sur disque en cours") ;
ObsrvSave( "d:\\Mat\\E000004" ) ; // pas de répertoire par défaut sous RTX
#endif

if ( _NbErreur ) printf( "\nTests Sherpa Fin : %d erreurs\n" , _NbErreur ) ;
else             printf( "\nTests Sherpa Fin : OK\n" ) ;

return _NbErreur ;
} // main
