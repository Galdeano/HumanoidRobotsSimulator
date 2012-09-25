/* Reel.h  définition du type Reel

17:06 25/07/06 MB V

Reel est défini comme double ou double suivant la précision souhaitée.

Il est utilisé dans le module Mat et déclaré séparément ici.

La plupart du temps le type double est suffisant.
Toutefois l'utilisation de MatPseudoInv avec une matrice mal conditionnée peut nécessiter le type double.
*/

typedef double Reel ;
//typedef double Reel ;
