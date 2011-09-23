/* Reel.h  définition du type Reel

17:06 25/07/06 MB V

Reel est défini comme float ou double suivant la précision souhaitée.

Il est utilisé dans le module Mat et déclaré séparément ici.

La plupart du temps le type float est suffisant.
Toutefois l'utilisation de MatPseudoInv avec une matrice mal conditionnée peut nécessiter le type double.
*/

typedef float Reel ;
//typedef double Reel ;
