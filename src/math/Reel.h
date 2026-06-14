/* Reel.h  Definition of Reel type

17:06 25/07/06 MB V

Reel is defined as double depending on the desired precision.

It is used in the Mat module and declared separately here.

Most of the time the double type is sufficient.
However, using MatPseudoInv with an ill-conditioned matrix may require the double type.
*/

typedef double Reel ;
//typedef double Reel ;
