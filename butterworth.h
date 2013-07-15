#ifndef H_butterworth
#define H_butterworth


#define NZEROS 3
#define NPOLES 3



typedef struct
{
    double xv[NZEROS+1];
    double yv[NPOLES+1];
    double a[NZEROS+1];
    double b[NPOLES+1];
} ButterworthData;


void ButterworthFilterInit (ButterworthData *data,double wc);
void ButterworthFilterValue (ButterworthData *data,double value);

double ButterworthFilter (ButterworthData *data, double input);


#endif
