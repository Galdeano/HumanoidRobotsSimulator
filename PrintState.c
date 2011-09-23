#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include "uLINK.h"
#include "PrintState.h"



void PrintState(SuLINK uLINK[])
{
    FILE *f=fopen("SherpaState.xml","w");
////    gsl_vector_fprintf(f,tmp,"%.5g");
////    fprintf(f," \n");
    fprintf(f,"<Sherpa>\n");
    int i;
    for(i=1; i<14; i++)
    {
        fprintf(f,"\t<Link no=\"%d\">\n",i);
        fprintf(f,"\t\t<Name>\n");
        fprintf(f,"\t\t\t%s\n",uLINK[i].name);
        fprintf(f,"\t\t</Name>\n");
        fprintf(f,"\t\t<q>\n");
        fprintf(f,"\t\t\t%f\n",uLINK[i].q);
        fprintf(f,"\t\t</q>\n");
        fprintf(f,"\t\t<dq>\n");
        fprintf(f,"\t\t\t%f\n",uLINK[i].dq);
        fprintf(f,"\t\t</dq>\n");
        fprintf(f,"\t\t<ddq>\n");
        fprintf(f,"\t\t\t%f\n",uLINK[i].ddq);
        fprintf(f,"\t\t</ddq>\n");
        fprintf(f,"\t\t<u>\n");
        fprintf(f,"\t\t\t%f\n",uLINK[i].u);
        fprintf(f,"\t\t</u>\n");
        fprintf(f,"\t</Link>\n");
    }
    fprintf(f,"</Sherpa>\n");


//    char name[8];
//    unsigned char sister,child,mother,color,isvertex;
//    double m,q,dq,ddq,Ir,gr,u,u_joint;
//    gsl_vector * a;
//    gsl_vector * b;
//    gsl_vector * c;
//    gsl_vector * p;
//    gsl_vector * v;
//    gsl_vector * vo;
//    gsl_vector * w;
//    gsl_vector * dvo;
//    gsl_vector * dw;
//    gsl_vector * hw;
//    gsl_vector * hv;
//    float com[3];
//    float shape[3];
//    gsl_matrix * R;
//    gsl_matrix * I;

    fclose(f);
}



