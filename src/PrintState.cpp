#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <stdio.h>
#include "uLink.h"
#include "Setup.h"
#include "PrintState.h"



void SaveStateXML(SuLINK uLINK[],State *Status,double t)
{
    FILE *f=fopen("SherpaState.xml","w");
////    gsl_vector_fprintf(f,tmp,"%.5g");
////    fprintf(f," \n");
    fprintf(f,"<Sherpa t=\"%f\">\n",t);
    int i;
    for(i=1; i<((Status->ddl)-6+2); i++)
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
        fprintf(f,"\t\t<ug>\n");
        fprintf(f,"\t\t\t%f\n",uLINK[i].ug);
        fprintf(f,"\t\t</ug>\n");
        fprintf(f,"\t\t<uef>\n");
        fprintf(f,"\t\t\t%f\n",uLINK[i].uef);
        fprintf(f,"\t\t</uef>\n");
        fprintf(f,"\t\t<u_joint>\n");
        fprintf(f,"\t\t\t%f\n",uLINK[i].u_joint);
        fprintf(f,"\t\t</u_joint>\n");

        fprintf(f,"\t\t<p>\n");
        fprintf(f,"\t\t\t%f %f %f\n",gsl_vector_get(uLINK[i].p,0),gsl_vector_get(uLINK[i].p,1),gsl_vector_get(uLINK[i].p,2));
        fprintf(f,"\t\t</p>\n");
        fprintf(f,"\t\t<v>\n");
        fprintf(f,"\t\t\t%f %f %f\n",gsl_vector_get(uLINK[i].v,0),gsl_vector_get(uLINK[i].v,1),gsl_vector_get(uLINK[i].v,2));
        fprintf(f,"\t\t</v>\n");
        fprintf(f,"\t\t<vo>\n");
        fprintf(f,"\t\t\t%f %f %f\n",gsl_vector_get(uLINK[i].vo,0),gsl_vector_get(uLINK[i].vo,1),gsl_vector_get(uLINK[i].vo,2));
        fprintf(f,"\t\t</vo>\n");
        fprintf(f,"\t\t<w>\n");
        fprintf(f,"\t\t\t%f %f %f\n",gsl_vector_get(uLINK[i].w,0),gsl_vector_get(uLINK[i].w,1),gsl_vector_get(uLINK[i].w,2));
        fprintf(f,"\t\t</w>\n");
        fprintf(f,"\t\t<dvo>\n");
        fprintf(f,"\t\t\t%f %f %f\n",gsl_vector_get(uLINK[i].dvo,0),gsl_vector_get(uLINK[i].dvo,1),gsl_vector_get(uLINK[i].dvo,2));
        fprintf(f,"\t\t</dvo>\n");
        fprintf(f,"\t\t<dw>\n");
        fprintf(f,"\t\t\t%f %f %f\n",gsl_vector_get(uLINK[i].dw,0),gsl_vector_get(uLINK[i].dw,1),gsl_vector_get(uLINK[i].dw,2));
        fprintf(f,"\t\t</dw>\n");
        fprintf(f,"\t\t<hw>\n");
        fprintf(f,"\t\t\t%f %f %f\n",gsl_vector_get(uLINK[i].hw,0),gsl_vector_get(uLINK[i].hw,1),gsl_vector_get(uLINK[i].hw,2));
        fprintf(f,"\t\t</hw>\n");
        fprintf(f,"\t\t<hv>\n");
        fprintf(f,"\t\t\t%f %f %f\n",gsl_vector_get(uLINK[i].hv,0),gsl_vector_get(uLINK[i].hv,1),gsl_vector_get(uLINK[i].hv,2));
        fprintf(f,"\t\t</hv>\n");

        fprintf(f,"\t\t<R>\n");
        fprintf(f,"\t\t\t%f %f %f, %f %f %f, %f %f %f\n",gsl_matrix_get(uLINK[i].R,0,0),gsl_matrix_get(uLINK[i].R,0,1),gsl_matrix_get(uLINK[i].R,0,2),
                                     gsl_matrix_get(uLINK[i].R,1,0),gsl_matrix_get(uLINK[i].R,1,1),gsl_matrix_get(uLINK[i].R,1,2),
                                     gsl_matrix_get(uLINK[i].R,2,0),gsl_matrix_get(uLINK[i].R,2,1),gsl_matrix_get(uLINK[i].R,2,2));
        fprintf(f,"\t\t</R>\n");

        fprintf(f,"\t</Link>\n");
    }
    fprintf(f,"</Sherpa>\n");

    fclose(f);
}


void SaveRobotXML(SuLINK uLINK[],State *Status)
{
    FILE *f=fopen("SherpaState.xml","w");
////    gsl_vector_fprintf(f,tmp,"%.5g");
////    fprintf(f," \n");
    fprintf(f,"<Robot>\n");
    int i;
        fprintf(f,"\t<DoF>\n");
        fprintf(f,"\t\t%d\n",((Status->ddl)-6));
        fprintf(f,"\t</DoF>\n");
        fprintf(f,"\t<Name>\n");
        fprintf(f,"\t\t%s\n","Generic");
        fprintf(f,"\t</Name>\n");

        fprintf(f,"\t<State>\n");
        fprintf(f,"\t\t<right_foot_ID>\n");
        fprintf(f,"\t\t\t%d\n",Status->right_foot_ID);
        fprintf(f,"\t\t</right_foot_ID>\n");
        fprintf(f,"\t\t<left_foot_ID>\n");
        fprintf(f,"\t\t\t%d\n",Status->left_foot_ID);
        fprintf(f,"\t\t</left_foot_ID>\n");
        fprintf(f,"\t</State>\n");



    for(i=1; i<(Status->ddl-4); i++)
    {
        fprintf(f,"\t<Link no=\"%d\">\n",i);
        fprintf(f,"\t\t<Name>\n");
        fprintf(f,"\t\t\t%s\n",uLINK[i].name);
        fprintf(f,"\t\t</Name>\n");
        fprintf(f,"\t\t<m>\n");
        fprintf(f,"\t\t\t%f\n",uLINK[i].m);
        fprintf(f,"\t\t</m>\n");
        fprintf(f,"\t\t<color>\n");
        fprintf(f,"\t\t\t%d\n",uLINK[i].color);
        fprintf(f,"\t\t</color>\n");
        fprintf(f,"\t\t<sister>\n");
        fprintf(f,"\t\t\t%d\n",uLINK[i].sister);
        fprintf(f,"\t\t</sister>\n");
        fprintf(f,"\t\t<child>\n");
        fprintf(f,"\t\t\t%d\n",uLINK[i].child);
        fprintf(f,"\t\t</child>\n");

        fprintf(f,"\t\t<a>\n");
        fprintf(f,"\t\t\t%f %f %f\n",gsl_vector_get(uLINK[i].a,0),gsl_vector_get(uLINK[i].a,1),gsl_vector_get(uLINK[i].a,2));
        fprintf(f,"\t\t</a>\n");

        fprintf(f,"\t\t<b>\n");
        fprintf(f,"\t\t\t%f %f %f\n",gsl_vector_get(uLINK[i].b,0),gsl_vector_get(uLINK[i].b,1),gsl_vector_get(uLINK[i].b,2));
        fprintf(f,"\t\t</b>\n");

        fprintf(f,"\t\t<p>\n");
        fprintf(f,"\t\t\t%f %f %f\n",gsl_vector_get(uLINK[i].p,0),gsl_vector_get(uLINK[i].p,1),gsl_vector_get(uLINK[i].p,2));
        fprintf(f,"\t\t</p>\n");

        fprintf(f,"\t\t<c>\n");
        fprintf(f,"\t\t\t%f %f %f\n",gsl_vector_get(uLINK[i].c,0),gsl_vector_get(uLINK[i].c,1),gsl_vector_get(uLINK[i].c,2));
        fprintf(f,"\t\t</c>\n");
        fprintf(f,"\t\t<R>\n");
        fprintf(f,"\t\t\t%f %f %f %f %f %f %f %f %f\n",gsl_matrix_get(uLINK[i].R,0,0),gsl_matrix_get(uLINK[i].R,0,1),gsl_matrix_get(uLINK[i].R,0,2),
                                     gsl_matrix_get(uLINK[i].R,1,0),gsl_matrix_get(uLINK[i].R,1,1),gsl_matrix_get(uLINK[i].R,1,2),
                                     gsl_matrix_get(uLINK[i].R,2,0),gsl_matrix_get(uLINK[i].R,2,1),gsl_matrix_get(uLINK[i].R,2,2));
        fprintf(f,"\t\t</R>\n");

        fprintf(f,"\t\t<I>\n");
        fprintf(f,"\t\t\t%f %f %f %f %f %f %f %f %f\n",gsl_matrix_get(uLINK[i].I,0,0),gsl_matrix_get(uLINK[i].I,0,1),gsl_matrix_get(uLINK[i].I,0,2),
                                     gsl_matrix_get(uLINK[i].I,1,0),gsl_matrix_get(uLINK[i].I,1,1),gsl_matrix_get(uLINK[i].I,1,2),
                                     gsl_matrix_get(uLINK[i].I,2,0),gsl_matrix_get(uLINK[i].I,2,1),gsl_matrix_get(uLINK[i].I,2,2));
        fprintf(f,"\t\t</I>\n");

        fprintf(f,"\t</Link>\n");

    }
    fprintf(f,"</Robot>\n");

    fclose(f);
}



void SaveState(SuLINK uLINK[],State *Status,long *t)
{

    SaveLINK *CopyuLINK;
    CopyuLINK = (SaveLINK *)calloc((Status->ddl)+2-6,sizeof(SaveLINK));
    SaveStuctLink(uLINK,CopyuLINK,Status);

    FILE *f=fopen("SherpaState.bin","wb");
    fwrite(CopyuLINK, ((Status->ddl)+2-6) * sizeof(SaveLINK), 1, f);
    fclose(f);

    FILE *ft=fopen("SherpaStateT.bin","wb");
    fwrite(t, sizeof(double), 1, ft);
    fclose(ft);
//Bad idea because of the pointer in the structure Sulink

}

void SaveStuctLink(SuLINK uLINK[],SaveLINK CopyuLINK[],State *Status)
{
    int i,j,k;

    for(i=1; i<((Status->ddl)-6+2); i++)
    {
        CopyuLINK[i].q = uLINK[i].q ;
        CopyuLINK[i].dq = uLINK[i].dq;
        CopyuLINK[i].ddq = uLINK[i].ddq;
        CopyuLINK[i].u = uLINK[i].u;
        CopyuLINK[i].ug = uLINK[i].ug;
        CopyuLINK[i].uef = uLINK[i].uef;
        CopyuLINK[i].u_joint = uLINK[i].u_joint;
        for(j=0;j<3;j++)
        {
            CopyuLINK[i].p[j]=gsl_vector_get (uLINK[i].p, j);
            CopyuLINK[i].v[j]=gsl_vector_get (uLINK[i].v, j);
            CopyuLINK[i].vo[j]=gsl_vector_get (uLINK[i].vo, j);
            CopyuLINK[i].w[j]=gsl_vector_get (uLINK[i].w, j);
            CopyuLINK[i].dvo[j]=gsl_vector_get (uLINK[i].dvo, j);
            CopyuLINK[i].dw[j]=gsl_vector_get (uLINK[i].dw, j);
            CopyuLINK[i].hw[j]=gsl_vector_get (uLINK[i].hw, j);
            CopyuLINK[i].hv[j]=gsl_vector_get (uLINK[i].hv, j);
            for(k=0;k<3;k++)
            {
                CopyuLINK[i].R[3*j+k]=gsl_matrix_get (uLINK[i].R, j,k);
            }
        }
    }
}


void LoadState(SuLINK uLINK[],State *Status,long *t)
{
    SaveLINK *CopyuLINK;
    CopyuLINK = (SaveLINK *)calloc((Status->ddl)+2-6,sizeof(SaveLINK));

    FILE *f=fopen("SherpaState.bin","rb");
    fread(CopyuLINK, ((Status->ddl)+2-6)* sizeof(SaveLINK), 1, f);
    fclose(f);

    LoadStuctLink(uLINK,CopyuLINK,Status);

    FILE *ft=fopen("SherpaStateT.bin","rb");
    fread(t, sizeof(double), 1, ft);
    fclose(ft);
}

void LoadStuctLink(SuLINK uLINK[],SaveLINK CopyuLINK[],State *Status)
{
    int i,j,k;

    for(i=1; i<((Status->ddl)-6+2); i++)
    {
        uLINK[i].q = CopyuLINK[i].q ;
        uLINK[i].dq = CopyuLINK[i].dq;
        uLINK[i].ddq = CopyuLINK[i].ddq;
        uLINK[i].u = CopyuLINK[i].u;
        uLINK[i].ug = CopyuLINK[i].ug;
        uLINK[i].uef = CopyuLINK[i].uef;
        uLINK[i].u_joint = CopyuLINK[i].u_joint;
        for(j=0;j<3;j++)
        {
            gsl_vector_set (uLINK[i].p, j, CopyuLINK[i].p[j]);
            gsl_vector_set (uLINK[i].v, j, CopyuLINK[i].v[j]);
            gsl_vector_set (uLINK[i].vo, j, CopyuLINK[i].vo[j]);
            gsl_vector_set (uLINK[i].w, j, CopyuLINK[i].w[j]);
            gsl_vector_set (uLINK[i].dvo, j, CopyuLINK[i].dvo[j]);
            gsl_vector_set (uLINK[i].dw, j, CopyuLINK[i].dw[j]);
            gsl_vector_set (uLINK[i].hw, j, CopyuLINK[i].hw[j]);
            gsl_vector_set (uLINK[i].hv, j, CopyuLINK[i].hv[j]);
            for(k=0;k<3;k++)
            {
                gsl_matrix_set(uLINK[i].R, j,k,CopyuLINK[i].R[3*j+k]);
            }
        }
    }
}
