
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include "uLINK.h"
#include "TotalMass.h"
#include "FindRoute.h"
#include "Cross.h"
#include "CalcCoMJacobian.h"


void CalcCoMJacobian( SuLINK uLINK[],State *Status, gsl_matrix * J, int base_call)
{
    int i,j,k,l,n,o,err;
    int nbre_elements,nbre_tmp;
    static double mass;


    static gsl_vector * a;
    static gsl_vector * error;
    static gsl_matrix * mc;
    static gsl_vector * m;
    static gsl_matrix * store_mc;
    static gsl_vector * store_m;
    static gsl_vector * tmpv3;
    static gsl_vector * tmpv3_2;
    static gsl_vector * mother;
    static gsl_vector * double_path;
    static int *path;

    static int init_tmp=1;
    if (init_tmp==1)
    {
        a = gsl_vector_calloc (3);
        error = gsl_vector_calloc (3);
        mc = gsl_matrix_calloc((Status->ddl)-3,3);
        m = gsl_vector_calloc((Status->ddl)-3);
        store_mc = gsl_matrix_calloc((Status->ddl)-3,3);
        store_m = gsl_vector_calloc((Status->ddl)-3);
        tmpv3 = gsl_vector_calloc (3);
        tmpv3_2 = gsl_vector_calloc (3);
        mother = gsl_vector_calloc((Status->ddl)-3);
        double_path = gsl_vector_calloc((Status->ddl)-3);
        path=calloc((Status->ddl)+2-6,sizeof(int));

        mass=TotalMass(uLINK,1);
        init_tmp=0;
    }

    //gsl_vector_set_zero(a);
    //gsl_vector_set_zero(error);
    gsl_vector_set_zero(mother);
    //gsl_vector_set_zero(double_path);

    gsl_vector_set_zero(m);
    gsl_vector_set_zero(store_m);
    gsl_vector_set_zero(double_path);
    gsl_matrix_set_zero(mc);
    gsl_matrix_set_zero(store_mc);



    for(i=0; i<(Status->nb_limb); i++)
    {
        if (gsl_vector_get(Status->limbID,i)!=base_call)
        {
            gsl_matrix_set_zero(mc);
            gsl_vector_set_zero(m);
            nbre_elements=FindRoute(uLINK,path,gsl_vector_get(Status->limbID,i));

            for (j=0; j<nbre_elements; j++)
            {
                k=path[j];

                if (j==0)
                {
                    gsl_blas_dgemv(CblasNoTrans, 1.0, uLINK[k].R, uLINK[k].c, 0.0, tmpv3);
                    gsl_vector_add (tmpv3, uLINK[k].p);
                    gsl_vector_scale (tmpv3, uLINK[k].m);
                    gsl_matrix_set_row(mc,k,tmpv3);

                    gsl_vector_set(m,k,uLINK[k].m);
                }
                else
                {
                    l=path[j-1];

                    gsl_blas_dgemv(CblasNoTrans, 1.0, uLINK[k].R, uLINK[k].c, 0.0, tmpv3);
                    gsl_vector_add (tmpv3, uLINK[k].p);
                    gsl_vector_scale (tmpv3, uLINK[k].m);
                    gsl_matrix_get_row(tmpv3_2,mc,l);
                    gsl_vector_add(tmpv3,tmpv3_2);
                    gsl_matrix_set_row(mc,k,tmpv3);

                    gsl_vector_set(m,k,uLINK[k].m+gsl_vector_get(m,l));
                    gsl_vector_set(mother,l,k);

                }

            }
            gsl_matrix_add(store_mc,mc);
            gsl_vector_add(store_m,m);

            if (i==0)
            {
                for (j=0; j<nbre_elements; j++)
                {
                    gsl_vector_set(double_path,path[j],1);
                }
            }

            if (i>0)
            {
                gsl_matrix_set_zero(mc);
                gsl_vector_set_zero(m);
                nbre_tmp=0;
                for (j=0; j<nbre_elements; j++)
                {
                    if (gsl_vector_get(double_path,path[j])==1)
                    {
                        nbre_tmp++;
                    }
                    gsl_vector_set(double_path,path[j],1);
                }

                for (j=0; j<nbre_tmp; j++)
                {
                    k=path[j+(nbre_elements-nbre_tmp)];

                    if (j==0)
                    {
                        gsl_blas_dgemv(CblasNoTrans, 1.0, uLINK[k].R, uLINK[k].c, 0.0, tmpv3);
                        gsl_vector_add (tmpv3, uLINK[k].p);
                        gsl_vector_scale (tmpv3, uLINK[k].m);
                        //gsl_vector_scale (tmpv3, -1.0);
                        gsl_matrix_set_row(mc,k,tmpv3);

                        gsl_vector_set(m,k,uLINK[k].m);
                    }
                    else
                    {
                        l=path[j-1+(nbre_elements-nbre_tmp)];

                        gsl_blas_dgemv(CblasNoTrans, 1.0, uLINK[k].R, uLINK[k].c, 0.0, tmpv3);
                        gsl_vector_add (tmpv3, uLINK[k].p);
                        gsl_vector_scale (tmpv3, uLINK[k].m);
                        //gsl_vector_scale (tmpv3, -1.0);
                        gsl_matrix_get_row(tmpv3_2,mc,l);
                        gsl_vector_add(tmpv3,tmpv3_2);
                        gsl_matrix_set_row(mc,k,tmpv3);

                        gsl_vector_set(m,k,uLINK[k].m+gsl_vector_get(m,l));
                        gsl_vector_set(mother,l,k);

                    }

                }

                gsl_matrix_sub(store_mc,mc);
                gsl_vector_sub(store_m,m);

            }
        }
    }



    if (base_call!=1)
    {

        gsl_matrix_set_zero(mc);
        gsl_vector_set_zero(m);
        nbre_elements=FindRoute(uLINK,path,base_call);

        for (j=1; j<nbre_elements; j++)
        {
            k=path[nbre_elements-j-1];
            l=path[nbre_elements-(j-1)-1];

            if (l==1)
            {
                gsl_matrix_get_row(tmpv3,store_mc,1);
                gsl_matrix_set_row(mc,1,tmpv3);
                gsl_vector_set(m,1,gsl_vector_get(store_m,1));
            }

            gsl_blas_dgemv(CblasNoTrans, 1.0, uLINK[k].R, uLINK[k].c, 0.0, tmpv3);
            gsl_vector_add (tmpv3, uLINK[k].p);
            gsl_vector_scale (tmpv3, uLINK[k].m);
            gsl_matrix_get_row(tmpv3_2,mc,l);
            gsl_vector_add(tmpv3,tmpv3_2);
            gsl_matrix_set_row(mc,k,tmpv3);

            gsl_vector_set(m,k,uLINK[k].m+gsl_vector_get(m,l));
            gsl_vector_set(mother,l,k);



        }
        gsl_vector_set_zero(tmpv3);
        gsl_matrix_set_row(mc,1,tmpv3);
        gsl_vector_set(m,1,0.0);
        gsl_matrix_add(store_mc,mc);
        gsl_vector_add(store_m,m);
    }




//PrintGSLMatrix(store_mc);
//PrintGSLVector(store_m);
//printf("%d \n",k);
    //printf("%f \n",mass);



    for(n=1; n<((Status->ddl)+2-6+1); n++)//attention aux indices qui partent de zero en c
    {
        j=n;
        err=0;
        for (i=1; i<((Status->ddl)+2-6+1); i++)
        {
            if (j==gsl_vector_get(mother,i))
            {

                path[err]=i;
                k=path[0];
                err++;
            }
        }
        if (err==0)
            k=j;


        if(err==1)
        {
            if (k==uLINK[j].mother)
            {
                gsl_blas_dgemv(CblasNoTrans, 1.0, uLINK[j].R, uLINK[j].a, 0.0, a);
                gsl_vector_scale (a, -1.0);

                gsl_matrix_get_row(error,store_mc,k);
                gsl_vector_scale(error,1/gsl_vector_get(store_m,k));
                gsl_vector_sub(error,uLINK[j].p);

                Cross(a, error, 1);
                gsl_vector_scale(error,gsl_vector_get(store_m,k)/mass);

                for (i=0; i<3; i++)
                {
                    gsl_matrix_set(J,i,j-2,gsl_vector_get(error,i));
                    if (J->size1==6)
                    gsl_matrix_set(J,i+3,j-2,gsl_vector_get(a,i));
                }
            }
            else if (j==uLINK[k].mother)
            {
                gsl_blas_dgemv(CblasNoTrans, 1.0, uLINK[k].R, uLINK[k].a, 0.0, a);

                gsl_matrix_get_row(error,store_mc,k);
                gsl_vector_scale(error,1/gsl_vector_get(store_m,k));
                gsl_vector_sub(error,uLINK[k].p);

                Cross(a, error, 1);
                gsl_vector_scale(error,gsl_vector_get(store_m,k)/mass);

                for (i=0; i<3; i++)
                {
                    gsl_matrix_set(J,i,k-2,gsl_vector_get(error,i));
                    if (J->size1==6)
                    gsl_matrix_set(J,i+3,k-2,gsl_vector_get(a,i));
                }
            }
            else if (k==j)
            {
                gsl_blas_dgemv(CblasNoTrans, 1.0, uLINK[k].R, uLINK[k].a, 0.0, a);

                gsl_matrix_get_row(error,store_mc,k);
                gsl_vector_scale(error,1/gsl_vector_get(store_m,k));
                gsl_vector_sub(error,uLINK[k].p);

                Cross(a, error, 1);
                gsl_vector_scale(error,gsl_vector_get(store_m,k)/mass);

                for (i=0; i<3; i++)
                {
                    gsl_matrix_set(J,i,k-2,gsl_vector_get(error,i));
                    if (J->size1==6)
                    gsl_matrix_set(J,i+3,k-2,gsl_vector_get(a,i));
                }
            }
            else
            {
                printf(" \n Error CalcJcom, link path need verification!!! \n");
            }
        }
        else
        {
            for (l=0; l<err; l++)
            {
             o=path[l];
                gsl_blas_dgemv(CblasNoTrans, 1.0, uLINK[o].R, uLINK[o].a, 0.0, a);

                gsl_matrix_get_row(error,store_mc,o);
                gsl_vector_scale(error,1/gsl_vector_get(store_m,o));
                gsl_vector_sub(error,uLINK[o].p);

                Cross(a, error, 1);
                gsl_vector_scale(error,gsl_vector_get(store_m,o)/mass);

                for (i=0; i<3; i++)
                {
                    gsl_matrix_set(J,i,o-2,gsl_vector_get(error,i));
                    if (J->size1==6)
                    gsl_matrix_set(J,i+3,o-2,gsl_vector_get(a,i));
                }
            }
        }
    }
}

//function J = CalcCoMJacobian(base)
//%base=13;
//% Jacobian matrix of current configration in World frame
//global uLINK
//
//
//J = zeros(6,length(uLINK));
//M  = TotalMass(1);
//
//
//extrem=Find_extremum(base,0);
//% extrem=[7,13,18,21];
//% extrem(find(extrem==base))=[];
//
//
//idx = zeros(4,8);
//store_mc=zeros(2*length(extrem),length(uLINK),3);
//store_m=zeros(2*length(extrem),length(uLINK));
//
//for i=1:length(extrem)
//    tmp=[1 FindRoute(extrem(1,i))];
//    idx(i,1:length(tmp))=tmp;
//    mc=zeros(length(uLINK),3);
//    m=zeros(length(uLINK),1);
//    for j=1:length(tmp)ping(int i)
//        k=idx(i,(length(tmp)-j+1));
//        if j==1
//            mc(k,1:3) = (uLINK(k).m * (uLINK(k).p + uLINK(k).R * uLINK(k).c));
//            m(k)=uLINK(k).m ;
//        else
//            l=idx(i,(length(tmp)-j+2));
//            mc(k,1:3) = (uLINK(k).m * (uLINK(k).p + uLINK(k).R * uLINK(k).c))+ mc(l,1:3)';
//            m(k)=uLINK(k).m + m(l);
//            mother(l)=k;
//        end
//    end
//    store_mc(i,1:length(uLINK),1:3)=mc(1:length(uLINK),1:3);
//    store_m(i,1:length(uLINK))=m(1:length(uLINK));
//
//    if i>1
//    mc=zeros(length(uLINK),3);
//    m=zeros(length(uLINK),1);
//        for j=1:(i-1)
//            tmp_min=tmp(find(idx(j,1:(length(tmp)))==idx(i,1:(length(tmp)))));
//        end
//
//        for j=1:length(tmp_min)
//            k=tmp_min(length(tmp_min)-j+1);
//            if j==1
//                mc(k,1:3) = -(uLINK(k).m * (uLINK(k).p + uLINK(k).R * uLINK(k).c));
//                m(k)=-uLINK(k).m ;
//            else
//                l=tmp_min(length(tmp_min)-j+2);
//                mc(k,1:3) = -(uLINK(k).m * (uLINK(k).p + uLINK(k).R * uLINK(k).c))+ mc(l,1:3)';
//                m(k)=-uLINK(k).m + m(l);
//            end
//        end
//    store_mc(i+length(extrem),1:length(uLINK),1:3)=mc(1:length(uLINK),1:3);
//    store_m(i+length(extrem),1:length(uLINK))=m(1:length(uLINK));
//    end
//
//end
//
//mc_sum=squeeze(sum(store_mc));
//m_sum=sum(store_m);
//
//if base~=1
//idx = [1 FindRoute(base)];
//    mc=zeros(length(uLINK),3);
//    m=zeros(length(uLINK),1);
//    for j=2:length(idx)
//        k=idx(j);
//        l=idx(j-1);
//        if l==1
//            mc(1,1:3)=mc_sum(1,1:3);
//            m(1,1:3)=m_sum(1,1:3);
//        end
//
//        mc(k,1:3) = (uLINK(k).m * (uLINK(k).p + uLINK(k).R * uLINK(k).c))+ mc(l,1:3)';
//        m(k)=uLINK(k).m + m(l);
//        mother(l)=k;
//    end
//    mc(1,1:3)=zeros(1,3);
//    m(1,1:3)=zeros(1,3);
//
//    store_mc(i+2*length(extrem),1:length(uLINK),1:3)=mc(1:length(uLINK),1:3);
//    store_m(i+2*length(extrem),1:length(uLINK))=m(1:length(uLINK));
//end
//
//% store_mc=zeros(2*length(extrem),21,3);
//% store_m=zeros(2*length(extrem),21);
//% for i=1:21
//%     for j=1:3
//%         mc_sum(i,j)=squeeze(sum(store_mc(:,i,j)));
//%     end
//%     m_sum(i)=squeeze(sum(store_m(:,i)));
//% end
//mc_sum=squeeze(sum(store_mc));
//m_sum=sum(store_m);
//
//% mc_sum
//% m_sum
//
//
//
//for n=1:length(uLINK)
//
//    j = n;
//    k = find(mother(:)==j);
//    if isempty(k)
//        k=j;
//    end
//
//    if length(k)==1
//        if k==uLINK(j).mother
//            %x_com=uLINK(j).R*(mc_sum(j,1:3)'/m_sum(1,j));
//            x_com=(mc_sum(k,1:3)'/m_sum(1,k))-uLINK(j).p;
//            a = uLINK(j).R * (-uLINK(j).a);
//            J(:,j) = [(m_sum(1,k)/M)*cross(a, x_com) ; a ];
//        elseif j==uLINK(k).mother
//            x_com=(mc_sum(k,1:3)'/m_sum(1,k))-uLINK(k).p;
//            a = uLINK(k).R * uLINK(k).a;
//            J(:,k) = [(m_sum(1,k)/M)*cross(a, x_com) ; a ];
//        elseif k==j
//            x_com=(mc_sum(k,1:3)'/m_sum(1,k))-uLINK(k).p;
//            a = uLINK(k).R * uLINK(k).a;
//            J(:,k) = [(m_sum(1,k)/M)*cross(a, x_com) ; a ];
//        else
//            disp('error CalcJcom, verif link path;');
//        end
//    else
//        for l=1:length(k)
//            m=k(l);
//            x_com=(mc_sum(m,1:3)'/m_sum(1,m))-uLINK(m).p;
//            a = uLINK(m).R * uLINK(m).a;
//            J(:,m) = [(m_sum(1,m)/M)*cross(a, x_com) ; a ];
//        end
//    end
//
//end


