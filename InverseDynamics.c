#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include "uLINK.h"
#include "Hat.h"
#include "Cross.h"
#include "InverseDynamics.h"


void InverseDynamics(SuLINK uLINK[],State *Status,int j,gsl_vector * f,gsl_vector * t)
{
    if (j == 0)
    {
        gsl_vector_set_zero(f);
        gsl_vector_set_zero(t);
        return;
    }
//if j == 0
//    f=[0,0,0]';
//    t=[0,0,0]';
//    return;
//end
//
    double res;

    static gsl_vector * c;
    static gsl_vector * P;
    static gsl_vector * L;
    static gsl_vector * tmp3;
    static gsl_matrix * I;
    static gsl_matrix * tmp;
    static gsl_matrix * tmp2;
    static int init_tmp=1;
    if (init_tmp==1)
    {
        c = gsl_vector_calloc (3);
        P = gsl_vector_calloc (3);
        L = gsl_vector_calloc (3);
        tmp3 = gsl_vector_calloc (3);
        I = gsl_matrix_calloc (3, 3);
        tmp = gsl_matrix_calloc (3, 3);
        tmp2 = gsl_matrix_calloc (3, 3);

        init_tmp=0;
    }

    gsl_vector_set_zero(f);
    gsl_vector_set_zero(t);
//    gsl_vector * c = gsl_vector_calloc (3);
//    gsl_matrix * I = gsl_matrix_calloc (3,3);
//    gsl_matrix * tmp = gsl_matrix_calloc (3,3);
//    gsl_matrix * tmp2 = gsl_matrix_calloc (3,3);
//
//    gsl_vector * P = gsl_vector_calloc (3);
//    gsl_vector * L = gsl_vector_calloc (3);
//    gsl_vector * tmp3 = gsl_vector_calloc (3);

    gsl_vector * ftmp = gsl_vector_calloc (3);
    gsl_vector * ttmp = gsl_vector_calloc (3);

    gsl_blas_dgemv(CblasNoTrans, 1.0, uLINK[j].R, uLINK[j].c, 0.0, c);
    gsl_vector_add (c, uLINK[j].p);
    gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, uLINK[j].R, uLINK[j].I, 0.0, tmp);
    gsl_blas_dgemm (CblasNoTrans, CblasTrans, 1.0, tmp, uLINK[j].R, 0.0, I);
//c = uLINK(j).R * uLINK(j).c + uLINK(j).p;   % center of mass
//I = uLINK(j).R * uLINK(j).I * uLINK(j).R';  % inertia in world frame

    Hat(tmp,c);
//c_hat = hat(c);
    gsl_blas_dgemm (CblasNoTrans, CblasTrans, 1.0, tmp, tmp, 0.0, tmp2);
    gsl_matrix_scale (tmp2, uLINK[j].m);
    gsl_matrix_add (I, tmp2);
//I = I + uLINK(j).m * c_hat * c_hat';

    gsl_vector_memcpy (P, c);
    Cross(uLINK[j].w,P,1);
    gsl_vector_add (P, uLINK[j].vo);
    gsl_vector_scale (P, uLINK[j].m);
//P = uLINK(j).m * (uLINK(j).vo + cross(uLINK(j).w,c));   % linear  momentum
    gsl_vector_memcpy (L, c);
    Cross(L,uLINK[j].vo,0);
    gsl_vector_scale (L, uLINK[j].m);
    gsl_blas_dgemv(CblasNoTrans, 1.0, I, uLINK[j].w, 0.0, tmp3);
    gsl_vector_add (L, tmp3);
//L = uLINK(j).m * cross(c,uLINK(j).vo) + I * uLINK(j).w; % angular momentum

    gsl_vector_memcpy (f, c);
    Cross(uLINK[j].dw,f,1);
    gsl_vector_add (f, uLINK[j].dvo);
    gsl_vector_scale (f, uLINK[j].m);

    gsl_vector_memcpy (tmp3, P);
    Cross(uLINK[j].w,tmp3,1);
    gsl_vector_add (f, tmp3);
//f = uLINK(j).m * (uLINK(j).dvo + cross(uLINK(j).dw,c))   + cross(uLINK(j).w,P);

    gsl_vector_memcpy (t, c);
    Cross(t,uLINK[j].dvo,0);
    gsl_vector_scale (t, uLINK[j].m);

    gsl_blas_dgemv(CblasNoTrans, 1.0, I, uLINK[j].dw, 0.0, tmp3);
    gsl_vector_add (t, tmp3);

    gsl_vector_memcpy (tmp3, P);
    Cross(uLINK[j].vo,tmp3,1);
    gsl_vector_add (t, tmp3);

    gsl_vector_memcpy (tmp3, L);
    Cross(uLINK[j].w,tmp3,1);
    gsl_vector_add (t, tmp3);
//t = uLINK(j).m * cross(c,uLINK(j).dvo) + I * uLINK(j).dw + cross(uLINK(j).vo,P) + cross(uLINK(j).w,L);




//
//
//    gsl_vector_set_zero(ftmp);
//    gsl_vector_set_zero(ttmp);
//    gsl_vector * w_c = gsl_vector_calloc (3);
//    gsl_blas_dgemv(CblasNoTrans, 1.0, uLINK[j].R, uLINK[j].c, 0.0, w_c);
//    gsl_vector_add (w_c, uLINK[j].p);
//
//    gsl_vector_set (ftmp, 2, 9.81*uLINK[j].m);
//    //printf(" j: %d %f \n",j,uLINK[j].m);
//////w_c = uLINK(j).R * uLINK(j).c + uLINK(j).p; % Centre de masse
//////f3 = -[0 0 -uLINK(j).m * G]'; % Gravite
//    gsl_vector_memcpy (ttmp, ftmp);
//    Cross(w_c,ttmp,1);
//////t3 = cross(w_c, f3);
//////DrawForceMarker(w_c',-f3*0.005,'g');
////    gsl_vector * visu = gsl_vector_calloc (3);
////    gsl_vector_memcpy (visu, ftmp);
////    gsl_vector_scale (visu, -0.005);
////    DrawForceMarker(w_c,visu);
////    gsl_vector_free(visu);
//    gsl_vector_add (f, ftmp);
//    gsl_vector_add (t, ttmp);
//    gsl_vector_free(w_c);

////f = f + f3;
////t = t + t3;
////




//
//
//    gsl_vector_set_zero(ftmp);
//    gsl_vector_set_zero(ttmp);
//    if (uLINK[j].isPolygon==1)
//    {
//        //int Kf = 1000000;        //1.0E+4 Rigidite (N/m),           //40000
//        //int Df = 1000;        //1.0E+3 viscosite (N/(m/s)) du sol    //
//        int Kf = 1000000;        //1.0E+4 Rigidite (N/m),           //40000
//        int Df = 1000;        //1.0E+3 viscosite (N/(m/s)) du sol    //
//        int i,k;
//        double val;
//        gsl_matrix * vert = gsl_matrix_calloc (3, 8);
//        gsl_vector * v = gsl_vector_calloc (3);
//        gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, uLINK[j].R, uLINK[j].vert, 0.0, vert);
//        for (i = 0; i < 3; ++i)
//        {
//            for (k = 0; k < 8; ++k)
//            {
//                val=gsl_matrix_get (vert,i,k)+gsl_vector_get (uLINK[j].p,i);
//                gsl_matrix_set (vert,i,k,val);
//            }
//        }
//
//        if (j==Status->right_foot_ID)
//        {
//            //right foot
//            gsl_vector_set (Status->FootCenter_R,0,(gsl_matrix_get(vert,0,0)+gsl_matrix_get(vert,0,2))/2);
//            gsl_vector_set (Status->FootCenter_R,1,(gsl_matrix_get(vert,1,0)+gsl_matrix_get(vert,1,2))/2);
//            gsl_vector_set (Status->FootCenter_R,2,(gsl_matrix_get(vert,2,0)+gsl_matrix_get(vert,2,2))/2);
//            gsl_vector_set_zero(Status->forCoP_R);
//            gsl_vector_set_zero(Status->posCoP_R);
//            Status->right_scale=0.0;
//            Status->support &= (~0x01); //foot right support=0
//        }
//
//        if (j==Status->left_foot_ID)
//        {
//            //left foot
//            gsl_vector_set (Status->FootCenter_L,0,(gsl_matrix_get(vert,0,0)+gsl_matrix_get(vert,0,2))/2);
//            gsl_vector_set (Status->FootCenter_L,1,(gsl_matrix_get(vert,1,0)+gsl_matrix_get(vert,1,2))/2);
//            gsl_vector_set (Status->FootCenter_L,2,(gsl_matrix_get(vert,2,0)+gsl_matrix_get(vert,2,2))/2);
//            gsl_vector_set_zero(Status->forCoP_L);
//            gsl_vector_set_zero(Status->posCoP_L);
//            Status->left_scale=0.0;
//            Status->support &= (~0x02); //foot left support=0
//        }
//
//        for (i = 0; i < 8; ++i)
//        {
//            if (gsl_matrix_get(vert,2,i)<uLINK[j].supportHeight)
//            {
//                gsl_matrix_get_col(v, vert, i);
//                Cross(uLINK[j].w,v,1);
//                gsl_vector_add (v, uLINK[j].vo);
//                gsl_vector_set (ftmp,0,Df*gsl_vector_get(v,0));
//                gsl_vector_set (ftmp,1,Df*gsl_vector_get(v,1));
//                gsl_vector_set (ftmp,2,Kf*(gsl_matrix_get(vert,2,i)-uLINK[j].supportHeight)+Df*gsl_vector_get(v,2));
//                if (gsl_vector_get (ftmp,2)<0)
//                {
//                    gsl_vector_add(f, ftmp);
//                    gsl_vector_memcpy (ttmp, ftmp);
//                    gsl_matrix_get_col(v, vert, i);
//                    //gsl_vector_sub(v,uLINK[j].p);
//                    Cross(v,ttmp,1);
//                    gsl_vector_add(t, ttmp);
//                    //gsl_vector_add(v,uLINK[j].p);
//
//                    //gsl_vector_scale (ftmp, -0.01);
//                    //DrawForceMarker(v,ftmp);
//                    gsl_vector_set(uLINK[j].isContact,i,1);
//                    gsl_matrix_set_col (uLINK[j].posContact, i, v);
//                    gsl_matrix_set_col (uLINK[j].forContact, i, ftmp);
//
//                    if (j==Status->right_foot_ID)
//                    {
//                    gsl_vector_set (v, 2, 0);
//                    gsl_vector_scale (v, gsl_vector_get (ftmp,2));
//                    Status->right_scale=Status->right_scale+gsl_vector_get (ftmp,2);
//                    //gsl_vector_scale (ftmp, -0.0025);
//                    gsl_vector_add(Status->forCoP_R,ftmp);
//                    gsl_vector_add(Status->posCoP_R,v);
//                    Status->support |= 0x01;
//                    }
//
//                    if (j==Status->left_foot_ID)
//                    {
//                    gsl_vector_set (v, 2, 0);
//                    gsl_vector_scale (v, gsl_vector_get (ftmp,2));
//                    Status->left_scale=Status->left_scale+gsl_vector_get (ftmp,2);
//                    //gsl_vector_scale (ftmp, -0.0025);
//                    gsl_vector_add(Status->forCoP_L,ftmp);
//                    gsl_vector_add(Status->posCoP_L,v);
//                    Status->support |= 0x02;
//                    }
//
//                }
//                else
//                {
//                    gsl_vector_set(uLINK[j].isContact,i,0);
//                }
//            }
//            else
//            {
//                gsl_vector_set(uLINK[j].isContact,i,0);
//            }
//        }
//
//        gsl_matrix_free(vert);
//        gsl_vector_free(v);
//    }

//if ~isempty(uLINK(j).vertex)
//    vert = uLINK(j).R * uLINK(j).vertex;
//    for k = 1:3
//        vert(k,:) = vert(k,:) + uLINK(j).p(k); % adding x,y,z to all vertex
//    end
//
//    for i = 1:size(vert(3,:),2)
//        if vert(3,i) < 0 % Contact detecte
//            Kf = 1.0E+4;        %1.0E+4 Rigidite (N/m),
//            Df = 1.0E+3;        %1.0E+3 viscosite (N/(m/s)) du sol
//            v = uLINK(j).vo + cross(uLINK(j).w,vert(:,i));  % Vitesse de contact
//            fc = -[-Df*v(1)  -Df*v(2) -Kf*vert(3,i)-Df*v(3)]';
//            if fc(3)<0
//                f = f + fc;
//                %t = t + cross(uLINK(j).p, fc);
//                t = t + cross(vert(:,i), fc);
//                DrawForceMarker([vert(1,i),vert(2,i),vert(3,i)]',-[fc(1),fc(2),fc(3)]'*0.01,'b');
//            end
//        end
//    end





//% from child link
    gsl_vector_set_zero(ftmp);
    gsl_vector_set_zero(ttmp);
    InverseDynamics(uLINK,Status,uLINK[j].child,ftmp,ttmp);
//[f1,t1] = InverseDynamics(uLINK(j).child);
//
    gsl_vector_add (f, ftmp);
    gsl_vector_add (t, ttmp);
//f = f + f1;
//t = t + t1;





    if (j != 1)
    {
        gsl_blas_ddot (uLINK[j].hv, f, &res);
        uLINK[j].u = res;
        gsl_blas_ddot (uLINK[j].hw, t, &res);
        uLINK[j].u = res + uLINK[j].u ;
    }
//if j ~= 1
//    uLINK(j).u = uLINK(j).hv' * f + uLINK(j).hw' * t;  % joint driving force
//end
//




//% return force to mother, with sisters
    gsl_vector_set_zero(ftmp);
    gsl_vector_set_zero(ttmp);
    InverseDynamics(uLINK,Status,uLINK[j].sister,ftmp,ttmp);
//[f2,t2] = InverseDynamics(uLINK(j).sister);
    gsl_vector_add (f, ftmp);
    gsl_vector_add (t, ttmp);
//f = f + f2;
//t = t + t2;

//    gsl_vector_free(c);
//    gsl_matrix_free(I);
//    gsl_matrix_free(tmp);
//    gsl_matrix_free(tmp2);
//
//    gsl_vector_free(P);
//    gsl_vector_free(L);
//    gsl_vector_free(tmp3);
    gsl_vector_free(ftmp);
    gsl_vector_free(ttmp);
}


