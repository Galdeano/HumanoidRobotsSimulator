#include <Eigen/Dense>
#include <vector>
#include <stdio.h>
#include "uLink.h"
#include "TotalMass.h"
#include "FindRoute.h"
#include "Cross.h"
#include "CalcCoMJacobian.h"

void CalcCoMJacobian(SuLINK uLINK[], State *Status, Eigen::MatrixXd & J, int base_call)
{
    int i, j, k, l, n, o, err;
    int nbre_elements, nbre_tmp;
    static double mass;

    static Eigen::Vector3d a;
    static Eigen::Vector3d error;
    static Eigen::MatrixXd mc;
    static Eigen::VectorXd m;
    static Eigen::MatrixXd store_mc;
    static Eigen::VectorXd store_m;
    static Eigen::Vector3d tmpv3;
    static Eigen::Vector3d tmpv3_2;
    static Eigen::VectorXd mother;
    static Eigen::VectorXd double_path;
    static std::vector<int> path;

    static int init_tmp = 1;
    if (init_tmp == 1)
    {
        a.setZero();
        error.setZero();
        mc = Eigen::MatrixXd::Zero((Status->ddl) - 3, 3);
        m = Eigen::VectorXd::Zero((Status->ddl) - 3);
        store_mc = Eigen::MatrixXd::Zero((Status->ddl) - 3, 3);
        store_m = Eigen::VectorXd::Zero((Status->ddl) - 3);
        tmpv3.setZero();
        tmpv3_2.setZero();
        mother = Eigen::VectorXd::Zero((Status->ddl) - 3);
        double_path = Eigen::VectorXd::Zero((Status->ddl) - 3);
        path.resize((Status->ddl) + 2 - 6);

        mass = TotalMass(uLINK, 1);
        init_tmp = 0;
    }

    mother.setZero();
    m.setZero();
    store_m.setZero();
    double_path.setZero();
    mc.setZero();
    store_mc.setZero();

    for (i = 0; i < (Status->nb_limb); i++)
    {
        if (Status->limbID(i) != base_call)
        {
            mc.setZero();
            m.setZero();
            nbre_elements = FindRoute(uLINK, path.data(), (int)Status->limbID(i));

            for (j = 0; j < nbre_elements; j++)
            {
                k = path[j];

                if (j == 0)
                {
                    tmpv3 = uLINK[k].R * uLINK[k].c + uLINK[k].p;
                    tmpv3 *= uLINK[k].m;
                    mc.row(k) = tmpv3;

                    m(k) = uLINK[k].m;
                }
                else
                {
                    l = path[j - 1];

                    tmpv3 = uLINK[k].R * uLINK[k].c + uLINK[k].p;
                    tmpv3 *= uLINK[k].m;
                    tmpv3_2 = mc.row(l);
                    tmpv3 += tmpv3_2;
                    mc.row(k) = tmpv3;

                    m(k) = uLINK[k].m + m(l);
                    mother(l) = k;
                }
            }
            store_mc += mc;
            store_m += m;

            if (i == 0)
            {
                for (j = 0; j < nbre_elements; j++)
                {
                    double_path(path[j]) = 1.0;
                }
            }

            if (i > 0)
            {
                mc.setZero();
                m.setZero();
                nbre_tmp = 0;
                for (j = 0; j < nbre_elements; j++)
                {
                    if (double_path(path[j]) == 1.0)
                    {
                        nbre_tmp++;
                    }
                    double_path(path[j]) = 1.0;
                }

                for (j = 0; j < nbre_tmp; j++)
                {
                    k = path[j + (nbre_elements - nbre_tmp)];

                    if (j == 0)
                    {
                        tmpv3 = uLINK[k].R * uLINK[k].c + uLINK[k].p;
                        tmpv3 *= uLINK[k].m;
                        mc.row(k) = tmpv3;

                        m(k) = uLINK[k].m;
                    }
                    else
                    {
                        l = path[j - 1 + (nbre_elements - nbre_tmp)];

                        tmpv3 = uLINK[k].R * uLINK[k].c + uLINK[k].p;
                        tmpv3 *= uLINK[k].m;
                        tmpv3_2 = mc.row(l);
                        tmpv3 += tmpv3_2;
                        mc.row(k) = tmpv3;

                        m(k) = uLINK[k].m + m(l);
                        mother(l) = k;
                    }
                }

                store_mc -= mc;
                store_m -= m;
            }
        }
    }

    if (base_call != 1)
    {
        mc.setZero();
        m.setZero();
        nbre_elements = FindRoute(uLINK, path.data(), base_call);

        for (j = 1; j < nbre_elements; j++)
        {
            k = path[nbre_elements - j - 1];
            l = path[nbre_elements - (j - 1) - 1];

            if (l == 1)
            {
                tmpv3 = store_mc.row(1);
                mc.row(1) = tmpv3;
                m(1) = store_m(1);
            }

            tmpv3 = uLINK[k].R * uLINK[k].c + uLINK[k].p;
            tmpv3 *= uLINK[k].m;
            tmpv3_2 = mc.row(l);
            tmpv3 += tmpv3_2;
            mc.row(k) = tmpv3;

            m(k) = uLINK[k].m + m(l);
            mother(l) = k;
        }
        tmpv3.setZero();
        mc.row(1) = tmpv3;
        m(1) = 0.0;
        store_mc += mc;
        store_m += m;
    }

    for (n = 1; n < ((Status->ddl) + 2 - 6 + 1); n++)
    {
        j = n;
        err = 0;
        for (i = 1; i < ((Status->ddl) + 2 - 6 + 1); i++)
        {
            if (j == (int)mother(i))
            {
                path[err] = i;
                k = path[0];
                err++;
            }
        }
        if (err == 0)
            k = j;

        if (err == 1)
        {
            if (k == uLINK[j].mother)
            {
                a = uLINK[j].R * (-uLINK[j].a);

                error = store_mc.row(k);
                error *= (1.0 / store_m(k));
                error -= uLINK[j].p;

                error = a.cross(error);
                error *= (store_m(k) / mass);

                for (i = 0; i < 3; i++)
                {
                    J(i, j - 2) = error(i);
                    if (J.rows() == 6)
                        J(i + 3, j - 2) = a(i);
                }
            }
            else if (j == uLINK[k].mother)
            {
                a = uLINK[k].R * uLINK[k].a;

                error = store_mc.row(k);
                error *= (1.0 / store_m(k));
                error -= uLINK[k].p;

                error = a.cross(error);
                error *= (store_m(k) / mass);

                for (i = 0; i < 3; i++)
                {
                    J(i, k - 2) = error(i);
                    if (J.rows() == 6)
                        J(i + 3, k - 2) = a(i);
                }
            }
            else if (k == j)
            {
                a = uLINK[k].R * uLINK[k].a;

                error = store_mc.row(k);
                error *= (1.0 / store_m(k));
                error -= uLINK[k].p;

                error = a.cross(error);
                error *= (store_m(k) / mass);

                for (i = 0; i < 3; i++)
                {
                    J(i, k - 2) = error(i);
                    if (J.rows() == 6)
                        J(i + 3, k - 2) = a(i);
                }
            }
            else
            {
                printf(" \n Error CalcJcom, link path need verification!!! \n");
            }
        }
        else
        {
            for (l = 0; l < err; l++)
            {
                o = path[l];
                a = uLINK[o].R * uLINK[o].a;

                error = store_mc.row(o);
                error *= (1.0 / store_m(o));
                error -= uLINK[o].p;

                error = a.cross(error);
                error *= (store_m(o) / mass);

                for (i = 0; i < 3; i++)
                {
                    J(i, o - 2) = error(i);
                    if (J.rows() == 6)
                        J(i + 3, o - 2) = a(i);
                }
            }
        }
    }
}
