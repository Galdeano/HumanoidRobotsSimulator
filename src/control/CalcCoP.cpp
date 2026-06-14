#include <Eigen/Dense>
#include "uLink.h"
#include "CalcCoP.h"

double CalcCoP(SuLINK uLINK[], Eigen::Vector3d & CoP, int j)
{
    if (j == 0)
    {
        return 0.0;
    }
    else
    {
        double f = 0.0;
        if (uLINK[j].isPolygon == 1)
        {
            for (int i = 0; i < 8; ++i)
            {
                if (uLINK[j].isContact(i) == 1.0)
                {
                    Eigen::Vector3d v = uLINK[j].posContact.col(i);
                    v(2) = 0.0;
                    Eigen::Vector3d ftmp = uLINK[j].forContact.col(i);
                    v *= ftmp(2);
                    CoP += v;
                    f += ftmp(2);
                }
            }
        }

        f += CalcCoP(uLINK, CoP, uLINK[j].child);
        f += CalcCoP(uLINK, CoP, uLINK[j].sister);
        return f;
    }
}
