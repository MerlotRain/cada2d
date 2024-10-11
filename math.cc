#include "cadsa.h"

using namespace cadsa;

bool isAngleBetween(double a, double a1, double a2, bool reversed)
{
    a = getNormalizedAngle(a);
    a1 = getNormalizedAngle(a1);
    a2 = getNormalizedAngle(a2);

    bool ret = false;

    if (reversed)
    {
        double tmp = a1;
        a1 = a2;
        a2 = tmp;
    }

    if (a1 >= a2 - AngleTolerance)
    {
        if (a >= a1 - AngleTolerance || a <= a2 + AngleTolerance)
        {
            ret = true;
        }
    }
    else
    {
        if (a >= a1 - AngleTolerance && a <= a2 + AngleTolerance)
        {
            ret = true;
        }
    }
    return ret;
}
  