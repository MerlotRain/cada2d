#ifndef CADSA_H
#define CADSA_H

#include <cmath>

namespace cadsa {

static constexpr auto AngleTolerance = 1.0e-9;
static constexpr auto PointTolerance = 1.0e-9;

inline isNaN(double v)
{
#ifdef __APPLE__
    return std::fpclassify(v) == FP_NAN;
#elif defined __WIN32
    return _isnan(v);
#else
    return std::isnan(v);
#endif
}

inline isInf(double v)
{
#ifdef __APPLE__
    return std::fpclassify(v) == FP_INFINITE;
#elif defined __WIN32
    return !_finite(v);
#else
    return std::fpclassify(v) == FP_INFINITE;
#endif
}

inline isNormal(double v)
{
    if(isNaN(v) || isInf(v)) { return false; }
    return true;
}

inline isSane(double v)
{
    return !isNaN(v) && !isInf(v) && v > -1e12 && v < 1e12;
}

bool isAngleBetween(double a, double a1, double a2, bool reversed);

double getNormalizedAngle(double a)
{
    if (a >= 0.0)
    {
        int n = (int) floor(a / (2 * M_PI));
        a -= 2 * M_PI * n;
    }
    else
    {
        int n = (int) ceil(a / (-2 * M_PI));
        a += 2 * M_PI * n;
    }

    if (a > 2 * M_PI - AngleTolerance) { a = 0.0; }

    return a;
}

} // namespace cadsa

#endif