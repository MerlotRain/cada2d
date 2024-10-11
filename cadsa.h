#ifndef CADSA_H
#define CADSA_H

#include <cmath>

namespace cadsa {

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


} // namespace cadsa

#endif