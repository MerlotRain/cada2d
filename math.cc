/**
 * Copyright (c) 2024-present Merlot.Rain
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#include "cada_math.h"
#include <cmath>
#include "cada_ns.h"

using namespace cada;

bool Math::isNaN(double v)
{
#ifdef __APPLE__
    return std::fpclassify(v) == FP_NAN;
#elif defined __WIN32
    return _isnan(v);
#else
    return std::isnan(v);
#endif
}

bool Math::isInf(double v)
{
#ifdef __APPLE__
    return std::fpclassify(v) == FP_INFINITE;
#elif defined __WIN32
    return !_finite(v);
#else
    return std::fpclassify(v) == FP_INFINITE;
#endif
}

bool Math::isNormal(double v)
{
    if (isNaN(v) || isInf(v)) {
        return false;
    }
    return true;
}

bool Math::isSane(double v)
{
    return !isNaN(v) && !isInf(v) && v > -1e12 && v < 1e12;
}

bool Math::isAngleBetween(double a, double a1, double a2, bool reversed)
{
    a = getNormalizedAngle(a);
    a1 = getNormalizedAngle(a1);
    a2 = getNormalizedAngle(a2);

    bool ret = false;

    if (reversed) {
        double tmp = a1;
        a1 = a2;
        a2 = tmp;
    }

    if (a1 >= a2 - NS::AngleTolerance) {
        if (a >= a1 - NS::AngleTolerance || a <= a2 + NS::AngleTolerance) {
            ret = true;
        }
    }
    else {
        if (a >= a1 - NS::AngleTolerance && a <= a2 + NS::AngleTolerance) {
            ret = true;
        }
    }
    return ret;
}

double Math::getNormalizedAngle(double a)
{
    if (a >= 0.0) {
        int n = (int)floor(a / (2 * M_PI));
        a -= 2 * M_PI * n;
    }
    else {
        int n = (int)ceil(a / (-2 * M_PI));
        a += 2 * M_PI * n;
    }

    if (a > 2 * M_PI - NS::AngleTolerance) {
        a = 0.0;
    }

    return a;
}

double Math::getAngleDifference(double a1, double a2)
{
    double ret;

    if (a1 >= a2) {
        a2 += 2 * M_PI;
    }
    ret = a2 - a1;

    if (ret >= 2 * M_PI) {
        ret = 0.0;
    }

    return ret;
}

double Math::getAngleDifference180(double a1, double a2)
{
    double ret;

    ret = a2 - a1;
    if (ret > M_PI) {
        ret = -(2 * M_PI - ret);
    }
    if (ret < -M_PI) {
        ret = 2 * M_PI + ret;
    }

    return ret;
}

bool cada::Math::fuzzyCompare(double v1, double v2, double tolerance)
{
    return fabs(v1 - v2) < tolerance;
}

bool cada::Math::fuzzyAngleCompare(double v1, double v2, double tolerance)
{
    return fabs(getAngleDifference180(v1, v2)) < tolerance;
}
