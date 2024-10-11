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

#ifndef CADSA_H
#define CADSA_H

#include <cmath>

namespace cadsa {

class NS {

public:
    static constexpr auto AngleTolerance = 1.0e-9;
    static constexpr auto PointTolerance = 1.0e-9;

    static bool isNaN(double v)
    {
#ifdef __APPLE__
        return std::fpclassify(v) == FP_NAN;
#elif defined __WIN32
        return _isnan(v);
#else
        return std::isnan(v);
#endif
    }

    static bool isInf(double v)
    {
#ifdef __APPLE__
        return std::fpclassify(v) == FP_INFINITE;
#elif defined __WIN32
        return !_finite(v);
#else
        return std::fpclassify(v) == FP_INFINITE;
#endif
    }

    static bool isNormal(double v)
    {
        if (isNaN(v) || isInf(v)) {
            return false;
        }
        return true;
    }

    static bool isSane(double v)
    {
        return !isNaN(v) && !isInf(v) && v > -1e12 && v < 1e12;
    }

    static bool isAngleBetween(double a, double a1, double a2, bool reversed);
    static double getNormalizedAngle(double a);
};

} // namespace cadsa

#endif