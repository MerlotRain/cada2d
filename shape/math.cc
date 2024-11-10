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
#include "cada_ns.h"
#include <cmath>
#include <algorithm>

namespace cada {

bool Math::isNaN(double v)
{
#ifdef __APPLE__
    return std::fpclassify(v) == FP_NAN;
#elif defined _MSC_VER
    return _isnan(v);
#else
    return std::isnan(v);
#endif
}

bool Math::isInf(double v)
{
#ifdef __APPLE__
    return std::fpclassify(v) == FP_INFINITE;
#elif defined _MSC_VER
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

/**
 * Converts radians to degrees.
 *
 * \param a angle in radians
 */
double Math::rad2deg(double a)
{
    return (a / (2.0 * M_PI) * 360.0);
}

/**
 * Converts grads to degrees.
 *
 * \param a angle in grad (gon)
 */
double Math::gra2deg(double a)
{
    return a / 400.0 * 360.0;
}

/**
 * Converts degrees to radians.
 *
 * \param a angle in degrees
 */
double Math::deg2rad(double a)
{
    return ((a / 360.0) * (2.0 * M_PI));
}

/**
 * Converts radians to gradians.
 *
 * \param a angle in radians
 */
double Math::rad2gra(double a)
{
    return (a / (2.0 * M_PI) * 400.0);
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

bool Math::fuzzyCompare(double v1, double v2, double tolerance)
{
    return fabs(v1 - v2) < tolerance;
}

bool Math::fuzzyAngleCompare(double v1, double v2, double tolerance)
{
    return fabs(getAngleDifference180(v1, v2)) < tolerance;
}

bool Math::isBetween(double value, double limit1, double limit2, bool inclusive,
                     double tolerance)
{
    if (fuzzyCompare(value, limit1, tolerance) ||
        fuzzyCompare(value, limit2, tolerance)) {
        return inclusive;
    }
    double min = std::min(limit1, limit2);
    double max = std::max(limit1, limit2);
    return (value >= min && value <= max);
}

/**
 * Finds greatest common divider using Euclid's algorithm.
 * \sa http://en.wikipedia.org/wiki/Greatest_common_divisor
 *
 * \param a the first number
 * \param b the second number
 * \return The greatest common divisor of \c a and \c b.
 */
int Math::getGcd(int a, int b)
{
    int rem;

    while (b != 0) {
        rem = a % b;
        a = b;
        b = rem;
    }

    return a;
}

/**
 * \return Angle a as angle relative to baseAngle.
 *         Result is in range -PI < result < PI.
 */
double Math::getRelativeAngle(double a, double baseAngle)
{
    double ret = a - baseAngle;
    if (ret > M_PI) {
        ret -= 2 * M_PI;
    }
    if (ret < -M_PI) {
        ret += 2 * M_PI;
    }
    return ret;
}

/**
 * Adds 180° to the given angle if a text constructed with that angle
 * otherwise wouldn't be readable.
 * Used for dimension texts and for mirroring texts.
 *
 * \param angle the original angle
 *
 * \param readable true: make angle readable, false: unreadable
 *
 * \param corrected Pointer to boolean that will point to true if the given
 * angle was corrected, false otherwise, or null.
 *
 * \return The given angle or the given \c angle + pi, depending which one
 * is readable from the bottom or right.
 */
double Math::makeAngleReadable(double angle, bool readable, bool *corrected)
{
    double ret;

    bool cor = isAngleReadable(angle) ^ readable;

    // quadrant 1 & 4
    if (!cor) {
        ret = angle;
    }
    // quadrant 2 & 3
    else {
        ret = angle + M_PI;
    }

    if (corrected != NULL) {
        *corrected = cor;
    }

    return ret;
}

/**
 * \param angle The text angle in rad
 *
 * \param tolerance The tolerance by which the angle still maybe
 * in the unreadable range.
 *
 * \return true: If the given angle is in a range that is readable
 * for texts created with that angle.
 */
bool Math::isAngleReadable(double angle, double tolerance)
{
    double angleCorrected = getNormalizedAngle(angle);
    if (angleCorrected > M_PI / 2.0 * 3.0 + tolerance ||
        angleCorrected < M_PI / 2.0 + tolerance) {
        return true;
    }
    else {
        return false;
    }
}

bool Math::isSameDirection(double dir1, double dir2, double tol)
{
    double diff = fabs(dir1 - dir2);
    if (diff < tol || diff > 2 * M_PI - tol) {
        return true;
    }
    else {
        return false;
    }
}

int Math::absmod(int a, int b)
{
    if (b == 0) {
        return a;
    }
    int m = a % b;
    if ((b < 0 && m > 0) || (b > 0 && m < 0)) {
        return b + m;
    }
    return m;
}

double Math::java_math_round(double val)
{
    double n;
    double f = std::fabs(std::modf(val, &n));

    if (val >= 0) {
        if (f < 0.5) {
            return std::floor(val);
        }
        else if (f > 0.5) {
            return std::ceil(val);
        }
        else {
            return (n + 1.0);
        }
    }
    else {
        if (f < 0.5) {
            return std::ceil(val);
        }
        else if (f > 0.5) {
            return std::floor(val);
        }
        else {
            return n;
        }
    }
} // java_math_round

} // namespace cada