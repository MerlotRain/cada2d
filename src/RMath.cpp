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

#include <cada2d/RMath.h>
#include <cada2d/RS.h>
#include <cmath>
#include <algorithm>

bool RMath::isNaN(double v)
{
#ifdef __APPLE__
    return std::fpclassify(v) == FP_NAN;
#elif defined _MSC_VER
    return _isnan(v);
#else
    return std::isnan(v);
#endif
}

bool RMath::isInf(double v)
{
#ifdef __APPLE__
    return std::fpclassify(v) == FP_INFINITE;
#elif defined _MSC_VER
    return !_finite(v);
#else
    return std::fpclassify(v) == FP_INFINITE;
#endif
}

bool RMath::isNormal(double v)
{
    if (isNaN(v) || isInf(v)) {
        return false;
    }
    return true;
}

bool RMath::isSane(double v)
{
    return !isNaN(v) && !isInf(v) && v > -1e12 && v < 1e12;
}

double RMath::rad2deg(double a)
{
    return (a / (2.0 * M_PI) * 360.0);
}

double RMath::gra2deg(double a)
{
    return a / 400.0 * 360.0;
}

double RMath::deg2rad(double a)
{
    return ((a / 360.0) * (2.0 * M_PI));
}

double RMath::rad2gra(double a)
{
    return (a / (2.0 * M_PI) * 400.0);
}

bool RMath::isAngleBetween(double a, double a1, double a2, bool reversed)
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

    if (a1 >= a2 - RS::AngleTolerance) {
        if (a >= a1 - RS::AngleTolerance || a <= a2 + RS::AngleTolerance) {
            ret = true;
        }
    }
    else {
        if (a >= a1 - RS::AngleTolerance && a <= a2 + RS::AngleTolerance) {
            ret = true;
        }
    }
    return ret;
}

double RMath::getNormalizedAngle(double a)
{
    if (a >= 0.0) {
        int n = (int)floor(a / (2 * M_PI));
        a -= 2 * M_PI * n;
    }
    else {
        int n = (int)ceil(a / (-2 * M_PI));
        a += 2 * M_PI * n;
    }

    if (a > 2 * M_PI - RS::AngleTolerance) {
        a = 0.0;
    }

    return a;
}

double RMath::getAngleDifference(double a1, double a2)
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

double RMath::getAngleDifference180(double a1, double a2)
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

bool RMath::fuzzyCompare(double v1, double v2, double tolerance)
{
    return fabs(v1 - v2) < tolerance;
}

bool RMath::fuzzyAngleCompare(double v1, double v2, double tolerance)
{
    return fabs(getAngleDifference180(v1, v2)) < tolerance;
}

bool RMath::isBetween(double value, double limit1, double limit2,
                      bool inclusive, double tolerance)
{
    if (fuzzyCompare(value, limit1, tolerance) ||
        fuzzyCompare(value, limit2, tolerance)) {
        return inclusive;
    }
    double min = std::min(limit1, limit2);
    double max = std::max(limit1, limit2);
    return (value >= min && value <= max);
}

int RMath::getGcd(int a, int b)
{
    int rem;

    while (b != 0) {
        rem = a % b;
        a = b;
        b = rem;
    }

    return a;
}

double RMath::getRelativeAngle(double a, double baseAngle)
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

double RMath::makeAngleReadable(double angle, bool readable, bool *corrected)
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

bool RMath::isAngleReadable(double angle, double tolerance)
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

bool RMath::isSameDirection(double dir1, double dir2, double tol)
{
    double diff = fabs(dir1 - dir2);
    if (diff < tol || diff > 2 * M_PI - tol) {
        return true;
    }
    else {
        return false;
    }
}

int RMath::absmod(int a, int b)
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

double RMath::java_math_round(double val)
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
}

bool RMath::linearSolver(const std::vector<std::vector<double>> &mt,
                         std::vector<double> &sn)
{
    // verify the matrix size
    size_t mSize = mt.size(); // rows
    size_t aSize(mSize + 1);  // columns of augmented matrix
    if (std::any_of(mt.begin(), mt.end(),
                    [&aSize](const std::vector<double> &v) -> bool {
                        return v.size() != aSize;
                    }))
        return false;
    sn.resize(mSize); // to hold the solution

    // solve the linear equation by Gauss-Jordan elimination
    std::vector<std::vector<double>> mt0(mt); // copy the matrix;
    for (size_t i = 0; i < mSize; ++i) {
        size_t imax(i);
        double cmax(fabs(mt0[i][i]));
        for (size_t j = i + 1; j < mSize; ++j) {
            if (fabs(mt0[j][i]) > cmax) {
                imax = j;
                cmax = fabs(mt0[j][i]);
            }
        }
        if (cmax < 1.0e-20)
            return false; // singular matrix
        if (imax != i) {  // move the line with largest absolute value at column
                          // i to row i, to avoid division by zero
            std::swap(mt0[i], mt0[imax]);
        }
        for (size_t k = i + 1; k <= mSize; ++k) { // normalize the i-th row
            mt0[i][k] /= mt0[i][i];
        }
        mt0[i][i] = 1.;
        for (size_t j = 0; j < mSize; ++j) { // Gauss-Jordan
            if (j != i) {
                double &a = mt0[j][i];
                for (size_t k = i + 1; k <= mSize; ++k) {
                    mt0[j][k] -= mt0[i][k] * a;
                }
                a = 0.;
            }
        }
    }
    for (size_t i = 0; i < mSize; ++i) {
        sn[i] = mt0[i][mSize];
    }

    return true;
}

int RMath::sign(double x)
{
    if (std::isnan(x)) {
        return 4;
    }
    if (x == 0) {
        return std::signbit(x) ? 3 : 2;
    }
    return x > 0 ? 0 : 1;
}