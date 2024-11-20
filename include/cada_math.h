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

#ifndef CADA_MATH_H
#define CADA_MATH_H

#include <float.h>
#include <math.h>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846 // pi
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923 // pi/2
#endif

#ifndef M_PI_4
#define M_PI_4 0.785398163397448309616 // pi/4
#endif

namespace cada {

class Math {
public:
    static bool isNaN(double v);
    static bool isInf(double v);
    static bool isNormal(double v);
    static bool isSane(double v);

    static double rad2deg(double a);
    static double deg2rad(double a);
    static double rad2gra(double a);
    static double gra2deg(double a);

    static bool isAngleBetween(double a, double a1, double a2, bool reversed);
    static double getNormalizedAngle(double a);
    static double getAngleDifference(double a1, double a2);
    static double getAngleDifference180(double a1, double a2);
    static bool fuzzyCompare(double v1, double v2,
                             double tolerance = DBL_EPSILON);
    static bool fuzzyAngleCompare(double v1, double v2,
                                  double tolerance = DBL_EPSILON);

    static bool isBetween(double value, double limit1, double limit2,
                          bool inclusive, double tolerance = DBL_EPSILON);
    static int getGcd(int a, int b);
    static double getRelativeAngle(double a, double baseAngle);
    static double makeAngleReadable(double angle, bool readable = true,
                                    bool *corrected = nullptr);
    static bool isAngleReadable(double angle, double tolerance = 0.01);
    static bool isSameDirection(double dir1, double dir2,
                                double tol = DBL_EPSILON);
    static int absmod(int a, int b);
    static double round(double val) { return java_math_round(val); }
    static double java_math_round(double val);

    static bool linearSolver(const std::vector<std::vector<double>> &mt,
                             std::vector<double> &sn);
};

} // namespace cada

#endif