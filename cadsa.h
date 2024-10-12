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
    enum Side {
        NO_SIDE,
        LEFT_HAND,
        RIGHT_HAND,
        BOTH_SIDES,
    };

    enum Ending {
        ENDING_START,
        ENDING_END,
        ENDING_NONE,
    };

    enum From {
        FROM_START = 0x01,
        FROM_END = 0x02,
        FROM_ANY = FROM_START | FROM_END,
        ALONG_POLYLINE = 0x04,
    };

    enum Orientation
    {
        UNKNOWN_ORIENTATION = -1,
        ANY = 0,
        CW,
        CCW,
    };

public:
    static constexpr auto AngleTolerance = 1.0e-9;
    static constexpr auto PointTolerance = 1.0e-9;

    static bool isNaN(double v);
    static bool isInf(double v);
    static bool isNormal(double v);
    static bool isSane(double v);

    static bool isAngleBetween(double a, double a1, double a2, bool reversed);
    static double getNormalizedAngle(double a);
    static double getAngleDifference(double a1, double a2);
    static double getAngleDifference180(double a1, double a2);
};

} // namespace cadsa

#endif