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

#include "cadsa.h"

using namespace cadsa;

bool NS::isAngleBetween(double a, double a1, double a2, bool reversed)
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

    if (a1 >= a2 - AngleTolerance) {
        if (a >= a1 - AngleTolerance || a <= a2 + AngleTolerance) {
            ret = true;
        }
    }
    else {
        if (a >= a1 - AngleTolerance && a <= a2 + AngleTolerance) {
            ret = true;
        }
    }
    return ret;
}

double NS::getNormalizedAngle(double a)
{
    if (a >= 0.0) {
        int n = (int)floor(a / (2 * M_PI));
        a -= 2 * M_PI * n;
    }
    else {
        int n = (int)ceil(a / (-2 * M_PI));
        a += 2 * M_PI * n;
    }

    if (a > 2 * M_PI - AngleTolerance) {
        a = 0.0;
    }

    return a;
}