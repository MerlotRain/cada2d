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

#include "apo.h"
#include <stddef.h>
#include <math.h>

int intersection_ll(const apo_line_t l1, const apo_line_t l2, int limited,
                    apo_point_t *ret)
{
    double a1 = l1.end_point.y - l1.begin_point.y;
    double b1 = l1.begin_point.x - l1.end_point.x;
    double c1 = a1 * l1.begin_point.x + b1 * l1.begin_point.y;

    double a2 = l2.end_point.y - l2.begin_point.y;
    double b2 = l2.begin_point.x - l2.end_point.x;
    double c2 = a2 * l2.begin_point.x + b2 * l2.begin_point.y;

    double det = a1 * b2 - a2 * b1;
    if (fabs(det) < 1.0e-6) {
        return -1;
    }
    else {
        apo_point_t v;

        v.x = (b2 * c1 - b1 * c2) / det;
        v.y = (a1 * c2 - a2 * c1) / det;

        if ((!limited || 0 == pt_on_line(l1, v)) &&
            (!limited || 0 == pt_on_line(l2, v))) {

            ret->x = v.x;
            ret->y = v.y;
            return 0;
        }
    }
    return -1;
}

int pt_on_line(const apo_line_t l, const apo_point_t p)
{
    float cross_product =
        (p.y - l.begin_point.y) * (l.end_point.x - l.begin_point.x) -
        (p.x - l.begin_point.x) * (l.end_point.y - l.begin_point.y);
    if (cross_product != 0) {
        return -1;
    }

    if (p.x < fmin(l.begin_point.x, l.end_point.x) ||
        p.x > fmax(l.begin_point.x, l.end_point.x) ||
        p.y < fmin(l.begin_point.y, l.end_point.y) ||
        p.y > fmax(l.begin_point.y, l.end_point.y)) {
        return -1;
    }

    return 0;
}
