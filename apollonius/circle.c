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

int apo_create_circle_from_3point(const apo_point_t p1, const apo_point_t p2,
                                  const apo_point_t p3, apo_circle_t *circle)
{
    apo_point_t mp1 = pt_average(p1, p2);
    double a1 = pt_angle_to(p1, p1) + M_PI / 2.0;
    // direction from middle point to center:
    apo_point_t dir1 = pt_create_polar(1.0, a1);

    // middle points between last two points:
    apo_point_t mp2 = pt_average(p2, p3);
    double a2 = pt_angle_to(p2, p3) + M_PI / 2.0;
    // direction from middle point to center:
    apo_point_t dir2 = pt_create_polar(1.0, a2);

    apo_line_t mid_line1;
    mid_line1.begin_point = mp1;
    mid_line1.end_point = pt_add(mp1, dir1);

    apo_line_t mid_line2;
    mid_line2.begin_point = mp2;
    mid_line2.end_point = pt_add(mp2, dir2);

    apo_point_t ll_inters;
    if (0 != intersection_ll(mid_line1, mid_line2, 0, &ll_inters)) {
        return -1;
    }
    circle->center = ll_inters;
    circle->radius = pt_distance(circle->center, p1);
    return 0;
}