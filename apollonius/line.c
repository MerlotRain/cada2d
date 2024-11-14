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
#include <float.h>

boolean apo_intersection_ll(const apo_line_t l1, const apo_line_t l2,
                            boolean limited, apo_point_t *ret)
{
    double a1 = l1.end_point.y - l1.begin_point.y;
    double b1 = l1.begin_point.x - l1.end_point.x;
    double c1 = a1 * l1.begin_point.x + b1 * l1.begin_point.y;

    double a2 = l2.end_point.y - l2.begin_point.y;
    double b2 = l2.begin_point.x - l2.end_point.x;
    double c2 = a2 * l2.begin_point.x + b2 * l2.begin_point.y;

    double det = a1 * b2 - a2 * b1;
    if (fabs(det) < 1.0e-6) {
        return APO_FALSE;
    }
    else {
        apo_point_t v;

        v.x = (b2 * c1 - b1 * c2) / det;
        v.y = (a1 * c2 - a2 * c1) / det;

        if ((!limited || 0 == pt_on_line(l1, v, limited)) &&
            (!limited || 0 == pt_on_line(l2, v, limited))) {

            ret->x = v.x;
            ret->y = v.y;
            return APO_TRUE;
        }
    }
    return APO_FALSE;
}

static apo_point_t pt_closest_to_line_end(const apo_line_t l,
                                          const apo_point_t p)
{
    double d1 = pt_distance(l.begin_point, p);
    double d2 = pt_distance(l.end_point, p);
    return d1 > d2 ? l.end_point : l.begin_point;
}

boolean apo_line_closest_point(const apo_line_t l, const apo_point_t p,
                               boolean limited, apo_point_t *closest)
{
    apo_point_t ae = pt_sub(l.end_point, l.begin_point);
    apo_point_t ap = pt_sub(p, l.begin_point);

    if (pt_magnitude(ae) < 1.0e-6) {
        return APO_FALSE;
    }
    if (pt_magnitude(ap) < 1.0e-6) {
        closest->x = 0.0;
        closest->y = 0.0;
        return APO_TRUE;
    }

    double b = pt_dot_product(ap, ae) / pt_dot_product(ae, ae);
    if (limited && (b < 0 || b > 1.0)) {
        *closest = pt_closest_to_line_end(l, p);
        if (pt_magnitude(*closest) < DBL_MAX) {
            return APO_TRUE;
        }
        else {
            return APO_FALSE;
        }
    }

    apo_point_t closest_point = pt_add(l.begin_point, pt_mul(ae, b));
    *closest = pt_sub(p, closest_point);
    return APO_TRUE;
}

boolean apo_intersection_lc(const apo_line_t l, const apo_circle_t c,
                            boolean limited, apo_point_t *rets, int *ret_size)
{
    return APO_FALSE;
}

double apo_line_angle(const apo_line_t l)
{
    return pt_angle_to(l.begin_point, l.end_point);
}

apo_point_t apo_line_middle_point(const apo_line_t l)
{
    return pt_average(l.begin_point, l.end_point);
}

void apo_set_line_length(apo_line_t *l, double length, boolean from_start)
{
    if (from_start) {
        l->end_point =
            pt_add(l->begin_point,
                   apo_create_point_from_polar(length, apo_line_angle(*l)));
    }
    else {
        l->begin_point =
            pt_sub(l->end_point,
                   apo_create_point_from_polar(length, apo_line_angle(*l)));
    }
}

void apo_set_line_rotate(apo_line_t *l, double angle, const apo_point_t center)
{
    if (angle < APO_TOLERANCE)
        return;

    l->begin_point = pt_rotate(l->begin_point, angle, center);
    l->end_point = pt_rotate(l->end_point, angle, center);
}