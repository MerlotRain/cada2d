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
#include <math.h>
#include <assert.h>

double pt_magnitude(const apo_point_t p)
{
    return sqrt(p.x * p.x + p.y * p.y);
}

apo_point_t apo_create_point_from_polar(double r, double ang)
{
    return (apo_point_t){
        .x = r * cos(ang),
        .y = r * sin(ang),
    };
}

apo_point_t pt_average(const apo_point_t a, const apo_point_t b)
{
    return (apo_point_t){
        .x = (a.x + b.x) / 2,
        .y = (a.y + b.y) / 2,
    };
}

double pt_distance(const apo_point_t a, const apo_point_t b)
{
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

double pt_angle_to(const apo_point_t a, const apo_point_t b)
{
    return pt_angle(pt_sub(b, a));
}

double pt_angle(const apo_point_t p)
{
    double ret = 0.0;
    double m = sqrt(p.x * p.x + p.y * p.y);

    if (m > 1.0e-6) {
        double dp = (p.x * 1.0 + p.y * 0.0);
        if (dp / m >= 1.0) {
            ret = 0.0;
        }
        else if (dp / m < -1.0) {
            ret = M_PI;
        }
        else {
            ret = acos(dp / m);
        }
        if (p.y < 0.0) {
            ret = 2 * M_PI - ret;
        }
    }
    return ret;
}

double pt_dot_product(const apo_point_t p1, const apo_point_t p2)
{
    return p1.x * p2.x + p1.y * p2.y;
}

void pt_set_maguitude(apo_point_t *u, double r)
{
    assert(u);
    double a = pt_angle(*u);
    u->x = r * cos(a);
    u->y = r * sin(a);
}

apo_point_t pt_sub(const apo_point_t a, const apo_point_t b)
{
    return (apo_point_t){
        .x = a.x - b.x,
        .y = a.y - b.y,
    };
}

apo_point_t pt_add(const apo_point_t a, const apo_point_t b)
{
    return (apo_point_t){
        .x = a.x + b.x,
        .y = a.y + b.y,
    };
}

apo_point_t pt_mul(const apo_point_t p, const double s)
{
    return (apo_point_t){
        .x = p.x * s,
        .y = p.y * s,
    };
}

boolean pt_on_line(const apo_line_t l, const apo_point_t p, boolean limited)
{
    apo_point_t vt;
    if (!apo_line_closest_point(l, p, limited, &vt)) {
        return APO_FALSE;
    }

    double vtm = pt_magnitude(vt);
    if (vtm < 1.0e-4) {
        return APO_TRUE;
    }
    return APO_FALSE;
}

boolean pt_on_circle(const apo_circle_t c, const apo_point_t p)
{
    return APO_FALSE;
}

boolean pt_equal(const apo_point_t p1, const apo_point_t p2)
{
    if (fabs(p1.x - p2.x) < APO_TOLERANCE &&
        fabs(p1.y - p2.y) < APO_TOLERANCE) {
        return APO_TRUE;
    }
    return APO_FALSE;
}

apo_point_t pt_rotate(const apo_point_t p, double rotation,
                      const apo_point_t center)
{
    double r = pt_magnitude(p);
    double a = pt_angle(p) + rotation;

    double x = cos(a) * r;
    double y = sin(a) * r;
    x = x <= APO_TOLERANCE ? 0.0 : x;
    y = y <= APO_TOLERANCE ? 0.0 : y;
    return APO_POINT(x, y);
}