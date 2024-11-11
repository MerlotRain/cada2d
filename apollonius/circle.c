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

boolean apo_create_circle_from_3point(const apo_point_t p1,
                                      const apo_point_t p2,
                                      const apo_point_t p3,
                                      apo_circle_t *circle)
{
    apo_point_t mp1 = pt_average(p1, p2);
    double a1 = pt_angle_to(p1, p1) + M_PI / 2.0;
    // direction from middle point to center:
    apo_point_t dir1 = apo_create_point_from_polar(1.0, a1);

    // middle points between last two points:
    apo_point_t mp2 = pt_average(p2, p3);
    double a2 = pt_angle_to(p2, p3) + M_PI / 2.0;
    // direction from middle point to center:
    apo_point_t dir2 = apo_create_point_from_polar(1.0, a2);

    apo_line_t mid_line1;
    mid_line1.begin_point = mp1;
    mid_line1.end_point = pt_add(mp1, dir1);

    apo_line_t mid_line2;
    mid_line2.begin_point = mp2;
    mid_line2.end_point = pt_add(mp2, dir2);

    apo_point_t ll_inters;
    if (!apo_intersection_ll(mid_line1, mid_line2, 0, &ll_inters)) {
        return APO_FALSE;
    }
    circle->center = ll_inters;
    circle->radius = pt_distance(circle->center, p1);
    return APO_TRUE;
}

apo_circle_t apo_create_circle_from_2point(const apo_point_t p1,
                                           const apo_point_t p2)
{
    return APO_CIRCLE2((p1.x + p2.x) / 2.0, (p1.y + p2.y) / 2.0,
                       pt_distance(p1, p2) / 2.0);
}

boolean apo_circle_contains_point(const apo_circle_t circle,
                                  const apo_point_t p)
{
    if (pt_distance(p, circle.center) < circle.radius) {
        return APO_TRUE;
    }
    return APO_FALSE;
}

void apo_intersection_cc(const apo_circle_t circle1, const apo_circle_t circle2,
                         apo_point_t *ips, int *ip_size)
{
    double r1 = circle1.radius;
    double r2 = circle2.radius;
    if (r1 < r2) {
        apo_intersection_cc(circle2, circle1, ips, ip_size);
        return;
    }

    apo_point_t c1 = circle1.center;
    apo_point_t c2 = circle2.center;

    apo_point_t u = pt_sub(c2, c1);
    double u_mag = pt_magnitude(u);

    // concentric
    if (u_mag < APO_TOLERANCE) {
        return;
    }

    double tol = (r1 + r2) / 200000;

    // the two circles (almost) touch externally / internally in one point
    // (tangent):
    if (fabs(u_mag - (r1 + r2)) < tol || fabs(u_mag - fabs(r1 - r2)) < tol) {
        pt_set_maguitude(&u, r1);
        ips[0] = pt_add(c1, u);
        *ip_size = 1;
        return;
    }

    apo_point_t v = APO_POINT(u.y, -u.x);

    double s, t1, t2, term;
    s = 1.0 / 2.0 * ((r1 * r1 - r2 * r2) / (pow(u_mag, 2.0)) + 1.0);

    term = (r1 * r1) / (pow(u_mag, 2.0))-s * s;

    // no intersection:
    if (term < 0.0) {
        return;
    }

    // one or two intersections:
    t1 = sqrt(term);
    t2 = -sqrt(term);

    apo_point_t sol1 = pt_add(pt_add(c1, pt_mul(u, s)), pt_mul(v, t1));
    apo_point_t sol2 = pt_add(pt_add(c1, pt_mul(u, s)), pt_mul(v, t2));

    if (fabs(sol1.x - sol2.x) < tol && fabs(sol1.y - sol2.y) < tol) {
        ips[0] = sol1;
        *ip_size = 1;
    }
    else {
        ips[0] = sol1;
        ips[1] = sol2;
        *ip_size = 2;
    }
}
