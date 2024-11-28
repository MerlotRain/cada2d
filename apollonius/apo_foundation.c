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

#include "apo_foundation.h"
#include <math.h>
#include <stddef.h>
#include <float.h>

/* ------------------------- apo_circle_t functions ------------------------- */

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

/* -------------------------- apo_line_t functions -------------------------- */

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

/* -------------------------- apo_point_t functions ------------------------- */

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