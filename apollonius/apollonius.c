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

#include "apollonius.h"
#include "apo_foundation.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#define APOLLONIUS_CIRCLE_CAPACITY      (5)
#define APOLLONIUS_CIRCLE_EXPAND_FACTOR (2)

/* ---------------- apollonius inner structures and functions --------------- */

struct apollonius_s {
    apo_object_t *obj[3];
    size_t obj_size;
};

typedef struct apo_solution_real_s {
    size_t count;
    apollonius_circle *circle_objs;
    size_t capacity;
} apo_solution_real_t;

/* ------------------------- static solve functions ------------------------- */

static boolean apo_inverse_shape(const apo_object_t *shp,
                                 const apo_object_t *inversion_circle,
                                 apo_object_t *inversed);

static void apo_circle_tangents_through_point(const apo_circle_t circle,
                                              const apo_point_t p,
                                              apo_line_t *lines,
                                              int *line_size);

static void apo_tangents(const apo_object_t *obj1, const apo_object_t *obj2,
                         apo_line_t *lines, int *line_size);

static boolean apollonius_solution_from_PPP(apo_object_t *point1,
                                            apo_object_t *point2,
                                            apo_object_t *point3,
                                            apo_solution_real_t *result);

static boolean apollonius_solution_from_PPC(apo_object_t *point1,
                                            apo_object_t *point2,
                                            apo_object_t *circle,
                                            apo_solution_real_t *result);

static boolean apollonius_solution_from_PPL(apo_object_t *point1,
                                            apo_object_t *point2,
                                            apo_object_t *line,
                                            apo_solution_real_t *result);

static boolean apollonius_solution_from_PCC(apo_object_t *point,
                                            apo_object_t *circle1,
                                            apo_object_t *circle2,
                                            apo_solution_real_t *result);

static boolean apollonius_solution_from_PLL(apo_object_t *point,
                                            apo_object_t *line1,
                                            apo_object_t *line2,
                                            apo_solution_real_t *result);

static boolean apollonius_solution_from_PLC(apo_object_t *point,
                                            apo_object_t *line,
                                            apo_object_t *circle,
                                            apo_solution_real_t *result);

static boolean apollonius_solution_from_LLL(apo_object_t *line1,
                                            apo_object_t *line2,
                                            apo_object_t *line3,
                                            apo_solution_real_t *result);

static boolean apollonius_solution_from_LLC(apo_object_t *line1,
                                            apo_object_t *line2,
                                            apo_object_t *circle,
                                            apo_solution_real_t *result);

static boolean apollonius_solution_from_LCC(apo_object_t *line,
                                            apo_object_t *circle2,
                                            apo_object_t *circle3,
                                            apo_solution_real_t *result);

static boolean apollonius_solution_from_CCC(apo_object_t *circle1,
                                            apo_object_t *circle2,
                                            apo_object_t *circle3,
                                            apo_solution_real_t *result);

/* -------------------- apollonius result real functions -------------------- */

static apo_solution_real_t *apo_solution_real_new()
{
    apo_solution_real_t *real =
        (apo_solution_real_t *)malloc(sizeof(apo_solution_real_t));
    if (!real) {
        return NULL;
    }
    real->circle_objs = (apollonius_circle *)malloc(APOLLONIUS_CIRCLE_CAPACITY *
                                                    sizeof(apollonius_circle));
    if (!real->circle_objs) {
        free(real);
        return NULL;
    }
    real->capacity = APOLLONIUS_CIRCLE_CAPACITY;
    real->count = 0;
    return real;
}

static void apo_solution_real_append(apo_solution_real_t *result,
                                     const apollonius_circle circle)
{
    assert(result);
    if (result->count >= result->capacity) {
        result->circle_objs =
            realloc(result->circle_objs, result->capacity *
                                             APOLLONIUS_CIRCLE_EXPAND_FACTOR *
                                             sizeof(apollonius_circle));
        if (!result->circle_objs) {
            return;
        }
        result->capacity *= APOLLONIUS_CIRCLE_EXPAND_FACTOR;
    }
    result->circle_objs[result->count++] = circle;
}

apollonius_t *apollonius_init()
{
    apollonius_t *apollonius = (apollonius_t *)malloc(sizeof(apollonius_t));
    if (!apollonius) {
        return NULL;
    }
    apollonius->obj_size = 0;
    for (size_t i = 0; i < 3; i++) {
        apollonius->obj[i] = (apo_object_t *)malloc(sizeof(apo_object_t));
    }
    return apollonius;
}

void apollonius_free(apollonius_t *apo)
{
    assert(apo);
    for (size_t i = 0; i < 3; i++) {
        free(apo->obj[i]);
    }
    free(apo);
}

int apollonius_add_point(apollonius_t *apo, double x, double y)
{
    assert(apo);
    assert(apo->obj_size < 3);

    apo->obj[apo->obj_size]->type = APOLLONIUS_POINT_TYPE;
    apo->obj[apo->obj_size]->point.x = x;
    apo->obj[apo->obj_size]->point.y = y;
    apo->obj_size++;
    return APO_TRUE;
}

int apollonius_add_line(apollonius_t *apo, double x1, double y1, double x2,
                        double y2)
{
    assert(apo);
    assert(apo->obj_size < 3);
    apo->obj[apo->obj_size]->type = APOLLONIUS_LINE_TYPE;
    apo->obj[apo->obj_size]->line.begin_point.x = x1;
    apo->obj[apo->obj_size]->line.begin_point.y = y1;
    apo->obj[apo->obj_size]->line.end_point.x = x2;
    apo->obj[apo->obj_size]->line.end_point.y = y2;
    apo->obj_size++;
    return APO_TRUE;
}

int apollonius_add_circle(apollonius_t *apo, double cx, double cy, double r)
{
    assert(apo);
    assert(apo->obj_size < 3);
    apo->obj[apo->obj_size]->type = APOLLONIUS_CIRCLE_TYPE;
    apo->obj[apo->obj_size]->circle.center.x = cx;
    apo->obj[apo->obj_size]->circle.center.y = cy;
    apo->obj[apo->obj_size]->circle.radius = r;
    apo->obj_size++;
    return APO_TRUE;
}

int apollonius_solve(apollonius_t *apo, apollonius_solution **result)
{
    assert(apo);
    *result = (apollonius_solution *)apo_solution_real_new();
    if (*result == NULL) {
        return APO_FALSE;
    }

    assert(apo->obj_size == 3);

    int point_objs_count = 0;
    int line_objs_count = 0;
    int circle_objs_count = 0;
    apo_object_t *point_objs[3];
    apo_object_t *line_objs[3];
    apo_object_t *circle_objs[3];
#define object_classification(obj)              \
    switch (obj->type) {                        \
    case APOLLONIUS_POINT_TYPE:                 \
        point_objs[point_objs_count++] = obj;   \
        break;                                  \
    case APOLLONIUS_LINE_TYPE:                  \
        line_objs[line_objs_count++] = obj;     \
        break;                                  \
    case APOLLONIUS_CIRCLE_TYPE:                \
        circle_objs[circle_objs_count++] = obj; \
        break;                                  \
    };

    object_classification(apo->obj[0]);
    object_classification(apo->obj[1]);
    object_classification(apo->obj[2]);

    if (point_objs_count == 3) {
        return apollonius_solution_from_PPP(point_objs[0], point_objs[1],
                                            point_objs[2], *result);
    }

    else if (point_objs_count == 2) {
        if (circle_objs_count == 1) {
            return apollonius_solution_from_PPC(point_objs[0], point_objs[1],
                                                circle_objs[0], *result);
        }
        else if (line_objs_count == 1) {
            return apollonius_solution_from_PPL(point_objs[0], point_objs[1],
                                                line_objs[0], *result);
        }
    }

    else if (point_objs_count == 1) {
        if (circle_objs_count == 2) {
            return apollonius_solution_from_PCC(point_objs[0], circle_objs[0],
                                                circle_objs[1], *result);
        }
        else if (line_objs_count == 2) {
            return apollonius_solution_from_PLL(point_objs[0], line_objs[0],
                                                line_objs[1], *result);
        }
        else if (circle_objs_count == 1 && line_objs_count == 1) {
            return apollonius_solution_from_PLC(point_objs[0], line_objs[0],
                                                circle_objs[0], *result);
        }
    }

    else if (point_objs_count == 0) {
        if (line_objs_count == 3) {
            return apollonius_solution_from_LLL(line_objs[0], line_objs[1],
                                                line_objs[2], *result);
        }
        else if (line_objs_count == 2 && circle_objs_count == 1) {
            return apollonius_solution_from_LLC(line_objs[0], line_objs[1],
                                                circle_objs[0], *result);
        }
        else if (line_objs_count == 1 && circle_objs_count == 2) {
            return apollonius_solution_from_LCC(line_objs[0], circle_objs[0],
                                                circle_objs[1], *result);
        }
        else if (circle_objs_count == 3) {
            return apollonius_solution_from_CCC(circle_objs[0], circle_objs[1],
                                                circle_objs[2], *result);
        }
    }

    return APO_FALSE;

#undef object_classification

    return APO_TRUE;
}

void apollonius_solution_free(apollonius_solution *result)
{
    assert(result);
    apo_solution_real_t *real = (apo_solution_real_t *)result;
    free(real->circle_objs);
    free(real);
}

/* --------------------------------- solves --------------------------------- */

/// http://www.geometer.org/mathcircles/inversion.pdf
boolean apo_inverse_shape(const apo_object_t *shp,
                          const apo_object_t *inversion_circle,
                          apo_object_t *inversed)
{
    assert(shp);
    assert(inversion_circle);
    assert(inversed);

    // inverse point
    // https://mathworld.wolfram.com/InversePoints.html
    //
    // Points, also called polar reciprocals, which are transformed into each
    // other through inversion about a given inversion circle C (or inversion
    // sphere). The points P and P^' are inverse points with respect to the
    // inversion circle if
    // OP * OP' = r^2
    // In this case, P^' is called the inversion pole and the line L through P
    // and perpendicular to OP is called the polar. In the above figure, the
    // quantity r^2 is called the circle power of the point P relative to the
    // circle C.
    if (APOLLONIUS_IS_POINT(shp)) {
        double r = inversion_circle->circle.radius;
        apo_point_t center = inversion_circle->circle.center;
        double d = pt_distance(shp->point, center);
        if (fabs(d) < APO_TOLERANCE) {
            *inversed = APO_POINT_OBJ(shp->point);
            return APO_TRUE;
        }

        double d_inverse = pow(r, 2) / d;
        *inversed = APO_POINT_OBJ2(
            center.x + (shp->point.x - center.x) * d_inverse / d,
            center.y + (shp->point.y - center.y) * d_inverse / d);
        return APO_TRUE;
    }
    // line inverse
    if (APOLLONIUS_IS_LINE(shp)) {
        apo_point_t center = inversion_circle->circle.center;
        // A “line” that passes through O is inverted to itself. Note, of course
        // that the individual points of the “line” are inverted to other points
        // on the “line” except for the two points where it passes through k.
        if (pt_on_line(shp->line, center, APO_FALSE)) {
            *inversed = APO_LINE_OBJ(shp->line);
            return APO_TRUE;
        }
        else {
            // Every “line” that does not pass through O is inverted to a circle
            // (no quotes: a real circle) that passes through O.
            apo_line_t s;
            s.begin_point = center;
            apo_line_closest_point(shp->line, center, APO_FALSE, &s.end_point);
            // intersection_points from shp.line and s
            apo_point_t p;
            if (apo_intersection_ll(shp->line, s, APO_FALSE, &p)) {
                apo_object_t pinverse;
                if (apo_inverse_shape(&(APO_POINT_OBJ(p)), inversion_circle,
                                      &pinverse)) {
                    *inversed = APO_CIRCLE_OBJ(
                        apo_create_circle_from_2point(center, p));
                    return APO_TRUE;
                }
            }
        }
    }

    if (APOLLONIUS_IS_CIRCLE(shp)) {
        /// concentric circles
        apo_circle_t circle = shp->circle;
        if (pt_equal(circle.center, inversion_circle->circle.center)) {
            apo_object_t inversed_point_obj;
            apo_inverse_shape(&(APO_POINT_OBJ2(circle.center.x + circle.radius,
                                               circle.center.y)),
                              inversion_circle, &inversed_point_obj);
            double radius = circle.center.x - inversed_point_obj.point.x;
            if (radius < 0) {
                radius = fabs(radius);
            }
            *inversed = APO_CIRCLE_OBJ2(circle.center, radius);
            return APO_TRUE;
        }
        else if (pt_on_circle(circle, inversion_circle->circle.center)) {
            apo_line_t s =
                APO_LINE(inversion_circle->circle.center, circle.center);
            apo_point_t ips[2];
            int ips_size = 0;
            if (apo_intersection_lc(s, circle, APO_FALSE, ips, &ips_size)) {
                return APO_FALSE;
            }

            apo_point_t p = ips[0];
            if (pt_equal(p, inversion_circle->circle.center)) {
                if (ips_size < 2) {
                    return APO_FALSE;
                }
                p = ips[1];
            }

            apo_object_t pinverse;
            if (!apo_inverse_shape(&(APO_POINT_OBJ(p)), inversion_circle,
                                   &pinverse)) {
                return APO_FALSE;
            }
            *inversed = APO_LINE_OBJ3(pinverse.point,
                                      apo_line_angle(s) + M_PI / 2.0, 1.0);
            return APO_TRUE;
        }
        else {
            apo_line_t l =
                APO_LINE(inversion_circle->circle.center, circle.center);
            apo_point_t ips[2];
            int ips_size = 0;
            if (!apo_intersection_lc(l, circle, APO_FALSE, ips, &ips_size)) {
                return APO_FALSE;
            }

            apo_point_t p1 = ips[0];
            apo_point_t p2 = ips[1];

            apo_object_t p1inverse, p2inverse;
            if (!apo_inverse_shape(&(APO_POINT_OBJ(p1)), inversion_circle,
                                   &p1inverse)) {
                return APO_FALSE;
            }
            if (!apo_inverse_shape(&(APO_POINT_OBJ(p2)), inversion_circle,
                                   &p2inverse)) {
                return APO_FALSE;
            }
            *inversed = APO_CIRCLE_OBJ(apo_create_circle_from_2point(
                p1inverse.point, p2inverse.point));
            return APO_TRUE;
        }
    }

    return APO_FALSE;
}

void apo_circle_tangents_through_point(const apo_circle_t circle,
                                       const apo_point_t p, apo_line_t *lines,
                                       int *line_size)
{
    // used when creating tangential circles to two parallel lines and point:
    if (fabs(circle.radius) < APO_TOLERANCE) {
        lines[0] = APO_LINE(p, circle.center);
        lines[1] = APO_LINE(p, circle.center);
        *line_size = 2;
        return;
    }
    // point on the circle line (produces error):
    else if (pt_on_circle(circle, p)) {
        apo_line_t s = APO_LINE(p, circle.center);
        lines[0] = APO_LINE3(p, apo_line_angle(s) + M_PI / 2.0, 1.0);
        *line_size = 1;
        return;
    }
    // pointis inside the circle:
    else if (apo_circle_contains_point(circle, p)) {
        return;
    }
    // point outside circle:
    else {
        apo_circle_t circle2 = apo_create_circle_from_2point(p, circle.center);
        apo_point_t touching_points[2];
        int touching_size = 0;
        apo_intersection_cc(circle2, circle, touching_points, &touching_size);
        if (touching_size == 1) {
            lines[0] = APO_LINE(p, touching_points[0]);
            *line_size = 1;
        }
        else if (touching_size == 2) {
            lines[0] = APO_LINE(p, touching_points[0]);
            lines[1] = APO_LINE(p, touching_points[1]);
            *line_size = 2;
        }
    }
}

void apo_tangents(const apo_object_t *obj1, const apo_object_t *obj2,
                  apo_line_t *lines, int *line_size)
{
    assert(obj1);
    assert(obj2);

    if (APOLLONIUS_IS_CIRCLE(obj1) && APOLLONIUS_IS_CIRCLE(obj2)) {
        apo_circle_t c1 = obj1->circle;
        apo_circle_t c2 = obj2->circle;
        if (obj1->circle.radius < APO_TOLERANCE) {
            c1.radius = 0.0;
        }
        if (obj2->circle.radius < APO_TOLERANCE) {
            c2.radius = 0.0;
        }
        if (c1.radius > c2.radius) {
            apo_circle_t tmp = c1;
            c1 = c2;
            c2 = tmp;
        }

        double c1Radius = c1.radius;
        double c2Radius = c2.radius;
        apo_point_t c1Center = c1.center;
        apo_point_t c2Center = c2.center;

        double dc1c2 = pt_distance(c1Center, c2Center);
        if (dc1c2 < 1.0e-6)
            return;

        // internally touching circles:
        if (fabs(dc1c2 + c1Radius - c2Radius) < APO_TOLERANCE) {
            apo_line_t tangent = APO_LINE(c2Center, c1Center);
            // with 2 radii larger than zero:
            if (c1Radius > 0.0) {
                apo_set_line_length(&tangent, (dc1c2 + c1Radius) * 2.0,
                                    APO_TRUE);
            }
            else {
                apo_set_line_length(&tangent, c2Radius * 2, APO_TRUE);
            }
            apo_set_line_rotate(&tangent, M_PI / 2.0,
                                apo_line_middle_point(tangent));

            lines[0] = tangent;
            *line_size = 1;
            return;
        }
    }
}

boolean apollonius_solution_from_PPP(apo_object_t *point1, apo_object_t *point2,
                                     apo_object_t *point3,
                                     apo_solution_real_t *result)
{
    if (!APOLLONIUS_IS_POINT(point1) || !APOLLONIUS_IS_POINT(point2) ||
        !APOLLONIUS_IS_POINT(point3)) {
        return APO_FALSE;
    }

    apo_circle_t c;
    if (0 == apo_create_circle_from_3point(point1->point, point2->point,
                                           point3->point, &c)) {
        apo_solution_real_append(result, (apollonius_circle){.cx = c.center.x,
                                                             .cy = c.center.y,
                                                             .r = c.radius});
        return APO_TRUE;
    }
    return APO_FALSE;
}

boolean apollonius_solution_from_PPC(apo_object_t *point1, apo_object_t *point2,
                                     apo_object_t *circle,
                                     apo_solution_real_t *result)
{
    if (!APOLLONIUS_IS_POINT(point1) || !APOLLONIUS_IS_POINT(point2) ||
        !APOLLONIUS_IS_CIRCLE(circle)) {
        return APO_FALSE;
    }

    if (0 == pt_on_circle(circle->circle, point1->point) &&
        0 == pt_on_circle(circle->circle, point2->point)) {
        apo_solution_real_append(
            result, (apollonius_circle){.cx = circle->circle.center.x,
                                        circle->circle.center.y,
                                        circle->circle.radius});
        return APO_TRUE;
    }

    apo_point_t p1;
    apo_point_t p2;
    if (0 == pt_on_circle(circle->circle, point1->point)) {
        p1 = point1->point;
        p2 = point2->point;
        goto point_on_circle;
    }
    if (0 == pt_on_circle(circle->circle, point2->point)) {
        p1 = point2->point;
        p2 = point1->point;
        goto point_on_circle;
    }

    apo_object_t inversion_cirlce;
    inversion_cirlce.type = APOLLONIUS_CIRCLE_TYPE;
    inversion_cirlce.circle.center = p1;
    inversion_cirlce.circle.radius = 10.0;
    apo_object_t circle_inverse;
    apo_object_t point2_inverse;
    if (0 != apo_inverse_shape(circle, &inversion_cirlce, &circle_inverse) ||
        0 != apo_inverse_shape(point2, &inversion_cirlce, &point2_inverse)) {
        return APO_FALSE;
    }

    apo_line_t lines[2];
    int lines_size = 0;
    apo_circle_tangents_through_point(circle_inverse.circle,
                                      point2_inverse.point, lines, &lines_size);
    for (int i = 0; i < lines_size; ++i) {
        apo_object_t res_circle;
        apo_inverse_shape(&(APO_LINE_OBJ(lines[i])), &inversion_cirlce,
                          &res_circle);
        apo_solution_real_append(
            result, (apollonius_circle){.cx = res_circle.circle.center.x,
                                        .cy = res_circle.circle.center.y,
                                        .r = res_circle.circle.radius});
    }
    return APO_TRUE;

point_on_circle: {
    apo_line_t l1 = APO_LINE(circle->circle.center, p1);
    apo_point_t m = pt_average(p1, p2);
    apo_line_t l2 = APO_LINE3(m, pt_angle_to(p2, p1) + M_PI / 2.0, 1.0);

    apo_point_t ip;
    if (apo_intersection_ll(l1, l2, 0, &ip)) {
        return APO_FALSE;
    }
    apo_solution_real_append(
        result,
        (apollonius_circle){.cx = ip.x, .cy = ip.y, pt_distance(ip, p1)});
    return APO_TRUE;
}
}

boolean apollonius_solution_from_PPL(apo_object_t *point1, apo_object_t *point2,
                                     apo_object_t *line,
                                     apo_solution_real_t *result)
{
    if (!APOLLONIUS_IS_POINT(point1) || !APOLLONIUS_IS_POINT(point2) ||
        !APOLLONIUS_IS_LINE(line))
        return APO_FALSE;

    if (pt_on_line(line->line, point1->point, APO_TRUE)) {
        apo_point_t tmp = point1->point;
        point1->point = point2->point;
        point2->point = tmp;
        if (pt_on_line(line->line, point1->point, APO_TRUE)) {
            return APO_FALSE;
        }
    }

    apo_object_t inversion_cirlce = APO_CIRCLE_OBJ2(point1->point, 10);
    apo_object_t line_inverse;
    apo_object_t point2_inverse;
    if (0 != apo_inverse_shape(line, &inversion_cirlce, &line_inverse) ||
        0 != apo_inverse_shape(point2, &inversion_cirlce, &point2_inverse)) {
        return APO_FALSE;
    }

    apo_line_t lines[2];
    int lines_size = 0;
    apo_circle_tangents_through_point(line_inverse.circle, point2_inverse.point,
                                      lines, &lines_size);
    for (int i = 0; i < lines_size; ++i) {
        apo_object_t res_circle;
        apo_inverse_shape(&(APO_LINE_OBJ(lines[i])), &inversion_cirlce,
                          &res_circle);
        apo_solution_real_append(
            result, (apollonius_circle){.cx = res_circle.circle.center.x,
                                        .cy = res_circle.circle.center.y,
                                        .r = res_circle.circle.radius});
    }
    return APO_TRUE;
}

boolean apollonius_solution_from_PCC(apo_object_t *point, apo_object_t *circle1,
                                     apo_object_t *circle2,
                                     apo_solution_real_t *result)
{
    if (!APOLLONIUS_IS_POINT(point) || !APOLLONIUS_IS_CIRCLE(circle1) ||
        !APOLLONIUS_IS_CIRCLE(circle2))
        return APO_FALSE;

    // relative sized inversion circle:
    double r_inv = circle1->circle.radius;
    if (circle2->circle.radius > circle1->circle.radius) {
        r_inv = circle2->circle.radius;
    }
    apo_object_t inversion_circle = APO_CIRCLE_OBJ2(point->point, r_inv);

    // construct inversion shape:
    apo_object_t c1inverse;
    apo_object_t c2inverse;
    if (!apo_inverse_shape(circle1, &inversion_circle, &c1inverse) ||
        !apo_inverse_shape(circle2, &inversion_circle, &c2inverse)) {
        return APO_FALSE;
    }
}

boolean apollonius_solution_from_PLL(apo_object_t *point, apo_object_t *line1,
                                     apo_object_t *line2,
                                     apo_solution_real_t *result)
{
    return APO_FALSE;
}

boolean apollonius_solution_from_PLC(apo_object_t *point, apo_object_t *line,
                                     apo_object_t *circle,
                                     apo_solution_real_t *result)
{
    return APO_FALSE;
}

boolean apollonius_solution_from_LLL(apo_object_t *line1, apo_object_t *line2,
                                     apo_object_t *line3,
                                     apo_solution_real_t *result)
{
    return APO_FALSE;
}

boolean apollonius_solution_from_LLC(apo_object_t *line1, apo_object_t *line2,
                                     apo_object_t *circle,
                                     apo_solution_real_t *result)
{
    return APO_FALSE;
}

boolean apollonius_solution_from_LCC(apo_object_t *line, apo_object_t *circle2,
                                     apo_object_t *circle3,
                                     apo_solution_real_t *result)
{
    return APO_FALSE;
}

boolean apollonius_solution_from_CCC(apo_object_t *circle1,
                                     apo_object_t *circle2,
                                     apo_object_t *circle3,
                                     apo_solution_real_t *result)
{
    return APO_FALSE;
}