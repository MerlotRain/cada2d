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

#ifndef APOLLONIUS_GRAPHICS_H
#define APOLLONIUS_GRAPHICS_H

typedef int boolean;
#define APO_TRUE  (1)
#define APO_FALSE (0)

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define APO_TOLERANCE            (1.0e-9)

#define APOLLONIUS_IS_POINT(obj) ((obj) && (obj)->type == APOLLONIUS_POINT_TYPE)
#define APOLLONIUS_IS_LINE(obj)  ((obj) && (obj)->type == APOLLONIUS_LINE_TYPE)
#define APOLLONIUS_IS_CIRCLE(obj) \
    ((obj) && (obj)->type == APOLLONIUS_CIRCLE_TYPE)

#define APOLLONIUS_POINT_TYPE  (0)
#define APOLLONIUS_LINE_TYPE   (1)
#define APOLLONIUS_CIRCLE_TYPE (2)

typedef struct apo_point_s {
    double x;
    double y;
} apo_point_t;

typedef struct apo_line_s {
    apo_point_t begin_point;
    apo_point_t end_point;
} apo_line_t;

typedef struct apo_circle_s {
    apo_point_t center;
    double radius;
} apo_circle_t;

typedef struct apo_object_s {
    unsigned char type;
    union {
        apo_point_t point;
        apo_line_t line;
        apo_circle_t circle;
    };
} apo_object_t;

/* ----------------------------- apo constructor ---------------------------- */

#define APO_POINT(X, Y) \
    (apo_point_t)       \
    {                   \
        .x = X, .y = Y, \
    }
#define APO_LINE(B, E)                    \
    (apo_line_t)                          \
    {                                     \
        .begin_point = B, .end_point = E, \
    }
#define APO_LINE3(B, A, D) \
    APO_LINE(B, pt_add(B, apo_create_point_from_polar(A, D)))
#define APO_LINE4(A, B, C, D)       \
    (apo_line_t)                    \
    {                               \
        .begin_point =              \
            (apo_point_t){          \
                .x = A,             \
                .y = B,             \
            },                      \
        .end_point = (apo_point_t){ \
            .x = C,                 \
            .y = D,                 \
        },                          \
    }
#define APO_CIRCLE(C, R)          \
    (apo_circle_t)                \
    {                             \
        .center = C, .radius = R, \
    }
#define APO_CIRCLE2(X, Y, R) \
    (apo_circle_t)           \
    {                        \
        .center =            \
            (apo_point_t){   \
                .x = X,      \
                .y = Y,      \
            },               \
        .radius = R,         \
    }

#define APO_POINT_OBJ(p)                           \
    (apo_object_t)                                 \
    {                                              \
        .type = APOLLONIUS_POINT_TYPE, .point = p, \
    }
#define APO_POINT_OBJ2(X, Y)           \
    (apo_object_t)                     \
    {                                  \
        .type = APOLLONIUS_POINT_TYPE, \
        .point = (apo_point_t){        \
            .x = X,                    \
            .y = Y,                    \
        },                             \
    }
#define APO_LINE_OBJ(l)                          \
    (apo_object_t)                               \
    {                                            \
        .type = APOLLONIUS_LINE_TYPE, .line = l, \
    }
#define APO_LINE_OBJ2(l, B, E)        \
    (apo_object_t)                    \
    {                                 \
        .type = APOLLONIUS_LINE_TYPE, \
        .line = (apo_line_t){         \
            .begin_point = B,         \
            .end_point = E,           \
        },                            \
    }
#define APO_LINE_OBJ3(B, A, D)                                    \
    (apo_object_t)                                                \
    {                                                             \
        .type = APOLLONIUS_LINE_TYPE, .line = APO_LINE3(B, A, D), \
    }
#define APO_LINE_OBJ4(l, A, B, C, D)  \
    (apo_object_t)                    \
    {                                 \
        .type = APOLLONIUS_LINE_TYPE, \
        .line = (apo_line_t){         \
            .begin_point =            \
                (apo_point_t){        \
                    .x = A,           \
                    .y = B,           \
                },                    \
            .end_point =              \
                (apo_point_t){        \
                    .x = C,           \
                    .y = D,           \
                },                    \
        },                            \
    }
#define APO_CIRCLE_OBJ(c)                            \
    (apo_object_t)                                   \
    {                                                \
        .type = APOLLONIUS_CIRCLE_TYPE, .circle = c, \
    }
#define APO_CIRCLE_OBJ2(C, R)           \
    (apo_object_t)                      \
    {                                   \
        .type = APOLLONIUS_CIRCLE_TYPE, \
        .circle = (apo_circle_t){       \
            .center = C,                \
            .radius = R,                \
        },                              \
    }
#define APO_CIRCLE_OBJ3(X, Y, R)        \
    (apo_object_t)                      \
    {                                   \
        .type = APOLLONIUS_CIRCLE_TYPE, \
        .circle = (apo_circle_t){       \
            .center =                   \
                (apo_point_t){          \
                    .x = X,             \
                    .y = Y,             \
                },                      \
            .radius = R,                \
        },                              \
    }

extern apo_point_t apo_create_point_from_polar(double r, double ang);
extern boolean apo_create_circle_from_3point(const apo_point_t p1,
                                             const apo_point_t p2,
                                             const apo_point_t p3,
                                             apo_circle_t *circle);
extern apo_circle_t apo_create_circle_from_2point(const apo_point_t p1,
                                                  const apo_point_t p2);

/* -------------------------------- apo point ------------------------------- */

extern double pt_magnitude(const apo_point_t p);
extern apo_point_t pt_average(const apo_point_t a, const apo_point_t b);
extern double pt_distance(const apo_point_t a, const apo_point_t b);
extern double pt_angle_to(const apo_point_t a, const apo_point_t b);
extern double pt_angle(const apo_point_t p);
extern double pt_dot_product(const apo_point_t p1, const apo_point_t p2);
extern void pt_set_maguitude(apo_point_t *u, double r);

extern apo_point_t pt_sub(const apo_point_t a, const apo_point_t b);
extern apo_point_t pt_add(const apo_point_t a, const apo_point_t b);
extern apo_point_t pt_mul(const apo_point_t p, const double s);

extern boolean pt_on_line(const apo_line_t l, const apo_point_t p,
                          boolean limited);
extern boolean pt_on_circle(const apo_circle_t c, const apo_point_t p);
extern boolean pt_equal(const apo_point_t p1, const apo_point_t p2);

/* -------------------------------- apo line -------------------------------- */
extern boolean
apo_intersection_ll(const apo_line_t l1, const apo_line_t l2, boolean limited,
                    apo_point_t *ret); // limited: 0 for infinite, 1 for limited

extern boolean apo_intersection_lc(const apo_line_t l, const apo_circle_t c,
                                   boolean limited, apo_point_t *rets,
                                   int *ret_size);

extern boolean apo_line_closest_point(const apo_line_t l, const apo_point_t p,
                                      boolean limited, apo_point_t *closest);

extern double apo_line_angle(const apo_line_t l);
/* ------------------------------- apo circle ------------------------------- */
extern boolean apo_circle_contains_point(const apo_circle_t circle,
                                         const apo_point_t p);
extern void apo_intersection_cc(const apo_circle_t circle1,
                                const apo_circle_t circle2, apo_point_t *ips,
                                int *ip_size);

#endif