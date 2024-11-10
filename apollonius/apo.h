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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

/* -------------------------------- apo point ------------------------------- */

extern double pt_magnitude(const apo_point_t p);

extern apo_point_t pt_create_polar(double r, double ang);
extern apo_point_t pt_average(const apo_point_t a, const apo_point_t b);
extern double pt_distance(const apo_point_t a, const apo_point_t b);
extern double pt_angle_to(const apo_point_t a, const apo_point_t b);
extern double pt_angle(const apo_point_t p);

extern apo_point_t pt_sub(const apo_point_t a, const apo_point_t b);
extern apo_point_t pt_add(const apo_point_t a, const apo_point_t b);

/* -------------------------------- apo line -------------------------------- */
extern int
intersection_ll(const apo_line_t l1, const apo_line_t l2, int limited,
                apo_point_t *ret); // limited: 0 for infinite, 1 for limited

extern int pt_on_line(const apo_line_t l, const apo_point_t p);

/* ------------------------------- apo circle ------------------------------- */
extern int apo_create_circle_from_3point(const apo_point_t p1,
                                         const apo_point_t p2,
                                         const apo_point_t p3,
                                         apo_circle_t *circle);

extern int pt_on_circle(const apo_circle_t c, const apo_point_t p);

#endif