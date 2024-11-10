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
#include "apo.h"
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
    result->count++;
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
    return 0;
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
    return 0;
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
    return 0;
}

int apollonius_solve(apollonius_t *apo, apollonius_solution **result)
{
    assert(apo);
    *result = (apollonius_solution *)apo_solution_real_new();
    if (*result == NULL) {
        return -1;
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

    return -1;

#undef object_classification

    return 0;
}

void apollonius_solution_free(apollonius_solution *result)
{
    assert(result);
    apo_solution_real_t *real = (apo_solution_real_t *)result;
    free(real->circle_objs);
    free(real);
}

/* ------------------------- static solve functions ------------------------- */

static int apollonius_solution_from_PPP(apo_object_t *point1,
                                        apo_object_t *point2,
                                        apo_object_t *point3,
                                        apo_solution_real_t *result);

static int apollonius_solution_from_PPC(apo_object_t *point1,
                                        apo_object_t *point2,
                                        apo_object_t *circle,
                                        apo_solution_real_t *result);

static int apollonius_solution_from_PPL(apo_object_t *point1,
                                        apo_object_t *point2,
                                        apo_object_t *line,
                                        apo_solution_real_t *result);

static int apollonius_solution_from_PCC(apo_object_t *point,
                                        apo_object_t *circle1,
                                        apo_object_t *circle2,
                                        apo_solution_real_t *result);

static int apollonius_solution_from_PLL(apo_object_t *point,
                                        apo_object_t *line1,
                                        apo_object_t *line2,
                                        apo_solution_real_t *result);

static int apollonius_solution_from_PLC(apo_object_t *point, apo_object_t *line,
                                        apo_object_t *circle,
                                        apo_solution_real_t *result);

static int apollonius_solution_from_LLL(apo_object_t *line1,
                                        apo_object_t *line2,
                                        apo_object_t *line3,
                                        apo_solution_real_t *result);

static int apollonius_solution_from_LLC(apo_object_t *line1,
                                        apo_object_t *line2,
                                        apo_object_t *circle,
                                        apo_solution_real_t *result);

static int apollonius_solution_from_LCC(apo_object_t *line,
                                        apo_object_t *circle2,
                                        apo_object_t *circle3,
                                        apo_solution_real_t *result);

static int apollonius_solution_from_CCC(apo_object_t *circle1,
                                        apo_object_t *circle2,
                                        apo_object_t *circle3,
                                        apo_solution_real_t *result);

/* --------------------------------- solves --------------------------------- */

int apollonius_solution_from_PPP(apo_object_t *point1, apo_object_t *point2,
                                 apo_object_t *point3,
                                 apo_solution_real_t *result)
{
    if (!APOLLONIUS_IS_POINT(point1) || !APOLLONIUS_IS_POINT(point2) ||
        !APOLLONIUS_IS_POINT(point3)) {
        return -1;
    }

    apo_circle_t c;
    if (0 == apo_create_circle_from_3point(point1->point, point2->point,
                                           point3->point, &c)) {
        apo_solution_real_append(result, (apollonius_circle){.cx = c.center.x,
                                                             .cy = c.center.y,
                                                             .r = c.radius});
        return 0;
    }
    return -1;
}

int apollonius_solution_from_PPC(apo_object_t *point1, apo_object_t *point2,
                                 apo_object_t *circle,
                                 apo_solution_real_t *result)
{
    if (!APOLLONIUS_IS_POINT(point1) || !APOLLONIUS_IS_POINT(point2) ||
        !APOLLONIUS_IS_CIRCLE(circle)) {
        return -1;
    }

        return -1;
}

int apollonius_solution_from_PPL(apo_object_t *point1, apo_object_t *point2,
                                 apo_object_t *line,
                                 apo_solution_real_t *result)
{
    return -1;
}

int apollonius_solution_from_PCC(apo_object_t *point, apo_object_t *circle1,
                                 apo_object_t *circle2,
                                 apo_solution_real_t *result)
{
    return -1;
}

int apollonius_solution_from_PLL(apo_object_t *point, apo_object_t *line1,
                                 apo_object_t *line2,
                                 apo_solution_real_t *result)
{
    return -1;
}

int apollonius_solution_from_PLC(apo_object_t *point, apo_object_t *line,
                                 apo_object_t *circle,
                                 apo_solution_real_t *result)
{
    return -1;
}

int apollonius_solution_from_LLL(apo_object_t *line1, apo_object_t *line2,
                                 apo_object_t *line3,
                                 apo_solution_real_t *result)
{
    return -1;
}

int apollonius_solution_from_LLC(apo_object_t *line1, apo_object_t *line2,
                                 apo_object_t *circle,
                                 apo_solution_real_t *result)
{
    return -1;
}

int apollonius_solution_from_LCC(apo_object_t *line, apo_object_t *circle2,
                                 apo_object_t *circle3,
                                 apo_solution_real_t *result)
{
    return -1;
}

int apollonius_solution_from_CCC(apo_object_t *circle1, apo_object_t *circle2,
                                 apo_object_t *circle3,
                                 apo_solution_real_t *result)
{
    return -1;
}