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

#ifndef APOLLONIUS_SOLUTION_H
#define APOLLONIUS_SOLUTION_H

typedef struct apollonius_circle {
    double cx;
    double cy;
    double r;
} apollonius_circle;

/*
 * apollonius problem solution
 */
typedef struct apollonius_solution {
    unsigned int count;         // circle count
    apollonius_circle *circles; // circle array
} apollonius_solution;

/**
 * @brief apollonius problem object
 *
 * This problem describes solving all circles obtained by plotting three
 * elements (any combination of points, lines, and circles) in a plane.
 *
 */
typedef struct apollonius_s apollonius_t;

apollonius_t *apollonius_init();
void apollonius_free(apollonius_t *apo);

/**
 * @brief add a point object
 * @return 1: add success; 0: add failure
 */
int apollonius_add_point(apollonius_t *apo, double x, double y);
/**
 * @brief add a line object
 * @return 1: add success; 0: add failure
 */
int apollonius_add_line(apollonius_t *apo, double x1, double y1, double x2,
                        double y2);
/**
 * @brief add a circle object
 * @return 1: add success; 0: add failure
 */
int apollonius_add_circle(apollonius_t *apo, double cx, double cy, double r);
/**
 * @brief claculate solution circles
 * @return 1: add success; 0: add failure
 */
int apollonius_solve(apollonius_t *apo, apollonius_solution **result);
void apollonius_solution_free(apollonius_solution *result);

#endif