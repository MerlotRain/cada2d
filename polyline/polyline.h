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

#ifndef POLYLINE_H
#define POLYLINE_H

#include <stddef.h>

typedef struct point_s {
    double x;
    double y;
} point_t;

typedef struct vertex_s {
    point_t pos;
    double bulge;
} vertex_t;

typedef struct polyline_s polyline_t;

polyline_t *polyline_create();
void polyline_free(polyline_t *t);

int polyline_append_vertex(polyline_t *poly, const vertex_t v);
vertex_t polyline_vertex_at(polyline_t *poly, size_t index);
size_t polyline_size(const polyline_t *poly);

polyline_t **polyline_parallel_offset(const polyline_t *pline, double offset,
                                      int hasSelfIntersects, int *res_size);

#endif