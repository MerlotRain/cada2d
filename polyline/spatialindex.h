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

#ifndef SPATIALINDEX_H
#define SPATIALINDEX_H

#include "arraylist.h"

typedef struct spatialIndex_s spatialIndex_t;
typedef bool (*spatial_index_boxes_visitor)(spatialIndex_t *, size_t, double,
                                            double, double, double);
typedef bool (*spatial_index_visitor)(spatialIndex_t *, size_t);

#define SPATIAL_DEFAULT_NODE_SIZE (16)

spatialIndex_t *spidx_create(size_t num_item, size_t node_size);
void spidx_free(const spatialIndex_t *idx);

double spidx_minx(const spatialIndex_t *idx);
double spidx_miny(const spatialIndex_t *idx);
double spidx_maxx(const spatialIndex_t *idx);
double spidx_maxy(const spatialIndex_t *idx);

void spidx_add(spatialIndex_t *idx, double minx, double miny, double maxx,
               double maxy);
void spidx_finish(spatialIndex_t *idx);
void spidx_visit_item_boxes(spatialIndex_t *idx,
                            spatial_index_boxes_visitor visitor);
void spidx_query(spatialIndex_t *idx, double minx, double miny, double maxx,
                 double maxy, array_t *result);
void spidx_query2(spatialIndex_t *idx, double minx, double miny, double maxx,
                  double maxy, array_t *result, array_t *stack);

#endif