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

#ifndef QUICKINDEX_H
#define QUICKINDEX_H

#include "arraylist.h"

#define QINDEX_DEFAULT_NODE_SIZE (16)

typedef struct quickIndex_s quickIndex_t;
typedef bool (*qindex_IB_visitor)(size_t, double, double, double, double,
                                  void *);
typedef bool (*qindex_Q_visitor)(size_t, void *);

struct quickIndex_s {
    double minX;
    double minY;
    double maxX;
    double maxY;
};

quickIndex_t *qindex_create(size_t num_item, size_t node_size);
void qindex_free(quickIndex_t *idx);

void qindex_add(quickIndex_t *idx, double minx, double miny, double maxx,
                double maxy);
void qindex_finish(quickIndex_t *idx);
void qindex_visit_item_boxes(quickIndex_t *idx, void *data,
                             qindex_IB_visitor visitor);
void qindex_query(quickIndex_t *idx, double minx, double miny, double maxx,
                  double maxy, array_t *result);
void qindex_query2(quickIndex_t *idx, double minx, double miny, double maxx,
                   double maxy, array_t *result, array_t *stack);
void qindex_visitor_query(quickIndex_t *idx, double minx, double miny,
                          double maxx, double maxy, void *data,
                          qindex_Q_visitor visitor);
void qindex_visitor_query2(quickIndex_t *idx, double minx, double miny,
                           double maxx, double maxy, void *data,
                           qindex_Q_visitor visitor, array_t *stack);

#endif
