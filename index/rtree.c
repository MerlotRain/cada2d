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

#include "rtree.h"
#include <assert.h>
#include <math.h>
#include <float.h>
#include <stdlib.h>
#include <string.h>

#define MAX_ITEM    (64)
#define MAX(a, b)   ((a) > (b) ? (a) : (b))
#define MIN(a, b)   ((a) > (b) ? (b) : (a))
#define EQUAL(a, b) (fabs(a - b) < DBL_EPSILON)

typedef enum {
    contains,
    intersects,
    onedge,
    equal,
    equal_bin,
} rect_relation;

typedef struct rect_s {
    double min[2];
    double max[2];
} rect_t;

typedef enum {
    leaf,
    branch,
} node_kind;

typedef struct item_s {
    const void *data;
} item_t;

typedef struct node_s node_t;

typedef struct node_s {
    node_kind kind;
    size_t count;
    rect_t rect[MAX_ITEM];
    union {
        node_t *nodes[MAX_ITEM];
        item_t datas[MAX_ITEM];
    };
} node_t;

struct rtree_s {
    rect_t rect;
    node_t *root;
    size_t count;
    size_t height;
    int path_hint[16];
    void *udata;
    user_data_clone item_clone;
    user_data_free item_free;
};

/* ---------------------------- static functions ---------------------------- */

static void rect_expand(rect_t *rect, const rect_t *rhs);
static double rect_area(const rect_t *rect);
static double rect_union_area(const rect_t *r1, const rect_t *r2);
static bool rect_relations(const rect_t *r1, const rect_t *r2,
                           rect_relation relation);
static int rect_largest_axis(const rect_t *rect);

static node_t *node_new(rtree_t *rt, node_kind kind);
static void node_free(rtree_t *rt, node_t *node);
static void node_swap(node_t *node, int i, int j);
static void node_qsort(node_t *node, int s, int e, int index);
static void node_sort_by_axis(node_t *node, int axis, bool max);
static void node_move_rect_at_index_into(node_t *from, int index, node_t *into);
static bool node_split_largest_axis_edge_snap(rtree_t *rt, rect_t *rect,
                                              node_t *node, node_t **right_out);
static bool node_split(rtree_t *rt, rect_t *rect, node_t *node,
                       node_t **right_out);
static int node_choose_least_enlargement(const node_t *node,
                                         const rect_t *rect);
static int node_choose(rtree_t *rt, const node_t *node, const rect_t *rect,
                       int depth);
static rect_t node_rect_calc(const node_t *node);
static bool node_insert(rtree_t *rt, rect_t *nr, node_t *node, rect_t *ir,
                        item_t item, int depth, bool *split);
static bool node_search(node_t *node, rect_t *rect, rtree_cursor cursor,
                        void *udata);
static bool node_scan(node_t *node, rtree_cursor cursor, void *udata);
static bool node_delete(rtree_t *rt, rect_t *nr, node_t *node, rect_t *ir,
                        item_t item, int depth, bool *removed, bool *shrunk,
                        rtree_delete_compare compare, void *udata);
static bool rtree_delete0(rtree_t *rt, const double *min, const double *max,
                          const void *data, rtree_delete_compare compare,
                          void *udata);

/* ---------------------------------- impls --------------------------------- */

rtree_t *rtree_new(void)
{
    rtree_t *rt = (rtree_t *)malloc(sizeof(rtree_t));
    if (rt == NULL)
        return NULL;
    memset(rt, 0, sizeof(rtree_t));
    return rt;
}

void rtree_free(rtree_t *rt)
{
}

void rtree_set_item_callbacks(rtree_t *rt, user_data_clone clone,
                              user_data_free free)
{
    assert(rt);
    rt->item_clone = clone;
    rt->item_free = free;
}

void rtree_set_udata(rtree_t *rt, void *udata)
{
}

bool rtree_insert(rtree_t *rt, const double *min, const double *max,
                  const void *data)
{
    assert(rt);
    // copy input rect
    rect_t rect;
    memcpy(&rect.min, min, sizeof(double) * 2);
    memcpy(&rect.max, max, sizeof(double) * 2);

    item_t item;
    if (rt->item_clone) {
        if (!rt->item_clone(data, (void *)(&item.data), rt->udata)) {
            return false;
        }
    }
    else {
        memcpy(&item.data, &data, sizeof(void *));
    }
    return false;
}

void rtree_search(const rtree_t *rt, const double *min, const double *max,
                  rtree_cursor cursor, void *udata)
{
}

void rtree_scan(const rtree_t *rt, rtree_cursor cursor, void *udata)
{
}

size_t rtree_count(const rtree_t *rt)
{
    return 0;
}

bool rtree_delete(rtree_t *rt, const double *min, const double *max,
                  const void *data)
{
    return false;
}

bool rtree_delete_with_comparator(rtree_t *rt, const double *min,
                                  const double *max, const void *data,
                                  rtree_delete_compare compare, void *udata)
{
    return false;
}

/* -------------------------- static function impls ------------------------- */

void rect_expand(rect_t *rect, const rect_t *rhs)
{
}

double rect_area(const rect_t *rect)
{
    return 0.0;
}

double rect_union_area(const rect_t *r1, const rect_t *r2)
{
    return 0.0;
}

bool rect_relations(const rect_t *r1, const rect_t *r2, rect_relation relation)
{
    return false;
}

int rect_largest_axis(const rect_t *rect)
{
    return 0;
}

node_t *node_new(rtree_t *rt, node_kind kind)
{
    return NULL;
}

void node_free(rtree_t *rt, node_t *node)
{
}

void node_swap(node_t *node, int i, int j)
{
}

void node_qsort(node_t *node, int s, int e, int index)
{
}

void node_sort_by_axis(node_t *node, int axis, bool max)
{
}

void node_move_rect_at_index_into(node_t *from, int index, node_t *into)
{
}

bool node_split_largest_axis_edge_snap(rtree_t *rt, rect_t *rect, node_t *node,
                                       node_t **right_out)
{
    return false;
}

bool node_split(rtree_t *rt, rect_t *rect, node_t *node, node_t **right_out)
{
    return false;
}

int node_choose_least_enlargement(const node_t *node, const rect_t *rect)
{
    return 0;
}

int node_choose(rtree_t *rt, const node_t *node, const rect_t *rect, int depth)
{
    return 0;
}

rect_t node_rect_calc(const node_t *node)
{
    rect_t rect;
    return rect;
}

bool node_insert(rtree_t *rt, rect_t *nr, node_t *node, rect_t *ir, item_t item,
                 int depth, bool *split)
{
    return false;
}

bool node_search(node_t *node, rect_t *rect, rtree_cursor cursor, void *udata)
{
    return false;
}

bool node_scan(node_t *node, rtree_cursor cursor, void *udata)
{
    return false;
}

bool node_delete(rtree_t *rt, rect_t *nr, node_t *node, rect_t *ir, item_t item,
                 int depth, bool *removed, bool *shrunk,
                 rtree_delete_compare compare, void *udata)
{
    return false;
}

bool rtree_delete0(rtree_t *rt, const double *min, const double *max,
                   const void *data, rtree_delete_compare compare, void *udata)
{
    return false;
}