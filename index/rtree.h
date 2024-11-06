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

#ifndef CADA_RTREE_H
#define CADA_RTREE_H 1

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdbool.h>

typedef struct rtree_s rtree_t;

typedef bool (*user_data_clone)(const void *item, const void **into,
                                void *udata);
typedef void (*user_data_free)(const void *item, void *udata);
typedef bool (*rtree_cursor)(const double *min, const double *max,
                             const void *data, void *udata);
typedef int (*rtree_delete_compare)(const void *a, const void *b, void *udata);

rtree_t *rtree_new(void);
void rtree_free(rtree_t *rt);
void rtree_set_item_callbacks(rtree_t *rt, user_data_clone clone,
                              user_data_free free);
void rtree_set_udata(rtree_t *rt, void *udata);
bool rtree_insert(rtree_t *rt, const double *min, const double *max,
                  const void *data);
void rtree_search(const rtree_t *rt, const double *min, const double *max,
                  rtree_cursor cursor, void *udata);
void rtree_scan(const rtree_t *rt, rtree_cursor cursor, void *udata);
size_t rtree_count(const rtree_t *rt);
bool rtree_delete(rtree_t *rt, const double *min, const double *max,
                  const void *data);
bool rtree_delete_with_comparator(rtree_t *rt, const double *min,
                                  const double *max, const void *data,
                                  rtree_delete_compare compare, void *udata);

#ifdef __cplusplus
}
#endif

#endif