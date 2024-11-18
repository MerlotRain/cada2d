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

#ifndef ARRAY_LIST_H
#define ARRAY_LIST_H

#include <stddef.h>
#include <stdbool.h>

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

typedef void (*array_clear_callback)(void *);
typedef struct array_s {
    char *data;
    size_t size;
    size_t capacity;
    size_t ele_size;
    array_clear_callback clear_func;
} array_t;

array_t *array_new(size_t ele_size);
char *array_free(array_t *a, bool free_segment);
array_t *array_append_vals(array_t *a, void *data, size_t len);
array_t *array_prepend_vals(array_t *a, void *data, size_t len);
array_t *array_insert_vals(array_t *a, size_t index_, void *data, size_t len);
array_t *array_set_size(array_t *a, size_t length);
array_t *array_remove_range(array_t *a, size_t index_, size_t length);
void array_set_clear_func(array_t *a, array_clear_callback func);
void array_maybe_expand(array_t *array, size_t len);
size_t array_nearest_pow(size_t v);

#define array_index(a, t, i)      (((t *)(void *)(a)->data)[(i)])
#define array_append_val(a, v)    array_append_vals(a, &(v), 1)
#define array_prepend_val(a, v)   array_prepend_vals(a, &(v), 1)
#define array_insert_val(a, i, v) array_insert_vals(a, i, &(v), 1)
#define array_front(a, t)         array_index(a, t, 0)
#define array_back(a, t)          array_index(a, t, (a)->size - 1)

#define array_elt_len(array, i)   ((size_t)(array)->ele_size * (i))
#define array_elt_pos(array, i)   ((array)->data + array_elt_len((array), (i)))
#define array_elt_zero(array, pos, len) \
    (memset(array_elt_pos((array), pos), 0, array_elt_len((array), len)))

#endif