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

#include "arraylist.h"
#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

array_t *array_new(size_t element_size)
{
    if (element_size <= 0 || element_size >= 128)
        return NULL;

    array_t *array = (array_t *)malloc(sizeof(array_t));
    if (!array)
        return NULL;

    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
    array->ele_size = element_size;
    array->clear_func = 0;
    return (array_t *)array;
}

char *array_free(array_t *array, bool free_segment)
{
    if (!array)
        return NULL;

    char *segment;
    if (free_segment) {
        if (array->clear_func != NULL) {
            for (size_t i = 0; i < array->size; i++)
                array->clear_func(array_elt_pos(array, i));
        }

        free(array->data);
        segment = NULL;
    }
    else {
        segment = (char *)array->data;
    }

    free(array);
    return segment;
}

array_t *array_append_vals(array_t *array, void *data, size_t len)
{
    if (!array)
        return NULL;

    if (len == 0)
        return array;

    array_maybe_expand(array, len);
    memcpy(array_elt_pos(array, array->size), data, array_elt_len(array, len));

    array->size += len;
    return array;
}

array_t *array_prepend_vals(array_t *a, void *data, size_t len)
{
    if (a == NULL)
        return NULL;

    if (len == 0)
        return a;

    array_maybe_expand(a, len);

    memmove(array_elt_pos(a, len), array_elt_pos(a, 0),
            array_elt_len(a, a->size));

    memcpy(array_elt_pos(a, 0), data, array_elt_len(a, len));

    a->size += len;

    return a;
}

array_t *array_insert_vals(array_t *a, size_t index_, void *data, size_t len)
{
    if (!a)
        return NULL;

    if (len == 0)
        return a;

    /* Is the index off the end of the array, and hence do we need to
     * over-allocate and clear some elements? */
    if (index_ >= a->size) {
        array_maybe_expand(a, index_ - a->size + len);
        return array_append_vals(array_set_size(a, index_), data, len);
    }

    array_maybe_expand(a, len);

    memmove(array_elt_pos(a, len + index_), array_elt_pos(a, index_),
            array_elt_len(a, a->size - index_));

    memcpy(array_elt_pos(a, index_), data, array_elt_len(a, len));

    a->size += len;

    return a;
}

array_t *array_set_size(array_t *a, size_t length)
{
    if (!a)
        return NULL;

    if (length > a->size) {
        array_maybe_expand(a, length - a->size);
    }
    else if (length < a->size)
        array_remove_range(a, length, a->size - length);

    a->size = length;

    return a;
}

array_t *array_remove_range(array_t *a, size_t index_, size_t length)
{
    if (!a)
        return NULL;
    if (index_ > a->size)
        return NULL;
    if (index_ > UINT_MAX - length)
        return NULL;
    if (index_ + length > a->size)
        return NULL;

    if (a->clear_func != NULL) {
        size_t i;
        for (i = 0; i < length; i++)
            a->clear_func(array_elt_pos(a, index_ + i));
    }

    if (index_ + length != a->size)
        memmove(array_elt_pos(a, index_), array_elt_pos(a, index_ + length),
                (a->size - (index_ + length)) * a->ele_size);

    a->size -= length;
    array_elt_zero(a, a->size, length);

    return a;
}

void array_set_clear_func(array_t *a, array_clear_callback func)
{
    if (a == NULL)
        return;

    a->clear_func = func;
}

void array_maybe_expand(array_t *a, size_t len)
{
    size_t max_len, want_len;
    max_len = MIN(SIZE_MAX / 2 / a->ele_size, UINT_MAX);

    /* Detect potential overflow */
    if ((max_len - a->size) < len)
        abort();

    want_len = a->size + len;
    if (want_len > a->capacity) {
        size_t want_alloc = array_nearest_pow(array_elt_len(a, want_len));
        assert(want_alloc >= array_elt_len(a, want_len));
        want_alloc = MAX(want_alloc, 16);

        a->data = realloc(a->data, want_alloc);

        memset(array_elt_pos(a, a->capacity), 0,
               array_elt_len(a, want_len - a->capacity));

        a->capacity = MIN(want_alloc / a->ele_size, UINT_MAX);
    }
}

size_t array_nearest_pow(size_t v)
{
    size_t n = 1;

    while (n < v && n > 0)
        n <<= 1;

    return n ? n : v;
}
