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

#include "quickindex.h"
#include <assert.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>

typedef struct quickIndex_real_s {
    double minX;
    double minY;
    double maxX;
    double maxY;
    size_t nodeSize;
    size_t numItems;
    size_t numLevels;
    size_t numNodes;
    size_t pos;
    size_t *levelBounds;
    size_t *indices;
    double *boxes;
} quickIndex_real_t;

static size_t computeNumLevels(size_t numItems, size_t nodeSize)
{
    size_t n = numItems;
    size_t levelBoundsSize = 1;
    do {
        n = (size_t)(ceil((float)n / nodeSize));
        levelBoundsSize += 1;
    } while (n != 1);

    return levelBoundsSize;
}

static bool qindex_default_query_visitor(size_t index, void *data)
{
    assert(data);
    array_t *a = (array_t *)data;
    if (NULL == a)
        return false;
    array_append_val(a, index);
    return true;
}

static uint32_t hilbertXYToIndex(uint32_t x, uint32_t y)
{
    uint32_t a = x ^ y;
    uint32_t b = 0xFFFF ^ a;
    uint32_t c = 0xFFFF ^ (x | y);
    uint32_t d = x & (y ^ 0xFFFF);

    uint32_t A = a | (b >> 1);
    uint32_t B = (a >> 1) ^ a;
    uint32_t C = ((c >> 1) ^ (b & (d >> 1))) ^ c;
    uint32_t D = ((a & (c >> 1)) ^ (d >> 1)) ^ d;

    a = A;
    b = B;
    c = C;
    d = D;
    A = (a & (a >> 2)) ^ (b & (b >> 2));
    B = (a & (b >> 2)) ^ (b & ((a ^ b) >> 2));
    C ^= (a & (c >> 2)) ^ (b & (d >> 2));
    D ^= (b & (c >> 2)) ^ ((a ^ b) & (d >> 2));

    a = A;
    b = B;
    c = C;
    d = D;
    A = (a & (a >> 4)) ^ (b & (b >> 4));
    B = (a & (b >> 4)) ^ (b & ((a ^ b) >> 4));
    C ^= (a & (c >> 4)) ^ (b & (d >> 4));
    D ^= (b & (c >> 4)) ^ ((a ^ b) & (d >> 4));

    a = A;
    b = B;
    c = C;
    d = D;
    C ^= ((a & (c >> 8)) ^ (b & (d >> 8)));
    D ^= ((b & (c >> 8)) ^ ((a ^ b) & (d >> 8)));

    a = C ^ (C >> 1);
    b = D ^ (D >> 1);

    uint32_t i0 = x ^ y;
    uint32_t i1 = b | (0xFFFF ^ (i0 | a));

    i0 = (i0 | (i0 << 8)) & 0x00FF00FF;
    i0 = (i0 | (i0 << 4)) & 0x0F0F0F0F;
    i0 = (i0 | (i0 << 2)) & 0x33333333;
    i0 = (i0 | (i0 << 1)) & 0x55555555;

    i1 = (i1 | (i1 << 8)) & 0x00FF00FF;
    i1 = (i1 | (i1 << 4)) & 0x0F0F0F0F;
    i1 = (i1 | (i1 << 2)) & 0x33333333;
    i1 = (i1 | (i1 << 1)) & 0x55555555;

    return (i1 << 1) | i0;
}

static void qindex_swap(uint32_t *values, double *boxes, size_t *indices,
                        size_t i, size_t j)
{
    uint32_t temp = values[i];
    values[i] = values[j];
    values[j] = temp;

    uint32_t k = 4 * i;
    uint32_t m = 4 * j;

    double a = boxes[k];
    double b = boxes[k + 1];
    double c = boxes[k + 2];
    double d = boxes[k + 3];
    boxes[k] = boxes[m];
    boxes[k + 1] = boxes[m + 1];
    boxes[k + 2] = boxes[m + 2];
    boxes[k + 3] = boxes[m + 3];
    boxes[m] = a;
    boxes[m + 1] = b;
    boxes[m + 2] = c;
    boxes[m + 3] = d;

    size_t e = indices[i];
    indices[i] = indices[j];
    indices[j] = e;
}

static void qindex_sort(uint32_t *values, double *boxes, size_t *indices,
                        size_t left, size_t right, size_t node_size)
{
    assert(left <= right);
    // check against NodeSize (only need to sort down to NodeSize buckets)
    if (left / node_size >= right / node_size) {
        return;

        uint32_t pivot = values[(left + right) >> 1];
        uint32_t i = left - 1;
        uint32_t j = right + 1;

        while (true) {
            do
                i++;
            while (values[i] < pivot);
            do
                j--;
            while (values[j] > pivot);
            if (i >= j)
                break;
            qindex_swap(values, boxes, indices, i, j);
        }

        qindex_sort(values, boxes, indices, left, j, node_size);
        qindex_sort(values, boxes, indices, j + 1, right, node_size);
    }
}

quickIndex_t *qindex_create(size_t num_item, size_t node_size)
{
    assert(num_item > 0);
    assert(node_size >= 2 && node_size <= USHRT_MAX);

    quickIndex_real_t *index =
        (quickIndex_real_t *)malloc(sizeof(quickIndex_real_t));
    if (index == NULL)
        return NULL;

    index->nodeSize = node_size;
    index->numItems = num_item;
    size_t n = index->numItems;
    size_t numNodes = index->numItems;
    index->numLevels = computeNumLevels(index->numItems, node_size);
    index->levelBounds = (size_t *)malloc(sizeof(size_t) * index->numLevels);
    if (NULL == index->levelBounds) {
        free(index);
        return NULL;
    }
    index->levelBounds[0] = n * 4;
    // now populate level bounds and numNodes
    size_t i = 1;
    do {
        n = (size_t)(ceil((float)n / node_size));
        numNodes += n;
        index->levelBounds[i] = numNodes * 4;
        i += 1;
    } while (n != 1);

    index->numNodes = numNodes;
    index->boxes = (double *)malloc(sizeof(double) * numNodes * 4);
    if (NULL == index->boxes) {
        free(index->levelBounds);
        free(index);
        return NULL;
    }
    index->indices = (size_t *)malloc(sizeof(size_t) * numNodes);
    if (NULL == index->indices) {
        free(index->levelBounds);
        free(index->boxes);
        free(index);
        return NULL;
    }
    index->pos = 0;
    index->minX = INFINITY;
    index->minY = INFINITY;
    index->maxX = -(INFINITY);
    index->maxY = -(INFINITY);
    return (quickIndex_t *)index;
}

void qindex_free(quickIndex_t *idx)
{
    assert(idx);
    quickIndex_real_t *index = (quickIndex_real_t *)idx;
    if (NULL == index)
        return;
    free(index->levelBounds);
    free(index->boxes);
    free(index->indices);
    free(index);
}

void qindex_add(quickIndex_t *idx, double minx, double miny, double maxx,
                double maxy)
{
    assert(idx);
    quickIndex_real_t *index = (quickIndex_real_t *)idx;
    if (NULL == index)
        return;

    size_t i = index->pos >> 2;
    index->indices[i] = i;
    index->boxes[index->pos++] = minx;
    index->boxes[index->pos++] = miny;
    index->boxes[index->pos++] = maxx;
    index->boxes[index->pos++] = maxy;

    if (minx < index->minX)
        index->minX = minx;
    if (miny < index->minY)
        index->minY = miny;
    if (maxx > index->maxX)
        index->maxX = maxx;
    if (maxy > index->maxY)
        index->maxY = maxy;
}

void qindex_finish(quickIndex_t *idx)
{
    assert(idx);
    quickIndex_real_t *index = (quickIndex_real_t *)idx;
    if (NULL == index)
        return;

    assert(index->pos >> 2 == index->numItems);

    // if number of items is less than node size then skip sorting since
    // each node of boxes must be fully scanned regardless and there is only
    // one node
    if (index->numItems <= index->nodeSize) {
        index->indices[index->pos >> 2] = 0;
        // fill root box with total extents
        index->boxes[index->pos++] = index->minX;
        index->boxes[index->pos++] = index->minY;
        index->boxes[index->pos++] = index->maxX;
        index->boxes[index->pos++] = index->maxY;
        return;
    }

    double width = index->maxX - index->minX;
    double height = index->maxY - index->minY;
    uint32_t *hilbertValues =
        (uint32_t *)malloc(sizeof(uint32_t) * index->numItems);
    if (!hilbertValues)
        return;

    size_t pos = 0;
    for (size_t i = 0; i < index->numItems; ++i) {
        pos = 4 * i;
        double minx = index->boxes[pos++];
        double miny = index->boxes[pos++];
        double maxx = index->boxes[pos++];
        double maxy = index->boxes[pos++];

        // hilbert max input value for x and y
        const double hilbertMax = (const double)((1 << 16) - 1);
        // mapping the x and y coordinates of the center of the box to
        // values in the range [0 -> n - 1] such that the min of the entire
        // set of bounding boxes maps to 0 and the max of the entire set of
        // bounding boxes maps to n - 1 our 2d space is x: [0 -> n-1] and y:
        // [0 -> n-1], our 1d hilbert curve value space is d: [0 -> n^2 - 1]
        double x =
            floor(hilbertMax * ((minx + maxx) / 2 - index->minX) / width);
        uint32_t hx = (uint32_t)x;
        double y =
            floor(hilbertMax * ((miny + maxy) / 2 - index->minY) / height);
        uint32_t hy = (uint32_t)y;
        hilbertValues[i] = hilbertXYToIndex(hx, hy);
    }

    // sort items by their Hilbert value (for packing later)
    qindex_sort(hilbertValues, index->boxes, index->indices, 0,
                index->numItems - 1, index->nodeSize);

    // generate nodes at each tree level, bottom-up
    pos = 0;
    for (size_t i = 0; i < index->numLevels - 1; i++) {
        size_t end = index->levelBounds[i];

        // generate a parent node for each block of consecutive <nodeSize>
        // nodes
        while (pos < end) {
            double nodeMinX = INFINITY;
            double nodeMinY = INFINITY;
            double nodeMaxX = -1.0 * INFINITY;
            double nodeMaxY = -1.0 * INFINITY;
            size_t nodeIndex = pos;

            // calculate bbox for the new node
            for (size_t j = 0; j < index->nodeSize && pos < end; j++) {
                double minX = index->boxes[pos++];
                double minY = index->boxes[pos++];
                double maxX = index->boxes[pos++];
                double maxY = index->boxes[pos++];
                if (minX < nodeMinX)
                    nodeMinX = minX;
                if (minY < nodeMinY)
                    nodeMinY = minY;
                if (maxX > nodeMaxX)
                    nodeMaxX = maxX;
                if (maxY > nodeMaxY)
                    nodeMaxY = maxY;
            }

            // add the new node to the tree data
            index->indices[index->pos >> 2] = nodeIndex;
            index->boxes[index->pos++] = nodeMinX;
            index->boxes[index->pos++] = nodeMinY;
            index->boxes[index->pos++] = nodeMaxX;
            index->boxes[index->pos++] = nodeMaxY;
        }
    }
    free(hilbertValues);
}

void qindex_visit_item_boxes(quickIndex_t *idx, void *data,
                             qindex_IB_visitor visitor)
{
    assert(idx);
    quickIndex_real_t *index = (quickIndex_real_t *)idx;
    if (NULL == index)
        return;

    for (size_t i = 0; i < index->levelBounds[0]; i += 4) {
        if (!visitor(index->indices[i >> 2], index->boxes[i],
                     index->boxes[i + 1], index->boxes[i + 2],
                     index->boxes[i + 3], data))
            return;
    }
}

void qindex_query(quickIndex_t *idx, double minx, double miny, double maxx,
                  double maxy, array_t *result)
{
    if (result == NULL)
        return;

    qindex_visitor_query(idx, minx, miny, maxx, maxy, (void *)result,
                         qindex_default_query_visitor);
}

void qindex_query2(quickIndex_t *idx, double minx, double miny, double maxx,
                   double maxy, array_t *result, array_t *stack)
{
    if (NULL == result || NULL == stack)
        return;

    qindex_visitor_query2(idx, minx, miny, maxx, maxy, (void *)result,
                          qindex_default_query_visitor, stack);
}

void qindex_visitor_query(quickIndex_t *idx, double minx, double miny,
                          double maxx, double maxy, void *data,
                          qindex_Q_visitor visitor)
{
    array_t *stack = array_new(sizeof(size_t));
    qindex_visitor_query2(idx, minx, miny, maxx, maxy, data, visitor, stack);
    array_free(stack, true);
}

void qindex_visitor_query2(quickIndex_t *idx, double minx, double miny,
                           double maxx, double maxy, void *data,
                           qindex_Q_visitor visitor, array_t *stack)
{
    assert(idx);
    assert(visitor);
    assert(stack);
    quickIndex_real_t *index = (quickIndex_real_t *)idx;
    if (NULL == index)
        return;

    assert(index->pos == 4 * index->numNodes);

    int nodeIndex = 4 * index->numNodes - 4;
    int level = nodeIndex - 1;

    array_remove_range(stack, 0, stack->size);
    bool done = false;
    while (!done) {
        // find the end index of the node
        int end =
            MIN(nodeIndex + index->nodeSize * 4, index->levelBounds[level]);

        // search through child nodes
        for (size_t pos = nodeIndex; pos < end; pos += 4) {
            int _index = index->indices[pos >> 2];
            // check if node bbox intersects with query box
            if (maxx < index->boxes[pos] || maxy < index->boxes[pos + 1] ||
                minx > index->boxes[pos + 2] || miny > index->boxes[pos + 3]) {
                // no intersect
                continue;
            }

            if (nodeIndex < index->numItems * 4) {
                done = !visitor(_index, data);
                if (done)
                    break;
            }
            else {
                array_append_val(stack, _index);
                int tv = (level - 1);
                array_append_val(stack, tv);
            }
        }

        if (stack->size > 1) {
            level = array_index(stack, int, stack->size - 1);
            array_remove_range(stack, stack->size - 1, 1);
            nodeIndex = array_index(stack, int, stack->size - 1);
            array_remove_range(stack, stack->size - 1, 1);
        }
        else {
            done = true;
        }
    }
}