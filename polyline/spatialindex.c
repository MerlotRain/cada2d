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

#include "spatialindex.h"
#include <assert.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>

struct spatialIndex_s {
    double minX;
    double minY;
    double maxX;
    double maxY;
    size_t numItems;
    size_t numLevels;
    size_t *levelBounds;
    size_t numNodes;
    double *boxes;
    size_t *indices;
    size_t pos;
};

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

spatialIndex_t *spidx_create(size_t numItems, size_t nodeSize)
{
    assert(numItems > 0);
    assert(nodeSize >= 2 && nodeSize <= USHRT_MAX);

    spatialIndex_t *idx = (spatialIndex_t *)malloc(sizeof(spatialIndex_t));
    if (idx == NULL)
        return NULL;

    idx->numItems = numItems;
    size_t n = idx->numItems;
    size_t numNodes = idx->numItems;
    idx->numLevels = computeNumLevels(idx->numItems, nodeSize);
    idx->levelBounds = (size_t *)malloc(sizeof(size_t) * idx->numLevels);
    if (NULL == idx->levelBounds) {
        free(idx);
        return NULL;
    }
    idx->levelBounds[0] = n * 4;
    // now populate level bounds and numNodes
    size_t i = 1;
    do {
        n = (size_t)(ceil((float)n / nodeSize));
        numNodes += n;
        idx->levelBounds[i] = numNodes * 4;
        i += 1;
    } while (n != 1);

    idx->numNodes = numNodes;
    idx->boxes = (double *)malloc(sizeof(double) * numNodes * 4);
    if (NULL == idx->boxes) {
        spidx_free(idx);
        return NULL;
    }
    idx->indices = (size_t *)malloc(sizeof(size_t) * numNodes);
    if (NULL == idx->indices) {
        spidx_free(idx);
        return NULL;
    }
    idx->pos = 0;
    idx->minX = INFINITY;
    idx->minY = INFINITY;
    idx->maxX = -(INFINITY);
    idx->maxY = -(INFINITY);
}

void spidx_free(const spatialIndex_t *idx)
{
    assert(idx);
    free(idx->levelBounds);
    free(idx->boxes);
    free(idx->indices);
    free(idx);
}

double spidx_minx(const spatialIndex_t *idx)
{
    assert(idx);
    return idx->minX;
}

double spidx_miny(const spatialIndex_t *idx)
{
    assert(idx);
    return idx->minY;
}

double spidx_maxx(const spatialIndex_t *idx)
{
    assert(idx);
    return idx->maxX;
}

double spidx_maxy(const spatialIndex_t *idx)
{
    assert(idx);
    return idx->maxY;
}

void spidx_add(spatialIndex_t *idx, double minx, double miny, double maxx,
               double maxy)
{
    assert(idx);
    size_t index = idx->pos >> 2;
    idx->indices[index] = index;
    idx->boxes[idx->pos++] = minx;
    idx->boxes[idx->pos++] = miny;
    idx->boxes[idx->pos++] = maxx;
    idx->boxes[idx->pos++] = maxy;

    if (minx < idx->minX)
        idx->minX = minx;
    if (miny < idx->minY)
        idx->minY = miny;
    if (maxx > idx->maxX)
        idx->maxX = maxx;
    if (maxy > idx->maxY)
        idx->maxY = maxy;
}

void spidx_finish(spatialIndex_t *idx)
{
    assert(idx);
}

void spidx_visit_item_boxes(spatialIndex_t *idx,
                            spatial_index_boxes_visitor visitor)
{
    assert(idx);
    assert(visitor);
    for (size_t i = 0; i < idx->levelBounds[0]; i += 4) {
        if (!visitor(idx, idx->indices[i >> 2], idx->boxes[i],
                     idx->boxes[i + 1], idx->boxes[i + 2], idx->boxes[i + 3])) {
            return;
        }
    }
}
void spidx_query(spatialIndex_t *idx, double minx, double miny, double maxx,
                 double maxy, array_t *result);
void spidx_query2(spatialIndex_t *idx, double minx, double miny, double maxx,
                  double maxy, array_t *result, array_t *stack);