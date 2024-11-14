#include "polyline.h"

#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>
#include <stdint.h>
#include <limits.h>
#include <math.h>

#define THRESHOLD_ELLIPSE (1.0e-8)
#define MIN(a, b)         ((a) < (b) ? (a) : (b))
#define MAX(a, b)         ((a) > (b) ? (a) : (b))

typedef void (*array_clear_callback)(void *);
typedef struct array_s {
    char *data;
    size_t size;
    size_t capacity;
    size_t ele_size;
    array_clear_callback clear_func;
} array_t;

typedef struct pline_offset_segment_s {
    vertex_t v1;
    vertex_t v2;
    point_t orig_pos;
    bool collapsed_arc;
} pline_offset_segment_t;

typedef struct arc_s {
    point_t center;
    double radius;
} arc_t;

struct polyline_s {
    bool closed;
    size_t vertex_num;
    array_t *vertices;
};

/* ----------------------------- math functions ---------------------------- */

double normalizeRadians(double angle);
double deltaAngle(double a1, double a2);

/* ---------------------------- array_t functions --------------------------- */

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

/* ---------------------------- point_t functions --------------------------- */

point_t pt_sub(const point_t p1, const point_t p2);
point_t pt_add(const point_t p1, const point_t p2);
point_t pt_mul(double s, const point_t p);
point_t pt_unit_perp(const point_t p);
double pt_angle(const point_t p1, const point_t p2);

/* ----------------------------- arc_t functions ---------------------------- */

arc_t arc_radius_and_center(const vertex_t v1, const vertex_t v2);

/* -------------------------- polyline_t functions -------------------------- */

polyline_t *create_raw_offset_polyline(const polyline_t *pline, double offset);
vertex_t polyline_last_vertex(const polyline_t *pline);
void polyline_set_pos(polyline_t *pline, size_t index, const point_t pos);
void polyline_set_bulge(polyline_t *pline, size_t index, const double bulge);

pline_offset_segment_t *
create_untrimmed_offset_segments(const polyline_t *pline, double offset,
                                 int *seg_size);

#define PL_VAT(res, index)    polyline_vertex_at(res, index)
#define PL_UPP(res, index, p) polyline_set_pos(res, index, p)
#define PL_UPB(res, index, b) polyline_set_bulge(res, index, b)
#define PL_SIZE(res)          polyline_size(res)
#define PL_ERASE(p, i)

/* ------------------------ polyline_t main functions ----------------------- */

polyline_t **polyline_parallel_offset(const polyline_t *pline, double offset,
                                      int hasSelfIntersects, int *res_size)
{
    assert(pline);
    assert(pline->vertices);
    if (pline->vertices->size < 2)
        return NULL;
    polyline_t *rawOffset = create_raw_offset_polyline(pline, offset);
    if (pline->closed && !hasSelfIntersects) {
        size_t slice_size = 0;
        polyline_t **slices =
            slices_from_raw_offset(pline, rawOffset, offset, &slice_size);
        polyline_t **result =
            stitch_offset_slices_together(slices, slice_size, pline->closed,
                                          PL_SIZE(rawOffset) - 1, res_size);
        polyline_free(rawOffset);
        for (size_t i = 0; i < slice_size; ++i) {
            polyline_free(slices[i]);
        }
        return result;
    }
    else {
        // not closed polyline or has self intersects, must apply dual clipping
        polyline_t *dualRawOffset = create_raw_offset_polyline(pline, offset);
        size_t slice_size = 0;
        polyline_t **slices = dual_slice_at_intersects_for_offset(
            pline, rawOffset, dualRawOffset, offset, &slice_size);
        polyline_t **result =
            stitch_offset_slices_together(slices, slice_size, pline->closed,
                                          PL_SIZE(rawOffset) - 1, res_size);
        polyline_free(rawOffset);
        polyline_free(dualRawOffset);
        for (size_t i = 0; i < slice_size; ++i) {
            polyline_free(slices[i]);
        }
        return result;
    }

    return NULL;
}

polyline_t *create_raw_offset_polyline(const polyline_t *pline, double offset)
{
    assert(pline);
    assert(pline->vertices);
    if (pline->vertices->size < 2)
        return NULL;
    polyline_t *res = create_polyline();

    size_t rosize = 0;
    pline_offset_segment_t *raw_offset =
        create_untrimmed_offset_segments(pline, offset, &rosize);
    if (raw_offset == NULL || rosize == 0)
        return res;

    // detect single collapsed arc segment (this may be removed in the future if
    // invalid segments are tracked in join functions to be pruned at slice
    // creation)
    array_reserve(res->vertices, pline->vertex_num);
    // join first two segments and determine if first vertex was replaced (to
    // know how to handle last two segment joins for closed polyline)
    if (rosize > 1) {
        pline_offset_segment_t seg01 = raw_offset[0];
        pline_offset_segment_t seg12 = raw_offset[1];
        // join
    }
    const bool firstVertexReplaced = (res->vertex_num == 1) ? true : false;
    for (size_t i = 2; i < rosize; ++i) {
        pline_offset_segment_t seg1 = raw_offset[i - 1];
        pline_offset_segment_t seg2 = raw_offset[i];
        // join
    }

    if (pline->closed && res->vertex_num > 1) {
        pline_offset_segment_t s1 = raw_offset[rosize - 1];
        pline_offset_segment_t s2 = raw_offset[0];

        // tmp ,need free
        polyline_t *closingPartResult = create_polyline();
        polyline_append_vertex(closingPartResult, polyline_last_vertex(res));
        // join

        // update last vertex
        polyline_set_vertex(res, res->vertex_num - 1,
                            PL_VAT(closingPartResult, 0));
        for (size_t i = 1; i < closingPartResult->vertex_num - 1; ++i) {
            polyline_append_vertex(res, PL_VAT(closingPartResult, i));
        }

        // update first vertex
        if (!firstVertexReplaced) {
            point_t updatedFirstPos =
                polyline_last_vertex(closingPartResult).pos;
            if (PL_VAT(res, 0).bulge == 0.0) {
                PL_UPP(res, 0, updatedFirstPos);
            }
            else if (res->vertex_num > 1) {
                arc_t arc =
                    arc_radius_and_center(PL_VAT(res, 0), PL_VAT(res, 1));
                const double a1 = pt_angle(arc.center, updatedFirstPos);
                const double a2 = pt_angle(arc.center, PL_VAT(res, 1).pos);
                const double updatedTheta = deltaAngle(a1, a2);
                if ((updatedTheta < 0.0 && PL_VAT(res, 0).bulge > 0.0) ||
                    (updatedTheta > 0.0 && PL_VAT(res, 0).bulge < 0.0))
                    PL_UPP(res, 0, updatedFirstPos);
                else {
                    PL_UPP(res, 0, updatedFirstPos);
                    PL_UPB(res, 0, tan(updatedTheta / 4.0));
                }
            }
        }

        // must do final singularity prune between first and second vertex after
        // joining curves (n, 0) and (0, 1)
        if (PL_SIZE(res) > 1) {
            if (fuzzyEqual(PL_VAT(res, 0).pos, PL_VAT(res, 1).pos,
                           THRESHOLD_ELLIPSE)) {
                PL_ERASE(res, 0);
            }
        }
    }
    else {
        pline_add_or_replace_if_same_pos(res, raw_offset[rosize - 1].v2);
    }

    if (PL_SIZE(res) == 1) {
        PL_CLEAR(res);
    }
    return res;
}

pline_offset_segment_t *
create_untrimmed_offset_segments(const polyline_t *pline, double offset,
                                 int *seg_size)
{
    *seg_size = 0;
    size_t segment_count =
        pline->closed ? pline->vertex_num : pline->vertex_num - 1;
    pline_offset_segment_t *segments = (pline_offset_segment_t *)malloc(
        sizeof(pline_offset_segment_t) * segment_count);
    assert(segments);

    for (size_t i = 1; i < pline->vertex_num; ++i) {
        vertex_t v1 = PL_VAT(pline, i - 1);
        vertex_t v2 = PL_VAT(pline, i);

        if (v1.bulge == 0.0) {
            pline_offset_segment_t seg;
            seg.collapsed_arc = false;
            seg.orig_pos = v2.pos;
            point_t edge = pt_sub(v2.pos, v1.pos);
            point_t offsetV = pt_mul(offset, pt_unit_perp(edge));
            seg.v1.pos = pt_add(v1.pos, offsetV);
            seg.v1.bulge = v1.bulge;
            seg.v2.pos = pt_add(v2.pos, offsetV);
            seg.v2.bulge = v2.bulge;
            segments[(*seg_size)++] = seg;
        }
        else {
            arc_t arc = arc_radius_and_center(v1, v2);
            double ofs = v1.bulge < 0.0 ? offset : -offset;
            double radius_after_offset = arc.radius + offset;
            point_t v1ToCenter = pt_sub(v1.pos, arc.center);
            pt_normalize(&v1ToCenter);
            point_t v2ToCenter = pt_sub(v2.pos, arc.center);
            pt_normalize(&v2ToCenter);

            pline_offset_segment_t seg;
            seg.orig_pos = v2.pos;
            seg.v1.pos = pt_add(v1.pos, pt_mul(ofs, v1ToCenter));
            seg.v2.pos = pt_add(v2.pos, pt_mul(ofs, v2ToCenter));
            seg.v2.bulge = v2.bulge;

            if (radius_after_offset < THRESHOLD_ELLIPSE) {
                seg.collapsed_arc = true;
                seg.v1.bulge = 0.0;
            }
            else {
                seg.collapsed_arc = false;
                seg.v1.bulge = v1.bulge;
            }
            segments[(*seg_size)++] = seg;
        }
    }
    return 0;
}

/* -------------------------- maths function impls -------------------------- */
double normalizeRadians(double angle)
{
    if (angle >= 0.0 && angle <= (M_PI * 2.0)) {
        return angle;
    }

    return angle - floor(angle / (M_PI * 2.0)) * (M_PI * 2.0);
}

double deltaAngle(double a1, double a2)
{
    double diff = normalizeRadians(a2 - a1);
    if (diff > M_PI) {
        diff -= (M_PI * 2.0);
    }

    return diff;
}

/* ------------------------- array_t function impls ------------------------- */

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

static void array_maybe_expand(array_t *a, size_t len)
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

        a->data = doubleloc(a->data, want_alloc);

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

/* ---------------------------- point_t functions --------------------------- */

point_t pt_sub(const point_t p1, const point_t p2)
{
    return (point_t){p1.x - p2.x, p1.y - p2.y};
}

point_t pt_add(const point_t p1, const point_t p2)
{
    return (point_t){p1.x + p2.x, p1.y + p2.y};
}

point_t pt_mul(double s, const point_t p)
{
    return (point_t){p.x * s, p.y * s};
}

point_t pt_unit_perp(const point_t p)
{
    point_t result = {-p.y, p.x};

    double length = hypot(p.x, p.y);
    if (length == 0) {
        return result;
    }
    return pt_mul(1.0 / length, result);
}

double pt_angle(const point_t p1, const point_t p2)
{
    return atan2(p2.y - p1.y, p2.x - p1.x);
}

/* ----------------------------- arc_t functions ---------------------------- */

arc_t arc_radius_and_center(const vertex_t v1, const vertex_t v2)
{
    assert(v1.bulge != 0);
    assert(!pt_fequal(v1.pos, v2.pos));

    // compute radius
    double b = fabs(v1.bulge);
    point_t v = pt_sub(v2.pos, v1.pos);
    double d = hypot(v.x, v.y);
    double r = d * (b * b + 1.0) / (4.0 * b);

    // compute center
    double s = b * d / 2.0;
    double m = r - s;
    double offsX = -m * v.y / d;
    double offsY = m * v.x / d;
    if (v1.bulge < 0.0) {
        offsX = -offsX;
        offsY = -offsY;
    }

    point_t c = {v1.pos.x + v.x / 2.0 + offsX, v1.pos.y + v.y / 2.0 + offsY};
    return (arc_t){.radius = r, .center = c};
}

#undef MIN
#undef MAX