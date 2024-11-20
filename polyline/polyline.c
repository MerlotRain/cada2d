#include "polyline.h"
#include "arraylist.h"
#include "quickindex.h"

#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>
#include <stdint.h>
#include <limits.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define THRESHOLD_ELLIPSE (1.0e-8)
#define PERCISION_ELLIPSE (1.0e-5)

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

// axis aligned bounding box
typedef struct aabb_s {
    double xmin;
    double xmax;
    double ymin;
    double ymax;
} aabb_t;

struct polyline_s {
    bool closed;
    size_t vertex_num; // index plus one equal vertex_num;
    array_t *vertices;
};

/* ------------------ intersection structure and functions ------------------ */

typedef enum {
    // no intersect (segments are parallel and not collinear)
    LL_NONE,
    // true intersect between line segments
    LL_TRUE,
    // segments overlap each other by some amount
    LL_COINCIDENT,
    // false intersect between line segments (one or both of the segments must
    // be extended)
    LL_FALSE
} IntrLLType;

typedef struct intr_LL {
    // holds the type of intersect, if True or False then point holds the point
    // that they intersect, if True then t0 and t1 are undefined, if False then
    // t0 is the parametric value of the first segment and t1 is the parametric
    // value of the second segment, if Coincident then point is undefined and t0
    // holds the parametric value start of coincidence and t1 holds the
    // parametric value of the end of the coincidence for the second segment's
    // equation
    IntrLLType intrType;
    double t0;
    double t1;
    point_t point;
} intr_LL;

typedef enum {
    // no intersect between circles
    CC_NO_INTERSECT,
    // one intersect between circles (tangent)
    CC_ONE_INTERSECT,
    // two intersects between circles
    CC_TWO_INTERSECTS,
    // circles are coincident
    CC_COINCIDENT
} IntrCCType;

typedef struct intr_CC {
    // type of intersect
    IntrCCType intrType;
    // first intersect point if intrType is OneIntersect or TwoIntersects,
    // undefined otherwise
    point_t point1;
    // second intersect point if intrType is TwoIntersects, undefined otherwise
    point_t point2;
} intr_CC;

typedef struct intr_LC {
    // number of interescts found (0, 1, or 2)
    int numIntersects;
    // parametric value for first intersect (if numIntersects > 0) otherwise
    // undefined
    double t0;
    // parametric value for second intersect (if numintersects > 1) otherwise
    // undefined
    double t1;
} intr_LC;

// Find intersect between two circles in 2D.
intr_CC intersection_CC(double radius1, const point_t center1, double radius2,
                        const point_t center2);
// Gets the intersect between a segment and a circle, returning the parametric
// solution t to the segment equation P(t) = v1 + t * (v2 - v1) for t = 0 to t =
// 1, if t < 0 or t > 1 then intersect occurs only when extending the segment
// out past the points given (if t < 0 intersect nearest v1, if t > 0 then
// intersect nearest v2), intersects are "sticky" and "snap" to tangent points,
// e.g. a segment very close to being a tangent will be returned as a single
// intersect point
intr_LC intersection_LC(const point_t p0, const point_t p1, double radius,
                        const point_t center);
// http://geomalgorithms.com/a05-_intersect-1.html
intr_LL intersection_LL(const point_t u1, const point_t u2, const point_t v1,
                        const point_t v2);

/* ----------------------------- math functions ---------------------------- */

double normalizeRadians(double angle);
double deltaAngle(double a1, double a2);
bool fuzzyEqual(double a, double b, double epsilon);
bool fuzzInRange(double min, double a, double max, double epsilon);

/* ---------------------------- point_t functions --------------------------- */

point_t pt_sub(const point_t p1, const point_t p2);
point_t pt_add(const point_t p1, const point_t p2);
point_t pt_mul(double s, const point_t p);
point_t pt_div(const point_t p, double s);
point_t pt_unit_perp(const point_t p);
double pt_angle(const point_t p1, const point_t p2);
double pt_normalize(point_t *p);
bool pt_fuzzyEqual(const point_t p1, const point_t p2, double tol);
double pt_distance(const point_t p0, const point_t p1);
double pt_distance2(const point_t p0, const point_t p1);
point_t pt_from_parametric(const point_t p0, const point_t p1, const double t);

/* ----------------------------- arc_t functions ---------------------------- */

arc_t arc_radius_and_center(const vertex_t v1, const vertex_t v2);
double bulge_for_connection(const point_t center, const point_t sp,
                            const point_t ep, bool ccw);
bool point_within_arc_sweep_angle(const point_t center, const point_t start,
                                  const point_t end, double bulge,
                                  const point_t point);

/* ---------------------------- aabb_t functions ---------------------------- */
void aabb_expand(aabb_t *aabb, double val);
aabb_t create_fast_approx_boundingbox(const vertex_t v1, const vertex_t v2);

/* ------------------------ polyline_t base functions ----------------------- */

void polyline_set_pos(polyline_t *pline, size_t index, const point_t pos);
void polyline_set_bulge(polyline_t *pline, size_t index, const double bulge);
void polyline_set_vertex(polyline_t *pline, size_t index, const vertex_t v);
void polyline_erase_vertex(polyline_t *pline, size_t index);
void polyline_clear_vertices(polyline_t *pline);

#define pl_at(p, i)       polyline_vertex_at(p, i)
#define pl_upp(p, i, pos) polyline_set_pos(p, i, pos)
#define pl_upb(p, i, b)   polyline_set_bulge(p, i, b)
#define pl_up(p, i, v)    polyline_set_vertex(p, i, v)
#define pl_size(p)        polyline_size(p)
#define pl_backv(p)       polyline_vertex_at(p, p->vertex_num - 1)
#define pl_frontv(p)      polyline_vertex_at(p, 0)
#define pl_erase(p, i)    polyline_erase_vertex(p, i)
#define pl_clear(p)       polyline_clear_vertices(p)

/* -------------------------- polyline_t functions -------------------------- */

polyline_t *create_raw_offset_polyline(const polyline_t *pline, double offset);
pline_offset_segment_t *
create_untrimmed_offset_segments(const polyline_t *pline, double offset,
                                 size_t *seg_size);
void pline_add_or_replace_if_same_pos(polyline_t *pline, const vertex_t v,
                                      double epsilon);
polyline_t **slices_from_raw_offset(const polyline_t *pline,
                                    const polyline_t *rawOffsetPline,
                                    double offset, size_t *slice_size);
quickIndex_t *create_approx_qIndex(const polyline_t *pline);

#define false_intersect(t) ((t) < 0.0 || (t) > 1.0)

void pline_internal_join_visitor(const pline_offset_segment_t s1,
                                 const pline_offset_segment_t s2, bool ccw,
                                 polyline_t *result);
void pline_internal_join_LL(const pline_offset_segment_t s1,
                            const pline_offset_segment_t s2, bool ccw,
                            polyline_t *result);
void pline_internal_join_LA(const pline_offset_segment_t s1,
                            const pline_offset_segment_t s2, bool ccw,
                            polyline_t *result);
void pline_internal_join_AL(const pline_offset_segment_t s1,
                            const pline_offset_segment_t s2, bool ccw,
                            polyline_t *result);
void pline_internal_join_AA(const pline_offset_segment_t s1,
                            const pline_offset_segment_t s2, bool ccw,
                            polyline_t *result);

void pline_internal_join_processIntersect1(const pline_offset_segment_t s1,
                                           const pline_offset_segment_t s2,
                                           bool ccw, const arc_t arc, double t,
                                           const point_t intersect,
                                           polyline_t *result);

void pline_internal_join_processIntersect2(const pline_offset_segment_t s1,
                                           const pline_offset_segment_t s2,
                                           bool ccw, const arc_t arc, double t,
                                           const point_t intersect,
                                           polyline_t *result);
void pline_internal_join_processIntersect3(const pline_offset_segment_t s1,
                                           const pline_offset_segment_t s2,
                                           bool ccw, const arc_t arc1,
                                           const arc_t arc2, double t,
                                           const point_t intersect,
                                           polyline_t *result);

/* --------------------- polyline_t base function impls --------------------- */

polyline_t *polyline_create()
{
    polyline_t *pline = (polyline_t *)malloc(sizeof(polyline_t));
    if (NULL == pline)
        return NULL;
    pline->closed = false;
    pline->vertex_num = 0;
    pline->vertices = array_new(sizeof(vertex_t));
    if (NULL == pline->vertices) {
        free(pline);
        return NULL;
    }
    return pline;
}

void polyline_free(polyline_t *pline)
{
    assert(pline);
    if (pline->vertices) {
        array_free(pline->vertices, true);
    }
    free(pline);
}

int polyline_append_vertex(polyline_t *pline, vertex_t v)
{
    assert(pline);
    assert(pline->vertices);
    size_t org_size = pline->vertices->size;
    array_append_val(pline->vertices, v);
    if (org_size + 1 == pline->vertices->size) {
        pline->vertex_num++;
        return true;
    }
    return false;
}

vertex_t polyline_vertex_at(const polyline_t *pline, size_t index)
{
    assert(pline);
    assert(pline->vertices);
    assert(pline->vertex_num >= index + 1);
    return array_index(pline->vertices, vertex_t, index);
}

size_t polyline_size(const polyline_t *pline)
{
    assert(pline);
    return pline->vertex_num;
}

void polyline_set_pos(polyline_t *pline, size_t index, const point_t pos)
{
    assert(pline);
    assert(pline->vertices);
    array_index(pline->vertices, vertex_t, index).pos = pos;
}

void polyline_set_bulge(polyline_t *pline, size_t index, const double bulge)
{
    assert(pline);
    assert(pline->vertices);
    array_index(pline->vertices, vertex_t, index).bulge = bulge;
}

void polyline_set_vertex(polyline_t *pline, size_t index, const vertex_t v)
{
    polyline_set_pos(pline, index, v.pos);
    polyline_set_bulge(pline, index, v.bulge);
}

void polyline_erase_vertex(polyline_t *pline, size_t index)
{
    assert(pline);
    assert(pline->vertices);
    array_remove_range(pline->vertices, index, 1);
    pline->vertex_num--;
}

void polyline_clear_vertices(polyline_t *pline)
{
    assert(pline);
    assert(pline->vertices);
    array_remove_range(pline->vertices, 0, pline->vertices->size);
    pline->vertex_num = 0;
}

/* ------------------------ polyline_t function impls ----------------------- */

polyline_t *create_raw_offset_polyline(const polyline_t *pline, double offset)
{
    assert(pline);
    assert(pline->vertices);
    if (pline->vertices->size < 2)
        return NULL;
    polyline_t *res = polyline_create();

    size_t rosize = 0;
    pline_offset_segment_t *raw_offset =
        create_untrimmed_offset_segments(pline, offset, &rosize);
    if (raw_offset == NULL || rosize == 0)
        return res;

    const double connectionArcArcCCW = offset < 0.0;

    // detect single collapsed arc segment (this may be removed in the
    // future if invalid segments are tracked in join functions to be pruned
    // at slice creation)
    array_set_size(res->vertices, pl_size(pline));
    // join first two segments and determine if first vertex was replaced
    // (to know how to handle last two segment joins for closed polyline)
    if (rosize > 1) {
        pline_offset_segment_t seg01 = raw_offset[0];
        pline_offset_segment_t seg12 = raw_offset[1];
        pline_internal_join_visitor(seg01, seg12, connectionArcArcCCW, res);
    }
    const bool firstVertexReplaced = (pl_size(res) == 1) ? true : false;
    for (size_t i = 2; i < rosize; ++i) {
        pline_offset_segment_t seg1 = raw_offset[i - 1];
        pline_offset_segment_t seg2 = raw_offset[i];
        pline_internal_join_visitor(seg1, seg2, connectionArcArcCCW, res);
    }

    if (pline->closed && pl_size(res) > 1) {
        // joining segments at vertex indexes (n, 0) and (0, 1)
        pline_offset_segment_t s1 = raw_offset[rosize - 1];
        pline_offset_segment_t s2 = raw_offset[0];

        // temp polyline to capture results of joining (to avoid mutating
        // result)
        polyline_t *closingPartResult = polyline_create();
        polyline_append_vertex(closingPartResult, pl_backv(res));
        pline_internal_join_visitor(s1, s2, connectionArcArcCCW,
                                    closingPartResult);

        // update last vertex
        pl_up(res, pl_size(res) - 1, pl_at(closingPartResult, 0));
        for (size_t i = 1; i < closingPartResult->vertex_num - 1; ++i) {
            polyline_append_vertex(res, pl_at(closingPartResult, i));
        }

        // update first vertex
        if (!firstVertexReplaced) {
            point_t updatedFirstPos = pl_backv(closingPartResult).pos;
            if (pl_at(res, 0).bulge == 0.0) {
                pl_upp(res, 0, updatedFirstPos);
            }
            else if (pl_size(res) > 1) {
                arc_t arc = arc_radius_and_center(pl_at(res, 0), pl_at(res, 1));
                const double a1 = pt_angle(arc.center, updatedFirstPos);
                const double a2 = pt_angle(arc.center, pl_at(res, 1).pos);
                const double updatedTheta = deltaAngle(a1, a2);
                if ((updatedTheta < 0.0 && pl_at(res, 0).bulge > 0.0) ||
                    (updatedTheta > 0.0 && pl_at(res, 0).bulge < 0.0))
                    pl_upp(res, 0, updatedFirstPos);
                else {
                    pl_upp(res, 0, updatedFirstPos);
                    pl_upb(res, 0, tan(updatedTheta / 4.0));
                }
            }
        }

        // must do final singularity prune between first and second vertex
        // after joining curves (n, 0) and (0, 1)
        if (pl_size(res) > 1) {
            if (pt_fuzzyEqual(pl_at(res, 0).pos, pl_at(res, 1).pos,
                              THRESHOLD_ELLIPSE)) {
                pl_erase(res, 0);
            }
        }
    }
    else {
        pline_add_or_replace_if_same_pos(res, raw_offset[rosize - 1].v2,
                                         PERCISION_ELLIPSE);
    }

    free(raw_offset);
    if (pl_size(res) == 1) {
        pl_clear(res);
    }
    return res;
}

pline_offset_segment_t *
create_untrimmed_offset_segments(const polyline_t *pline, double offset,
                                 size_t *seg_size)
{
    *seg_size = 0;
    size_t segment_count =
        pline->closed ? pline->vertex_num : pline->vertex_num - 1;
    pline_offset_segment_t *segments = (pline_offset_segment_t *)malloc(
        sizeof(pline_offset_segment_t) * segment_count);
    assert(segments);

    for (size_t i = 1; i < pline->vertex_num; ++i) {
        vertex_t v1 = pl_at(pline, i - 1);
        vertex_t v2 = pl_at(pline, i);

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

void pline_add_or_replace_if_same_pos(polyline_t *pline, const vertex_t v,
                                      double epsilon)
{
    assert(pline);
    assert(pline->vertices);
    if (pl_size(pline) == 0) {
        polyline_append_vertex(pline, v);
        return;
    }

    if (pt_fuzzyEqual(pl_backv(pline).pos, v.pos, epsilon)) {
        pl_upb(pline, pl_size(pline) - 1, v.bulge);
        return;
    }

    polyline_append_vertex(pline, v);
}

polyline_t **slices_from_raw_offset(const polyline_t *originalPline,
                                    const polyline_t *rawOffsetPline,
                                    double offset, size_t *slice_size)
{
    assert(originalPline);
    assert(rawOffsetPline);
    assert(originalPline->closed);

    if (pl_size(rawOffsetPline) < 2) {
        *slice_size = 0;
        return NULL;
    }

    quickIndex_t *origPlineSpatialIndex = create_approx_qIndex(originalPline);
    quickIndex_t *rawOffsetPlineSpatialIndex =
        create_approx_qIndex(rawOffsetPline);

    return NULL;
}

quickIndex_t *create_approx_qIndex(const polyline_t *pline)
{
    assert(pline);
    assert(pl_size(pline) > 0);

    size_t segmentCount = pline->closed ? pl_size(pline) : pl_size(pline) - 1;
    quickIndex_t *result =
        qindex_create(segmentCount, QINDEX_DEFAULT_NODE_SIZE);
    if (!result)
        return NULL;

    for (size_t i = 0; i < pl_size(pline) - 1; ++i) {
        aabb_t approxBB = create_fast_approx_boundingbox(pl_at(pline, i),
                                                         pl_at(pline, i + 1));
        qindex_add(result, approxBB.xmin, approxBB.ymin, approxBB.xmax,
                   approxBB.ymax);
    }

    if (pline->closed) {
        // add final segment from last to first
        aabb_t approxBB =
            create_fast_approx_boundingbox(pl_backv(pline), pl_frontv(pline));
        qindex_add(result, approxBB.xmin, approxBB.ymin, approxBB.xmax,
                   approxBB.ymax);
    }

    qindex_finish(result);

    return result;
    return NULL;
}

void pline_internal_join_visitor(const pline_offset_segment_t s1,
                                 const pline_offset_segment_t s2, bool ccw,
                                 polyline_t *result)
{
    const bool s1IsLine = s1.v1.bulge == 0.0;
    const bool s2IsLine = s2.v1.bulge == 0.0;

    if (s1IsLine && s2IsLine) {
        pline_internal_join_LL(s1, s2, ccw, result);
    }
    else if (s1IsLine) {
        pline_internal_join_LA(s1, s2, ccw, result);
    }
    else if (s2IsLine) {
        pline_internal_join_AL(s1, s2, ccw, result);
    }
    else {
        pline_internal_join_AA(s1, s2, ccw, result);
    }
}

void pline_internal_join_LL(const pline_offset_segment_t s1,
                            const pline_offset_segment_t s2, bool ccw,
                            polyline_t *result)
{
    const vertex_t v1 = s1.v1;
    const vertex_t v2 = s1.v2;
    const vertex_t u1 = s2.v1;
    const vertex_t u2 = s2.v2;
    // both segs should be lines
    assert(v1.bulge == 0.0 && u1.bulge == 0.0);

    if (s1.collapsed_arc || s2.collapsed_arc) {
        // connecting to/from collapsed arc, always connect using arc
        goto connect_using_arc;
    }
    else {
        intr_LL intrResult = intersection_LL(v1.pos, v2.pos, u1.pos, u2.pos);
        switch (intrResult.intrType) {
        case LL_NONE:
            pline_add_or_replace_if_same_pos(result, (vertex_t){v2.pos, 0.0},
                                             THRESHOLD_ELLIPSE);
            pline_add_or_replace_if_same_pos(result, u1, THRESHOLD_ELLIPSE);
            break;
        case LL_TRUE:
            pline_add_or_replace_if_same_pos(
                result, (vertex_t){intrResult.point, 0.0}, THRESHOLD_ELLIPSE);
            break;
        case LL_COINCIDENT:
            pline_add_or_replace_if_same_pos(result, (vertex_t){v2.pos, 0.0},
                                             THRESHOLD_ELLIPSE);
            break;
        case LL_FALSE:
            if (intrResult.t0 > 1.0 && false_intersect(intrResult.t1)) {
                // extend and join the lines together using an arc
                goto connect_using_arc;
            }
            else {
                pline_add_or_replace_if_same_pos(
                    result, (vertex_t){v2.pos, 0.0}, THRESHOLD_ELLIPSE);
                pline_add_or_replace_if_same_pos(result, u1, THRESHOLD_ELLIPSE);
            }
            break;
        };
        return;
    }

connect_using_arc: {
    const point_t arcCenter = s1.orig_pos;
    const point_t sp = v2.pos;
    const point_t ep = u1.pos;
    double bulge = bulge_for_connection(arcCenter, sp, ep, ccw);
    pline_add_or_replace_if_same_pos(
        result, (vertex_t){.pos = sp, .bulge = bulge}, PERCISION_ELLIPSE);
    pline_add_or_replace_if_same_pos(
        result, (vertex_t){.pos = ep, .bulge = 0.0}, PERCISION_ELLIPSE);
}
}

void pline_internal_join_LA(const pline_offset_segment_t s1,
                            const pline_offset_segment_t s2, bool ccw,
                            polyline_t *result)
{
    const vertex_t v1 = s1.v1;
    const vertex_t v2 = s1.v2;
    const vertex_t u1 = s2.v1;
    const vertex_t u2 = s2.v2;
    // first seg should be arc, second seg should be line
    assert(v1.bulge == 0.0 && u1.bulge != 0.0);

    const arc_t arc = arc_radius_and_center(u1, u2);

    intr_LC intrResult =
        intersection_LC(v1.pos, v2.pos, arc.radius, arc.center);
    if (intrResult.numIntersects == 0) {
        const point_t arcCenter = s1.orig_pos;
        const point_t sp = v2.pos;
        const point_t ep = u1.pos;
        double bulge = bulge_for_connection(arcCenter, sp, ep, ccw);
        pline_add_or_replace_if_same_pos(
            result, (vertex_t){.pos = sp, .bulge = bulge}, PERCISION_ELLIPSE);
        pline_add_or_replace_if_same_pos(
            result, (vertex_t){.pos = ep, .bulge = 0.0}, PERCISION_ELLIPSE);
    }
    else if (intrResult.numIntersects == 1) {
        pline_internal_join_processIntersect1(
            s1, s2, ccw, arc, intrResult.t0,
            pt_from_parametric(v1.pos, v2.pos, intrResult.t0), result);
    }
    else {
        point_t i1 = pt_from_parametric(v1.pos, v2.pos, intrResult.t0);
        double dist1 = pt_distance2(i1, s1.orig_pos);
        point_t i2 = pt_from_parametric(v1.pos, v2.pos, intrResult.t1);
        double dist2 = pt_distance2(i2, s1.orig_pos);

        if (dist1 < dist2) {
            pline_internal_join_processIntersect1(s1, s2, ccw, arc,
                                                  intrResult.t0, i1, result);
        }
        else {
            pline_internal_join_processIntersect1(s1, s2, ccw, arc,
                                                  intrResult.t1, i2, result);
        }
    }
}

void pline_internal_join_AL(const pline_offset_segment_t s1,
                            const pline_offset_segment_t s2, bool ccw,
                            polyline_t *result)
{
    const vertex_t v1 = s1.v1;
    const vertex_t v2 = s1.v2;
    const vertex_t u1 = s2.v1;
    const vertex_t u2 = s2.v2;
    // first seg should be line, second seg should be arc
    assert(v1.bulge != 0.0 && u1.bulge == 0.0);

    const arc_t arc = arc_radius_and_center(v1, v2);

    intr_LC intrResult =
        intersection_LC(u1.pos, u2.pos, arc.radius, arc.center);
    if (intrResult.numIntersects == 0) {
    }
    else if (intrResult.numIntersects == 1) {
        pline_internal_join_processIntersect2(
            s1, s2, ccw, arc, intrResult.t0,
            pt_from_parametric(u1.pos, u2.pos, intrResult.t0), result);
    }
    else {
        const point_t origPoint = s2.collapsed_arc ? u1.pos : s1.orig_pos;
        point_t i1 = pt_from_parametric(u1.pos, u2.pos, intrResult.t0);
        double dist1 = pt_distance2(i1, origPoint);
        point_t i2 = pt_from_parametric(u1.pos, u2.pos, intrResult.t1);
        double dist2 = pt_distance2(i2, origPoint);

        if (dist1 < dist2) {
            pline_internal_join_processIntersect2(s1, s2, ccw, arc,
                                                  intrResult.t0, i1, result);
        }
        else {
            pline_internal_join_processIntersect2(s1, s2, ccw, arc,
                                                  intrResult.t1, i2, result);
        }
    }
}

void pline_internal_join_AA(const pline_offset_segment_t s1,
                            const pline_offset_segment_t s2, bool ccw,
                            polyline_t *result)
{
}

void pline_internal_join_processIntersect1(const pline_offset_segment_t s1,
                                           const pline_offset_segment_t s2,
                                           bool ccw, const arc_t arc, double t,
                                           const point_t intersect,
                                           polyline_t *result)
{
    const vertex_t v1 = s1.v1;
    const vertex_t v2 = s1.v2;
    const vertex_t u1 = s2.v1;
    const vertex_t u2 = s2.v2;

    const bool trueSegIntersect = !false_intersect(t);
    const bool trueArcIntersect = point_within_arc_sweep_angle(
        arc.center, u1.pos, u2.pos, u1.bulge, intersect);
    if (trueSegIntersect && trueArcIntersect) {
        // trim at intersect
        double a = pt_angle(arc.center, intersect);
        double arcEndAngle = pt_angle(arc.center, u2.pos);
        double theta = deltaAngle(a, arcEndAngle);
        // ensure the sign matches (may get flipped if intersect is at the very
        // end of the arc, in which case we do not want to update the bulge)
        if ((theta > 0.0) == (u1.bulge > 0.0)) {
            pline_add_or_replace_if_same_pos(
                result, (vertex_t){intersect, tan(theta / 4.0)},
                PERCISION_ELLIPSE);
        }
        else {
            pline_add_or_replace_if_same_pos(
                result, (vertex_t){intersect, u1.bulge}, PERCISION_ELLIPSE);
        }
    }
    else if (t > 1.0 && !trueArcIntersect) {
        goto connect_using_arc;
    }
    else if (s1.collapsed_arc) {
        // collapsed arc connecting to arc, connect using arc
        goto connect_using_arc;
    }
    else {
        // connect using line
        pline_add_or_replace_if_same_pos(result, (vertex_t){v2.pos, 0.0},
                                         PERCISION_ELLIPSE);
        pline_add_or_replace_if_same_pos(result, u1, PERCISION_ELLIPSE);
    }

connect_using_arc: {
    const point_t arcCenter = s1.orig_pos;
    const point_t sp = v2.pos;
    const point_t ep = u1.pos;
    double bulge = bulge_for_connection(arcCenter, sp, ep, ccw);
    pline_add_or_replace_if_same_pos(
        result, (vertex_t){.pos = sp, .bulge = bulge}, PERCISION_ELLIPSE);
    pline_add_or_replace_if_same_pos(
        result, (vertex_t){.pos = ep, .bulge = 0.0}, PERCISION_ELLIPSE);
}
}

void pline_internal_join_processIntersect2(const pline_offset_segment_t s1,
                                           const pline_offset_segment_t s2,
                                           bool ccw, const arc_t arc, double t,
                                           const point_t intersect,
                                           polyline_t *result)
{
    const vertex_t v1 = s1.v1;
    const vertex_t v2 = s1.v2;
    const vertex_t u1 = s2.v1;
    const vertex_t u2 = s2.v2;

    const bool trueSegIntersect = !false_intersect(t);
    const bool trueArcIntersect = point_within_arc_sweep_angle(
        arc.center, v1.pos, v2.pos, v1.bulge, intersect);
    if (trueSegIntersect && trueArcIntersect) {
        vertex_t prevVertex = pl_backv(result);

        if (prevVertex.bulge != 0 &&
            !pt_fuzzyEqual(prevVertex.pos, v2.pos, THRESHOLD_ELLIPSE)) {
            // modify previous bulge and trim at intersect
            double a = pt_angle(arc.center, intersect);
            arc_t prevArc = arc_radius_and_center(prevVertex, v2);
            double prevArcStartAngle = pt_angle(prevArc.center, prevVertex.pos);
            double updatedPrevTheta = deltaAngle(prevArcStartAngle, a);

            // ensure the sign matches (may get flipped if intersect is at the
            // very end of the arc, in which case we do not want to update the
            // bulge)
            if ((updatedPrevTheta > 0.0) == (prevVertex.bulge > 0.0)) {
                pl_upb(result, pl_size(result) - 1,
                       tan(updatedPrevTheta / 4.0));
            }
        }
        pline_add_or_replace_if_same_pos(result, (vertex_t){intersect, 0.0},
                                         THRESHOLD_ELLIPSE);
    }
    else {
        const point_t arcCenter = s1.orig_pos;
        const point_t sp = v2.pos;
        const point_t ep = u1.pos;
        double bulge = bulge_for_connection(arcCenter, sp, ep, ccw);
        pline_add_or_replace_if_same_pos(result, (vertex_t){sp, bulge},
                                         PERCISION_ELLIPSE);
        pline_add_or_replace_if_same_pos(result, u1, PERCISION_ELLIPSE);
    }
}
void pline_internal_join_processIntersect3(const pline_offset_segment_t s1,
                                           const pline_offset_segment_t s2,
                                           bool ccw, const arc_t arc1,
                                           const arc_t arc2, double t,
                                           const point_t intersect,
                                           polyline_t *result)
{
    const vertex_t v1 = s1.v1;
    const vertex_t v2 = s1.v2;
    const vertex_t u1 = s2.v1;
    const vertex_t u2 = s2.v2;

    const bool trueArcIntersect1 = point_within_arc_sweep_angle(
        arc1.center, v1.pos, v2.pos, v1.bulge, intersect);
    const bool trueArcIntersect2 = point_within_arc_sweep_angle(
        arc2.center, u1.pos, u2.pos, u1.bulge, intersect);

    if (trueArcIntersect1 && trueArcIntersect2) {
        vertex_t prevVertex = pl_backv(result);
        if (prevVertex.bulge != 0 &&
            !pt_fuzzyEqual(prevVertex.pos, v2.pos, THRESHOLD_ELLIPSE)) {
            // modify previous bulge and trim at intersect
            double a1 = pt_angle(arc1.center, intersect);
            arc_t prevArc = arc_radius_and_center(prevVertex, v2);
            double prevArcStartAngle = pt_angle(prevArc.center, prevVertex.pos);
            double updatedPrevTheta = deltaAngle(prevArcStartAngle, a1);

            // ensure the sign matches (may get flipped if intersect is at the
            // very end of the arc, in which case we do not want to update the
            // bulge)
            if ((updatedPrevTheta > 0.0) == (prevVertex.bulge > 0.0)) {
                pl_upb(result, pl_size(result) - 1,
                       tan(updatedPrevTheta / 4.0));
            }
        }

        // add the vertex at our current trim/join point
        double a2 = pt_angle(arc2.center, intersect);
        double endAngle = pt_angle(arc2.center, u2.pos);
        double theta = deltaAngle(a2, endAngle);

        // ensure the sign matches (may get flipped if intersect is at the very
        // end of the arc, in which case we do not want to update the bulge)
        if ((theta > 0.0) == (u1.bulge > 0.0)) {
            pline_add_or_replace_if_same_pos(
                result, (vertex_t){intersect, tan(theta / 4.0)},
                THRESHOLD_ELLIPSE);
        }
        else {
            pline_add_or_replace_if_same_pos(
                result, (vertex_t){intersect, u1.bulge}, THRESHOLD_ELLIPSE);
        }
    }
    else {
        const point_t arcCenter = s1.orig_pos;
        const point_t sp = v2.pos;
        const point_t ep = u1.pos;
        double bulge = bulge_for_connection(arcCenter, sp, ep, ccw);
        pline_add_or_replace_if_same_pos(result, (vertex_t){sp, bulge},
                                         THRESHOLD_ELLIPSE);
        pline_add_or_replace_if_same_pos(result, u1, THRESHOLD_ELLIPSE);
    }
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

bool fuzzyEqual(double a, double b, double epsilon)
{
    return fabs(a - b) < epsilon;
}

bool fuzzInRange(double min, double a, double max, double epsilon)
{
    return (a + epsilon > min) && (a < max + epsilon);
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

point_t pt_div(const point_t p, double s)
{
    assert(s != 0);
    return (point_t){p.x / s, p.y / s};
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

double pt_normalize(point_t *p)
{
    double len = hypot(p->x, p->y);
    assert(len != 0);
    p->x = p->x / len;
    p->y = p->y / len;
    return len;
}

bool pt_fuzzyEqual(const point_t p1, const point_t p2, double tol)
{
    if (fabs(p1.x - p2.x) < tol && fabs(p1.y - p2.y) < tol)
        return true;
    return false;
}

double pt_distance(const point_t p0, const point_t p1)
{
    double dx = p1.x - p0.x;
    double dy = p1.y - p0.x;
    return dx * dx + dy * dy;
}

double pt_distance2(const point_t p0, const point_t p1)
{
    return sqrt(pt_distance(p0, p1));
}

point_t pt_from_parametric(const point_t p0, const point_t p1, const double t)
{
    // p0 + t * (p1 - p0)
    return pt_add(p0, pt_mul(t, pt_sub(p1, p0)));
}

/* ----------------------------- arc_t functions ---------------------------- */

arc_t arc_radius_and_center(const vertex_t v1, const vertex_t v2)
{
    assert(v1.bulge != 0);
    assert(!pt_fuzzyEqual(v1.pos, v2.pos, THRESHOLD_ELLIPSE));

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

double bulge_for_connection(const point_t center, const point_t sp,
                            const point_t ep, bool ccw)
{
    double a1 = pt_angle(center, sp);
    double a2 = pt_angle(center, ep);
    double sweep_angle = fabs(deltaAngle(a1, a2));
    double bulge = tan(sweep_angle / 4.0);
    if (ccw)
        return bulge;
    else
        return -bulge;
}

bool point_within_arc_sweep_angle(const point_t center, const point_t start,
                                  const point_t end, double bulge,
                                  const point_t point)
{
    assert(fabs(bulge) > THRESHOLD_ELLIPSE);
    assert(fabs(bulge) <= 1.0);

#define left_or_coincident(p0, p1, point, epsilon)  \
    (((p1).x - (p0).x) * ((point).y - (p0).y) -     \
         ((p1).y - (p0).y) * ((point).x - (p0).x) > \
     -(epsilon))
#define right_or_coincident(p0, p1, point, epsilon) \
    (((p1).x - (p0).x) * ((point).y - (p0).y) -     \
         ((p1).y - (p0).y) * ((point).x - (p0).x) < \
     (epsilon))

    if (bulge > 0.0) {
        return left_or_coincident(center, start, point, THRESHOLD_ELLIPSE) &&
               right_or_coincident(center, end, point, THRESHOLD_ELLIPSE);
    }
    else {
        return left_or_coincident(center, end, point, THRESHOLD_ELLIPSE) &&
               right_or_coincident(center, start, point, THRESHOLD_ELLIPSE);
    }
}

/* ---------------------------- aabb_t functions ---------------------------- */
void aabb_expand(aabb_t *aabb, double val)
{
    assert(aabb);
    aabb->xmin -= val;
    aabb->ymin -= val;
    aabb->xmax += val;
    aabb->ymax += val;
}

aabb_t create_fast_approx_boundingbox(const vertex_t v1, const vertex_t v2)
{
    aabb_t result;
    if (v1.bulge == 0.0) {
        if (v1.pos.x < v2.pos.x) {
            result.xmin = v1.pos.x;
            result.xmax = v2.pos.x;
        }
        else {
            result.xmin = v2.pos.x;
            result.xmax = v1.pos.x;
        }

        if (v1.pos.y < v2.pos.y) {
            result.ymin = v1.pos.y;
            result.ymax = v2.pos.y;
        }
        else {
            result.ymin = v2.pos.y;
            result.ymax = v1.pos.y;
        }

        return result;
    }

    // For arcs we don't compute the actual extents which is slower, instead we
    // create an approximate bounding box from the rectangle formed by extending
    // the chord by the sagitta, NOTE: this approximate bounding box is always
    // equal to or bigger than the true bounding box
    double b = v1.bulge;
    double offsX = b * (v2.pos.y - v1.pos.y) / 2.0;
    double offsY = -b * (v2.pos.x - v1.pos.x) / 2.0;

    double pt1X = v1.pos.x + offsX;
    double pt2X = v2.pos.x + offsX;
    double pt1Y = v1.pos.y + offsY;
    double pt2Y = v2.pos.y + offsY;

    double endPointXMin, endPointXMax;
    if (v1.pos.x < v2.pos.x) {
        endPointXMin = v1.pos.x;
        endPointXMax = v2.pos.x;
    }
    else {
        endPointXMin = v2.pos.x;
        endPointXMax = v1.pos.x;
    }

    double ptXMin, ptXMax;
    if (pt1X < pt2X) {
        ptXMin = pt1X;
        ptXMax = pt2X;
    }
    else {
        ptXMin = pt2X;
        ptXMax = pt1X;
    }

    double endPointYMin, endPointYMax;
    if (v1.pos.y < v2.pos.y) {
        endPointYMin = v1.pos.y;
        endPointYMax = v2.pos.y;
    }
    else {
        endPointYMin = v2.pos.y;
        endPointYMax = v1.pos.y;
    }

    double ptYMin, ptYMax;
    if (pt1Y < pt2Y) {
        ptYMin = pt1Y;
        ptYMax = pt2Y;
    }
    else {
        ptYMin = pt2Y;
        ptYMax = pt1Y;
    }

    result.xmin = MIN(endPointXMin, ptXMin);
    result.ymin = MIN(endPointYMin, ptYMin);
    result.xmax = MAX(endPointXMax, ptXMax);
    result.ymax = MAX(endPointYMax, ptYMax);
    return result;
}

/* ------------------------ polyline_t main functions ----------------------- */

polyline_t **polyline_parallel_offset(const polyline_t *pline, double offset,
                                      int hasSelfIntersects, int *res_size)
{
    assert(pline);
    assert(pline->vertices);
    if (pline->vertices->size < 2)
        return NULL;
    polyline_t *rawOffset = create_raw_offset_polyline(pline, offset);
    // if (pline->closed && !hasSelfIntersects) {
    //     size_t slice_size = 0;
    //     polyline_t **slices =
    //         slices_from_raw_offset(pline, rawOffset, offset, &slice_size);
    //     polyline_t **result =
    //         stitch_offset_slices_together(slices, slice_size, pline->closed,
    //                                       pl_size(rawOffset) - 1, res_size);
    //     polyline_free(rawOffset);
    //     for (size_t i = 0; i < slice_size; ++i) {
    //         polyline_free(slices[i]);
    //     }
    //     return result;
    // }
    // else {
    //     // not closed polyline or has self intersects, must apply dual
    //     clipping polyline_t *dualRawOffset =
    //     create_raw_offset_polyline(pline, offset); size_t slice_size = 0;
    //     polyline_t **slices = dual_slice_at_intersects_for_offset(
    //         pline, rawOffset, dualRawOffset, offset, &slice_size);
    //     polyline_t **result =
    //         stitch_offset_slices_together(slices, slice_size, pline->closed,
    //                                       pl_size(rawOffset) - 1, res_size);
    //     polyline_free(rawOffset);
    //     polyline_free(dualRawOffset);
    //     for (size_t i = 0; i < slice_size; ++i) {
    //         polyline_free(slices[i]);
    //     }
    //     return result;
    // }

    return NULL;
}

/* ------------------ intersection structure and functions ------------------ */

// http://paulbourke.net/geometry/circlesphere/
// https://github.com/MerlotRain/circlesphere/blob/main/intersection_2circles.c
intr_CC intersection_CC(double radius1, const point_t center1, double radius2,
                        const point_t center2)
{
    intr_CC result;
    point_t cv = pt_sub(center2, center1);
    double d2 = hypot(cv.x, cv.y);
    double d = sqrt(d2);
    if (d < THRESHOLD_ELLIPSE) {
        if (fuzzyEqual(radius1, radius2, THRESHOLD_ELLIPSE))
            result.intrType = CC_COINCIDENT;
        else
            result.intrType = CC_NO_INTERSECT;
    }
    else {
        if (d > radius1 + radius2 || d < fabs(radius1 - radius2))
            result.intrType = CC_NO_INTERSECT;
        else {
            double rad11 = radius1 * radius1;
            double rad22 = radius2 * radius2;
            double a = (rad11 - rad22 + d2) / (2.0 * d);
            point_t midPoint = pt_add(center1, pt_div(pt_mul(a, cv), d));
            double diff = rad11 - a * a;
            if (diff < 0.0) {
                result.intrType = CC_ONE_INTERSECT;
                result.point1 = midPoint;
            }
            else {
                double h = sqrt(diff);
                double hOverD = h / d;
                double xTerm = hOverD * cv.y;
                double yTerm = hOverD * cv.x;
                double x1 = midPoint.x + xTerm;
                double y1 = midPoint.y - yTerm;
                double x2 = midPoint.x - xTerm;
                double y2 = midPoint.y + yTerm;
                result.point1 = (point_t){x1, y1};
                result.point2 = (point_t){x2, y2};
                if (pt_fuzzyEqual(result.point1, result.point2,
                                  THRESHOLD_ELLIPSE)) {
                    result.intrType = CC_ONE_INTERSECT;
                }
                else {
                    result.intrType = CC_TWO_INTERSECTS;
                }
            }
        }
    }
    return result;
}

// http://paulbourke.net/geometry/circlesphere/
// https://github.com/MerlotRain/circlesphere/blob/main/intersection_with_line.c
intr_LC intersection_LC(const point_t p0, const point_t p1, double radius,
                        const point_t center)
{
    // This function solves for t by substituting the parametric equations for
    // the segment x = v1.X + t * (v2.X - v1.X) and y = v1.Y + t * (v2.Y - v1.Y)
    // for t = 0 to t = 1 into the circle equation (x-h)^2 + (y-k)^2 = r^2 and
    // then solving the resulting equation in the form a*t^2 + b*t + c = 0 using
    // the quadratic formula
    intr_LC result;
    double dx = p1.x - p0.x;
    double dy = p1.y - p0.y;
    double h = center.x;
    double k = center.y;

    double a = dx * dx + dy * dy;
    if (fabs(a) < THRESHOLD_ELLIPSE) {
        // v1 = v2 test if point is on the circle
        double xh = p0.x - h;
        double yk = p0.y - k;
        if (fuzzyEqual(xh * xh + yk * yk, radius * radius, THRESHOLD_ELLIPSE)) {
            result.numIntersects = 1;
            result.t0 = 0.0;
        }
        else {
            result.numIntersects = 0;
        }
    }
    else {
        double b = 2.0 * (dx * (p0.x - h) + dy * (p0.y - k));
        double c = (p0.x * p0.x - 2.0 * h * p0.x + h * h) +
                   (p0.y * p0.y - 2.0 * k * p0.y + k * k) - radius * radius;
        double discr = b * b - 4.0 * a * c;

        if (fabs(discr) < THRESHOLD_ELLIPSE) {
            // 1 solution (tangent line)
            result.numIntersects = 1;
            result.t0 = -b / (2.0 * a);
        }
        else if (discr < 0.0) {
            result.numIntersects = 0;
        }
        else {
            double sqrtDiscr = sqrt(discr);
            double denom = 2.0 * a;
            double sol1;
            if (b < 0.0) {
                sol1 = (-b + sqrtDiscr) / denom;
            }
            else {
                sol1 = (-b - sqrtDiscr) / denom;
            }
            double sol2 = (c / a) / sol1;

            result.numIntersects = 2;
            result.t0 = sol1;
            result.t1 = sol2;
        }
    }
    return result;
}

static bool intr_LL_hint_segment(const point_t pt, const point_t start,
                                 const point_t end)
{
    if (fuzzyEqual(start.x, end.x, THRESHOLD_ELLIPSE)) {
        double min = MIN(start.y, end.y);
        double max = MAX(start.y, end.y);
        return fuzzInRange(min, pt.y, max, THRESHOLD_ELLIPSE);
    }
    else {
        double min = MIN(start.x, end.x);
        double max = MAX(start.x, end.x);
        return fuzzInRange(min, pt.x, max, THRESHOLD_ELLIPSE);
    }
}

intr_LL intersection_LL(const point_t u1, const point_t u2, const point_t v1,
                        const point_t v2)
{

#define perp_dot(v0, v1) ((v0).x * (v1).y - (v0).y * (v1).x)
    intr_LL result;
    point_t u = pt_sub(u2, u1);
    point_t v = pt_sub(v2, v1);
    double d = perp_dot(u, v);

    point_t w = pt_sub(u1, v1);

    // threshold check here to avoid almost parallel lines resulting in very
    // distant intersection
    if (fabs(d) > THRESHOLD_ELLIPSE) {
        // segments not parallel or collinear
        result.t0 = perp_dot(v, w) / d;
        result.t1 = perp_dot(u, w) / d;
        result.point = pt_add(v1, pt_mul(result.t1, v));
        if (result.t0 < 0.0 || result.t0 > 1.0 || result.t1 < 0.0 ||
            result.t1 > 1.0)
            result.intrType = LL_FALSE;
        else
            result.intrType = LL_TRUE;
    }
    else {
        // segments are parallel or collinear
        double a = perp_dot(u, w);
        double b = perp_dot(v, w);
        if (fabs(a) > THRESHOLD_ELLIPSE || fabs(b) > THRESHOLD_ELLIPSE)
            result.intrType = LL_NONE;
        else {
            // either collinear or degenerate (segments are single points)
            bool uIsPoint = pt_fuzzyEqual(u1, u2, THRESHOLD_ELLIPSE);
            bool vIsPoint = pt_fuzzyEqual(v1, v2, THRESHOLD_ELLIPSE);
            if (uIsPoint && vIsPoint) {
                // both segments are just points
                if (pt_fuzzyEqual(u1, v2, THRESHOLD_ELLIPSE)) {
                    // same point
                    result.point = u1;
                    result.intrType = LL_TRUE;
                }
                else {
                    // distinct point
                    result.intrType = LL_NONE;
                }
            }
            else if (uIsPoint) {
                if (intr_LL_hint_segment(u1, v1, v2)) {
                    result.intrType = LL_TRUE;
                    result.point = u1;
                }
                else {
                    result.intrType = LL_NONE;
                }
            }
            else if (vIsPoint) {
                if (intr_LL_hint_segment(v1, u1, u2)) {
                    result.intrType = LL_TRUE;
                    result.point = v1;
                }
                else {
                    result.intrType = LL_NONE;
                }
            }
            else {
                // neither segment is a point, check if they overlap
                point_t w2 = pt_sub(u2, u1);
                if (fabs(v.x) < THRESHOLD_ELLIPSE) {
                    result.t0 = w.y / v.y;
                    result.t1 = w2.y / v.y;
                }
                else {
                    result.t0 = w.x / v.x;
                    result.t1 = w2.x / v.x;
                }

                if (result.t0 > result.t1) {
                    double tmp = result.t0;
                    result.t0 = result.t1;
                    result.t1 = tmp;
                }

                // using threshold check here to make intersect "sticky" to
                // prefer considering it an intersect
                if (result.t0 > 1.0 || result.t1 < 0.0) {
                    // no overlap
                    result.intrType = LL_NONE;
                }
                else {
                    result.t0 = MAX(result.t0, 0.0);
                    result.t1 = MIN(result.t1, 1.0);
                    if (fabs(result.t1 - result.t0) < THRESHOLD_ELLIPSE) {
                        result.intrType = LL_TRUE;
                        result.point = pt_add(v1, pt_mul(result.t0, v));
                    }
                    else {
                        result.intrType = LL_COINCIDENT;
                    }
                }
            }
        }
    }
#undef perp_dot
    return result;
}
