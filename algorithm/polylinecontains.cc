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

#include "cada_shape.h"
#include <assert.h>

using namespace cada::shape;

namespace cada {
namespace algorithm {

struct CubicBezier {
    static CubicBezier fromPoints(const Vec2d &p1, const Vec2d &p2,
                                  const Vec2d &p3, const Vec2d &p4)
    {
        CubicBezier bezier;
        bezier.pt1 = p1;
        bezier.pt2 = p2;
        bezier.pt3 = p3;
        bezier.pt4 = p4;
    }

    BBox bounds() const;

    Vec2d pt1;
    Vec2d pt2;
    Vec2d pt3;
    Vec2d pt4;
    std::pair<CubicBezier, CubicBezier> split() const;
};

BBox CubicBezier::bounds() const
{
    double x1 = pt1.x;
    double y1 = pt1.y;
    double x2 = pt2.x;
    double y2 = pt2.y;
    double x3 = pt3.x;
    double y3 = pt3.y;
    double x4 = pt4.x;
    double y4 = pt4.y;

    double xmin = x1;
    double xmax = x1;
    if (x2 < xmin)
        xmin = x2;
    else if (x2 > xmax)
        xmax = x2;
    if (x3 < xmin)
        xmin = x3;
    else if (x3 > xmax)
        xmax = x3;
    if (x4 < xmin)
        xmin = x4;
    else if (x4 > xmax)
        xmax = x4;

    double ymin = y1;
    double ymax = y1;
    if (y2 < ymin)
        ymin = y2;
    else if (y2 > ymax)
        ymax = y2;
    if (y3 < ymin)
        ymin = y3;
    else if (y3 > ymax)
        ymax = y3;
    if (y4 < ymin)
        ymin = y4;
    else if (y4 > ymax)
        ymax = y4;
    return BBox(xmin, ymin, xmax, ymax);
}

std::pair<CubicBezier, CubicBezier> CubicBezier::split() const
{
    const auto mid = [](Vec2d lhs, Vec2d rhs) { return (lhs + rhs) * 0.5; };

    const Vec2d mid_12 = mid(pt1, pt2);
    const Vec2d mid_23 = mid(pt2, pt3);
    const Vec2d mid_34 = mid(pt3, pt4);
    const Vec2d mid_12_23 = mid(mid_12, mid_23);
    const Vec2d mid_23_34 = mid(mid_23, mid_34);
    const Vec2d mid_12_23__23_34 = mid(mid_12_23, mid_23_34);

    return {
        fromPoints(pt1, mid_12, mid_12_23, mid_12_23__23_34),
        fromPoints(mid_12_23__23_34, mid_23_34, mid_34, pt4),
    };
}

static void cada__path_isect_line(const Vec2d &p1, const Vec2d &p2,
                                  const Vec2d &pos, int *winding)
{
    double x1 = p1.x;
    double y1 = p1.y;
    double x2 = p2.x;
    double y2 = p2.y;
    double y = pos.y;

    int dir = 1;

    if (Math::fuzzyCompare(y1, y2)) {
        // ignore horizontal lines according to scan conversion rule
        return;
    }
    else if (y2 < y1) {
        double x_tmp = x2;
        x2 = x1;
        x1 = x_tmp;
        double y_tmp = y2;
        y2 = y1;
        y1 = y_tmp;
        dir = -1;
    }

    if (y >= y1 && y < y2) {
        double x = x1 + ((x2 - x1) / (y2 - y1)) * (y - y1);

        // count up the winding number if we're
        if (x <= pos.x) {
            (*winding) += dir;
        }
    }
}

static void cada__path_isect_bezier(const CubicBezier &bezier, const Vec2d &pt,
                                    int *winding, int depth = 0)
{
    double x = pt.x;
    double y = pt.y;
    BBox bounds = bezier.bounds();

    // potential intersection, divide and try again...
    // Please note that a sideeffect of the bottom exclusion is that
    // horizontal lines are dropped, but this is correct according to
    // scan conversion rules.
    if (y >= bounds.c1.y && y < bounds.c1.y + bounds.getHeight()) {

        // hit lower limit... This is a rough threshold, but its a
        // tradeoff between speed and precision.
        const double lower_bound = double(.001);
        if (depth == 32 || (bounds.getWidth() < lower_bound &&
                            bounds.getHeight() < lower_bound)) {
            // We make the assumption here that the curve starts to
            // approximate a line after while (i.e. that it doesn't
            // change direction drastically during its slope)
            if (bezier.pt1.x <= x) {
                (*winding) += (bezier.pt4.y > bezier.pt1.y ? 1 : -1);
            }
            return;
        }

        // split curve and try again...
        const auto halves = bezier.split();
        cada__path_isect_bezier(halves.first, pt, winding, depth + 1);
        cada__path_isect_bezier(halves.second, pt, winding, depth + 1);
    }
}

bool cada_polylineContains(const Polyline *pline, const Vec2d &pt)
{
    assert(pline);
    int winding_number = 0;
    for (int i = 0; i < pline->countSegments(); i++) {
        auto &&segment = pline->getSegmentAt(i);

        if (segment->getShapeType() == NS::Line) {
            auto &&line = dynamic_cast<Line *>(segment.get());
            cada__path_isect_line(line->getStartPoint(), line->getEndPoint(),
                                  pt, &winding_number);
        }

        if (segment->getShapeType() == NS::Arc) {
            auto &&splines =
                Spline::createSplineFromArc(dynamic_cast<Arc *>(segment.get()));
            for (auto &&spline : splines) {
                Vec2d v1 = spline->getControlPointAt(0);
                Vec2d v2 = spline->getControlPointAt(1);
                Vec2d v3 = spline->getControlPointAt(2);
                Vec2d v4 = spline->getControlPointAt(3);
                cada__path_isect_bezier(CubicBezier::fromPoints(v1, v2, v3, v3),
                                        pt, &winding_number);
            }
        }
    }

    return (winding_number != 0);
}

} // namespace algorithm
} // namespace cada
