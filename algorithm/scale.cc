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

#include <cada_shape.h>
#include <assert.h>

using namespace cada::shape;

namespace cada {
namespace algorithm {

extern bool cada_ellipse_mirror(shape::Ellipse *e, const shape::Vec2d &v1,
                                const shape::Vec2d &v2);

/* -------------------------------- functions ------------------------------- */

bool cada_scale(shape::Shape *shape, const shape::Vec2d &scaleFactors,
                const shape::Vec2d &c);

std::unique_ptr<Shape> cada_arc_scale_new(const shape::Shape *arc,
                                          const shape::Vec2d &scaleFactors,
                                          const shape::Vec2d &c);
bool cada_polyline_scale(shape::Polyline *polyline,
                         const shape::Vec2d &scaleFactors,
                         const shape::Vec2d &c);

/* ---------------------------------- Impl ---------------------------------- */

bool cada_scale(shape::Shape *shape, const shape::Vec2d &scaleFactors,
                const shape::Vec2d &c)
{
    assert(shape);
    switch (shape->getShapeType()) {
    case NS::Point: {
        auto pt = dynamic_cast<Point *>(shape);
        pt->getPosition().scale(scaleFactors, c);
        return true;
    }
    case NS::Line: {
        auto l = dynamic_cast<Line *>(shape);
        l->setStartPoint(l->getStartPoint().scale(scaleFactors, c));
        l->setEndPoint(l->getEndPoint().scale(scaleFactors, c));
        return true;
    }
    case NS::Arc: {
        auto a = dynamic_cast<Arc *>(shape);
        double radius = a->getRadius();
        Vec2d center = a->getCenter();
        if (scaleFactors.x < 0.0) {
            a->mirror(center, center + Vec2d(0.0, 1.0));
        }
        if (scaleFactors.y < 0.0) {
            a->mirror(center, center + Vec2d(1.0, 0.0));
        }

        a->setCenter(center.scale(scaleFactors, c));
        radius *= scaleFactors.x;
        if (radius < 0.0) {
            a->setRadius(radius *= -1.0);
        }
        else {
            a->setRadius(radius);
        }

        return true;
    }
    case NS::Circle: {
        auto circle = dynamic_cast<Circle *>(shape);
        Vec2d center = circle->getCenter();
        double radius = circle->getRadius();
        center.scale(scaleFactors, c);
        radius *= scaleFactors.x;
        if (radius < 0.0) {
            radius *= -1.0;
        }
        circle->setCenter(center);
        circle->setRadius(radius);
        return true;
    }
    case NS::Ellipse: {
        auto e = dynamic_cast<Ellipse *>(shape);
        if (fabs(fabs(scaleFactors.x) - fabs(scaleFactors.y)) >
            NS::PointTolerance) {
            return false;
        }

        Vec2d center = e->getCenter();
        Vec2d majorPoint = e->getMajorPoint();

        if (scaleFactors.x < 0.0) {
            cada_ellipse_mirror(e, center, center + Vec2d(0.0, 1.0));
        }
        if (scaleFactors.y < 0.0) {
            cada_ellipse_mirror(e, center, center + Vec2d(1.0, 0.0));
        }

        center.scale(scaleFactors, c);
        Vec2d f = Vec2d(fabs(scaleFactors.x), fabs(scaleFactors.y));
        majorPoint.scale(f);

        e->setCenter(center);
        e->setMajorPoint(majorPoint);

        return true;
    }
    case NS::XLine:
    case NS::Ray: {
        auto xl = dynamic_cast<XLine *>(shape);
        xl->setBasePoint(xl->getBasePoint().scale(scaleFactors, c));
        xl->setDirectionVector(xl->getDirectionVector().scale(scaleFactors));
        return true;
    }
    case NS::Polyline: {
        return cada_polyline_scale(dynamic_cast<Polyline *>(shape),
                                   scaleFactors, c);
    }
    case NS::BSpline:
    default:
        break;
    }
    return false;
}

std::unique_ptr<shape::Shape>
cada__ellipse_to_arc_circle_ellipse(const Ellipse *ellipse)
{
    assert(ellipse);
    if (ellipse->isCircular()) {
        if (ellipse->isFullEllipse()) {
            return ShapeFactory::instance()->createCircle(
                ellipse->getCenter(), ellipse->getMajorRadius());
        }
        else {

            Vec2d c = ellipse->getCenter();
            auto ret = ShapeFactory::instance()->createArc(
                c, ellipse->getMajorRadius(),
                0.0, 2 * M_PI, ellipse->isReversed());
            ret->setStartAngle(c.getAngleTo(ellipse->getStartPoint()));
            ret->setEndAngle(c.getAngleTo(ellipse->getEndPoint()));
            return ret;
        }
    }
    else {
        return ellipse->clone();
    }
}

std::unique_ptr<Shape> cada_arc_scale_new(const shape::Shape *shp,
                                          const shape::Vec2d &scaleFactors,
                                          const shape::Vec2d &center)
{
    assert(shp);
    const shape::Arc *arc = dynamic_cast<const shape::Arc *>(shp);

    Vec2d r1 = Vec2d(arc->getRadius(), 0);
    Vec2d r2 = Vec2d(0, arc->getRadius());
    Vec2d c = arc->getCenter();

    // corners of bounding box of untransformed arc:
    Vec2d v1 = c + r1 + r2;
    Vec2d v2 = c + r1 - r2;
    Vec2d v3 = c - r1 - r2;
    Vec2d v4 = c - r1 + r2;

    // transform conrners:
    v1 = v1.getScaled(scaleFactors, center);
    v2 = v2.getScaled(scaleFactors, center);
    v3 = v3.getScaled(scaleFactors, center);
    v4 = v4.getScaled(scaleFactors, center);

    auto &&ellipse =
        ShapeFactory::instance()->createEllipseFromInscribed(v1, v2, v3, v4);

    {
        Vec2d sp = shp->getStartPoint();
        Vec2d ep = shp->getEndPoint();
        Vec2d mp = shp->getMiddlePoint();

        sp = sp.getScaled(scaleFactors, center);
        ep = ep.getScaled(scaleFactors, center);
        mp = mp.getScaled(scaleFactors, center);

        ellipse->setStartParam(ellipse->getParamTo(sp));
        ellipse->setEndParam(ellipse->getParamTo(ep));

        double d1 = ellipse->getMiddlePoint().getDistanceTo(mp);
        ellipse->setReversed(true);
        double d2 = ellipse->getMiddlePoint().getDistanceTo(mp);

        if (d1 < d2) {
            ellipse->setReversed(false);
        }
    }

    return cada__ellipse_to_arc_circle_ellipse(ellipse.release());
}

bool cada_polyline_scale(shape::Polyline *polyline,
                         const shape::Vec2d &scaleFactors,
                         const shape::Vec2d &c)
{
    assert(polyline);
    if (polyline->hasArcSegments() &&
        !Math::fuzzyCompare(scaleFactors.x, scaleFactors.y)) {
        // non-uniform scaling of polyline with arcs:

        std::unique_ptr<Polyline> pl =
            ShapeFactory::instance()->createPolyline();
        for (int i = 0; i < polyline->countSegments(); i++) {
            auto seg = polyline->getSegmentAt(i);
            if (!seg) {
                continue;
            }

            std::unique_ptr<Shape> newSeg;
            if (seg->getShapeType() == NS::Line) {
                newSeg = std::move(seg);
                newSeg->scale(scaleFactors, c);
            }
            else {
                newSeg = cada_arc_scale_new(seg.release(), scaleFactors, c);
            }

            if (newSeg) {
                pl->appendShape(newSeg.release());
            }
        }
        polyline = pl.release();
        return true;
    }

    auto &poly_vertices = polyline->getVertices();
    auto &poly_startWidths = polyline->getStartWidths();
    auto &poly_endWidths = polyline->getEndWidths();
    auto &poly_bulges = polyline->getBulges();

    for (auto &v : poly_vertices) {
        v.scale(scaleFactors, c);
    }
    for (auto &sw : poly_startWidths) {
        if (sw > 0.0) {
            sw *= fabs(scaleFactors.x);
        }
    }
    for (auto &ew : poly_endWidths) {
        if (ew > 0.0) {
            ew *= fabs(scaleFactors.x);
        }
    }

    if ((scaleFactors.x < 0) != (scaleFactors.y < 0)) {
        for (auto &b : poly_bulges) {
            b *= -1;
        }
    }
    return true;
}

} // namespace algorithm
} // namespace cada
