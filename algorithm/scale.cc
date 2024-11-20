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

std::unique_ptr<Shape> cada_arc_scale_new(const shape::Shape *arc,
                                          const shape::Vec2d &scaleFactors,
                                          const shape::Vec2d &c)
{
    return std::unique_ptr<Shape>();
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
