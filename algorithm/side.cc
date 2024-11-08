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

NS::Side cada_line_sideOfPoint(const shape::Line *l, const shape::Vec2d &point)
{
    assert(l);
    double entityAngle = l->getAngle();
    double angleToCoord = l->getStartPoint().getAngleTo(point);
    double angleDiff = Math::getAngleDifference(entityAngle, angleToCoord);

    if (angleDiff < M_PI) {
        return NS::LeftHand;
    }
    else {
        return NS::RightHand;
    }
}

NS::Side cada_arc_sideOfPoint(const shape::Arc *a, const shape::Vec2d &point)
{
    assert(a);
    if (a->isReversed()) {
        if (a->getCenter().getDistanceTo(point) < a->getRadius()) {
            return NS::RightHand;
        }
        else {
            return NS::LeftHand;
        }
    }
    else {
        if (a->getCenter().getDistanceTo(point) < a->getRadius()) {
            return NS::LeftHand;
        }
        else {
            return NS::RightHand;
        }
    }
}

NS::Side cada_ellipse_sideOfPoint(const shape::Ellipse *a,
                                  const shape::Vec2d &point)
{
    assert(a);
    if (a->contains(point)) {
        if (!a->isReversed()) {
            return NS::RightHand;
        }
        else {
            return NS::LeftHand;
        }
    }
    else {
        if (!a->isReversed()) {
            return NS::LeftHand;
        }
        else {
            return NS::RightHand;
        }
    }
}

NS::Side cada_polyline_sideOfPoint(const shape::Polyline *poly,
                                   const shape::Vec2d &point)
{
    assert(poly);
    int i = poly->getClosestSegment(point);
    if (i < 0 || i >= poly->countSegments()) {
        return NS::NoSide;
    }

    auto segment = poly->getSegmentAt(i);
    if (!segment) {
        return NS::NoSide;
    }

    if (segment->getShapeType() == NS::Line) {
        return cada_line_sideOfPoint(
            dynamic_cast<shape::Line *>(segment.release()), point);
    }
    else if (segment->getShapeType() == NS::Arc) {
        return cada_arc_sideOfPoint(
            dynamic_cast<shape::Arc *>(segment.release()), point);
    }
    else {
        return NS::NoSide;
    }
}

NS::Side cada_getSideOfPoint(const shape::Shape *shape,
                             const shape::Vec2d &point)
{
    assert(shape);
    switch (shape->getShapeType()) {
    case NS::Line: {
        auto l = dynamic_cast<const shape::Line *>(shape);
        return cada_line_sideOfPoint(l, point);
    }
    case NS::Arc: {
        auto a = dynamic_cast<const shape::Arc *>(shape);
        return cada_arc_sideOfPoint(a, point);
    }
    case NS::Ellipse: {
        auto e = dynamic_cast<const shape::Ellipse *>(shape);
        return cada_ellipse_sideOfPoint(e, point);
    }
    case NS::XLine:
    case NS::Ray: {
        auto xl = dynamic_cast<const shape::XLine *>(shape);
        return cada_line_sideOfPoint(xl->getLineShape().release(), point);
    }
    case NS::Polyline: {
        auto poly = dynamic_cast<const shape::Polyline *>(shape);
        return cada_polyline_sideOfPoint(poly, point);
    }
    case NS::BSpline:
        // TODO
    default:
        break;
    };
    return NS::NoSide;
}

} // namespace algorithm
} // namespace cada
