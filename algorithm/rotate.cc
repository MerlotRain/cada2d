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

/* -------------------------------- functions ------------------------------- */
bool cada_arc_rotate(Arc *arc, double rotation, const shape::Vec2d &center);

bool cada_rotate(shape::Shape *shape, double rotation,
                 const shape::Vec2d &center)
{
    assert(shape != nullptr);

    if (fabs(rotation) < NS::AngleTolerance) {
        return false;
    }

    switch (shape->getShapeType()) {
    case NS::Point: {
        auto point = dynamic_cast<Point *>(shape);
        point->setPosition(point->getPosition().rotate(rotation, center));
        return true;
    }
    case NS::Line: {
        auto line = dynamic_cast<Line *>(shape);
        line->setStartPoint(line->getStartPoint().rotate(rotation, center));
        line->setEndPoint(line->getEndPoint().rotate(rotation, center));
        return true;
    }
    case NS::Arc: {
        return cada_arc_rotate(dynamic_cast<Arc *>(shape), rotation, center);
    }
    case NS::Circle: {
        auto circle = dynamic_cast<Circle *>(shape);
        circle->setCenter(circle->getCenter().rotate(rotation, center));
        return true;
    }
    case NS::Ellipse: {
        auto ellipse = dynamic_cast<Ellipse *>(shape);
        ellipse->setCenter(ellipse->getCenter().rotate(rotation, center));
        ellipse->setMajorPoint(
            ellipse->getMajorPoint().rotate(rotation, center));
        return true;
    }
    case NS::XLine:
    case NS::Ray: {
        auto xl = dynamic_cast<XLine *>(shape);
        xl->setBasePoint(xl->getBasePoint().rotate(rotation, center));
        xl->setDirectionVector(xl->getDirectionVector().rotate(rotation));
        return true;
    }
    case NS::Polyline: {
        auto polyline = dynamic_cast<Polyline *>(shape);
        auto &vertices = polyline->getVertices();
        for (auto &point : vertices) {
            point.rotate(rotation, center);
        }
        return true;
    }
    case NS::BSpline:
    default:
        break;
    }
    return false;
}

bool cada_arc_rotate(Arc *arc, double rotation, const shape::Vec2d &c)
{
    Vec2d center = arc->getCenter();
    double startAngle = arc->getStartAngle();
    double endAngle = arc->getEndAngle();

    center.rotate(rotation, c);

    // important for circle shaped in hatch boundaries:
    if (!arc->isFullCircle()) {
        startAngle = Math::getNormalizedAngle(startAngle + rotation);
        endAngle = Math::getNormalizedAngle(endAngle + rotation);
    }

    arc->setCenter(center);
    arc->setStartAngle(startAngle);
    arc->setEndAngle(endAngle);

    return true;
}

} // namespace algorithm
} // namespace cada