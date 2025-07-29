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

static double cada_line_getDirection(const shape::Line *l, bool first)
{
    assert(l);
    if (first) {
        return l->getStartPoint().getAngleTo(l->getEndPoint());
    }
    else {
        return l->getEndPoint().getAngleTo(l->getStartPoint());
    }
}

static double cada_arc_getDirection(const shape::Arc *a, bool first)
{
    assert(a);
    if (first) {
        if (a->isReversed()) {
            return Math::getNormalizedAngle(a->getStartAngle() - M_PI_2);
        }
        else {
            return Math::getNormalizedAngle(a->getStartAngle() + M_PI_2);
        }
    }
    else {
        if (a->isReversed()) {
            return Math::getNormalizedAngle(a->getEndAngle() + M_PI_2);
        }
        else {
            return Math::getNormalizedAngle(a->getEndAngle() - M_PI_2);
        }
    }
}

double cada_getDirection1(const shape::Shape *shape)
{
    assert(shape);
    switch (shape->getShapeType()) {
    case NS::Line: {
        auto l = dynamic_cast<const shape::Line *>(shape);
        return cada_line_getDirection(l, true);
    }
    case NS::Arc: {
        auto a = dynamic_cast<const shape::Arc *>(shape);
        return cada_arc_getDirection(a, true);
    }
    case NS::Ellipse: {
        auto e = dynamic_cast<const shape::Ellipse *>(shape);
        return e->getAngleAtPoint(e->getStartPoint());
    }
    case NS::XLine:
    case NS::Ray: {
        auto xl = dynamic_cast<const shape::XLine *>(shape);
        return xl->getDirectionVector().getAngle();
    }
    case NS::Polyline: {
        auto poly = dynamic_cast<const shape::Polyline *>(shape);
        if (poly->countVertices() == 0) {
            return std::numeric_limits<double>::quiet_NaN();
        }

        auto seg = poly->getSegmentAt(0);
        if (!seg) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        else {
            if (seg->getShapeType() == NS::Line) {
                return cada_line_getDirection(dynamic_cast<Line *>(seg.get()),
                                              true);
            }
            else if (seg->getShapeType() == NS::Arc) {
                return cada_arc_getDirection(dynamic_cast<Arc *>(seg.get()),
                                             true);
            }
            else {
                return std::numeric_limits<double>::quiet_NaN();
            }
        }
    }
    case NS::Spline:
        // TODO
    default:
        break;
    };
    return std::numeric_limits<double>::quiet_NaN();
}

double cada_getDirection2(const shape::Shape *shape)
{
    assert(shape);
    switch (shape->getShapeType()) {
    case NS::Line: {
        auto l = dynamic_cast<const shape::Line *>(shape);
        return cada_line_getDirection(l, false);
    }
    case NS::Arc: {
        auto a = dynamic_cast<const shape::Arc *>(shape);
        return cada_arc_getDirection(a, false);
    }
    case NS::Ellipse: {
        auto e = dynamic_cast<const shape::Ellipse *>(shape);
        return Math::getNormalizedAngle(e->getAngleAtPoint(e->getEndPoint()) +
                                        M_PI);
    }
    case NS::XLine:
    case NS::Ray: {
        auto xl = dynamic_cast<const shape::XLine *>(shape);
        Vec2d second_point = xl->getBasePoint() + xl->getDirectionVector();
        return second_point.getAngleTo(xl->getBasePoint());
    }
    case NS::Polyline: {
        auto poly = dynamic_cast<const shape::Polyline *>(shape);
        if (poly->countVertices() == 0) {
            return std::numeric_limits<double>::quiet_NaN();
        }

        int i = poly->countVertices() - 2;
        if (poly->isClosed()) {
            i++;
        }

        auto seg = poly->getSegmentAt(i);
        if (!seg) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        else {
            if (seg->getShapeType() == NS::Line) {
                return cada_line_getDirection(dynamic_cast<Line *>(seg.get()),
                                              false);
            }
            else if (seg->getShapeType() == NS::Arc) {
                return cada_arc_getDirection(dynamic_cast<Arc *>(seg.get()),
                                             false);
            }
            else {
                return std::numeric_limits<double>::quiet_NaN();
            }
        }
    }
    case NS::Spline:
        // TODO
    default:
        break;
    };
    return std::numeric_limits<double>::quiet_NaN();
}

} // namespace algorithm
} // namespace cada
