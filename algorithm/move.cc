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

bool cada_arc_move(shape::Arc *a, const shape::Vec2d &offset)
{
    assert(a);
    a->setCenter(a->getCenter().move(offset));
    return true;
}

bool cada_move(shape::Shape *shape, const shape::Vec2d &offset)
{
    assert(shape);
    if (!offset.isValid() || offset.getMagnitude() < NS::PointTolerance) {
        return false;
    }
    switch (shape->getShapeType()) {
    case NS::Point: {
        auto point = dynamic_cast<Point *>(shape);
        point->setPosition(point->getPosition().move(offset));
        return true;
    }
    case NS::Line: {
        auto l = dynamic_cast<Line *>(shape);
        l->setStartPoint(l->getStartPoint().move(offset));
        l->setEndPoint(l->getEndPoint().move(offset));
        return true;
    }
    case NS::Arc: {
        auto a = dynamic_cast<Arc *>(shape);
        return cada_arc_move(a, offset);
    }
    case NS::Circle: {
        auto c = dynamic_cast<Circle *>(shape);
        c->setCenter(c->getCenter().move(offset));
        return true;
    }
    case NS::Ellipse: {
        auto e = dynamic_cast<Ellipse *>(shape);
        e->setCenter(e->getCenter().move(offset));
        e->setMajorPoint(e->getMajorPoint().move(offset));
        return true;
    }
    case NS::XLine:
    case NS::Ray: {
        auto l = dynamic_cast<XLine *>(shape);
        l->setBasePoint(l->getBasePoint().move(offset));
        return true;
    }
    case NS::Polyline: {
        auto p = dynamic_cast<Polyline *>(shape);
        auto &vertices = p->getVertices();
        for (auto &v : vertices) {
            v.move(offset);
        }
        return true;
    }
    case NS::BSpline:
    default:
        break;
    }
    return false;
}

} // namespace algorithm
} // namespace cada
