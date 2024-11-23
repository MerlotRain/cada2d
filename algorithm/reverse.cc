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

bool cada_reverse(shape::Shape *shape)
{
    assert(shape);
    switch (shape->getShapeType()) {
    case NS::Line: {
        auto l = dynamic_cast<shape::Line *>(shape);
        Vec2d v1 = l->getStartPoint();
        Vec2d v2 = l->getEndPoint();
        l->setStartPoint(v2);
        l->setEndPoint(v1);
        return true;
    }
    case NS::Arc: {
        auto a = dynamic_cast<shape::Arc *>(shape);
        double d1 = a->getStartAngle();
        double d2 = a->getEndAngle();
        bool r = a->isReversed();
        a->setStartAngle(d2);
        a->setEndAngle(d1);
        a->setReversed(!r);
        return true;
    }
    case NS::Ellipse: {
        auto e = dynamic_cast<shape::Ellipse *>(shape);
        double a = e->getStartParam();
        double b = e->getEndParam();
        bool r = e->isReversed();
        e->setStartParam(b);
        e->setEndParam(a);
        e->setReversed(!r);
        return true;
    }
    case NS::XLine:
    case NS::Ray: {
        auto xl = dynamic_cast<shape::XLine *>(shape);
        Vec2d sp = xl->getBasePoint() + xl->getDirectionVector();
        Vec2d bp = xl->getBasePoint();
        xl->setBasePoint(sp);
        xl->setDirectionVector(bp - sp);
        return true;
    }
    case NS::Polyline: {
        auto poly = dynamic_cast<shape::Polyline *>(shape);

        std::vector<Vec2d> vs = poly->getVertices();
        std::vector<double> bs = poly->getBulges();
        if (poly->isClosed()) {
            vs.push_back(vs.front());
        }

        auto nPolyline = ShapeFactory::instance()->createPolyline();
        for (size_t i = vs.size() - 1, k = 0; i >= 0; i--, k++) {
            nPolyline->appendVertex(vs[i]);
            if (i > 0) {
                nPolyline->setBulgeAt(k, -bs[i - 1]);
            }
            if (poly->isClosed()) {
                nPolyline->convertToClosed();
            }

            poly->setVertices(nPolyline->getVertices());
            poly->setBulges(nPolyline->getBulges());
            poly->setClosed(nPolyline->isClosed());
            return true;
        }
    }
    case NS::BSpline:
        break;
    default:
        break;
    }
    return false;
}

} // namespace algorithm
} // namespace cada
