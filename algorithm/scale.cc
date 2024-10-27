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

#include "scale.h"
#include <cada_shape.h>
#include <assert.h>

using namespace cada::shape;

namespace cada {
namespace algorithm {

Scale::Scale(Shape *shape) : mShape(shape)
{
}

bool Scale::operator()(const shape::Vec2d &scaleFactor,
                       const shape::Vec2d &c) const
{
    assert(mShape);

    switch (mShape->getShapeType()) {
    case NS::Point: {
        auto pt = dynamic_cast<Point *>(mShape);
        pt->setPosition(pt->getPosition().scale(scaleFactor, c));
        return true;
    }
    case NS::Line: {
        auto l = dynamic_cast<Line *>(mShape);
        l->setStartPoint(l->getStartPoint().scale(scaleFactor, c));
        l->setEndPoint(l->getEndPoint().scale(scaleFactor, c));
        return true;
    }
    case NS::Arc: {
        auto a = dynamic_cast<Arc *>(mShape);
        double radius = a->getRadius();
        Vec2d center = a->getCenter();
        if (scaleFactor.x < 0.0) {
            a->mirror(center, center + Vec2d(0.0, 1.0));
        }
        if (scaleFactor.y < 0.0) {
            a->mirror(center, center + Vec2d(1.0, 0.0));
        }

        a->setCenter(center.scale(scaleFactor, c));
        radius *= scaleFactor.x;
        if (radius < 0.0) {
            a->setRadius(radius *= -1.0);
        }
        else {
            a->setRadius(radius);
        }

        return true;
    }
    case NS::Circle:
    case NS::Ellipse:
    case NS::XLine:
    case NS::Ray:
    case NS::Polyline:
    case NS::BSpline:
        break;
    default:
        break;
    };
}

} // namespace algorithm
} // namespace cada
