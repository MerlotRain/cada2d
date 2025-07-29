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
shape::Vec2d cada_getPointOnShape(const shape::Shape *shape);
shape::Vec2d cada_ellipse_getPointOnShape(const shape::Ellipse *e);

/* ---------------------------------- impls --------------------------------- */

shape::Vec2d cada_getPointOnShape(const shape::Shape *shape)
{
    assert(shape);
    if (shape->getShapeType() == NS::Ellipse) {
        return cada_ellipse_getPointOnShape(
            dynamic_cast<const shape::Ellipse *>(shape));
    }
    else {
        std::vector<Vec2d> midPoints = shape->getMiddlePoints();
        if (midPoints.size() > 0) {
            return midPoints[0];
        }

        std::vector<Vec2d> endPoints = shape->getEndPoints();
        if (endPoints.size() > 0) {
            return endPoints[0];
        }

        return shape->getClosestPointOnShape(Vec2d(0.0, 0.0));
    }
}

shape::Vec2d cada_ellipse_getPointOnShape(const shape::Ellipse *e)
{
    double sp = e->getStartParam();
    double ep = e->getEndParam();
    if (e->isReversed()) {
        if (sp < ep) {
            sp += M_PI * 2;
        }
    }
    else {
        if (ep < sp) {
            ep += M_PI * 2;
        }
    }
    double mp = (sp + ep) / 2.0;
    return e->getPointAt(mp);
}

} // namespace algorithm
} // namespace cada
