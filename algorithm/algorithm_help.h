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

 #ifndef CADA_ALGORITHM_HELP_H
 #define CADA_ALGORITHM_HELP_H

#include <cada_shape.h>
#include <assert.h>

namespace cada {
namespace algorithm {

inline bool isPointShape(const shape::Shape *shape)
{
    assert(shape);
    return shape->getShapeType() == NS::Point;
}

inline bool isLineShape(const shape::Shape* shape)
{
    assert(shape);
    return shape->getShapeType() == NS::Line;
}

inline bool isArcShape(const shape::Shape* shape)
{
    assert(shape);
    return shape->getShapeType() == NS::Arc;
}

inline bool isCircleShape(const shape::Shape* shape)
{
    assert(shape);
    return shape->getShapeType() == NS::Circle;
}

inline bool isEllipseShape(const shape::Shape* shape)
{
    assert(shape);
    return shape->getShapeType() == NS::Ellipse;
}

inline bool isPolylineShape(const shape::Shape* shape)
{
    assert(shape);
    return shape->getShapeType() == NS::Polyline;
}

inline bool isSplineShape(const shape::Shape* shape)
{
    assert(shape);
    return shape->getShapeType() == NS::BSpline;
}

inline bool isFullEllipseShape(const shape::Shape *shape)
{
    assert(shape);
    return shape->getShapeType()==NS::Ellipse && dynamic_cast<const shape::Ellipse*>(shape)->isFullEllipse();
}

inline bool isRayShape(const shape::Shape* shape)
{
    assert(shape);
    return shape->getShapeType() == NS::Ray;
}

inline bool isXLineShape(const shape::Shape* shape)
{
    assert(shape);
    return shape->getShapeType() == NS::XLine;
}


inline shape::Vec2d centerOfACE(const shape::Shape *shape)
{
    assert(shape);
    auto a = dynamic_cast<const shape::Arc *>(shape);
    auto c = dynamic_cast<const shape::Circle *>(shape);
    auto e = dynamic_cast<const shape::Ellipse *>(shape);
    if (a) {
        return a->getCenter();
    }
    if (c) {
        return c->getCenter();
    }
    if (e) {
        return e->getCenter();
    }
    return shape::Vec2d::nullVector;
}

} // namespace algorithm
} // namespace cada

#endif