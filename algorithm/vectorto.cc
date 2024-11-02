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

#include <assert.h>
#include <cmath>
#include <limits>
#include <numeric>
#include <cada_shape.h>

using namespace cada::shape;

namespace cada {
namespace algorithm {

// VectorTo::VectorTo(shape::Shape *shape, bool limited, double strictRange)
//     : mShape(shape), mLimited(limited), mStrictRange(strictRange)
// {
// }

// Vec2d VectorTo::getVectorTo(const shape::Vec2d &point) const
// {
//     assert(mShape);
//     switch (mShape->getShapeType()) {
//     case NS::Point: {
//         Point *pt = dynamic_cast<Point *>(mShape);
//         return point - pt->getPosition();
//     }
//     case NS::Line: {
//         return getLineVectorTo(point);
//     }
//     case NS::Arc: {
//         return getArcVectorTo(point);
//     }
//     case NS::Circle: {
//         return getCircleVectorTo(point);
//     }
//     case NS::Ellipse:
//         break;
//     case NS::XLine: {
//         return getXLineVectorTo(point);
//     }
//     case NS::Ray: {
//         return getRayVectorTo(point);
//     }
//     case NS::Polyline:
//     case NS::BSpline:
//         break;
//     }
//     return Vec2d::invalid;
// }

// Vec2d VectorTo::getLineVectorTo(const shape::Vec2d &point) const
// {
//     Line *l = dynamic_cast<Line *>(mShape);
//     assert(l);

//     Vec2d ae = l->getEndPoint() - l->getStartPoint();
//     Vec2d ap = point - l->getStartPoint();

//     if (ae.getMagnitude() < 1.0e-6) {
//         return Vec2d::invalid;
//     }
//     if (ap.getMagnitude() < 1.0e-6) {
//         return Vec2d(0, 0);
//     }
//     double b = Vec2d::getDotProduct(ap, ae) / Vec2d::getDotProduct(ae, ae);

//     if (mLimited && (b < 0 || b > 1.0)) {
//         // orthogonal to line does not cross line, use distance to end point:
//         Vec2d ret = l->getVectorFromEndpointTo(point);
//         if (ret.getMagnitude() < mStrictRange) {
//             return ret;
//         }
//         else {
//             // not within given range:
//             return Vec2d::invalid;
//         }
//     }

//     Vec2d closestPoint = l->getStartPoint() + ae * b;

//     return point - closestPoint;
// }

// shape::Vec2d VectorTo::getArcVectorTo(const shape::Vec2d &point) const
// {
//     Arc *a = dynamic_cast<Arc *>(mShape);
//     assert(a);

//     double angle = a->getCenter().getAngleTo(point);
//     if (mLimited && !Math::isAngleBetween(angle, a->getStartAngle(),
//                                           a->getEndAngle(), a->isReversed()))
//                                           {
//         return Vec2d::invalid;
//     }

//     Vec2d v = point - a->getCenter();
//     return Vec2d::createPolar(v.getMagnitude() - a->getRadius(),
//     v.getAngle());
// }

// shape::Vec2d VectorTo::getCircleVectorTo(const shape::Vec2d &point) const
// {
//     Circle *c = dynamic_cast<Circle *>(mShape);
//     assert(c);

//     Vec2d v = point - c->getCenter();

//     // point is at the center of the circle, infinite solutions:
//     if (v.getMagnitude() < NS::PointTolerance) {
//         return Vec2d::invalid;
//     }

//     return Vec2d::createPolar(v.getMagnitude() - c->getRadius(),
//     v.getAngle());
// }

// shape::Vec2d VectorTo::getXLineVectorTo(const shape::Vec2d &point) const
// {
//     XLine *xl = dynamic_cast<XLine *>(mShape);
//     assert(xl);

//     auto l = ShapeFactory::instance()->createLine(xl->getStartPoint(),
//                                                   xl->getEndPoint());
//     return l->getVectorTo(point, mLimited, mStrictRange);
// }

// shape::Vec2d VectorTo::getRayVectorTo(const shape::Vec2d &point) const
// {
//     Ray *r = dynamic_cast<Ray *>(mShape);
//     assert(r);

//     if (!mLimited) {
//         auto l = ShapeFactory::instance()->createLine(r->getStartPoint(),
//                                                       r->getEndPoint());
//         return l->getVectorTo(point, mLimited, mStrictRange);
//     }
//     else {
//         Vec2d p = r->getClosestPointOnShape(point, false);
//         if (fabs(Math::getAngleDifference180(
//                 r->getDirection1(), r->getStartPoint().getAngleTo(p))) < 0.1)
//                 {
//             return point - p;
//         }
//         return Vec2d::invalid;
//     }
// }

} // namespace algorithm
} // namespace cada
