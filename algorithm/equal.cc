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

bool cada_equalsPoint(const shape::Point *point1, const shape::Point *point2,
                      double tolerance)
{
    assert(point1);
    assert(point2);

    return point1->getPosition().equalsFuzzy(point2->getPosition(), tolerance);
}

bool cada_equalsLine(const shape::Line *line1, const shape::Line *line2,
                     double tolerance)
{
    assert(line1);
    assert(line2);
    return line1->getStartPoint().equalsFuzzy(line2->getStartPoint(),
                                              tolerance) &&
           line1->getEndPoint().equalsFuzzy(line2->getEndPoint(), tolerance);
}

bool cada_equalsArc(const shape::Arc *arc1, const shape::Arc *arc2,
                    double tolerance)
{
    assert(arc1);
    assert(arc2);
    return arc1->getCenter().equalsFuzzy(arc2->getCenter(), tolerance) &&
           Math::fuzzyCompare(arc1->getRadius(), arc2->getRadius(),
                              tolerance) &&
           Math::fuzzyCompare(arc1->getStartAngle(), arc2->getStartAngle(),
                              tolerance) &&
           Math::fuzzyCompare(arc1->getEndAngle(), arc2->getEndAngle(),
                              tolerance) &&
           (arc1->isReversed() == arc2->isReversed());
}

bool cada_equalsCircle(const shape::Circle *circle1,
                       const shape::Circle *circle2, double tolerance)
{
    assert(circle1);
    assert(circle2);
    return circle1->getCenter().equalsFuzzy(circle2->getCenter(), tolerance) &&
           Math::fuzzyCompare(circle1->getRadius(), circle2->getRadius(),
                              tolerance);
}

bool cada_equalsEllipse(const shape::Ellipse *ellipse1,
                        const shape::Ellipse *ellipse2, double tolerance)
{
    assert(ellipse1);
    assert(ellipse2);
    return ellipse1->getCenter().equalsFuzzy(ellipse2->getCenter(),
                                             tolerance) &&
           ellipse1->getMajorPoint().equalsFuzzy(ellipse2->getMajorPoint(),
                                                 tolerance) &&
           Math::fuzzyCompare(ellipse1->getStartParam(),
                              ellipse2->getStartParam(), tolerance) &&
           Math::fuzzyCompare(ellipse1->getEndParam(), ellipse2->getEndParam(),
                              tolerance) &&
           (ellipse1->isReversed() == ellipse2->isReversed());
}

bool cada_equalsXLine(const shape::XLine *xline1, const shape::XLine *xline2,
                      double tolerance)
{
    assert(xline1);
    assert(xline2);
    return xline1->getBasePoint().equalsFuzzy(xline2->getBasePoint(),
                                              tolerance) &&
           xline1->getDirectionVector().equalsFuzzy(
               xline2->getDirectionVector(), tolerance);
}

bool cada_equalsRay(const shape::Ray *ray1, const shape::Ray *ray2,
                    double tolerance)
{
    assert(ray1);
    assert(ray2);
    return ray1->getBasePoint().equalsFuzzy(ray2->getBasePoint(), tolerance) &&
           ray1->getDirectionVector().equalsFuzzy(ray2->getDirectionVector(),
                                                  tolerance);
}

bool cada_equalsPolyline(const shape::Polyline *polyline1,
                         const shape::Polyline *polyline2, double tolerance)
{
    assert(polyline1);
    assert(polyline2);
    auto poly_vertices1 = polyline1->getVertices();
    auto poly_vertices2 = polyline2->getVertices();

    if (poly_vertices1.size() != poly_vertices2.size()) {
        return false;
    }

    for (size_t i = 0; i < poly_vertices1.size(); ++i) {
        if (!poly_vertices1[i].equalsFuzzy(poly_vertices2[i], tolerance)) {
            return false;
        }
    }

    auto poly_bulges1 = polyline1->getBulges();
    auto poly_bulges2 = polyline2->getBulges();

    if (poly_bulges1.size() != poly_bulges2.size()) {
        return false;
    }

    for (size_t i = 0; i < poly_bulges1.size(); ++i) {
        if (!Math::fuzzyCompare(poly_bulges1[i], poly_bulges2[i], tolerance)) {
            return false;
        }
    }

    return true;
}

bool cada_equalsBSpline(const shape::BSpline *bspline1,
                        const shape::BSpline *bspline2, double tolerance)
{
    return false;
}

bool cada_equals(const shape::Shape *shape1, const shape::Shape *shape2,
                 double tolerance)
{
    assert(shape1);
    assert(shape2);

    if (shape1->getShapeType() != shape2->getShapeType()) {
        return false;
    }

    switch (shape1->getShapeType()) {
    case NS::Point:
        return cada_equalsPoint(dynamic_cast<const shape::Point *>(shape1),
                                dynamic_cast<const shape::Point *>(shape2),
                                tolerance);
    case NS::Line:
        return cada_equalsLine(dynamic_cast<const shape::Line *>(shape1),
                               dynamic_cast<const shape::Line *>(shape2),
                               tolerance);
    case NS::Arc:
        return cada_equalsArc(dynamic_cast<const shape::Arc *>(shape1),
                              dynamic_cast<const shape::Arc *>(shape2),
                              tolerance);
    case NS::Circle:
        return cada_equalsCircle(dynamic_cast<const shape::Circle *>(shape1),
                                 dynamic_cast<const shape::Circle *>(shape2),
                                 tolerance);
    case NS::Ellipse:
        return cada_equalsEllipse(dynamic_cast<const shape::Ellipse *>(shape1),
                                  dynamic_cast<const shape::Ellipse *>(shape2),
                                  tolerance);
    case NS::XLine:
        return cada_equalsXLine(dynamic_cast<const shape::XLine *>(shape1),
                                dynamic_cast<const shape::XLine *>(shape2),
                                tolerance);
    case NS::Ray:
        return cada_equalsRay(dynamic_cast<const shape::Ray *>(shape1),
                              dynamic_cast<const shape::Ray *>(shape2),
                              tolerance);
    case NS::Polyline:
        return cada_equalsPolyline(
            dynamic_cast<const shape::Polyline *>(shape1),
            dynamic_cast<const shape::Polyline *>(shape2), tolerance);
    case NS::BSpline:
        return cada_equalsBSpline(dynamic_cast<const shape::BSpline *>(shape1),
                                  dynamic_cast<const shape::BSpline *>(shape2),
                                  tolerance);
    default:
        break;
    }
    return false;
};

} // namespace algorithm
} // namespace cada
