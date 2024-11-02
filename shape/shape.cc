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

#include "cada_shape.h"
#include <assert.h>
#include <cmath>

#include "shape_algorithm_extern.inl"

using namespace cada::algorithm;

namespace cada {
namespace shape {

BBox Shape::getBoundingBox() const
{
    return cada_getBoundingBox(this);
}

double Shape::getLength() const
{
    return cada_getLength(this);
}

Vec2d Shape::getVectorTo(const Vec2d &point, bool limited,
                         double strictRange) const
{
    return cada_getVectorTo(this, point, limited, strictRange);
}

Vec2d Shape::getClosestPointOnShape(const Vec2d &p, bool limited,
                                    double strictRange) const
{
    return cada_getClosestPointOnShape(this, p, limited, strictRange);
}

bool Shape::equals(const Shape *other, double tolerance) const
{
    return cada_equals(this, other, tolerance);
}

double Shape::getDistanceTo(const Vec2d &point, bool limited,
                            double strictRange) const
{
    return cada_getDistanceTo(this, point, limited, strictRange);
}

double Shape::getMaxDistanceTo(const std::vector<Vec2d> &points, bool limited,
                               double strictRange) const
{
    return cada_getMaxDistanceTo(this, points, limited, strictRange);
}

bool Shape::isOnShape(const Vec2d &point, bool limited, double tolerance) const
{
    return cada_isOnShape(this, point, limited, tolerance);
}

std::vector<Vec2d> Shape::filterOnShape(const std::vector<Vec2d> &pointList,
                                        bool limited, double tolerance) const
{
    return cada_filterOnShape(this, pointList, limited, tolerance);
}

Vec2d Shape::getVectorFromEndpointTo(const Vec2d &point) const
{
    return cada_getVectorFromEndpointTo(this, point);
}

std::vector<Vec2d> Shape::getPointsWithDistanceToEnd(double distance,
                                                     int from) const
{
    return cada_getPointsWithDistanceToEnd(this, distance, from);
}

Vec2d Shape::getPointOnShape() const
{
    return cada_getPointOnShape(this);
}

Vec2d Shape::getPointWithDistanceToStart(double distance) const
{
    return cada_getPointWithDistanceToStart(this, distance);
}

Vec2d Shape::getPointWithDistanceToEnd(double distance) const
{
    return cada_getPointWithDistanceToEnd(this, distance);
}

double Shape::getAngleAt(double distance, NS::From from) const
{
    return cada_getAngleAt(this, distance, from);
}

double Shape::getAngleAtPoint(const Vec2d &pos) const
{
    return cada_getAngleAtPoint(this, pos);
}

Vec2d Shape::getPointAtPercent(double p) const
{
    return cada_getPointAtPercent(this, p);
}

double Shape::getAngleAtPercent(double p) const
{
    return cada_getAngleAtPercent(this, p);
}

bool Shape::intersectsWith(const Shape *other, bool limited) const
{
    return cada_intersectsWith(this, other, limited);
}

std::vector<Vec2d> Shape::getIntersectionPoints(const Shape *other,
                                                bool limited, bool same,
                                                bool force) const
{
    return cada_getIntersectionPoints(this, other, limited, same, force);
}

std::vector<Vec2d> Shape::getSelfIntersectionPoints(double tolerance) const
{
    return cada_getSelfIntersectionPoints(this, tolerance);
}

bool Shape::isDirected() const
{
    return cada_isDirected(this);
}

double Shape::getDirection1() const
{
    return cada_getDirection1(this);
}

double Shape::getDirection2() const
{
    return cada_getDirection2(this);
}

NS::Side Shape::getSideOfPoint(const Vec2d &point) const
{
    return cada_getSideOfPoint(this, point);
}

bool Shape::reverse()
{
    return cada_reverse(this);
}

bool Shape::trimStartPoint(const Vec2d &trimPoint, const Vec2d &clickPoint,
                           bool extend)
{
    return cada_trimStartPoint(this, trimPoint, clickPoint, extend);
}

bool Shape::trimStartPoint(double trimDist)
{
    return cada_trimEndPoint(this, trimDist);
}

bool Shape::trimEndPoint(const Vec2d &trimPoint, const Vec2d &clickPoint,
                         bool extend)
{
    return cada_trimEndPoint(this, trimPoint, clickPoint, extend);
}

bool Shape::trimEndPoint(double trimDist)
{
    return cada_trimEndPoint(this, trimDist);
}

NS::Ending Shape::getTrimEnd(const Vec2d &trimPoint, const Vec2d &clickPoint)
{
    return cada_getTrimEnd(this, trimPoint, clickPoint);
}

double Shape::getDistanceFromStart(const Vec2d &p) const
{
    return cada_getDistanceFromStart(this, p);
}

std::vector<double> Shape::getDistancesFromStart(const Vec2d &p) const
{
    return cada_getDistancesFromStart(this, p);
}

bool Shape::move(const Vec2d &offset)
{
    return cada_move(this, offset);
}

bool Shape::rotate(double rotation, const Vec2d &center)
{
    return cada_rotate(this, rotation, center);
}

bool Shape::scale(double scaleFactor, const Vec2d &center)
{
    return scale(Vec2d(scaleFactor, scaleFactor), center);
}

bool Shape::scale(const Vec2d &scaleFactors, const Vec2d &center)
{
    return cada_scale(this, scaleFactors, center);
}

bool Shape::mirror(const Vec2d &v1, const Vec2d &v2)
{
    return cada_mirror(this, v1, v2);
}

bool Shape::flipHorizontal()
{
    return mirror(Vec2d(0, 0), Vec2d(0, 1));
}

bool Shape::flipVertical()
{
    return mirror(Vec2d(0, 0), Vec2d(1, 0));
}

bool Shape::stretch(const std::vector<Vec2d> &vertex, const Vec2d &offset)
{
    return cada_stretch(this, vertex, offset);
}

std::vector<std::unique_ptr<Shape>>
Shape::getOffsetShapes(double distance, int number, NS::Side side,
                       const Vec2d &position)
{
    return cada_getOffsetShapes(this, distance, number, side, position);
}

std::vector<std::unique_ptr<Shape>>
Shape::splitAt(const std::vector<Vec2d> &points) const
{
    return cada_splitAt(this, points);
}

} // namespace shape
} // namespace cada