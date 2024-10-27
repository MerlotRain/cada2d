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

#include "algorithm/length.h"
#include "algorithm/boundingbox.h"

using namespace cada::algorithm;

namespace cada {
namespace shape {

BBox Shape::getBoundingBox() const
{
    BoundingBox bb(const_cast<Shape *>(this));
    return bb.getBoundingBox();
}

double Shape::getLength() const
{
    Length l(const_cast<Shape *>(this));
    return l.getLength();
}

Vec2d Shape::getVectorTo(const Vec2d &point, bool limited,
                         double strictRange) const
{
    return Vec2d();
}

Vec2d Shape::getClosestPointOnShape(const Vec2d &p, bool limited,
                                    double strictRange) const
{
    Vec2d dv = getVectorTo(p, limited, strictRange);
    if (!dv.isValid()) {
        return Vec2d::invalid;
    }
    return p - dv;
}

bool Shape::equals(const Shape *other, double tolerance) const
{
    return false;
}

double Shape::getDistanceTo(const Vec2d &point, bool limited,
                            double strictRange) const
{
    return 0.0;
}

double Shape::getMaxDistanceTo(const std::vector<Vec2d> &points, bool limited,
                               double strictRange) const
{
    double ret = 0.0;
    for (size_t i = 0; i < points.size(); i++) {
        double d = getDistanceTo(points[i], limited, strictRange);
        ret = std::max(ret, d);
    }

    return ret;
}

bool Shape::isOnShape(const Vec2d &point, bool limited, double tolerance) const
{
    return false;
}

std::vector<Vec2d> Shape::filterOnShape(const std::vector<Vec2d> &pointList,
                                        bool limited, double tolerance) const
{
    std::vector<Vec2d> ret;
    for (int i = 0; i < pointList.size(); i++) {
        if (isOnShape(pointList[i], limited, tolerance)) {
            ret.push_back(pointList[i]);
        }
    }
    return ret;
}

Vec2d Shape::getVectorFromEndpointTo(const Vec2d &point) const
{
    std::vector<Vec2d> endPoints = getEndPoints();
    Vec2d closest = point.getClosest(endPoints);
    return point - closest;
}

std::vector<Vec2d> Shape::getPointsWithDistanceToEnd(double distance,
                                                     int from) const
{
    return std::vector<Vec2d>();
}

Vec2d Shape::getPointOnShape() const
{
    return Vec2d();
}

Vec2d Shape::getPointWithDistanceToStart(double distance) const
{
    return Vec2d();
}

Vec2d Shape::getPointWithDistanceToEnd(double distance) const
{
    return Vec2d();
}

double Shape::getAngleAt(double distance, NS::From from) const
{
    return 0.0;
}

double Shape::getAngleAtPoint(const Vec2d &pos) const
{
    return 0.0;
}

Vec2d Shape::getPointAtPercent(double p) const
{
    return Vec2d();
}

double Shape::getAngleAtPercent(double p) const
{
    double length = getLength();
    double distance = p * length;
    return getAngleAt(distance);
}

bool Shape::intersectsWith(const Shape *other, bool limited) const
{
    return false;
}

std::vector<Vec2d> Shape::getIntersectionPoints(const Shape *other,
                                                bool limited, bool same,
                                                bool force) const
{
    return std::vector<Vec2d>();
}

std::vector<Vec2d> Shape::getSelfIntersectionPoints(double tolerance) const
{
    return std::vector<Vec2d>();
}

bool Shape::isDirected() const
{
    return false;
}

double Shape::getDirection1() const
{
    return 0.0;
}

double Shape::getDirection2() const
{
    return 0.0;
}

NS::Side Shape::getSideOfPoint(const Vec2d &point) const
{
    return NS::NoSide;
}

bool Shape::reverse()
{
    return false;
}

bool Shape::trimStartPoint(const Vec2d &trimPoint, const Vec2d &clickPoint,
                           bool extend)
{
    return false;
}

bool Shape::trimStartPoint(double trimDist)
{
    return false;
}

bool Shape::trimEndPoint(const Vec2d &trimPoint, const Vec2d &clickPoint,
                         bool extend)
{
    return false;
}

bool Shape::trimEndPoint(double trimDist)
{
    return false;
}

NS::Ending Shape::getTrimEnd(const Vec2d &trimPoint, const Vec2d &clickPoint)
{
    return NS::EndingStart;
}

double Shape::getDistanceFromStart(const Vec2d &p) const
{
    return 0.0;
}

std::vector<double> Shape::getDistancesFromStart(const Vec2d &p) const
{
    return std::vector<double>();
}

bool Shape::move(const Vec2d &offset)
{
    return false;
}

bool Shape::rotate(double rotation, const Vec2d &center)
{
    return false;
}

bool Shape::scale(double scaleFactor, const Vec2d &center)
{
    return false;
}

bool Shape::scale(const Vec2d &scaleFactors, const Vec2d &center)
{
    return false;
}

bool Shape::mirror(const Vec2d &v1, const Vec2d &v2)
{
    return false;
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
    return false;
}

std::vector<std::unique_ptr<Shape>>
Shape::getOffsetShapes(double distance, int number, NS::Side side,
                       const Vec2d &position)
{
    return std::vector<std::unique_ptr<Shape>>();
}

std::vector<std::unique_ptr<Shape>>
Shape::splitAt(const std::vector<Vec2d> &points) const
{
    return std::vector<std::unique_ptr<Shape>>();
}

} // namespace shape
} // namespace cada