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

namespace cada {


Ray::Ray() : XLine()
{
}

Ray::Ray(const Line &line) : XLine(line)
{
}

Ray::Ray(const Vec2d &basePoint, const Vec2d &directionVector)
    : XLine(basePoint, directionVector)
{
}

Ray::Ray(const Vec2d &basePoint, double angle, double distance)
    : XLine(basePoint, angle, distance)
{
}

Ray::~Ray()
{
}

Vec2d Ray::getVectorTo(const Vec2d &point, bool limited,
                       double strictRange) const
{
    if (!limited) {
        return XLine::getVectorTo(point, false, strictRange);
    }
    else {
        Vec2d p = XLine::getClosestPointOnShape(point, false);
        if (fabs(Math::getAngleDifference180(
                getDirection1(), getStartPoint().getAngleTo(p))) < 0.1) {
            return point - p;
        }
        return Vec2d::invalid;
    }
}

bool Ray::reverse()
{
    return false;
}

Line Ray::getClippedLine(const BBox &box) const
{
    Line ret = XLine::getClippedLine(box);

    if (box.contains(getBasePoint())) {
        ret.setStartPoint(getBasePoint());
    }

    if (!Math::isSameDirection(getDirection1(),
                               getBasePoint().getAngleTo(ret.getEndPoint()),
                               0.1)) {
        ret = getLineShape();
    }

    return ret;
}

bool Ray::trimEndPoint(const Vec2d &trimPoint, const Vec2d &clickPoint,
                       bool extend)
{
    Vec2d tp = getClosestPointOnShape(trimPoint, false);
    if (!tp.isValid()) {
        return false;
    }
    directionVector = tp - basePoint;
    return true;
}

std::vector<Vec2d> Ray::getPointsWithDistanceToEnd(double distance,
                                                   int from) const
{
    std::vector<Vec2d> ret;
    double a1 = getAngle();

    Vec2d dv;
    dv.setPolar(distance, a1);

    if (from & NS::FromStart) {
        ret.push_back(basePoint + dv);
    }

    return ret;
}

bool Ray::stretch(const Polyline &area, const Vec2d &offset)
{
    bool ret = false;

    if (area.contains(basePoint, true)) {
        basePoint += offset;
        ret = true;
    }

    return ret;
}

std::vector<std::shared_ptr<Shape>>
Ray::splitAt(const std::vector<Vec2d> &points) const
{
    if (points.size() == 0) {
        return Shape::splitAt(points);
    }

    std::vector<std::shared_ptr<Shape>> ret;

    std::vector<Vec2d> sortedPoints =
        Vec2d::getSortedByDistance(points, basePoint);

    if (!basePoint.equalsFuzzy(sortedPoints[0])) {
        sortedPoints.prepend(basePoint);
    }

    for (int i = 0; i < sortedPoints.size() - 1; i++) {
        if (sortedPoints[i].equalsFuzzy(sortedPoints[i + 1])) {
            continue;
        }

        ret.push_back(std::shared_ptr<Shape>(
            new Line(sortedPoints[i], sortedPoints[i + 1])));
    }

    ret.push_back(std::shared_ptr<Shape>(
        new Ray(sortedPoints[sortedPoints.size() - 1], directionVector)));

    return ret;
}

} // namespace cada