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

/**
 * Creates an xline object with invalid base point and direction.
 */
XLine::XLine() : basePoint(Vec2d::invalid), directionVector(Vec2d::invalid)
{
}

XLine::XLine(const Line &line)
    : basePoint(line.getStartPoint()),
      directionVector(line.getEndPoint() - line.getStartPoint())
{
}

/**
 * Creates an xline object with the given base point and direction.
 */
XLine::XLine(const Vec2d &basePoint, const Vec2d &directionVector)
    : basePoint(basePoint), directionVector(directionVector)
{
}

XLine::XLine(const Vec2d &basePoint, double angle, double distance)
    : basePoint(basePoint), directionVector(Vec2d::createPolar(distance, angle))
{
}

XLine::~XLine()
{
}

BBox XLine::getBoundingBox() const
{
    return BBox(Vec2d::getMinimum(basePoint, getSecondPoint()),
                Vec2d::getMaximum(basePoint, getSecondPoint()));
}

double XLine::getLength() const
{
    return std::numeric_limits<double>::quiet_NaN();
}

double XLine::getAngle() const
{
    return directionVector.getAngle();
}

void XLine::setAngle(double a)
{
    directionVector.setAngle(a);
}

void XLine::setLength(double l)
{
    return;
}

double XLine::getDirection1() const
{
    return directionVector.getAngle();
}

double XLine::getDirection2() const
{
    return getSecondPoint().getAngleTo(basePoint);
}

NS::Side XLine::getSideOfPoint(const Vec2d &point) const
{
    return getLineShape().getSideOfPoint(point);
}

Vec2d XLine::getStartPoint() const
{
    return basePoint;
}

Vec2d XLine::getEndPoint() const
{
    return getSecondPoint();
}

bool XLine::trimStartPoint(const Vec2d &trimPoint, const Vec2d &clickPoint,
                           bool extend)
{
    Vec2d tp = getClosestPointOnShape(trimPoint, false);
    if (!tp.isValid()) {
        return false;
    }
    basePoint = tp;
    return true;
}

bool XLine::trimEndPoint(const Vec2d &trimPoint, const Vec2d &clickPoint,
                         bool extend)
{
    Vec2d tp = getClosestPointOnShape(trimPoint, false);
    if (!tp.isValid()) {
        return false;
    }
    basePoint = tp;
    directionVector = -directionVector;
    return true;
}

NS::Ending XLine::getTrimEnd(const Vec2d &trimPoint, const Vec2d &clickPoint)
{
    return getLineShape().getTrimEnd(trimPoint, clickPoint);
}

double XLine::getDistanceFromStart(const Vec2d &p) const
{
    double ret = basePoint.getDistanceTo(p);

    Vec2d p2 = getClosestPointOnShape(p, false);
    double angle = basePoint.getAngleTo(p2);
    if (Math::isSameDirection(getAngle(), angle, M_PI / 2)) {
        return ret;
    }
    else {
        return -ret;
    }
}

Vec2d XLine::getBasePoint() const
{
    return basePoint;
}

void XLine::setBasePoint(const Vec2d &vector)
{
    basePoint = vector;
}

Vec2d XLine::getSecondPoint() const
{
    return basePoint + directionVector;
}

void XLine::setSecondPoint(const Vec2d &vector)
{
    directionVector = vector - basePoint;
}

Vec2d XLine::getDirectionVector() const
{
    return directionVector;
}

void XLine::setDirectionVector(const Vec2d &vector)
{
    directionVector = vector;
}

Vec2d XLine::getMiddlePoint() const
{
    return Vec2d::invalid;
}

std::vector<Vec2d> XLine::getEndPoints() const
{
    return std::vector<Vec2d>();
}

std::vector<Vec2d> XLine::getMiddlePoints() const
{
    return std::vector<Vec2d>();
}

std::vector<Vec2d> XLine::getCenterPoints() const
{
    return std::vector<Vec2d>();
}

std::vector<Vec2d> XLine::getPointsWithDistanceToEnd(double distance,
                                                     int from) const
{
    return std::vector<Vec2d>();
}

double XLine::getAngleAt(double distance, NS::From from) const
{
    return getAngle();
}

Vec2d XLine::getVectorTo(const Vec2d &point, bool limited,
                         double strictRange) const
{
    return getLineShape().getVectorTo(point, false, strictRange);
}

Line XLine::getClippedLine(const BBox &box) const
{
    Line ret = getLineShape();

    Polyline pl = box.getPolyline2d();

    std::vector<Vec2d> ips =
        Shape::getIntersectionPointsLX(getLineShape(), pl, false);
    std::vector<Vec2d> sol;
    for (int i = 0; i < ips.size(); i++) {
        if (pl.isOnShape(ips[i])) {
            Vec2d p = ips[i].getClosest(sol);
            if (p.equalsFuzzy(ips[i])) {
                continue;
            }
            sol.push_back(ips[i]);
        }
    }

    if (sol.size() == 2) {
        ret = Line(sol[0], sol[1]);
        if (!Math::isSameDirection(ret.getDirection1(), getDirection1(),
                                   1.0e-2)) {
            ret.reverse();
        }
    }

    return ret;
}

bool XLine::move(const Vec2d &offset)
{
    if (!offset.isValid() || offset.getMagnitude() < NS::PointTolerance) {
        return false;
    }
    basePoint += offset;
    return true;
}

bool XLine::rotate(double rotation, const Vec2d &center)
{
    if (fabs(rotation) < NS::AngleTolerance) {
        return false;
    }
    basePoint.rotate(rotation, center);
    directionVector.rotate(rotation);
    return true;
}

bool XLine::scale(const Vec2d &scaleFactors, const Vec2d &center)
{
    basePoint.scale(scaleFactors, center);
    directionVector.scale(scaleFactors);
    return true;
}

bool XLine::mirror(const Line &axis)
{
    Vec2d sp = getSecondPoint();
    basePoint.mirror(axis);
    sp.mirror(axis);
    setSecondPoint(sp);
    return true;
}

bool XLine::reverse()
{
    Vec2d sp = getSecondPoint();
    Vec2d bp = basePoint;
    setBasePoint(sp);
    setSecondPoint(bp);
    return true;
}

bool XLine::stretch(const Polyline &area, const Vec2d &offset)
{
    return false;
}

std::vector<std::shared_ptr<Shape>>
XLine::splitAt(const std::vector<Vec2d> &points) const
{
    if (points.size() == 0) {
        return Shape::splitAt(points);
    }

    std::vector<std::shared_ptr<Shape>> ret;

    std::vector<Vec2d> sortedPoints =
        Vec2d::getSortedByDistance(points, basePoint - directionVector * 1e9);

    ret.push_back(
        std::shared_ptr<Shape>(new Ray(sortedPoints[0], -directionVector)));

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