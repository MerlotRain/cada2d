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

Line::Line() : startPoint(Vec2d::invalid), endPoint(Vec2d::invalid)
{
}

Line::Line(double x1, double y1, double x2, double y2)
    : startPoint(x1, y1), endPoint(x2, y2)
{
}

Line::Line(const Vec2d &startPoint, const Vec2d &endPoint)
    : startPoint(startPoint), endPoint(endPoint)
{
}

Line::Line(const Vec2d &startPoint, double angle, double distance)
    : startPoint(startPoint)
{

    endPoint = startPoint + Vec2d::createPolar(distance, angle);
}

bool Line::isValid() const
{
    return startPoint.isSane() && endPoint.isSane();
}

double Line::getLength() const
{
    return startPoint.getDistanceTo(endPoint);
}

void Line::setLength(double l, bool fromStart)
{
    if (fromStart) {
        endPoint = startPoint + Vec2d::createPolar(l, getAngle());
    }
    else {
        startPoint = endPoint - Vec2d::createPolar(l, getAngle());
    }
}

double Line::getAngle() const
{
    return startPoint.getAngleTo(endPoint);
}

void Line::setAngle(double a)
{
    endPoint = startPoint + Vec2d::createPolar(getLength(), a);
}

bool Line::isParallel(const Line &line) const
{
    double a = getAngle();
    double oa = line.getAngle();

    return Math::isSameDirection(a, oa) || Math::isSameDirection(a, oa + M_PI);
}

bool Line::isVertical(double tolerance) const
{
    return Math::fuzzyCompare(startPoint.x, endPoint.x, tolerance);
}

bool Line::isHorizontal(double tolerance) const
{
    return Math::fuzzyCompare(startPoint.y, endPoint.y, tolerance);
}

double Line::getDirection1() const
{
    return startPoint.getAngleTo(endPoint);
}

double Line::getDirection2() const
{
    return endPoint.getAngleTo(startPoint);
}

Vec2d Line::getStartPoint() const
{
    return startPoint;
}

void Line::setStartPoint(const Vec2d &vector)
{
    startPoint = vector;
}

Vec2d Line::getEndPoint() const
{
    return endPoint;
}

void Line::setEndPoint(const Vec2d &vector)
{
    endPoint = vector;
}

Vec2d Line::getMiddlePoint() const
{
    return (startPoint + endPoint) / 2.0;
}

BBox Line::getBoundingBox() const
{
    return BBox(Vec2d::getMinimum(startPoint, endPoint),
                Vec2d::getMaximum(startPoint, endPoint));
}

std::vector<Vec2d> Line::getEndPoints() const
{
    std::vector<Vec2d> ret;
    ret.push_back(startPoint);
    ret.push_back(endPoint);
    return ret;
}

std::vector<Vec2d> Line::getMiddlePoints() const
{
    std::vector<Vec2d> ret;
    ret.push_back(getMiddlePoint());
    return ret;
}

std::vector<Vec2d> Line::getCenterPoints() const
{
    return getMiddlePoints();
}

std::vector<Vec2d> Line::getPointsWithDistanceToEnd(double distance,
                                                    int from) const
{
    std::vector<Vec2d> ret;

    if (from & NS::FromStart) {
        Vec2d normalStart = (endPoint - startPoint).getNormalized();
        ret.push_back(startPoint + normalStart * distance);
    }
    if (from & NS::FromEnd) {
        Vec2d normalEnd = (startPoint - endPoint).getNormalized();
        ret.push_back(endPoint + normalEnd * distance);
    }

    return ret;
}

std::vector<Vec2d> Line::getPointCloud(double segmentLength) const
{
    std::vector<Vec2d> ret;
    ret.push_back(startPoint);
    if (segmentLength > getLength() / 10000.0) {
        for (double d = segmentLength; d < getLength(); d += segmentLength) {
            ret.push_back(getPointWithDistanceToStart(d));
        }
    }
    ret.push_back(endPoint);
    return ret;
}

double Line::getAngleAt(double distance, NS::From from) const
{
    return getAngle();
}

Vec2d Line::getVectorTo(const Vec2d &point, bool limited,
                        double strictRange) const
{

    Vec2d ae = (endPoint - startPoint).get2D();
    Vec2d ap = (point - startPoint).get2D();

    if (ae.getMagnitude2D() < 1.0e-6) {
        return Vec2d::invalid;
    }

    if (ap.getMagnitude2D() < 1.0e-6) {
        // distance to start point is very small:
        return Vec2d(0, 0);
    }

    double b = Vec2d::getDotProduct(ap, ae) / Vec2d::getDotProduct(ae, ae);

    if (limited && (b < 0 || b > 1.0)) {
        // orthogonal to line does not cross line, use distance to end point:
        Vec2d ret = getVectorFromEndpointTo(point);
        if (ret.getMagnitude() < strictRange) {
            return ret;
        }
        else {
            // not within given range:
            return Vec2d::invalid;
        }
    }

    Vec2d closestPoint = startPoint + ae * b;

    return point - closestPoint;
}

NS::Side Line::getSideOfPoint(const Vec2d &point) const
{
    double entityAngle = getAngle();
    double angleToCoord = startPoint.getAngleTo(point);
    double angleDiff = Math::getAngleDifference(entityAngle, angleToCoord);

    if (angleDiff < M_PI) {
        return NS::LeftHand;
    }
    else {
        return NS::RightHand;
    }
}

void Line::clipToXY(const BBox &box)
{
    double x1 = startPoint.x;
    double y1 = startPoint.y;
    double x2 = endPoint.x;
    double y2 = endPoint.y;
    double xmin = box.getMinimum().x;
    double ymin = box.getMinimum().y;
    double xmax = box.getMaximum().x;
    double ymax = box.getMaximum().y;

    double deltaX, deltaY, p, q;
    double u1 = 0.0, u2 = 1.0;
    double r;

    deltaX = (x2 - x1);
    deltaY = (y2 - y1);

    // left edge, right edge, bottom edge and top edge checking
    double pPart[] = {-1 * deltaX, deltaX, -1 * deltaY, deltaY};
    double qPart[] = {x1 - xmin, xmax - x1, y1 - ymin, ymax - y1};

    bool accept = true;

    for (int i = 0; i < 4; i++) {
        p = pPart[i];
        q = qPart[i];

        if (p == 0 && q < 0) {
            accept = false;
            break;
        }

        r = q / p;

        if (p < 0) {
            u1 = qMax(u1, r);
        }

        if (p > 0) {
            u2 = qMin(u2, r);
        }

        if (u1 > u2) {
            accept = false;
            break;
        }
    }

    if (accept) {
        if (u2 < 1) {
            x2 = x1 + u2 * deltaX;
            y2 = y1 + u2 * deltaY;
        }
        if (u1 > 0) {
            x1 = x1 + u1 * deltaX;
            y1 = y1 + u1 * deltaY;
        }

        startPoint = Vec2d(x1, y1);
        endPoint = Vec2d(x2, y2);
    }
    else {
        startPoint = Vec2d::invalid;
        endPoint = Vec2d::invalid;
    }
}

bool Line::move(const Vec2d &offset)
{
    if (!offset.isValid() || offset.getMagnitude() < NS::PointTolerance) {
        return false;
    }
    startPoint += offset;
    endPoint += offset;
    return true;
}

bool Line::rotate(double rotation, const Vec2d &center)
{
    if (fabs(rotation) < NS::AngleTolerance) {
        return false;
    }
    startPoint.rotate(rotation, center);
    endPoint.rotate(rotation, center);
    return true;
}

bool Line::scale(const Vec2d &scaleFactors, const Vec2d &center)
{
    startPoint.scale(scaleFactors, center);
    endPoint.scale(scaleFactors, center);
    return true;
}

bool Line::mirror(const Line &axis)
{
    startPoint.mirror(axis);
    endPoint.mirror(axis);
    return true;
}

bool Line::flipHorizontal()
{
    startPoint.flipHorizontal();
    endPoint.flipHorizontal();
    return true;
}

bool Line::flipVertical()
{
    startPoint.flipVertical();
    endPoint.flipVertical();
    return true;
}

bool Line::reverse()
{
    Vec2d v = startPoint;
    startPoint = endPoint;
    endPoint = v;
    return true;
}

bool Line::stretch(const Polyline &area, const Vec2d &offset)
{
    bool ret = false;

    if (area.contains(startPoint, true)) {
        startPoint += offset;
        ret = true;
    }
    if (area.contains(endPoint, true)) {
        endPoint += offset;
        ret = true;
    }

    return ret;
}

bool Line::moveTo(const Vec2d &dest)
{
    Vec2d offset = dest - startPoint;
    return move(offset);
}

NS::Ending Line::getTrimEnd(const Vec2d &trimPoint, const Vec2d &clickPoint)
{
    double lineAngle = getAngle();
    double angleToClickPoint = trimPoint.getAngleTo(clickPoint);
    double angleDifference = lineAngle - angleToClickPoint;

    if (angleDifference < 0.0) {
        angleDifference *= -1.0;
    }
    if (angleDifference > M_PI) {
        angleDifference = 2 * M_PI - angleDifference;
    }

    if (angleDifference < M_PI / 2.0) {
        return NS::EndingStart;
    }
    else {
        return NS::EndingEnd;
    }
}

bool Line::trimStartPoint(const Vec2d &trimPoint, const Vec2d &clickPoint,
                          bool extend)
{
    Vec2d tp = getClosestPointOnShape(trimPoint, false);
    if (!tp.isValid()) {
        return false;
    }
    setStartPoint(tp);
    return true;
}

bool Line::trimEndPoint(const Vec2d &trimPoint, const Vec2d &clickPoint,
                        bool extend)
{
    Vec2d tp = getClosestPointOnShape(trimPoint, false);
    if (!tp.isValid()) {
        return false;
    }
    setEndPoint(tp);
    return true;
}

double Line::getDistanceFromStart(const Vec2d &p) const
{
    double ret = startPoint.getDistanceTo(p);

    Vec2d p2 = getClosestPointOnShape(p, false);
    double angle = startPoint.getAngleTo(p2);
    if (Math::isSameDirection(getAngle(), angle, M_PI / 2)) {
        return ret;
    }
    else {
        return -ret;
    }
}

std::vector<std::shared_ptr<Shape>>
Line::splitAt(const std::vector<Vec2d> &points) const
{
    if (points.size() == 0) {
        return Shape::splitAt(points);
    }

    std::vector<std::shared_ptr<Shape>> ret;

    std::vector<Vec2d> sortedPoints =
        Vec2d::getSortedByDistance(points, startPoint);

    if (!startPoint.equalsFuzzy(sortedPoints[0])) {
        sortedPoints.prepend(startPoint);
    }
    if (!endPoint.equalsFuzzy(sortedPoints[sortedPoints.size() - 1])) {
        sortedPoints.push_back(endPoint);
    }

    for (int i = 0; i < sortedPoints.size() - 1; i++) {
        if (sortedPoints[i].equalsFuzzy(sortedPoints[i + 1])) {
            continue;
        }

        ret.push_back(std::shared_ptr<Shape>(
            new Line(sortedPoints[i], sortedPoints[i + 1])));
    }

    return ret;
}

} // namespace cada