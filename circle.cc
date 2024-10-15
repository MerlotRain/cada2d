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
#include <numeric>
#include <limits>

using namespace cada;

/**
 * Creates a circle object with invalid center and 0 radius.
 */
Circle::Circle() : center(Vec3d::invalid), radius(0.0)
{
}

Circle::Circle(double cx, double cy, const double radius)
    : center(cx, cy), radius(radius)
{
}

Circle::Circle(const Vec3d &center, const double radius)
    : center(center), radius(radius)
{
}

Circle::~Circle()
{
}

Circle Circle::createFrom2Points(const Vec3d &p1, const Vec3d &p2)
{
    Vec3d center = (p1 + p2) / 2.0;
    double radius = p1.getDistanceTo(p2) / 2.0;
    return Circle(center, radius);
}

Circle Circle::createFrom3Points(const Vec3d &p1, const Vec3d &p2,
                                 const Vec3d &p3)
{
    // intersection of two middle lines

    // middle points between first two points:
    Vec3d mp1 = Vec3d::getAverage(p1, p2);
    double a1 = p1.getAngleTo(p2) + M_PI / 2.0;
    // direction from middle point to center:
    Vec3d dir1 = Vec3d::createPolar(1.0, a1);

    // middle points between last two points:
    Vec3d mp2 = Vec3d::getAverage(p2, p3);
    double a2 = p2.getAngleTo(p3) + M_PI / 2.0;
    // direction from middle point to center:
    Vec3d dir2 = Vec3d::createPolar(1.0, a2);

    Line midLine1(mp1, mp1 + dir1);
    Line midLine2(mp2, mp2 + dir2);

    std::vector<Vec3d> ips = midLine1.getIntersectionPoints(midLine2, false);
    if (ips.size() != 1) {
        return Circle();
    }

    Vec3d center = ips[0];
    double radius = center.getDistanceTo(p3);
    //    double angle1 = center.getAngleTo(p1);
    //    double angle2 = center.getAngleTo(p3);
    //    bool reversed = Math::isAngleBetween(center.getAngleTo(p2),
    //                                            angle1, angle2, true);

    return Circle(center, radius);
}

Arc Circle::toArc(double startAngle) const
{
    return Arc(getCenter(), getRadius(), startAngle, startAngle + 2 * M_PI,
               false);
}

Vec3d Circle::getCenter() const
{
    return center;
}

void Circle::setCenter(const Vec3d &vector)
{
    center = vector;
}

double Circle::getRadius() const
{
    return radius;
}

void Circle::setRadius(double r)
{
    radius = r;
}

BBox Circle::getBoundingBox() const
{
    return BBox(center - Vec3d(radius, radius), center + Vec3d(radius, radius));
}

double Circle::getLength() const
{
    return 2 * radius * M_PI;
}

double Circle::getDiameter() const
{
    return 2 * radius;
}

void Circle::setDiameter(double d)
{
    radius = d / 2.0;
}

double Circle::getCircumference() const
{
    return radius * 2 * M_PI;
}

void Circle::setCircumference(double c)
{
    radius = c / M_PI / 2.0;
}

double Circle::getArea() const
{
    return radius * radius * M_PI;
}

void Circle::setArea(double a)
{
    radius = sqrt(fabs(a) / M_PI);
}

bool Circle::contains(const Vec3d &p) const
{
    return p.getDistanceTo(center) < radius;
    // TODO: + NS::PointTolerance ?
}

// bool Circle::touchesCircleInternally(const Circle& other) const {
//     return contains(other.center) || other.contains(center);
// }

std::vector<Vec3d> Circle::getEndPoints() const
{
    std::vector<Vec3d> ret;
    return ret;
}

std::vector<Vec3d> Circle::getMiddlePoints() const
{
    std::vector<Vec3d> ret;
    return ret;
}

std::vector<Vec3d> Circle::getCenterPoints() const
{
    std::vector<Vec3d> ret;
    ret.push_back(center);
    return ret;
}

std::vector<Vec3d> Circle::getArcReferencePoints() const
{
    std::vector<Vec3d> ret;

    ret.push_back(center + Vec3d(radius, 0));
    ret.push_back(center + Vec3d(0, radius));
    ret.push_back(center - Vec3d(radius, 0));
    ret.push_back(center - Vec3d(0, radius));

    return ret;
}

std::vector<Vec3d> Circle::getPointsWithDistanceToEnd(double distance,
                                                      int from) const
{
    // Q_UNUSED(distance)
    // Q_UNUSED(from)

    std::vector<Vec3d> ret;
    return ret;
}

std::vector<Vec3d> Circle::getPointCloud(double segmentLength) const
{
    Arc arc = toArc();
    return arc.getPointCloud(segmentLength);
}

double Circle::getAngleAt(double distance, NS::From from) const
{
    // Q_UNUSED(distance)
    // Q_UNUSED(from)

    return std::numeric_limits<double>::quiet_NaN();
}

Vec3d Circle::getPointAtAngle(double a) const
{
    return Vec3d(center.x + cos(a) * radius, center.y + sin(a) * radius);
}

Vec3d Circle::getVectorTo(const Vec3d &point, bool limited,
                          double strictRange) const
{
    // Q_UNUSED(limited)
    // Q_UNUSED(strictRange)

    Vec3d v = (point - center).get2D();

    // point is at the center of the circle, infinite solutions:
    if (v.getMagnitude() < NS::PointTolerance) {
        return Vec3d::invalid;
    }

    return Vec3d::createPolar(v.getMagnitude() - radius, v.getAngle());
}

bool Circle::move(const Vec3d &offset)
{
    if (!offset.isValid() || offset.getMagnitude() < NS::PointTolerance) {
        return false;
    }
    center += offset;
    return true;
}

bool Circle::rotate(double rotation, const Vec3d &c)
{
    if (fabs(rotation) < NS::AngleTolerance) {
        return false;
    }
    center.rotate(rotation, c);
    return true;
}

bool Circle::scale(const Vec3d &scaleFactors, const Vec3d &c)
{
    center.scale(scaleFactors, c);
    radius *= scaleFactors.x;
    if (radius < 0.0) {
        radius *= -1.0;
    }
    return true;
}

bool Circle::mirror(const Line &axis)
{
    center.mirror(axis);
    return true;
}

bool Circle::flipHorizontal()
{
    center.flipHorizontal();
    return true;
}

bool Circle::flipVertical()
{
    center.flipVertical();
    return true;
}

std::vector<Line> Circle::getTangents(const Vec3d &point) const
{
    std::vector<Line> ret;

    // create temporary thales circle:
    Vec3d thalesCenter = (point + getCenter()) / 2;
    double thalesRadius = point.getDistanceTo(thalesCenter);

    if (thalesRadius < getRadius() / 2.0) {
        return ret;
    }

    Circle thalesCircle(thalesCenter, thalesRadius);

    // get the two intersection points which are the tangent points:
    std::vector<Vec3d> ips = thalesCircle.getIntersectionPoints(*this, false);

    if (ips.size() > 0) {
        ret.push_back(Line(point, ips[0]));
        if (ips.size() > 1) {
            ret.push_back(Line(point, ips[1]));
        }
    }

    return ret;
}

std::vector<std::shared_ptr<Shape>>
Circle::splitAt(const std::vector<Vec3d> &points) const
{
    if (points.size() == 0) {
        return Shape::splitAt(points);
    }

    std::vector<std::shared_ptr<Shape>> ret;

    double refAngle = center.getAngleTo(points[0]);
    Vec3d startPoint;
    Vec3d endPoint;

    startPoint = endPoint = center + Vec3d::createPolar(radius, refAngle);

    std::vector<Vec3d> sortedPoints =
        Vec3d::getSortedByAngle(points, center, refAngle);

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
            new Arc(center, radius, center.getAngleTo(sortedPoints[i]),
                    center.getAngleTo(sortedPoints[i + 1]), false)));
    }

    return ret;
}
