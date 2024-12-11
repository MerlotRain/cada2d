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

#include <cmath>

#include <cada2d/RArc.h>
#include <cada2d/RBox.h>
#include <cada2d/RCircle.h>
#include <cada2d/RLine.h>
#include <cada2d/RPolyline.h>

RCircle::RCircle() : m_center(RVector::invalid), m_radius(0.0)
{
}

RCircle::RCircle(double cx, double cy, const double radius)
    : m_center(cx, cy), m_radius(radius)
{
}

RCircle::RCircle(const RVector &center, const double radius)
    : m_center(center), m_radius(radius)
{
}

RCircle::~RCircle()
{
}

RS::ShapeType RCircle::getShapeType() const
{
    return RS::ShapeType();
}

RCircle *RCircle::clone() const
{
    return nullptr;
}

RCircle RCircle::createFrom2Points(const RVector &p1, const RVector &p2)
{
    RVector center = (p1 + p2) / 2.0;
    double radius = p1.getDistanceTo(p2) / 2.0;
    return RCircle(center, radius);
}

RCircle RCircle::createFrom3Points(const RVector &p1, const RVector &p2,
                                   const RVector &p3)
{
    // intersection of two middle lines

    // middle points between first two points:
    RVector mp1 = RVector::getAverage(p1, p2);
    double a1 = p1.getAngleTo(p2) + M_PI / 2.0;
    // direction from middle point to center:
    RVector dir1 = RVector::createPolar(1.0, a1);

    // middle points between last two points:
    RVector mp2 = RVector::getAverage(p2, p3);
    double a2 = p2.getAngleTo(p3) + M_PI / 2.0;
    // direction from middle point to center:
    RVector dir2 = RVector::createPolar(1.0, a2);

    RLine midLine1(mp1, mp1 + dir1);
    RLine midLine2(mp2, mp2 + dir2);

    std::vector<RVector> ips = midLine1.getIntersectionPoints(midLine2, false);
    if (ips.size() != 1) {
        return RCircle();
    }

    RVector center = ips[0];
    double radius = center.getDistanceTo(p3);

    return RCircle(center, radius);
}

RArc RCircle::toArc(double startAngle) const
{
    return RArc(getCenter(), getRadius(), startAngle, startAngle + 2 * M_PI,
                false);
}

bool RCircle::isValid() const
{
    return false;
}

RVector RCircle::getCenter() const
{
    return m_center;
}

void RCircle::setCenter(const RVector &vector)
{
    m_center = vector;
}

double RCircle::getRadius() const
{
    return m_radius;
}

void RCircle::setRadius(double r)
{
    m_radius = r;
}

RBox RCircle::getBoundingBox() const
{
    return RBox(m_center - RVector(m_radius, m_radius),
                m_center + RVector(m_radius, m_radius));
}

double RCircle::getLength() const
{
    return 2 * m_radius * M_PI;
}

double RCircle::getDiameter() const
{
    return 2 * m_radius;
}

void RCircle::setDiameter(double d)
{
    m_radius = d / 2.0;
}

double RCircle::getCircumference() const
{
    return m_radius * 2 * M_PI;
}

void RCircle::setCircumference(double c)
{
    m_radius = c / M_PI / 2.0;
}

double RCircle::getArea() const
{
    return m_radius * m_radius * M_PI;
}

void RCircle::setArea(double a)
{
    m_radius = sqrt(fabs(a) / M_PI);
}

bool RCircle::contains(const RVector &p) const
{
    return p.getDistanceTo(m_center) < m_radius;
}

std::vector<RVector> RCircle::getEndPoints() const
{
    std::vector<RVector> ret;
    return ret;
}

std::vector<RVector> RCircle::getMiddlePoints() const
{
    std::vector<RVector> ret;
    return ret;
}

std::vector<RVector> RCircle::getCenterPoints() const
{
    std::vector<RVector> ret;
    ret.push_back(m_center);
    return ret;
}

std::vector<RVector> RCircle::getArcReferencePoints() const
{
    std::vector<RVector> ret;

    ret.push_back(m_center + RVector(m_radius, 0));
    ret.push_back(m_center + RVector(0, m_radius));
    ret.push_back(m_center - RVector(m_radius, 0));
    ret.push_back(m_center - RVector(0, m_radius));

    return ret;
}

std::vector<RVector> RCircle::getPointsWithDistanceToEnd(double distance,
                                                         int from) const
{
    std::vector<RVector> ret;
    return ret;
}

std::vector<RVector> RCircle::getPointCloud(double segmentLength) const
{
    RArc arc = toArc();
    return arc.getPointCloud(segmentLength);
}

double RCircle::getAngleAt(double distance, RS::From from) const
{
    return RNANDOUBLE;
}

RVector RCircle::getPointAtAngle(double a) const
{
    return RVector(m_center.x + cos(a) * m_radius,
                   m_center.y + sin(a) * m_radius);
}

RVector RCircle::getVectorTo(const RVector &point, bool limited,
                             double strictRange) const
{
    RVector v = point - m_center;

    // point is at the center of the circle, infinite solutions:
    if (v.getMagnitude() < RS::PointTolerance) {
        return RVector::invalid;
    }

    return RVector::createPolar(v.getMagnitude() - m_radius, v.getAngle());
}

RVector RCircle::getPointOnShape() const
{
    return getCenter() + RVector(m_radius, 0);
}

bool RCircle::move(const RVector &offset)
{
    if (!offset.isValid() || offset.getMagnitude() < RS::PointTolerance) {
        return false;
    }
    m_center += offset;
    return true;
}

bool RCircle::rotate(double rotation, const RVector &c)
{
    if (fabs(rotation) < RS::AngleTolerance) {
        return false;
    }
    m_center.rotate(rotation, c);
    return true;
}

bool RCircle::scale(const RVector &scaleFactors, const RVector &c)
{
    m_center.scale(scaleFactors, c);
    m_radius *= scaleFactors.x;
    if (m_radius < 0.0) {
        m_radius *= -1.0;
    }
    return true;
}

bool RCircle::mirror(const RLine &axis)
{
    m_center.mirror(axis.getStartPoint(), axis.getEndPoint());
    return true;
}

bool RCircle::flipHorizontal()
{
    m_center.flipHorizontal();
    return true;
}

bool RCircle::flipVertical()
{
    m_center.flipVertical();
    return true;
}

std::vector<RLine> RCircle::getTangents(const RVector &point) const
{
    std::vector<RLine> ret;

    // create temporary thales circle:
    RVector thalesCenter = (point + getCenter()) / 2;
    double thalesRadius = point.getDistanceTo(thalesCenter);

    if (thalesRadius < getRadius() / 2.0) {
        return ret;
    }

    RCircle thalesCircle(thalesCenter, thalesRadius);

    // get the two intersection points which are the tangent points:
    std::vector<RVector> ips = thalesCircle.getIntersectionPoints(*this, false);

    if (ips.size() > 0) {
        ret.push_back(RLine(point, ips[0]));
        if (ips.size() > 1) {
            ret.push_back(RLine(point, ips[1]));
        }
    }

    return ret;
}

std::vector<std::shared_ptr<RShape>>
RCircle::getOffsetShapes(double distance, int number, RS::Side side,
                         const RVector &position)
{
    return std::vector<std::shared_ptr<RShape>>();
}

std::vector<std::shared_ptr<RShape>>
RCircle::splitAt(const std::vector<RVector> &points) const
{
    if (points.size() == 0) {
        return RShape::splitAt(points);
    }

    std::vector<std::shared_ptr<RShape>> ret;

    double refAngle = m_center.getAngleTo(points[0]);
    RVector startPoint;
    RVector endPoint;

    startPoint = endPoint = m_center + RVector::createPolar(m_radius, refAngle);

    std::vector<RVector> sortedPoints =
        RVector::getSortedByAngle(points, m_center, refAngle);

    if (!startPoint.equalsFuzzy(sortedPoints[0])) {
        sortedPoints.insert(sortedPoints.begin(), startPoint);
    }
    if (!endPoint.equalsFuzzy(sortedPoints[sortedPoints.size() - 1])) {
        sortedPoints.push_back(endPoint);
    }
    for (int i = 0; i < sortedPoints.size() - 1; i++) {
        if (sortedPoints[i].equalsFuzzy(sortedPoints[i + 1])) {
            continue;
        }

        ret.push_back(std::shared_ptr<RShape>(
            new RArc(m_center, m_radius, m_center.getAngleTo(sortedPoints[i]),
                     m_center.getAngleTo(sortedPoints[i + 1]), false)));
    }

    return ret;
}
