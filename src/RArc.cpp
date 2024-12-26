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

#include <cada2d/private/RShapePrivate.h>
#include <cada2d/RArc.h>
#include <cada2d/RCircle.h>
#include <cada2d/RBox.h>
#include <cada2d/RLine.h>
#include <cada2d/RMath.h>
#include <cada2d/RPolyline.h>

RArc::RArc()
    : m_center(RVector::invalid), m_radius(0.0), m_startAngle(0.0),
      m_endAngle(0.0), m_reversed(false)
{
}

RArc::RArc(double cx, double cy, double radius, double startAngle,
           double endAngle, bool reversed)
    : m_center(cx, cy), m_radius(radius), m_startAngle(startAngle),
      m_endAngle(endAngle), m_reversed(reversed)
{
}

RArc::RArc(const RVector &center, double radius, double startAngle,
           double endAngle, bool reversed)
    : m_center(center), m_radius(radius), m_startAngle(startAngle),
      m_endAngle(endAngle), m_reversed(reversed)
{
}

RS::ShapeType RArc::getShapeType() const
{
    return RS::Arc;
}

bool RArc::isDirected() const
{
    return true;
}

RArc *RArc::clone() const
{
    return new RArc(*this);
}

bool RArc::isValid() const
{
    return m_center.isValid() && m_radius > 0.0;
}

bool RArc::isFullCircle(double tolerance) const
{
    return fabs(RMath::getAngleDifference180(
               RMath::getNormalizedAngle(m_startAngle),
               RMath::getNormalizedAngle(m_endAngle))) < tolerance;
}

RArc RArc::createFrom3Points(const RVector &startPoint, const RVector &point,
                             const RVector &endPoint)
{
    // intersection of two middle lines

    // middle points between first two points:
    RVector mp1 = RVector::getAverage(startPoint, point);
    double a1 = startPoint.getAngleTo(point) + M_PI / 2.0;
    // direction from middle point to center:
    RVector dir1 = RVector::createPolar(1.0, a1);

    // middle points between last two points:
    RVector mp2 = RVector::getAverage(point, endPoint);
    double a2 = point.getAngleTo(endPoint) + M_PI / 2.0;
    // direction from middle point to center:
    RVector dir2 = RVector::createPolar(1.0, a2);

    RLine midLine1(mp1, mp1 + dir1);
    RLine midLine2(mp2, mp2 + dir2);

    std::vector<RVector> ips = midLine1.getIntersectionPoints(midLine2, false);
    if (ips.size() != 1) {
        return RArc();
    }

    RVector center = ips[0];
    double radius = center.getDistanceTo(endPoint);
    double angle1 = center.getAngleTo(startPoint);
    double angle2 = center.getAngleTo(endPoint);
    bool reversed =
        RMath::isAngleBetween(center.getAngleTo(point), angle1, angle2, true);

    return RArc(center, radius, angle1, angle2, reversed);
}

RArc RArc::createFrom2PBulge(const RVector &startPoint, const RVector &endPoint,
                             double bulge)
{

    RArc arc;

    arc.m_reversed = (bulge < 0.0);
    double alpha = atan(bulge) * 4.0;

    RVector middle = (startPoint + endPoint) / 2.0;
    double dist = startPoint.getDistanceTo(endPoint) / 2.0;

    // alpha can't be 0.0 at this point
    arc.m_radius = fabs(dist / sin(alpha / 2.0));

    double wu = fabs(std::pow(arc.m_radius, 2.0) - std::pow(dist, 2.0));
    double h = sqrt(wu);
    double angle = startPoint.getAngleTo(endPoint);

    if (bulge > 0.0) {
        angle += M_PI / 2.0;
    }
    else {
        angle -= M_PI / 2.0;
    }

    if (fabs(alpha) > M_PI) {
        h *= -1.0;
    }

    arc.m_center.setPolar(h, angle);
    arc.m_center += middle;
    arc.m_startAngle = arc.m_center.getAngleTo(startPoint);
    arc.m_endAngle = arc.m_center.getAngleTo(endPoint);

    return arc;
}

RArc RArc::createTangential(const RVector &startPoint, const RVector &pos,
                            double direction, double radius)
{
    RArc arc;

    arc.m_radius = radius;

    // orthogonal to base entity:
    RVector ortho;
    ortho.setPolar(radius, direction + M_PI / 2.0);

    // two possible center points for arc:
    RVector center1 = startPoint + ortho;
    RVector center2 = startPoint - ortho;
    if (center1.getDistanceTo(pos) < center2.getDistanceTo(pos)) {
        arc.m_center = center1;
    }
    else {
        arc.m_center = center2;
    }

    // angles:
    arc.m_startAngle = arc.m_center.getAngleTo(startPoint);
    arc.m_endAngle = arc.m_center.getAngleTo(pos);

    // handle arc direction:
    arc.m_reversed = false;
    double diff = RMath::getNormalizedAngle(arc.getDirection1() - direction);
    if (fabs(diff - M_PI) < 1.0e-1) {
        arc.m_reversed = true;
    }

    return arc;
}

std::vector<RArc> RArc::createBiarc(const RVector &startPoint,
                                    double startDirection,
                                    const RVector &endPoint,
                                    double endDirection, bool secondTry)
{

    double length = startPoint.getDistanceTo(endPoint);
    double angle = startPoint.getAngleTo(endPoint);

    double alpha = RMath::getAngleDifference180(startDirection, angle);
    double beta = RMath::getAngleDifference180(angle, endDirection);

    double theta;
    if ((alpha > 0 && beta > 0) || (alpha < 0 && beta < 0)) {
        // same sign: C-shaped curve:
        theta = alpha;
    }
    else {
        // different sign: S-shaped curve:
        theta = (3.0 * alpha - beta) / 2.0;
    }

    RVector startNormal(-sin(startDirection), cos(startDirection));
    RVector jointPointNormal(-sin(theta + startDirection),
                             cos(theta + startDirection));

    double term1 = (length / (2.0 * sin((alpha + beta) / 2.0)));

    double radius1 =
        term1 * (sin((beta - alpha + theta) / 2.0) / sin(theta / 2.0));
    double radius2 = term1 * (sin((2.0 * alpha - theta) / 2.0) /
                              sin((alpha + beta - theta) / 2.0));

    // failed, might succeed in reverse direction:
    if (std::fabs(radius1) < RS::PointTolerance ||
        std::fabs(radius2) < RS::PointTolerance || !RMath::isNormal(radius1) ||
        !RMath::isNormal(radius2)) {

        if (secondTry) {
            return std::vector<RArc>();
        }

        std::vector<RArc> list =
            RArc::createBiarc(endPoint, endDirection + M_PI, startPoint,
                              startDirection + M_PI, true);
        if (list.empty()) {
            return std::vector<RArc>();
        }

        for (int i = 0; i < list.size(); i++) {
            list[i].reverse();
        }
        return {list[1], list[0]};
        //        return std::vector<RArc>();
    }

    RVector jointPoint =
        startPoint + radius1 * (startNormal - jointPointNormal);

    RVector center1 = startPoint + startNormal * radius1;
    RVector center2 = jointPoint + jointPointNormal * radius2;

    RArc arc1(center1, std::fabs(radius1), center1.getAngleTo(startPoint),
              center1.getAngleTo(jointPoint));
    if (std::fabs(RMath::getAngleDifference180(arc1.getDirection1(),
                                               startDirection)) > 0.1) {
        arc1.setReversed(true);
    }

    RArc arc2(center2, std::fabs(radius2), center2.getAngleTo(jointPoint),
              center2.getAngleTo(endPoint));
    if (std::fabs(RMath::getAngleDifference180(arc2.getDirection2() + M_PI,
                                               endDirection)) > 0.1) {
        arc2.setReversed(true);
    }

    return {arc1, arc2};
}

double RArc::getDirection1() const
{
    if (!m_reversed) {
        return RMath::getNormalizedAngle(m_startAngle + M_PI / 2.0);
    }
    else {
        return RMath::getNormalizedAngle(m_startAngle - M_PI / 2.0);
    }
}

double RArc::getDirection2() const
{
    if (!m_reversed) {
        return RMath::getNormalizedAngle(m_endAngle - M_PI / 2.0);
    }
    else {
        return RMath::getNormalizedAngle(m_endAngle + M_PI / 2.0);
    }
}

RS::Side RArc::getSideOfPoint(const RVector &point) const
{
    if (m_reversed) {
        if (m_center.getDistanceTo(point) < m_radius) {
            return RS::RightHand;
        }
        else {
            return RS::LeftHand;
        }
    }
    else {
        if (m_center.getDistanceTo(point) < m_radius) {
            return RS::LeftHand;
        }
        else {
            return RS::RightHand;
        }
    }
}

void RArc::moveStartPoint(const RVector &pos, bool keepRadius)
{
    if (!keepRadius) {
        RArc a = RArc::createFrom3Points(pos, getMiddlePoint(), getEndPoint());
        if (a.isReversed() != isReversed()) {
            a.reverse();
        }
        *this = a;
    }
    else {
        double bulge = getBulge();

        // full circle: trim instead of move:
        if (bulge < 1.0e-6 || bulge > 1.0e6) {
            m_startAngle = m_center.getAngleTo(pos);
        }
        else {
            *this = RArc::createFrom2PBulge(pos, getEndPoint(), bulge);
        }
    }
}

void RArc::moveEndPoint(const RVector &pos, bool keepRadius)
{
    if (!keepRadius) {
        RArc a =
            RArc::createFrom3Points(pos, getMiddlePoint(), getStartPoint());
        if (a.isReversed() != isReversed()) {
            a.reverse();
        }
        *this = a;
    }
    else {
        double bulge = getBulge();

        // full circle: trim instead of move:
        if (bulge < 1.0e-6 || bulge > 1.0e6) {
            m_endAngle = m_center.getAngleTo(pos);
        }
        else {
            *this = RArc::createFrom2PBulge(getStartPoint(), pos, bulge);
        }
    }
}

void RArc::moveMiddlePoint(const RVector &pos)
{
    *this = RArc::createFrom3Points(getStartPoint(), pos, getEndPoint());
}

double RArc::getBulge() const
{
    double bulge = tan(fabs(getSweep()) / 4.0);
    if (isReversed()) {
        bulge *= -1;
    }
    return bulge;
}

double RArc::getLength() const
{
    return fabs(getAngleLength(false)) * m_radius;
}

double RArc::getDiameter() const
{
    return 2 * m_radius;
}

void RArc::setDiameter(double d)
{
    m_radius = d / 2.0;
}

void RArc::setLength(double l)
{
    double sweep = l / m_radius;
    if (sweep > 2 * M_PI) {
        sweep = 2 * M_PI;
    }
    if (m_reversed) {
        sweep *= -1;
    }

    m_endAngle = m_startAngle + sweep;
}

double RArc::getArea() const
{
    return (m_radius * m_radius * getAngleLength(false)) / 2.0;
}

void RArc::setArea(double a)
{
    double sweep = (a * 2.0) / (m_radius * m_radius);
    if (m_reversed) {
        m_endAngle = RMath::getNormalizedAngle(m_startAngle - sweep);
    }
    else {
        m_endAngle = RMath::getNormalizedAngle(m_startAngle + sweep);
    }
}

double RArc::getChordArea() const
{
    double sectorArea = 0.0;
    double angleLength = getAngleLength(false);
    double sweep = getSweep();
    if (sweep < M_PI) {
        sectorArea =
            ((m_radius * m_radius) * (angleLength - sin(angleLength))) / 2.0;
    }
    else if (sweep == M_PI) {
        sectorArea = 0.5 * (M_PI * m_radius * m_radius);
    }
    else {
        double remainAngle = (M_PI * 2) - sweep;
        double remainSliceArea = (m_radius * m_radius * remainAngle) / 2.0;
        double remainSectorArea =
            (m_radius * m_radius * (remainAngle - sin(remainAngle))) / 2.0;
        sectorArea = getArea() + (remainSliceArea - remainSectorArea);
    }

    return sectorArea;
}

double RArc::getAngleLength(bool allowForZeroLength) const
{
    double ret = fabs(getSweep());

    // full circle or zero length arc:
    if (!allowForZeroLength) {
        if (ret < RS::PointTolerance) {
            ret = 2 * M_PI;
        }
    }
    else {
        if (ret > 2 * M_PI - RS::PointTolerance) {
            ret = 0.0;
        }
    }

    return ret;
}

bool RArc::isAngleWithinArc(double a) const
{
    return RMath::isAngleBetween(a, m_startAngle, m_endAngle, m_reversed);
}

double RArc::getSweep() const
{
    double ret = 0.0;

    if (m_reversed) {
        if (m_startAngle <= m_endAngle) {
            ret = -(m_startAngle + 2 * M_PI - m_endAngle);
        }
        else {
            ret = -(m_startAngle - m_endAngle);
        }
    }
    else {
        if (m_endAngle <= m_startAngle) {
            ret = m_endAngle + 2 * M_PI - m_startAngle;
        }
        else {
            ret = m_endAngle - m_startAngle;
        }
    }

    return ret;
}

void RArc::setSweep(double s)
{
    m_endAngle = m_startAngle + s;
    m_reversed = (s < 0.0);
}

RVector RArc::getCenter() const
{
    return m_center;
}

void RArc::setCenter(const RVector &vector)
{
    m_center = vector;
}

double RArc::getRadius() const
{
    return m_radius;
}

void RArc::setRadius(double r)
{
    m_radius = r;
}

double RArc::getStartAngle() const
{
    return m_startAngle;
}

void RArc::setStartAngle(double a)
{
    m_startAngle = RMath::getNormalizedAngle(a);
}

double RArc::getEndAngle() const
{
    return m_endAngle;
}

void RArc::setEndAngle(double a)
{
    m_endAngle = RMath::getNormalizedAngle(a);
}

RVector RArc::getMiddlePoint() const
{
    double a;
    a = m_startAngle + getSweep() / 2.0;
    RVector v = RVector::createPolar(m_radius, a);
    v += m_center;
    return v;
}

RVector RArc::getStartPoint() const
{
    return getPointAtAngle(m_startAngle);
}

RVector RArc::getEndPoint() const
{
    return getPointAtAngle(m_endAngle);
}

RVector RArc::getPointAtAngle(double a) const
{
    return RVector(m_center.x + cos(a) * m_radius,
                   m_center.y + sin(a) * m_radius);
}

double RArc::getAngleAt(double distance, RS::From from) const
{
    std::vector<RVector> points = getPointsWithDistanceToEnd(distance, from);
    if (points.size() != 1) {
        return RNANDOUBLE;
    }
    return m_center.getAngleTo(points[0]) + (m_reversed ? -M_PI / 2 : M_PI / 2);
}

bool RArc::isReversed() const
{
    return m_reversed;
}

void RArc::setReversed(bool r)
{
    m_reversed = r;
}

RBox RArc::getBoundingBox() const
{
    if (!isValid()) {
        return RBox();
    }

    RVector minV;
    RVector maxV;
    double minX = qMin(getStartPoint().x, getEndPoint().x);
    double minY = qMin(getStartPoint().y, getEndPoint().y);
    double maxX = qMax(getStartPoint().x, getEndPoint().x);
    double maxY = qMax(getStartPoint().y, getEndPoint().y);

    if (getStartPoint().getDistanceTo(getEndPoint()) < 1.0e-6 &&
        getRadius() > 1.0e5) {
        minV = RVector(minX, minY);
        maxV = RVector(maxX, maxY);
        return RBox(minV, maxV);
    }

    double a1 =
        RMath::getNormalizedAngle(!isReversed() ? m_startAngle : m_endAngle);
    double a2 =
        RMath::getNormalizedAngle(!isReversed() ? m_endAngle : m_startAngle);

    // check for left limit:
    if ((a1 < M_PI && a2 > M_PI) || (a1 > a2 - 1.0e-12 && a2 > M_PI) ||
        (a1 > a2 - 1.0e-12 && a1 < M_PI)) {

        minX = qMin(m_center.x - m_radius, minX);
    }

    // check for right limit:
    if (a1 > a2 - 1.0e-12) {
        maxX = qMax(m_center.x + m_radius, maxX);
    }

    // check for bottom limit:
    if ((a1 < (M_PI_2 * 3) && a2 > (M_PI_2 * 3)) ||
        (a1 > a2 - 1.0e-12 && a2 > (M_PI_2 * 3)) ||
        (a1 > a2 - 1.0e-12 && a1 < (M_PI_2 * 3))) {

        minY = qMin(m_center.y - m_radius, minY);
    }

    // check for top limit:
    if ((a1 < M_PI_2 && a2 > M_PI_2) || (a1 > a2 - 1.0e-12 && a2 > M_PI_2) ||
        (a1 > a2 - 1.0e-12 && a1 < M_PI_2)) {

        maxY = qMax(m_center.y + m_radius, maxY);
    }

    minV = RVector(minX, minY);
    maxV = RVector(maxX, maxY);

    return RBox(minV, maxV);
}

std::vector<RVector> RArc::getEndPoints() const
{
    std::vector<RVector> ret;
    ret.push_back(getStartPoint());
    ret.push_back(getEndPoint());
    return ret;
}

std::vector<RVector> RArc::getMiddlePoints() const
{
    std::vector<RVector> ret;
    ret.push_back(getMiddlePoint());
    return ret;
}

std::vector<RVector> RArc::getCenterPoints() const
{
    std::vector<RVector> ret;
    ret.push_back(getCenter());
    return ret;
}

std::vector<RVector> RArc::getArcReferencePoints() const
{
    std::vector<RVector> ret;

    std::vector<RVector> p;
    p.push_back(m_center + RVector(m_radius, 0));
    p.push_back(m_center + RVector(0, m_radius));
    p.push_back(m_center - RVector(m_radius, 0));
    p.push_back(m_center - RVector(0, m_radius));

    for (int i = 0; i < p.size(); i++) {
        if (RMath::isAngleBetween(m_center.getAngleTo(p[i]), m_startAngle,
                                  m_endAngle, m_reversed)) {
            ret.push_back(p[i]);
        }
    }

    return ret;
}

std::vector<RVector> RArc::getPointsWithDistanceToEnd(double distance,
                                                      int from) const
{
    std::vector<RVector> ret;

    if (m_radius < RS::PointTolerance) {
        return ret;
    }

    double a1;
    double a2;
    RVector p;
    double aDist = distance / m_radius;

    if (isReversed()) {
        a1 = getStartAngle() - aDist;
        a2 = getEndAngle() + aDist;
    }
    else {
        a1 = getStartAngle() + aDist;
        a2 = getEndAngle() - aDist;
    }

    if (from & RS::FromStart) {
        p.setPolar(m_radius, a1);
        p += m_center;
        ret.push_back(p);
    }

    if (from & RS::FromEnd) {
        p.setPolar(m_radius, a2);
        p += m_center;
        ret.push_back(p);
    }

    return ret;
}

std::vector<RVector> RArc::getPointCloud(double segmentLength) const
{
    std::vector<RVector> ret;
    RPolyline pl = approximateWithLines(segmentLength);
    auto vertices = pl.getVertices();
    ret.insert(ret.end(), vertices.begin(), vertices.end());
    pl = approximateWithLinesTan(segmentLength);
    ret.insert(ret.end(), vertices.begin(), vertices.end());
    return ret;
}

RVector RArc::getVectorTo(const RVector &point, bool limited,
                          double strictRange) const
{
    double angle = m_center.getAngleTo(point);
    if (limited &&
        !RMath::isAngleBetween(angle, m_startAngle, m_endAngle, m_reversed)) {
        return RVector::invalid;
    }

    RVector v = point - m_center;
    return RVector::createPolar(v.getMagnitude() - m_radius, v.getAngle());
}

bool RArc::move(const RVector &offset)
{
    if (!offset.isValid() || offset.getMagnitude() < RS::PointTolerance) {
        return false;
    }
    m_center += offset;
    return true;
}

bool RArc::rotate(double rotation, const RVector &c)
{
    if (fabs(rotation) < RS::AngleTolerance) {
        return false;
    }

    m_center.rotate(rotation, c);

    // important for circle shaped in hatch boundaries:
    if (!isFullCircle()) {
        m_startAngle = RMath::getNormalizedAngle(m_startAngle + rotation);
        m_endAngle = RMath::getNormalizedAngle(m_endAngle + rotation);
    }

    return true;
}

bool RArc::scale(const RVector &scaleFactors, const RVector &c)
{
    // negative scaling: mirroring and scaling
    if (scaleFactors.x < 0.0) {
        mirror(RLine(m_center, m_center + RVector(0.0, 1.0)));
    }
    if (scaleFactors.y < 0.0) {
        mirror(RLine(m_center, m_center + RVector(1.0, 0.0)));
    }

    m_center.scale(scaleFactors, c);
    m_radius *= scaleFactors.x;
    if (m_radius < 0.0) {
        m_radius *= -1.0;
    }

    return true;
}

bool RArc::mirror(const RLine &axis)
{
    m_center.mirror(axis.getStartPoint(), axis.getEndPoint());

    if (isFullCircle()) {
        return true;
    }

    m_reversed = (!m_reversed);

    RVector v;
    v.setPolar(1.0, m_startAngle);
    v.mirror(RVector(0.0, 0.0), axis.getEndPoint() - axis.getStartPoint());
    m_startAngle = v.getAngle();

    v.setPolar(1.0, m_endAngle);
    v.mirror(RVector(0.0, 0.0), axis.getEndPoint() - axis.getStartPoint());
    m_endAngle = v.getAngle();

    return true;
}

bool RArc::reverse()
{
    double a = m_startAngle;
    m_startAngle = m_endAngle;
    m_endAngle = a;
    m_reversed = !m_reversed;
    return true;
}

bool RArc::stretch(const RPolyline &area, const RVector &offset)
{
    bool ret = false;

    if (area.contains(getStartPoint(), true) &&
        area.contains(getEndPoint(), true)) {
        return move(offset);
    }

    if (area.contains(getStartPoint(), true)) {
        moveStartPoint(getStartPoint() + offset);
        ret = true;
    }
    else if (area.contains(getEndPoint(), true)) {
        moveEndPoint(getEndPoint() + offset);
        ret = true;
    }

    return ret;
}

RS::Ending RArc::getTrimEnd(const RVector &trimPoint, const RVector &clickPoint)
{
    double angleToTrimPoint = m_center.getAngleTo(trimPoint);
    double angleToClickPoint = m_center.getAngleTo(clickPoint);

    if (RMath::getAngleDifference(angleToClickPoint, angleToTrimPoint) > M_PI) {
        if (m_reversed) {
            return RS::EndingEnd;
        }
        else {
            return RS::EndingStart;
        }
    }
    else {
        if (m_reversed) {
            return RS::EndingStart;
        }
        else {
            return RS::EndingEnd;
        }
    }
}

bool RArc::trimStartPoint(const RVector &trimPoint, const RVector &clickPoint,
                          bool extend)
{
    m_startAngle = m_center.getAngleTo(trimPoint);
    return true;
}

bool RArc::trimEndPoint(const RVector &trimPoint, const RVector &clickPoint,
                        bool extend)
{
    m_endAngle = m_center.getAngleTo(trimPoint);
    return true;
}

double RArc::getDistanceFromStart(const RVector &p) const
{
    double a1 = getStartAngle();
    double ap = m_center.getAngleTo(p);
    if (m_reversed) {
        return RMath::getAngleDifference(ap, a1) * m_radius;
    }
    else {
        return RMath::getAngleDifference(a1, ap) * m_radius;
    }
}

RPolyline RArc::approximateWithLines(double segmentLength, double angle) const
{
    RPolyline polyline;

    double aStep;
    if (segmentLength < RS::PointTolerance && angle > RS::PointTolerance) {
        aStep = angle;
    }
    else {
        // avoid a segment length of 0:
        if (segmentLength > 0.0 && segmentLength < 1.0e-6) {
            segmentLength = 1.0e-6;
        }
        if (segmentLength > 0.0) {
            aStep = segmentLength / m_radius;
        }
        else {
            // negative segment length: auto:
            aStep = 1.0;
        }
    }

    double a1 = getStartAngle();
    double a2 = getEndAngle();
    double a, cix, ciy;

    polyline.appendVertex(getStartPoint());
    if (!m_reversed) {
        // Arc Counterclockwise:
        if (a1 > a2 - 1.0e-10) {
            a2 += 2 * M_PI;
        }
        for (a = a1 + aStep; a <= a2; a += aStep) {
            cix = m_center.x + cos(a) * m_radius;
            ciy = m_center.y + sin(a) * m_radius;
            polyline.appendVertex(RVector(cix, ciy));
        }
    }
    else {
        // Arc Clockwise:
        if (a1 < a2 + 1.0e-10) {
            a2 -= 2 * M_PI;
        }
        for (a = a1 - aStep; a >= a2; a -= aStep) {
            cix = m_center.x + cos(a) * m_radius;
            ciy = m_center.y + sin(a) * m_radius;
            polyline.appendVertex(RVector(cix, ciy));
        }
    }
    polyline.appendVertex(getEndPoint());

    return polyline;
}

RPolyline RArc::approximateWithLinesTan(double segmentLength,
                                        double angle) const
{
    RPolyline polyline;

    double aStep;
    if (segmentLength < RS::PointTolerance && angle > RS::PointTolerance) {
        aStep = angle;
        double sw = fabs(getSweep());
        if (aStep > sw) {
            // make sure aStep is not too large for arc:
            aStep = sw / 2;
        }
    }
    else {
        // avoid a segment length of 0:
        if (segmentLength < 1.0e-6) {
            segmentLength = 1.0e-6;
        }

        // ideal angle step to satisfy segmentLength:
        aStep = segmentLength / m_radius;

        int steps = ceil(fabs(getSweep()) / aStep);
        // real angle step:
        aStep = fabs(getSweep()) / steps;
        if (fabs(cos(aStep / 2)) < RS::PointTolerance) {
            polyline.appendVertex(getStartPoint());
            polyline.appendVertex(getEndPoint());
            return polyline;
        }
    }

    double r2 = m_radius / cos(aStep / 2);

    double a1 = getStartAngle();
    double a2 = getEndAngle();

    double a, cix, ciy;

    polyline.appendVertex(getStartPoint());
    if (!m_reversed) {
        // Arc Counterclockwise:
        if (a1 > a2 - 1.0e-10) {
            a2 += 2 * M_PI;
        }
        for (a = a1 + aStep / 2; a < a2; a += aStep) {
            cix = m_center.x + cos(a) * r2;
            ciy = m_center.y + sin(a) * r2;
            polyline.appendVertex(RVector(cix, ciy));
        }
    }
    else {
        // Arc Clockwise:
        if (a1 < a2 + 1.0e-10) {
            a2 -= 2 * M_PI;
        }
        for (a = a1 - aStep / 2; a > a2; a -= aStep) {
            cix = m_center.x + cos(a) * r2;
            ciy = m_center.y + sin(a) * r2;
            polyline.appendVertex(RVector(cix, ciy));
        }
    }

    if (polyline.countVertices() == 1) {
        // only got start point, add point in the middle:
        a = getAngleAtPercent(0.5);
        cix = m_center.x + cos(a) * r2;
        ciy = m_center.y + sin(a) * r2;
        polyline.appendVertex(RVector(cix, ciy));
    }

    polyline.appendVertex(getEndPoint());

    return polyline;
}

std::vector<RLine> RArc::getTangents(const RVector &point) const
{
    RCircle circle(m_center, m_radius);
    return circle.getTangents(point);
}

std::vector<std::shared_ptr<RShape>>
RArc::splitAt(const std::vector<RVector> &points) const
{
    if (points.size() == 0) {
        return RShape::splitAt(points);
    }

    std::vector<std::shared_ptr<RShape>> ret;

    if (m_reversed) {
        RArc arc = *this;
        arc.reverse();
        ret = arc.splitAt(points);
        return RShapePrivate::getReversedShapeList(ret);
    }

    RVector startPoint = getStartPoint();
    RVector endPoint = getEndPoint();

    std::vector<RVector> sortedPoints =
        RVector::getSortedByAngle(points, m_center, getStartAngle());

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

        RArc *seg = clone();
        double a1 = m_center.getAngleTo(sortedPoints[i]);
        double a2 = m_center.getAngleTo(sortedPoints[i + 1]);
        if (fabs(RMath::getAngleDifference180(a1, a2) * m_radius) < 0.001) {
            continue;
        }
        seg->setStartAngle(a1);
        seg->setEndAngle(a2);
        ret.push_back(std::shared_ptr<RShape>(seg));
    }

    return ret;
}

std::vector<RArc> RArc::splitAtQuadrantLines() const
{
    std::vector<double> angles;
    angles.push_back(0.0);
    angles.push_back(M_PI / 2);
    angles.push_back(M_PI);
    angles.push_back(M_PI / 2 * 3);

    std::vector<RVector> points;
    for (int i = 0; i < angles.size(); i++) {
        if (isAngleWithinArc(angles[i])) {
            points.push_back(m_center +
                             RVector::createPolar(m_radius, angles[i]));
        }
    }

    std::vector<std::shared_ptr<RShape>> segments = splitAt(points);

    std::vector<RArc> ret;
    for (int i = 0; i < segments.size(); i++) {
        std::shared_ptr<RArc> seg =
            std::dynamic_pointer_cast<RArc>(segments[i]);
        ret.push_back(*seg);
    }
    return ret;
}