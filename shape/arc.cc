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


Arc::Arc()
    : center(Vec2d::invalid), radius(0.0), startAngle(0.0), endAngle(0.0),
      reversed(false)
{
}

Arc::Arc(double cx, double cy, double radius, double startAngle,
         double endAngle, bool reversed)
    : center(cx, cy), radius(radius), startAngle(startAngle),
      endAngle(endAngle), reversed(reversed)
{
}

Arc::Arc(const Vec2d &center, double radius, double startAngle, double endAngle,
         bool reversed)
    : center(center), radius(radius), startAngle(startAngle),
      endAngle(endAngle), reversed(reversed)
{
}

bool Arc::isValid() const
{
    return center.isValid() && radius > 0.0;
}

bool Arc::isFullCircle(double tolerance) const
{
    return fabs(Math::getAngleDifference180(
               Math::getNormalizedAngle(startAngle),
               Math::getNormalizedAngle(endAngle))) < tolerance;
}


Arc Arc::createFrom3Points(const Vec2d &startPoint, const Vec2d &point,
                           const Vec2d &endPoint)
{
    // intersection of two middle lines

    // middle points between first two points:
    Vec2d mp1 = Vec2d::getAverage(startPoint, point);
    double a1 = startPoint.getAngleTo(point) + M_PI / 2.0;
    // direction from middle point to center:
    Vec2d dir1 = Vec2d::createPolar(1.0, a1);

    // middle points between last two points:
    Vec2d mp2 = Vec2d::getAverage(point, endPoint);
    double a2 = point.getAngleTo(endPoint) + M_PI / 2.0;
    // direction from middle point to center:
    Vec2d dir2 = Vec2d::createPolar(1.0, a2);

    Line midLine1(mp1, mp1 + dir1);
    Line midLine2(mp2, mp2 + dir2);

    std::vector<Vec2d> ips = midLine1.getIntersectionPoints(midLine2, false);
    if (ips.size() != 1) {
        return Arc();
    }

    Vec2d center = ips[0];
    double radius = center.getDistanceTo(endPoint);
    double angle1 = center.getAngleTo(startPoint);
    double angle2 = center.getAngleTo(endPoint);
    bool reversed =
        Math::isAngleBetween(center.getAngleTo(point), angle1, angle2, true);

    return Arc(center, radius, angle1, angle2, reversed);
}

Arc Arc::createFrom2PBulge(const Vec2d &startPoint, const Vec2d &endPoint,
                           double bulge)
{

    Arc arc;

    arc.reversed = (bulge < 0.0);
    double alpha = atan(bulge) * 4.0;

    Vec2d middle = (startPoint + endPoint) / 2.0;
    double dist = startPoint.getDistanceTo(endPoint) / 2.0;

    // alpha can't be 0.0 at this point
    arc.radius = fabs(dist / sin(alpha / 2.0));

    double wu = fabs(Math::pow(arc.radius, 2.0) - Math::pow(dist, 2.0));
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

    arc.center.setPolar(h, angle);
    arc.center += middle;
    arc.startAngle = arc.center.getAngleTo(startPoint);
    arc.endAngle = arc.center.getAngleTo(endPoint);

    return arc;
}

Arc Arc::createTangential(const Vec2d &startPoint, const Vec2d &pos,
                          double direction, double radius)
{
    Arc arc;

    arc.radius = radius;

    // orthogonal to base entity:
    Vec2d ortho;
    ortho.setPolar(radius, direction + M_PI / 2.0);

    // two possible center points for arc:
    Vec2d center1 = startPoint + ortho;
    Vec2d center2 = startPoint - ortho;
    if (center1.getDistanceTo(pos) < center2.getDistanceTo(pos)) {
        arc.center = center1;
    }
    else {
        arc.center = center2;
    }

    // angles:
    arc.startAngle = arc.center.getAngleTo(startPoint);
    arc.endAngle = arc.center.getAngleTo(pos);

    // handle arc direction:
    arc.reversed = false;
    double diff = Math::getNormalizedAngle(arc.getDirection1() - direction);
    if (fabs(diff - M_PI) < 1.0e-1) {
        arc.reversed = true;
    }

    return arc;
}

std::vector<Arc> Arc::createBiarc(const Vec2d &startPoint,
                                  double startDirection, const Vec2d &endPoint,
                                  double endDirection, bool secondTry)
{

    double length = startPoint.getDistanceTo(endPoint);
    double angle = startPoint.getAngleTo(endPoint);

    double alpha = Math::getAngleDifference180(startDirection, angle);
    double beta = Math::getAngleDifference180(angle, endDirection);

    double theta;
    if ((alpha > 0 && beta > 0) || (alpha < 0 && beta < 0)) {
        // same sign: C-shaped curve:
        theta = alpha;
    }
    else {
        // different sign: S-shaped curve:
        theta = (3.0 * alpha - beta) / 2.0;
    }

    Vec2d startNormal(-sin(startDirection), cos(startDirection));
    Vec2d jointPointNormal(-sin(theta + startDirection),
                           cos(theta + startDirection));

    double term1 = (length / (2.0 * sin((alpha + beta) / 2.0)));

    double radius1 =
        term1 * (sin((beta - alpha + theta) / 2.0) / sin(theta / 2.0));
    double radius2 = term1 * (sin((2.0 * alpha - theta) / 2.0) /
                              sin((alpha + beta - theta) / 2.0));

    // failed, might succeed in reverse direction:
    if (qAbs(radius1) < NS::PointTolerance ||
        qAbs(radius2) < NS::PointTolerance || !Math::isNormal(radius1) ||
        !Math::isNormal(radius2)) {

        if (secondTry) {
            return std::vector<Arc>();
        }

        std::vector<Arc> list =
            Arc::createBiarc(endPoint, endDirection + M_PI, startPoint,
                             startDirection + M_PI, true);
        if (list.isEmpty()) {
            return std::vector<Arc>();
        }

        for (int i = 0; i < list.size(); i++) {
            list[i].reverse();
        }
        return std::vector<Arc>() << list[1] << list[0];
        //        return std::vector<Arc>();
    }

    Vec2d jointPoint = startPoint + radius1 * (startNormal - jointPointNormal);

    Vec2d center1 = startPoint + startNormal * radius1;
    Vec2d center2 = jointPoint + jointPointNormal * radius2;

    Arc arc1(center1, qAbs(radius1), center1.getAngleTo(startPoint),
             center1.getAngleTo(jointPoint));
    if (qAbs(Math::getAngleDifference180(arc1.getDirection1(),
                                         startDirection)) > 0.1) {
        arc1.setReversed(true);
    }

    Arc arc2(center2, qAbs(radius2), center2.getAngleTo(jointPoint),
             center2.getAngleTo(endPoint));
    if (qAbs(Math::getAngleDifference180(arc2.getDirection2() + M_PI,
                                         endDirection)) > 0.1) {
        arc2.setReversed(true);
    }

    return std::vector<Arc>() << arc1 << arc2;
}

double Arc::getDirection1() const
{
    if (!reversed) {
        return Math::getNormalizedAngle(startAngle + M_PI / 2.0);
    }
    else {
        return Math::getNormalizedAngle(startAngle - M_PI / 2.0);
    }
}

double Arc::getDirection2() const
{
    if (!reversed) {
        return Math::getNormalizedAngle(endAngle - M_PI / 2.0);
    }
    else {
        return Math::getNormalizedAngle(endAngle + M_PI / 2.0);
    }
}

NS::Side Arc::getSideOfPoint(const Vec2d &point) const
{
    if (reversed) {
        if (center.getDistanceTo(point) < radius) {
            return NS::RightHand;
        }
        else {
            return NS::LeftHand;
        }
    }
    else {
        if (center.getDistanceTo(point) < radius) {
            return NS::LeftHand;
        }
        else {
            return NS::RightHand;
        }
    }
}

void Arc::moveStartPoint(const Vec2d &pos, bool keepRadius)
{
    if (!keepRadius) {
        Arc a = Arc::createFrom3Points(pos, getMiddlePoint(), getEndPoint());
        if (a.isReversed() != isReversed()) {
            a.reverse();
        }
        *this = a;
    }
    else {
        double bulge = getBulge();

        // full circle: trim instead of move:
        if (bulge < 1.0e-6 || bulge > 1.0e6) {
            startAngle = center.getAngleTo(pos);
        }
        else {
            *this = Arc::createFrom2PBulge(pos, getEndPoint(), bulge);
        }
    }
}

void Arc::moveEndPoint(const Vec2d &pos, bool keepRadius)
{
    if (!keepRadius) {
        Arc a = Arc::createFrom3Points(pos, getMiddlePoint(), getStartPoint());
        if (a.isReversed() != isReversed()) {
            a.reverse();
        }
        *this = a;
    }
    else {
        double bulge = getBulge();

        // full circle: trim instead of move:
        if (bulge < 1.0e-6 || bulge > 1.0e6) {
            endAngle = center.getAngleTo(pos);
        }
        else {
            *this = Arc::createFrom2PBulge(getStartPoint(), pos, bulge);
        }
    }
}

void Arc::moveMiddlePoint(const Vec2d &pos)
{
    *this = Arc::createFrom3Points(getStartPoint(), pos, getEndPoint());
}

double Arc::getBulge() const
{
    // qDebug() << "sweep: " << getSweep();
    double bulge = tan(fabs(getSweep()) / 4.0);
    if (isReversed()) {
        bulge *= -1;
    }
    return bulge;
}

double Arc::getLength() const
{
    return fabs(getAngleLength(false)) * radius;
}

double Arc::getDiameter() const
{
    return 2 * radius;
}

void Arc::setDiameter(double d)
{
    radius = d / 2.0;
}

void Arc::setLength(double l)
{
    double sweep = l / radius;
    if (sweep > 2 * M_PI) {
        sweep = 2 * M_PI;
    }
    if (reversed) {
        sweep *= -1;
    }

    endAngle = startAngle + sweep;
}

double Arc::getArea() const
{
    return (radius * radius * getAngleLength(false)) / 2.0;
}

void Arc::setArea(double a)
{
    double sweep = (a * 2.0) / (radius * radius);
    if (reversed) {
        endAngle = Math::getNormalizedAngle(startAngle - sweep);
    }
    else {
        endAngle = Math::getNormalizedAngle(startAngle + sweep);
    }
}

double Arc::getChordArea() const
{
    double sectorArea = 0.0;
    double angleLength = getAngleLength(false);
    double sweep = getSweep();
    if (sweep < M_PI) {
        sectorArea =
            ((radius * radius) * (angleLength - sin(angleLength))) / 2.0;
    }
    else if (sweep == M_PI) {
        sectorArea = 0.5 * (M_PI * radius * radius);
    }
    else {
        double remainAngle = (M_PI * 2) - sweep;
        double remainSliceArea = (radius * radius * remainAngle) / 2.0;
        double remainSectorArea =
            (radius * radius * (remainAngle - sin(remainAngle))) / 2.0;
        sectorArea = getArea() + (remainSliceArea - remainSectorArea);
    }

    return sectorArea;
}

double Arc::getAngleLength(bool allowForZeroLength) const
{
    double ret = fabs(getSweep());

    // full circle or zero length arc:
    if (!allowForZeroLength) {
        if (ret < NS::PointTolerance) {
            ret = 2 * M_PI;
        }
    }
    else {
        if (ret > 2 * M_PI - NS::PointTolerance) {
            ret = 0.0;
        }
    }

    return ret;
}

double Arc::getSweep() const
{
    double ret = 0.0;

    if (reversed) {
        if (startAngle <= endAngle) {
            ret = -(startAngle + 2 * M_PI - endAngle);
        }
        else {
            ret = -(startAngle - endAngle);
        }
    }
    else {
        if (endAngle <= startAngle) {
            ret = endAngle + 2 * M_PI - startAngle;
        }
        else {
            ret = endAngle - startAngle;
        }
    }

    // full circle:
    //  if (!allowForZeroLength && fabs(ret) < 1.0e-6) {
    //      ret = 2 * M_PI;
    //  }

    return ret;
}

void Arc::setSweep(double s)
{
    endAngle = startAngle + s;
    reversed = (s < 0.0);
}

Vec2d Arc::getCenter() const
{
    return center;
}

void Arc::setCenter(const Vec2d &vector)
{
    center = vector;
}

double Arc::getRadius() const
{
    return radius;
}

void Arc::setRadius(double r)
{
    radius = r;
}

double Arc::getStartAngle() const
{
    return startAngle;
}

void Arc::setStartAngle(double a)
{
    startAngle = Math::getNormalizedAngle(a);
}

double Arc::getEndAngle() const
{
    return endAngle;
}

void Arc::setEndAngle(double a)
{
    endAngle = Math::getNormalizedAngle(a);
}

Vec2d Arc::getMiddlePoint() const
{
    double a;
    a = startAngle + getSweep() / 2.0;
    Vec2d v = Vec2d::createPolar(radius, a);
    v += center;
    return v;
}

Vec2d Arc::getStartPoint() const
{
    return getPointAtAngle(startAngle);
}

Vec2d Arc::getEndPoint() const
{
    return getPointAtAngle(endAngle);
}

Vec2d Arc::getPointAtAngle(double a) const
{
    return Vec2d(center.x + cos(a) * radius, center.y + sin(a) * radius,
                 center.z);
}

double Arc::getAngleAt(double distance, NS::From from) const
{
    std::vector<Vec2d> points = getPointsWithDistanceToEnd(distance, from);
    if (points.size() != 1) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return center.getAngleTo(points[0]) + (reversed ? -M_PI / 2 : M_PI / 2);
}

bool Arc::isReversed() const
{
    return reversed;
}

void Arc::setReversed(bool r)
{
    reversed = r;
}

BBox Arc::getBoundingBox() const
{
    if (!isValid()) {
        return BBox();
    }

    Vec2d minV;
    Vec2d maxV;
    double minX = qMin(getStartPoint().x, getEndPoint().x);
    double minY = qMin(getStartPoint().y, getEndPoint().y);
    double maxX = qMax(getStartPoint().x, getEndPoint().x);
    double maxY = qMax(getStartPoint().y, getEndPoint().y);

    if (getStartPoint().getDistanceTo(getEndPoint()) < 1.0e-6 &&
        getRadius() > 1.0e5) {
        minV = Vec2d(minX, minY);
        maxV = Vec2d(maxX, maxY);
        return BBox(minV, maxV);
    }

    double a1 = Math::getNormalizedAngle(!isReversed() ? startAngle : endAngle);
    double a2 = Math::getNormalizedAngle(!isReversed() ? endAngle : startAngle);

    // check for left limit:
    if ((a1 < M_PI && a2 > M_PI) || (a1 > a2 - 1.0e-12 && a2 > M_PI) ||
        (a1 > a2 - 1.0e-12 && a1 < M_PI)) {

        minX = qMin(center.x - radius, minX);
    }

    // check for right limit:
    if (a1 > a2 - 1.0e-12) {
        maxX = qMax(center.x + radius, maxX);
    }

    // check for bottom limit:
    if ((a1 < (M_PI_2 * 3) && a2 > (M_PI_2 * 3)) ||
        (a1 > a2 - 1.0e-12 && a2 > (M_PI_2 * 3)) ||
        (a1 > a2 - 1.0e-12 && a1 < (M_PI_2 * 3))) {

        minY = qMin(center.y - radius, minY);
    }

    // check for top limit:
    if ((a1 < M_PI_2 && a2 > M_PI_2) || (a1 > a2 - 1.0e-12 && a2 > M_PI_2) ||
        (a1 > a2 - 1.0e-12 && a1 < M_PI_2)) {

        maxY = qMax(center.y + radius, maxY);
    }

    minV = Vec2d(minX, minY);
    maxV = Vec2d(maxX, maxY);

    return BBox(minV, maxV);
}

std::vector<Vec2d> Arc::getEndPoints() const
{
    std::vector<Vec2d> ret;
    ret.push_back(getStartPoint());
    ret.push_back(getEndPoint());
    return ret;
}

std::vector<Vec2d> Arc::getMiddlePoints() const
{
    std::vector<Vec2d> ret;
    ret.push_back(getMiddlePoint());
    return ret;
}

std::vector<Vec2d> Arc::getCenterPoints() const
{
    std::vector<Vec2d> ret;
    ret.push_back(getCenter());
    return ret;
}

std::vector<Vec2d> Arc::getArcRefPoints() const
{
    std::vector<Vec2d> ret;

    std::vector<Vec2d> p;
    p.push_back(center + Vec2d(radius, 0));
    p.push_back(center + Vec2d(0, radius));
    p.push_back(center - Vec2d(radius, 0));
    p.push_back(center - Vec2d(0, radius));

    for (int i = 0; i < p.size(); i++) {
        if (Math::isAngleBetween(center.getAngleTo(p[i]), startAngle, endAngle,
                                 reversed)) {
            ret.push_back(p[i]);
        }
    }

    return ret;
}

std::vector<Vec2d> Arc::getPointsWithDistanceToEnd(double distance,
                                                   int from) const
{
    std::vector<Vec2d> ret;

    if (radius < NS::PointTolerance) {
        return ret;
    }

    double a1;
    double a2;
    Vec2d p;
    double aDist = distance / radius;

    if (isReversed()) {
        a1 = getStartAngle() - aDist;
        a2 = getEndAngle() + aDist;
    }
    else {
        a1 = getStartAngle() + aDist;
        a2 = getEndAngle() - aDist;
    }

    if (from & NS::FromStart) {
        p.setPolar(radius, a1);
        p += center;
        ret.push_back(p);
    }

    if (from & NS::FromEnd) {
        p.setPolar(radius, a2);
        p += center;
        ret.push_back(p);
    }

    return ret;
}

Vec2d Arc::getVectorTo(const Vec2d &point, bool limited,
                       double strictRange) const
{
    double angle = center.getAngleTo(point);
    if (limited &&
        !Math::isAngleBetween(angle, startAngle, endAngle, reversed)) {
        return Vec2d::invalid;
    }

    Vec2d v = (point - center).get2D();
    return Vec2d::createPolar(v.getMagnitude() - radius, v.getAngle());
}

bool Arc::move(const Vec2d &offset)
{
    if (!offset.isValid() || offset.getMagnitude() < NS::PointTolerance) {
        return false;
    }
    center += offset;
    return true;
}

bool Arc::rotate(double rotation, const Vec2d &c)
{
    if (fabs(rotation) < NS::AngleTolerance) {
        return false;
    }

    center.rotate(rotation, c);

    // important for circle shaped in hatch boundaries:
    if (!isFullCircle()) {
        startAngle = Math::getNormalizedAngle(startAngle + rotation);
        endAngle = Math::getNormalizedAngle(endAngle + rotation);
    }

    return true;
}

bool Arc::scale(const Vec2d &scaleFactors, const Vec2d &c)
{
    // negative scaling: mirroring and scaling
    if (scaleFactors.x < 0.0) {
        mirror(Line(center, center + Vec2d(0.0, 1.0)));
    }
    if (scaleFactors.y < 0.0) {
        mirror(Line(center, center + Vec2d(1.0, 0.0)));
    }

    center.scale(scaleFactors, c);
    radius *= scaleFactors.x;
    if (radius < 0.0) {
        radius *= -1.0;
    }

    return true;
}

bool Arc::mirror(const Line &axis)
{
    center.mirror(axis);

    if (isFullCircle()) {
        return true;
    }

    reversed = (!reversed);

    Vec2d v;
    v.setPolar(1.0, startAngle);
    v.mirror(Vec2d(0.0, 0.0), axis.endPoint - axis.startPoint);
    startAngle = v.getAngle();

    v.setPolar(1.0, endAngle);
    v.mirror(Vec2d(0.0, 0.0), axis.endPoint - axis.startPoint);
    endAngle = v.getAngle();

    return true;
}

bool Arc::reverse()
{
    double a = startAngle;
    startAngle = endAngle;
    endAngle = a;
    reversed = !reversed;
    return true;
}

bool Arc::stretch(const Polyline &area, const Vec2d &offset)
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

NS::Ending Arc::getTrimEnd(const Vec2d &trimPoint, const Vec2d &clickPoint)
{
    double angleToTrimPoint = center.getAngleTo(trimPoint);
    double angleToClickPoint = center.getAngleTo(clickPoint);

    if (Math::getAngleDifference(angleToClickPoint, angleToTrimPoint) > M_PI) {
        if (reversed) {
            return NS::EndingEnd;
        }
        else {
            return NS::EndingStart;
        }
    }
    else {
        if (reversed) {
            return NS::EndingStart;
        }
        else {
            return NS::EndingEnd;
        }
    }
}

bool Arc::trimStartPoint(const Vec2d &trimPoint, const Vec2d &clickPoint,
                         bool extend)
{
    startAngle = center.getAngleTo(trimPoint);
    return true;
}

bool Arc::trimEndPoint(const Vec2d &trimPoint, const Vec2d &clickPoint,
                       bool extend)
{
    endAngle = center.getAngleTo(trimPoint);
    return true;
}

double Arc::getDistanceFromStart(const Vec2d &p) const
{
    double a1 = getStartAngle();
    double ap = center.getAngleTo(p);
    if (reversed) {
        return Math::getAngleDifference(ap, a1) * radius;
    }
    else {
        return Math::getAngleDifference(a1, ap) * radius;
    }
}

Polyline Arc::approximateWithLines(double segmentLength, double angle) const
{
    Polyline polyline;

    double aStep;
    if (segmentLength < NS::PointTolerance && angle > NS::PointTolerance) {
        aStep = angle;
    }
    else {
        // avoid a segment length of 0:
        if (segmentLength > 0.0 && segmentLength < 1.0e-6) {
            segmentLength = 1.0e-6;
        }
        if (segmentLength > 0.0) {
            aStep = segmentLength / radius;
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
    if (!reversed) {
        // Arc Counterclockwise:
        if (a1 > a2 - 1.0e-10) {
            a2 += 2 * M_PI;
        }
        for (a = a1 + aStep; a <= a2; a += aStep) {
            cix = center.x + cos(a) * radius;
            ciy = center.y + sin(a) * radius;
            polyline.appendVertex(Vec2d(cix, ciy));
        }
    }
    else {
        // Arc Clockwise:
        if (a1 < a2 + 1.0e-10) {
            a2 -= 2 * M_PI;
        }
        for (a = a1 - aStep; a >= a2; a -= aStep) {
            cix = center.x + cos(a) * radius;
            ciy = center.y + sin(a) * radius;
            polyline.appendVertex(Vec2d(cix, ciy));
        }
    }
    polyline.appendVertex(getEndPoint());

    return polyline;
}

Polyline Arc::approximateWithLinesTan(double segmentLength, double angle) const
{
    Polyline polyline;

    double aStep;
    if (segmentLength < NS::PointTolerance && angle > NS::PointTolerance) {
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
        aStep = segmentLength / radius;

        int steps = ceil(fabs(getSweep()) / aStep);
        // real angle step:
        aStep = fabs(getSweep()) / steps;
        if (fabs(cos(aStep / 2)) < NS::PointTolerance) {
            qWarning() << "Arc::approximateWithLinesTan: segmentLength to "
                          "coarse to yield meaningful result";
            polyline.appendVertex(getStartPoint());
            polyline.appendVertex(getEndPoint());
            return polyline;
        }
    }

    double r2 = radius / cos(aStep / 2);

    double a1 = getStartAngle();
    double a2 = getEndAngle();

    double a, cix, ciy;

    polyline.appendVertex(getStartPoint());
    if (!reversed) {
        // Arc Counterclockwise:
        if (a1 > a2 - 1.0e-10) {
            a2 += 2 * M_PI;
        }
        for (a = a1 + aStep / 2; a < a2; a += aStep) {
            cix = center.x + cos(a) * r2;
            ciy = center.y + sin(a) * r2;
            polyline.appendVertex(Vec2d(cix, ciy));
        }
    }
    else {
        // Arc Clockwise:
        if (a1 < a2 + 1.0e-10) {
            a2 -= 2 * M_PI;
        }
        for (a = a1 - aStep / 2; a > a2; a -= aStep) {
            cix = center.x + cos(a) * r2;
            ciy = center.y + sin(a) * r2;
            polyline.appendVertex(Vec2d(cix, ciy));
        }
    }

    if (polyline.countVertices() == 1) {
        // only got start point, add point in the middle:
        a = getAngleAtPercent(0.5);
        cix = center.x + cos(a) * r2;
        ciy = center.y + sin(a) * r2;
        polyline.appendVertex(Vec2d(cix, ciy));
    }

    polyline.appendVertex(getEndPoint());

    return polyline;
}

std::vector<Line> Arc::getTangents(const Vec2d &point) const
{
    Circle circle(center, radius);
    return circle.getTangents(point);
}

std::vector<std::shared_ptr<Shape>>
Arc::splitAt(const std::vector<Vec2d> &points) const
{
    if (points.size() == 0) {
        return Shape::splitAt(points);
    }

    std::vector<std::shared_ptr<Shape>> ret;

    if (reversed) {
        Arc arc = *this;
        arc.reverse();
        ret = arc.splitAt(points);
        return Shape::getReversedShapeList(ret);
    }

    Vec2d startPoint = getStartPoint();
    Vec2d endPoint = getEndPoint();

    std::vector<Vec2d> sortedPoints =
        Vec2d::getSortedByAngle(points, center, getStartAngle());

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

        Arc *seg = clone();
        double a1 = center.getAngleTo(sortedPoints[i]);
        double a2 = center.getAngleTo(sortedPoints[i + 1]);
        if (fabs(Math::getAngleDifference180(a1, a2) * radius) < 0.001) {
            continue;
        }
        seg->setStartAngle(a1);
        seg->setEndAngle(a2);
        ret.push_back(std::shared_ptr<Shape>(seg));
    }

    return ret;
}

std::vector<Arc> Arc::splitAtQuadrantLines() const
{
    QVector<double> angles;
    angles.push_back(0.0);
    angles.push_back(M_PI / 2);
    angles.push_back(M_PI);
    angles.push_back(M_PI / 2 * 3);

    std::vector<Vec2d> points;
    for (int i = 0; i < angles.size(); i++) {
        if (isAngleWithinArc(angles[i])) {
            points.push_back(center + Vec2d::createPolar(radius, angles[i]));
        }
    }

    std::vector<std::shared_ptr<Shape>> segments = splitAt(points);

    std::vector<Arc> ret;
    for (int i = 0; i < segments.size(); i++) {
        std::shared_ptr<Arc> seg = segments[i].dynamicCast<Arc>();
        ret.push_back(*seg);
    }
    return ret;
}

} // namespace cada