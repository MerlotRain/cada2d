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
#include <cmath>

namespace cada {
namespace shape {

Arc::Arc()
    : mCenter(Vec2d::invalid), mRadius(0.0), mStartAngle(0.0), mEndAngle(0.0),
      mReversed(false)
{
}

Arc::Arc(double cx, double cy, double radius, double startAngle,
         double endAngle, bool reversed)
    : mCenter(cx, cy), mRadius(radius), mStartAngle(startAngle),
      mEndAngle(endAngle), mReversed(reversed)
{
}

Arc::Arc(const Vec2d &center, double radius, double startAngle, double endAngle,
         bool reversed)
    : mCenter(center), mRadius(radius), mStartAngle(startAngle),
      mEndAngle(endAngle), mReversed(reversed)
{
}

bool Arc::isValid() const
{
    return mCenter.isValid() && mRadius > 0.0;
}

NS::ShapeType Arc::getShapeType() const
{
    return NS::Arc;
}

Shape* Arc::clone() const
{
    Arc *pClone = new Arc();
    pClone->mCenter = mCenter;
    pClone->mRadius = mRadius;
    pClone->mStartAngle = mStartAngle;
    pClone->mEndAngle = mEndAngle;
    pClone->mReversed = mReversed;
    return pClone;
}

bool Arc::isFullCircle(double tolerance) const
{
    return fabs(Math::getAngleDifference180(
               Math::getNormalizedAngle(mStartAngle),
               Math::getNormalizedAngle(mEndAngle))) < tolerance;
}

bool Arc::isAngleWithinArc(double a) const
{
    return false;
}

Arc Arc::createFrom3Points(const Vec2d &startPoint, const Vec2d &point,
                           const Vec2d &endPoint)
{
    Vec2d mp1 = Vec2d::getAverage(startPoint, point);
    double a1 = startPoint.getAngleTo(point) + M_PI / 2.0;
    Vec2d dir1 = Vec2d::createPolar(1.0, a1);

    Vec2d mp2 = Vec2d::getAverage(point, endPoint);
    double a2 = point.getAngleTo(endPoint) + M_PI / 2.0;
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

    arc.mReversed = (bulge < 0.0);
    double alpha = atan(bulge) * 4.0;

    Vec2d middle = (startPoint + endPoint) / 2.0;
    double dist = startPoint.getDistanceTo(endPoint) / 2.0;

    // alpha can't be 0.0 at this point
    arc.mRadius = fabs(dist / sin(alpha / 2.0));

    double wu = fabs(std::pow(arc.mRadius, 2.0) - std::pow(dist, 2.0));
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

    arc.mCenter.setPolar(h, angle);
    arc.mCenter += middle;
    arc.mStartAngle = arc.mCenter.getAngleTo(startPoint);
    arc.mEndAngle = arc.mCenter.getAngleTo(endPoint);

    return arc;
}

Arc Arc::createTangential(const Vec2d &startPoint, const Vec2d &pos,
                          double direction, double radius)
{
    Arc arc;

    arc.mRadius = radius;
    Vec2d ortho;
    ortho.setPolar(radius, direction + M_PI / 2.0);

    Vec2d center1 = startPoint + ortho;
    Vec2d center2 = startPoint - ortho;
    if (center1.getDistanceTo(pos) < center2.getDistanceTo(pos)) {
        arc.mCenter = center1;
    }
    else {
        arc.mCenter = center2;
    }

    // angles:
    arc.mStartAngle = arc.mCenter.getAngleTo(startPoint);
    arc.mEndAngle = arc.mCenter.getAngleTo(pos);

    // handle arc direction:
    arc.mReversed = false;
    double diff = Math::getNormalizedAngle(arc.getDirection1() - direction);
    if (fabs(diff - M_PI) < 1.0e-1) {
        arc.mReversed = true;
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
    if (std::fabs(radius1) < NS::PointTolerance ||
        std::fabs(radius2) < NS::PointTolerance || !Math::isNormal(radius1) ||
        !Math::isNormal(radius2)) {

        if (secondTry) {
            return std::vector<Arc>();
        }

        std::vector<Arc> list =
            Arc::createBiarc(endPoint, endDirection + M_PI, startPoint,
                             startDirection + M_PI, true);
        if (list.empty()) {
            return std::vector<Arc>();
        }

        for (size_t i = 0; i < list.size(); i++) {
            list[i].reverse();
        }
        return std::vector<Arc>{list[1], list[0]};
    }

    Vec2d jointPoint = startPoint + (startNormal - jointPointNormal) * radius1;

    Vec2d center1 = startPoint + startNormal * radius1;
    Vec2d center2 = jointPoint + jointPointNormal * radius2;

    Arc arc1(center1, std::fabs(radius1), center1.getAngleTo(startPoint),
             center1.getAngleTo(jointPoint));
    if (std::fabs(Math::getAngleDifference180(arc1.getDirection1(),
                                              startDirection)) > 0.1) {
        arc1.setReversed(true);
    }

    Arc arc2(center2, std::fabs(radius2), center2.getAngleTo(jointPoint),
             center2.getAngleTo(endPoint));
    if (std::fabs(Math::getAngleDifference180(arc2.getDirection2() + M_PI,
                                              endDirection)) > 0.1) {
        arc2.setReversed(true);
    }

    return std::vector<Arc>{arc1, arc2};
}

double Arc::getBulge() const
{
    double bulge = tan(fabs(getSweep()) / 4.0);
    if (isReversed()) {
        bulge *= -1;
    }
    return bulge;
}

double Arc::getDiameter() const
{
    return 2 * mRadius;
}

void Arc::setDiameter(double d)
{
    mRadius = d / 2.0;
}

void Arc::setLength(double l)
{
    double sweep = l / mRadius;
    if (sweep > 2 * M_PI) {
        sweep = 2 * M_PI;
    }
    if (mReversed) {
        sweep *= -1;
    }

    mEndAngle = mStartAngle + sweep;
}

double Arc::getArea() const
{
    return (mRadius * mRadius * getAngleLength(false)) / 2.0;
}

void Arc::setArea(double a)
{
    double sweep = (a * 2.0) / (mRadius * mRadius);
    if (mReversed) {
        mEndAngle = Math::getNormalizedAngle(mStartAngle - sweep);
    }
    else {
        mEndAngle = Math::getNormalizedAngle(mStartAngle + sweep);
    }
}

double Arc::getChordArea() const
{
    double sectorArea = 0.0;
    double angleLength = getAngleLength(false);
    double sweep = getSweep();
    if (sweep < M_PI) {
        sectorArea =
            ((mRadius * mRadius) * (angleLength - sin(angleLength))) / 2.0;
    }
    else if (sweep == M_PI) {
        sectorArea = 0.5 * (M_PI * mRadius * mRadius);
    }
    else {
        double remainAngle = (M_PI * 2) - sweep;
        double remainSliceArea = (mRadius * mRadius * remainAngle) / 2.0;
        double remainSectorArea =
            (mRadius * mRadius * (remainAngle - sin(remainAngle))) / 2.0;
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

    if (mReversed) {
        if (mStartAngle <= mEndAngle) {
            ret = -(mStartAngle + 2 * M_PI - mEndAngle);
        }
        else {
            ret = -(mStartAngle - mEndAngle);
        }
    }
    else {
        if (mEndAngle <= mStartAngle) {
            ret = mEndAngle + 2 * M_PI - mStartAngle;
        }
        else {
            ret = mEndAngle - mStartAngle;
        }
    }

    return ret;
}

void Arc::setSweep(double s)
{
    mEndAngle = mStartAngle + s;
    mReversed = (s < 0.0);
}

Vec2d Arc::getCenter() const
{
    return mCenter;
}

void Arc::setCenter(const Vec2d &vector)
{
    mCenter = vector;
}

double Arc::getRadius() const
{
    return mRadius;
}

void Arc::setRadius(double r)
{
    mRadius = r;
}

double Arc::getStartAngle() const
{
    return mStartAngle;
}

void Arc::setStartAngle(double a)
{
    mStartAngle = Math::getNormalizedAngle(a);
}

double Arc::getEndAngle() const
{
    return mEndAngle;
}

void Arc::setEndAngle(double a)
{
    mEndAngle = Math::getNormalizedAngle(a);
}

Vec2d Arc::getMiddlePoint() const
{
    double a;
    a = mStartAngle + getSweep() / 2.0;
    Vec2d v = Vec2d::createPolar(mRadius, a);
    v += mCenter;
    return v;
}

Vec2d Arc::getStartPoint() const
{
    return getPointAtAngle(mStartAngle);
}

Vec2d Arc::getEndPoint() const
{
    return getPointAtAngle(mEndAngle);
}

Vec2d Arc::getPointAtAngle(double a) const
{
    return Vec2d(mCenter.x + cos(a) * mRadius, mCenter.y + sin(a) * mRadius);
}

bool Arc::isReversed() const
{
    return mReversed;
}

void Arc::setReversed(bool r)
{
    mReversed = r;
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
    p.push_back(mCenter + Vec2d(mRadius, 0));
    p.push_back(mCenter + Vec2d(0, mRadius));
    p.push_back(mCenter - Vec2d(mRadius, 0));
    p.push_back(mCenter - Vec2d(0, mRadius));

    for (size_t i = 0; i < p.size(); i++) {
        if (Math::isAngleBetween(mCenter.getAngleTo(p[i]), mStartAngle,
                                 mEndAngle, mReversed)) {
            ret.push_back(p[i]);
        }
    }

    return ret;
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
            aStep = segmentLength / mRadius;
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
    if (!mReversed) {
        // Arc Counterclockwise:
        if (a1 > a2 - 1.0e-10) {
            a2 += 2 * M_PI;
        }
        for (a = a1 + aStep; a <= a2; a += aStep) {
            cix = mCenter.x + cos(a) * mRadius;
            ciy = mCenter.y + sin(a) * mRadius;
            polyline.appendVertex(Vec2d(cix, ciy));
        }
    }
    else {
        // Arc Clockwise:
        if (a1 < a2 + 1.0e-10) {
            a2 -= 2 * M_PI;
        }
        for (a = a1 - aStep; a >= a2; a -= aStep) {
            cix = mCenter.x + cos(a) * mRadius;
            ciy = mCenter.y + sin(a) * mRadius;
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
        aStep = segmentLength / mRadius;

        int steps = ceil(fabs(getSweep()) / aStep);
        // real angle step:
        aStep = fabs(getSweep()) / steps;
        if (fabs(cos(aStep / 2)) < NS::PointTolerance) {
            polyline.appendVertex(getStartPoint());
            polyline.appendVertex(getEndPoint());
            return polyline;
        }
    }

    double r2 = mRadius / cos(aStep / 2);

    double a1 = getStartAngle();
    double a2 = getEndAngle();

    double a, cix, ciy;

    polyline.appendVertex(getStartPoint());
    if (!mReversed) {
        // Arc Counterclockwise:
        if (a1 > a2 - 1.0e-10) {
            a2 += 2 * M_PI;
        }
        for (a = a1 + aStep / 2; a < a2; a += aStep) {
            cix = mCenter.x + cos(a) * r2;
            ciy = mCenter.y + sin(a) * r2;
            polyline.appendVertex(Vec2d(cix, ciy));
        }
    }
    else {
        // Arc Clockwise:
        if (a1 < a2 + 1.0e-10) {
            a2 -= 2 * M_PI;
        }
        for (a = a1 - aStep / 2; a > a2; a -= aStep) {
            cix = mCenter.x + cos(a) * r2;
            ciy = mCenter.y + sin(a) * r2;
            polyline.appendVertex(Vec2d(cix, ciy));
        }
    }

    if (polyline.countVertices() == 1) {
        // only got start point, add point in the middle:
        a = getAngleAtPercent(0.5);
        cix = mCenter.x + cos(a) * r2;
        ciy = mCenter.y + sin(a) * r2;
        polyline.appendVertex(Vec2d(cix, ciy));
    }

    polyline.appendVertex(getEndPoint());

    return polyline;
}

std::vector<Line> Arc::getTangents(const Vec2d &point) const
{
    Circle circle(mCenter, mRadius);
    return circle.getTangents(point);
}

std::vector<Arc> Arc::splitAtQuadrantLines() const
{
    std::vector<double> angles;
    angles.push_back(0.0);
    angles.push_back(M_PI / 2);
    angles.push_back(M_PI);
    angles.push_back(M_PI / 2 * 3);

    std::vector<Vec2d> points;
    for (size_t i = 0; i < angles.size(); i++) {
        if (isAngleWithinArc(angles[i])) {
            points.push_back(mCenter + Vec2d::createPolar(mRadius, angles[i]));
        }
    }

    std::vector<Shape *> segments = splitAt(points);

    std::vector<Arc> ret;
    for (size_t i = 0; i < segments.size(); i++) {
        Arc *seg = dynamic_cast<Arc *>(segments[i]);
        ret.push_back(*seg);
    }
    return ret;
}

} // namespace shape
} // namespace cada