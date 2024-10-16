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

Ellipse::Ellipse()
    : center(Vec2d::invalid), majorPoint(Vec2d::invalid), ratio(0.0),
      startParam(0.0), endParam(0.0), reversed(false)
{
}

Ellipse::Ellipse(const Vec2d &center, const Vec2d &majorPoint, double ratio,
                 double startParam, double endParam, bool reversed)
    : center(center), majorPoint(majorPoint), ratio(ratio),
      startParam(startParam), endParam(endParam), reversed(reversed)
{

    correctMajorMinor();
}

Ellipse::~Ellipse()
{
}

Ellipse Ellipse::createInscribed(const Vec2d &p1, const Vec2d &p2,
                                 const Vec2d &p3, const Vec2d &p4,
                                 const Vec2d &centerHint)
{
    Ellipse ret;

    return ret;
}

Ellipse Ellipse::createFrom4Points(const Vec2d &p1, const Vec2d &p2,
                                   const Vec2d &p3, const Vec2d &p4)
{
    Ellipse ret;

    return ret;
}

bool Ellipse::isValid() const
{
    return center.isValid() && majorPoint.isValid() && !Math::isNaN(ratio) &&
           !Math::isNaN(startParam) && !Math::isNaN(endParam);
}

std::vector<Vec2d> Ellipse::getFoci() const
{
    Vec2d vp(getMajorPoint() * sqrt(1.0 - getRatio() * getRatio()));
    return std::vector<Vec2d>{getCenter() + vp, getCenter() - vp};
}

double Ellipse::getAngleAtPoint(const Vec2d &pos) const
{
    Vec2d posNormalized = pos;
    posNormalized.move(-getCenter());
    posNormalized.rotate(-getAngle());

    double angle;
    if (Math::fuzzyCompare(posNormalized.y, 0.0)) {
        if (posNormalized.x > 0) {
            angle = M_PI / 2;
        }
        else {
            angle = M_PI / 2 * 3;
        }
    }
    else {
        double slope = -(pow(getMinorRadius() * 2, 2) * posNormalized.x) /
                       (pow(getMajorRadius() * 2, 2) * posNormalized.y);
        angle = atan(slope) + M_PI;
    }

    if (reversed) {
        angle += M_PI;
    }

    if (posNormalized.y < 0) {
        angle += M_PI;
    }

    angle += getAngle();

    return Math::getNormalizedAngle(angle);
}

// previously: getEllipseAngle
double Ellipse::getParamTo(const Vec2d &pos) const
{
    Vec2d m = pos;
    m.rotate(-majorPoint.getAngle(), center);
    Vec2d v = m - center;
    v.scale(Vec2d(1.0, 1.0 / ratio));
    return v.getAngle();
}

double Ellipse::getRadiusAt(double param) const
{
    Vec2d v(cos(param) * getMajorRadius(), sin(param) * getMinorRadius());
    return v.getMagnitude();
}

Vec2d Ellipse::getPointAt(double param) const
{
    Vec2d v(cos(param) * getMajorRadius(), sin(param) * getMinorRadius());
    v.rotate(getAngle());
    v.move(center);
    return v;
}

Vec2d Ellipse::getMiddlePoint() const
{
    double a;
    a = getStartParam() + getSweep() / 2.0;
    return getPointAt(a);
}

Vec2d Ellipse::getCenter() const
{
    return center;
}

void Ellipse::setCenter(const Vec2d &vector)
{
    center = vector;
}

Vec2d Ellipse::getMajorPoint() const
{
    return majorPoint;
}

void Ellipse::setMajorPoint(const Vec2d &p)
{
    majorPoint = p;
    correctMajorMinor();
}

Vec2d Ellipse::getMinorPoint() const
{
    double angle = Math::getNormalizedAngle(getAngle() + M_PI / 2.0);
    Vec2d ret;
    ret.setPolar(getMinorRadius(), angle);
    return ret;
}

void Ellipse::setMinorPoint(const Vec2d &p)
{
    double angle = Math::getNormalizedAngle(p.getAngle() - M_PI / 2.0);
    majorPoint.setPolar(getMajorRadius(), angle);
    setRatio(p.getMagnitude() / getMajorRadius());
}

bool Ellipse::switchMajorMinor()
{
    if (fabs(ratio) < NS::PointTolerance) {
        return false;
    }
    Vec2d vp_start = getStartPoint();
    Vec2d vp_end = getStartPoint();
    Vec2d vp = getMajorPoint();
    setMajorPoint(Vec2d(-ratio * vp.y, ratio * vp.x));
    setRatio(1.0 / ratio);
    setStartParam(getParamTo(vp_start));
    setEndParam(getParamTo(vp_end));
    return true;
}

double Ellipse::getRatio() const
{
    return ratio;
}

void Ellipse::setRatio(double r)
{
    ratio = r;
    correctMajorMinor();
}

double Ellipse::getStartParam() const
{
    return startParam;
}

void Ellipse::setStartParam(double a)
{
    startParam = a;
}

double Ellipse::getEndParam() const
{
    return endParam;
}

void Ellipse::setEndParam(double a)
{
    endParam = a;
}

double Ellipse::getStartAngle() const
{
    return Math::getNormalizedAngle(center.getAngleTo(getStartPoint()) -
                                    getAngle());
}

void Ellipse::setStartAngle(double a)
{
    double p = angleToParam(a);
    if (Math::isNaN(p)) {
        return;
    }
    startParam = p;
}

double Ellipse::getEndAngle() const
{
    return Math::getNormalizedAngle(center.getAngleTo(getEndPoint()) -
                                    getAngle());
}

void Ellipse::setEndAngle(double a)
{
    double p = angleToParam(a);
    if (Math::isNaN(p)) {
        return;
    }
    endParam = p;
}

double Ellipse::angleToParam(double a) const
{
    double p;
    if (fabs(a - 2 * M_PI) < NS::AngleTolerance) {
        p = 2 * M_PI;
    }
    else if (fabs(a) < NS::AngleTolerance) {
        p = 0.0;
    }
    else {
        Ellipse normEllipse = *this;
        normEllipse.move(-center);
        normEllipse.rotate(-getAngle());
        normEllipse.setStartParam(0.0);
        normEllipse.setEndParam(2 * M_PI);

        Line line(Vec2d(0, 0), Vec2d::createPolar(getMajorRadius() * 2, a));
        std::vector<Vec2d> r = line.getIntersectionPoints(normEllipse, true);
        if (r.size() != 1) {
            return std::numeric_limits<double>::quiet_NaN();
        }

        p = acos(r[0].x / getMajorRadius());
    }

    if (Math::getNormalizedAngle(a) > M_PI) {
        p = 2 * M_PI - p;
    }

    return p;
}

double Ellipse::getAngleLength(bool allowForZeroLength) const
{
    double ret = 0.0;

    if (isReversed()) {
        if (startParam < endParam) {
            ret = startParam + 2 * M_PI - endParam;
        }
        else {
            ret = startParam - endParam;
        }
    }
    else {
        if (endParam < startParam) {
            ret = endParam + 2 * M_PI - startParam;
        }
        else {
            ret = endParam - startParam;
        }
    }

    // full ellipse or zero length ellipse arc:
    if (!allowForZeroLength) {
        if (fabs(ret) < NS::AngleTolerance) {
            ret = 2 * M_PI;
        }
    }
    else {
        if (ret > 2 * M_PI - NS::AngleTolerance) {
            ret = 0.0;
        }
    }

    return ret;
}

Vec2d Ellipse::getStartPoint() const
{
    Vec2d p(center.x + cos(startParam) * getMajorRadius(),
            center.y + sin(startParam) * getMinorRadius());
    p.rotate(getAngle(), center);
    return p;
}

Vec2d Ellipse::getEndPoint() const
{
    Vec2d p(center.x + cos(endParam) * getMajorRadius(),
            center.y + sin(endParam) * getMinorRadius());
    p.rotate(getAngle(), center);
    return p;
}

double Ellipse::getMajorRadius() const
{
    return majorPoint.getMagnitude();
}

double Ellipse::getMinorRadius() const
{
    return majorPoint.getMagnitude() * ratio;
}

double Ellipse::getAngle() const
{
    return majorPoint.getAngle();
}

void Ellipse::setAngle(double a)
{
    majorPoint = Vec2d::createPolar(majorPoint.getMagnitude(), a);
}

bool Ellipse::isFullEllipse() const
{
    double a1 = Math::getNormalizedAngle(startParam);
    double a2 = Math::getNormalizedAngle(endParam);
    return (a1 < NS::AngleTolerance && a2 > 2 * M_PI - NS::AngleTolerance) ||
           (fabs(a1 - a2) < NS::AngleTolerance);
}

bool Ellipse::isCircular() const
{
    return getRatio() > (1.0 - 0.001);
}

double Ellipse::getLength() const
{
    double a1, a2;

    if (isFullEllipse()) {
        a1 = 0.0;
        a2 = 2 * M_PI;

        double a = getMajorRadius();
        double b = getMinorRadius();
        if (Math::fuzzyCompare((a + b), 0.0)) {
            return 0.0;
        }
        double h = pow((a - b) / (a + b), 2);

        return M_PI * (a + b) *
               ((135168 - 85760 * h - 5568 * h * h + 3867 * h * h * h) /
                (135168 - 119552 * h + 22208 * h * h - 345 * h * h * h));
    }
    else {
        a1 = Math::getNormalizedAngle(startParam);
        a2 = Math::getNormalizedAngle(endParam);
    }

    if (reversed) {
        double t = a1;
        a1 = a2;
        a2 = t;
    }

    if (Math::fuzzyCompare(a2, 0.0)) {
        a2 = 2 * M_PI;
    }

    if (fabs(a1 - a2) < NS::AngleTolerance) {
        return 0.0;
    }

    if (a1 < a2) {
        if (a1 < M_PI && a2 <= M_PI) {
            return getSimpsonLength(a1, a2);
        }
        if (a1 < M_PI && a2 > M_PI) {
            return getSimpsonLength(a1, M_PI) + getSimpsonLength(M_PI, a2);
        }
        if (a1 >= M_PI && a2 > M_PI) {
            return getSimpsonLength(a1, a2);
        }
    }
    else {
        if (a1 > M_PI && a2 < M_PI) {
            return getSimpsonLength(a1, 2 * M_PI) + getSimpsonLength(0, a2);
        }
        if (a1 > M_PI && a2 > M_PI) {
            return getSimpsonLength(a1, 2 * M_PI) + getSimpsonLength(0, M_PI) +
                   getSimpsonLength(M_PI, a2);
        }
        if (a1 < M_PI && a2 < M_PI) {
            return getSimpsonLength(a1, M_PI) +
                   getSimpsonLength(M_PI, 2 * M_PI) + getSimpsonLength(0, a2);
        }
    }

    return std::numeric_limits<double>::quiet_NaN();
}

double Ellipse::getSimpsonLength(double a1, double a2) const
{
    int interval = 20;
    double df = (a2 - a1) / interval;
    double majorR = getMajorRadius();
    double minorR = getMinorRadius();

    double sum = 0.0;
    double q = 1.0;

    for (int i = 0; i <= interval; ++i) {
        double y = sqrt(::pow(majorR * sin(a1 + i * df), 2) +
                        ::pow(minorR * cos(a1 + i * df), 2));
        if (i == 0 || i == interval) {
            q = 1.0;
        }
        else {
            if (i % 2 == 0) {
                q = 2.0;
            }
            else {
                q = 4.0;
            }
        }

        sum += q * y;
    }

    return (df / 3.0) * sum;
}

bool Ellipse::contains(const Vec2d &p) const
{
    Vec2d pt = p;
    pt.move(-center);
    pt.rotate(-getAngle());
    double rx = getMajorRadius();
    double ry = getMinorRadius();
    return (pt.x * pt.x) / (rx * rx) + (pt.y * pt.y) / (ry * ry) <= 1.0;
}

// depends on implementation of getPointsWithDistanceToEnd:
// double Ellipse::getAngleAt(double distance, NS::From from) const {
//    Ellipse normal = *this;
//    normal.rotate(-getAngle());

//    std::vector<Vec2d> points = normal.getPointsWithDistanceToEnd(distance,
//    from); if (points.size()!=1) {
//        return std::numeric_limits<double>::quiet_NaN();
//    }

//    Vec2d p = points[0];

//    double minR = normal.getMinorRadius();
//    double majR = normal.getMajorRadius();

//    double ret = - ((minR*minR*p.x) / (majR*majR*p.y));
//    ret+=getAngle();
//    return ret;
//}

bool Ellipse::isReversed() const
{
    return reversed;
}

void Ellipse::setReversed(bool r)
{
    reversed = r;
}

std::vector<Vec2d> Ellipse::getEndPoints() const
{
    std::vector<Vec2d> ret;
    ret.push_back(getStartPoint());
    ret.push_back(getEndPoint());
    return ret;
}

std::vector<Vec2d> Ellipse::getMiddlePoints() const
{
    std::vector<Vec2d> ret;
    // ret.push_back(getMiddlePoint());
    return ret;
}

std::vector<Vec2d> Ellipse::getCenterPoints() const
{
    std::vector<Vec2d> ret;
    ret.push_back(getCenter());
    return ret;
}

std::vector<Vec2d> Ellipse::getBoxCorners()
{
    std::vector<Vec2d> ret;

    Vec2d minorPoint = getMinorPoint();
    ret.push_back(center + majorPoint + minorPoint);
    ret.push_back(center + majorPoint - minorPoint);
    ret.push_back(center - majorPoint - minorPoint);
    ret.push_back(center - majorPoint + minorPoint);

    return ret;
}

void Ellipse::correctMajorMinor()
{
    if (ratio > 1.0) {
        Vec2d mp = getMinorPoint();
        ratio = 1.0 / ratio;
        setMajorPoint(mp);
        startParam = Math::getNormalizedAngle(startParam - M_PI / 2.0);
        endParam = Math::getNormalizedAngle(endParam - M_PI / 2.0);
    }
}

double Ellipse::getSweep() const
{
    double ret = 0.0;

    if (reversed) {
        if (startParam <= endParam) {
            ret = -(startParam + 2 * M_PI - endParam);
        }
        else {
            ret = -(startParam - endParam);
        }
    }
    else {
        if (endParam <= startParam) {
            ret = endParam + 2 * M_PI - startParam;
        }
        else {
            ret = endParam - startParam;
        }
    }

    return ret;
}

std::vector<Line> Ellipse::getTangents(const Vec2d &point) const
{
    std::vector<Line> ret;

    if (getDistanceTo(point, false) < NS::PointTolerance) {
        // point is on ellipse:
        return ret;
    }

    // point is at center (prevents recursion when swapping ellipse minor /
    // major):
    if (point.getDistanceTo(getCenter()) < NS::PointTolerance) {
        return ret;
    }

    // swap ellipse minor / major if point is on minor axis
    // 20120928: and not also on major axis (prevent recursion):
    Line minorAxis(getCenter(), getCenter() + getMinorPoint());
    Line majorAxis(getCenter(), getCenter() + getMajorPoint());
    if (minorAxis.isOnShape(point, false) &&
        !majorAxis.isOnShape(point, false)) {
        Ellipse e2 = *this;
        e2.majorPoint = getMinorPoint();
        e2.ratio = 1.0 / ratio;
        return e2.getTangents(point);
    }

    double a = getMajorRadius(); // the length of the major axis / 2
    double b = getMinorRadius(); // the length of the minor axis / 2

    // rotate and move point:
    Vec2d point2 = point;
    point2.move(-getCenter());
    point2.rotate(-getAngle());

    double xp = point2.x; // coordinates of the given point
    double yp = point2.y;

    double xt1; // Tangent point 1
    double yt1;
    double xt2; // Tangent point 2
    double yt2;

    double a2 = a * a;
    double b2 = b * b;
    double d = a2 / b2 * yp / xp;
    double e = a2 / xp;
    double af = b2 * d * d + a2;
    double bf = -b2 * d * e * 2.0;
    double cf = b2 * e * e - a2 * b2;
    double t = sqrt(bf * bf - af * cf * 4.0);
    if (Math::isNaN(t)) {
        return ret;
    }

    yt1 = (t - bf) / (af * 2.0);
    xt1 = e - d * yt1;
    yt2 = (-t - bf) / (af * 2.0);
    xt2 = e - d * yt2;

    Vec2d s1(xt1, yt1);
    s1.rotate(getAngle());
    s1.move(getCenter());

    Vec2d s2(xt2, yt2);
    s2.rotate(getAngle());
    s2.move(getCenter());

    if (s1.isValid()) {
        ret.push_back(Line(point, s1));
    }

    if (s2.isValid()) {
        ret.push_back(Line(point, s2));
    }

    return ret;
}

Vec2d Ellipse::getTangentPoint(const Line &line) const
{
    Line lineNeutral = line;

    // translate line to ellipse's center
    lineNeutral.move(getCenter().getNegated());

    // rotate line points (inverse rotation of the ellipse)
    lineNeutral.rotate(-getAngle());

    if (lineNeutral.isVertical()) {
        // for vertical line, check if it passes through ellipse's major axis
        if (Math::fuzzyCompare(lineNeutral.getStartPoint().x,
                               getMajorRadius())) {
            return getCenter() + getMajorPoint();
        }

        if (Math::fuzzyCompare(lineNeutral.getStartPoint().x,
                               -getMajorRadius())) {
            return getCenter() - getMajorPoint();
        }

        return Vec2d::invalid;
    }

    double m = (lineNeutral.getEndPoint().y - lineNeutral.getStartPoint().y) /
               (lineNeutral.getEndPoint().x - lineNeutral.getStartPoint().x);

    // y-intersept:
    double c =
        lineNeutral.getStartPoint().y - m * lineNeutral.getStartPoint().x;

    double a = getMajorRadius();
    double b = getMinorRadius();

    double A = (b * b) + (a * a * m * m);
    double B = 2 * a * a * m * c;
    double C = (a * a * c * c) - (a * a * b * b);

    double discriminant = B * B - 4 * A * C;

    // for a tangent line, discriminant should be zero (one real root):
    if (Math::fuzzyCompare(discriminant, 0.0, 0.001)) {
        double x = -B / (2 * A);
        double y = m * x + c;

        Vec2d ret = Vec2d(x, y);
        ret.rotate(getAngle());
        ret.move(getCenter());
        return ret;
    }

    return Vec2d::invalid;
}

std::vector<BSpline> Ellipse::approximateWithSplines() const
{
    return std::vector<BSpline>();
}

Polyline Ellipse::approximateWithArcs(int segments) const
{
    return Polyline();
}

} // namespace cada