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
namespace shape {

Ellipse::Ellipse()
    : mCenter(Vec2d::invalid), mMajorPoint(Vec2d::invalid), mRatio(0.0),
      mStartParam(0.0), mEndParam(0.0), mReversed(false)
{
}

Ellipse::Ellipse(const Vec2d &center, const Vec2d &majorPoint, double ratio,
                 double startParam, double endParam, bool reversed)
    : mCenter(center), mMajorPoint(majorPoint), mRatio(ratio),
      mStartParam(startParam), mEndParam(endParam), mReversed(reversed)
{

    correctMajorMinor();
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
    return mCenter.isValid() && mMajorPoint.isValid() && !Math::isNaN(mRatio) &&
           !Math::isNaN(mStartParam) && !Math::isNaN(mEndParam);
}

NS::ShapeType Ellipse::getShapeType() const
{
    return NS::ShapeType();
}

Shape* Ellipse::clone() const
{
    return nullptr;
}

std::vector<Vec2d> Ellipse::getFoci() const
{
    Vec2d vp(getMajorPoint() * sqrt(1.0 - getRatio() * getRatio()));
    return std::vector<Vec2d>{getCenter() + vp, getCenter() - vp};
}

double Ellipse::getParamTo(const Vec2d &pos) const
{
    Vec2d m = pos;
    m.rotate(-mMajorPoint.getAngle(), mCenter);
    Vec2d v = m - mCenter;
    v.scale(Vec2d(1.0, 1.0 / mRatio));
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
    v.move(mCenter);
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
    return mCenter;
}

void Ellipse::setCenter(const Vec2d &vector)
{
    mCenter = vector;
}

Vec2d Ellipse::getMajorPoint() const
{
    return mMajorPoint;
}

void Ellipse::setMajorPoint(const Vec2d &p)
{
    mMajorPoint = p;
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
    mMajorPoint.setPolar(getMajorRadius(), angle);
    setRatio(p.getMagnitude() / getMajorRadius());
}

bool Ellipse::switchMajorMinor()
{
    if (fabs(mRatio) < NS::PointTolerance) {
        return false;
    }
    Vec2d vp_start = getStartPoint();
    Vec2d vp_end = getStartPoint();
    Vec2d vp = getMajorPoint();
    setMajorPoint(Vec2d(-mRatio * vp.y, mRatio * vp.x));
    setRatio(1.0 / mRatio);
    setStartParam(getParamTo(vp_start));
    setEndParam(getParamTo(vp_end));
    return true;
}

double Ellipse::getRatio() const
{
    return mRatio;
}

void Ellipse::setRatio(double r)
{
    mRatio = r;
    correctMajorMinor();
}

double Ellipse::getStartParam() const
{
    return mStartParam;
}

void Ellipse::setStartParam(double a)
{
    mStartParam = a;
}

double Ellipse::getEndParam() const
{
    return mEndParam;
}

void Ellipse::setEndParam(double a)
{
    mEndParam = a;
}

double Ellipse::getStartAngle() const
{
    return Math::getNormalizedAngle(mCenter.getAngleTo(getStartPoint()) -
                                    getAngle());
}

void Ellipse::setStartAngle(double a)
{
    double p = angleToParam(a);
    if (Math::isNaN(p)) {
        return;
    }
    mStartParam = p;
}

double Ellipse::getEndAngle() const
{
    return Math::getNormalizedAngle(mCenter.getAngleTo(getEndPoint()) -
                                    getAngle());
}

void Ellipse::setEndAngle(double a)
{
    double p = angleToParam(a);
    if (Math::isNaN(p)) {
        return;
    }
    mEndParam = p;
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
        normEllipse.move(-mCenter);
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
        if (mStartParam < mEndParam) {
            ret = mStartParam + 2 * M_PI - mEndParam;
        }
        else {
            ret = mStartParam - mEndParam;
        }
    }
    else {
        if (mEndParam < mStartParam) {
            ret = mEndParam + 2 * M_PI - mStartParam;
        }
        else {
            ret = mEndParam - mStartParam;
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
    Vec2d p(mCenter.x + cos(mStartParam) * getMajorRadius(),
            mCenter.y + sin(mStartParam) * getMinorRadius());
    p.rotate(getAngle(), mCenter);
    return p;
}

Vec2d Ellipse::getEndPoint() const
{
    Vec2d p(mCenter.x + cos(mEndParam) * getMajorRadius(),
            mCenter.y + sin(mEndParam) * getMinorRadius());
    p.rotate(getAngle(), mCenter);
    return p;
}

bool Ellipse::isAngleWithinArc(double a) const
{
    return false;
}

bool Ellipse::isParamWithinArc(double a) const
{
    return false;
}

double Ellipse::getMajorRadius() const
{
    return mMajorPoint.getMagnitude();
}

double Ellipse::getMinorRadius() const
{
    return mMajorPoint.getMagnitude() * mRatio;
}

double Ellipse::getAngle() const
{
    return mMajorPoint.getAngle();
}

void Ellipse::setAngle(double a)
{
    mMajorPoint = Vec2d::createPolar(mMajorPoint.getMagnitude(), a);
}

bool Ellipse::isFullEllipse() const
{
    double a1 = Math::getNormalizedAngle(mStartParam);
    double a2 = Math::getNormalizedAngle(mEndParam);
    return (a1 < NS::AngleTolerance && a2 > 2 * M_PI - NS::AngleTolerance) ||
           (fabs(a1 - a2) < NS::AngleTolerance);
}

bool Ellipse::isCircular() const
{
    return getRatio() > (1.0 - 0.001);
}

bool Ellipse::contains(const Vec2d &p) const
{
    Vec2d pt = p;
    pt.move(-mCenter);
    pt.rotate(-getAngle());
    double rx = getMajorRadius();
    double ry = getMinorRadius();
    return (pt.x * pt.x) / (rx * rx) + (pt.y * pt.y) / (ry * ry) <= 1.0;
}

bool Ellipse::isReversed() const
{
    return mReversed;
}

void Ellipse::setReversed(bool r)
{
    mReversed = r;
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
    ret.push_back(mCenter + mMajorPoint + minorPoint);
    ret.push_back(mCenter + mMajorPoint - minorPoint);
    ret.push_back(mCenter - mMajorPoint - minorPoint);
    ret.push_back(mCenter - mMajorPoint + minorPoint);

    return ret;
}

void Ellipse::correctMajorMinor()
{
    if (mRatio > 1.0) {
        Vec2d mp = getMinorPoint();
        mRatio = 1.0 / mRatio;
        setMajorPoint(mp);
        mStartParam = Math::getNormalizedAngle(mStartParam - M_PI / 2.0);
        mEndParam = Math::getNormalizedAngle(mEndParam - M_PI / 2.0);
    }
}

double Ellipse::getSweep() const
{
    double ret = 0.0;

    if (mReversed) {
        if (mStartParam <= mEndParam) {
            ret = -(mStartParam + 2 * M_PI - mEndParam);
        }
        else {
            ret = -(mStartParam - mEndParam);
        }
    }
    else {
        if (mEndParam <= mStartParam) {
            ret = mEndParam + 2 * M_PI - mStartParam;
        }
        else {
            ret = mEndParam - mStartParam;
        }
    }

    return ret;
}

std::vector<Line> Ellipse::getTangents(const Vec2d &point) const
{
    std::vector<Line> ret;

    if (getDistanceTo(point, false) < NS::PointTolerance) {
        return ret;
    }

    if (point.getDistanceTo(getCenter()) < NS::PointTolerance) {
        return ret;
    }

    Line minorAxis(getCenter(), getCenter() + getMinorPoint());
    Line majorAxis(getCenter(), getCenter() + getMajorPoint());
    if (minorAxis.isOnShape(point, false) &&
        !majorAxis.isOnShape(point, false)) {
        Ellipse e2 = *this;
        e2.mMajorPoint = getMinorPoint();
        e2.mRatio = 1.0 / mRatio;
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

    lineNeutral.move(getCenter().getNegated());

    lineNeutral.rotate(-getAngle());

    if (lineNeutral.isVertical()) {
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

Polyline Ellipse::approximateWithArcs(int segments) const
{
    return Polyline();
}

} // namespace shape
} // namespace cada