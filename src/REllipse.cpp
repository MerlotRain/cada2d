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

#include <cada2d/REllipse.h>
#include <cada2d/RBox.h>
#include <cada2d/RMath.h>
#include <cada2d/RLine.h>
#include <cada2d/private/RShapePrivate.h>

REllipse::REllipse()
    : m_center(RVector::invalid), m_majorPoint(RVector::invalid), m_ratio(0.0),
      m_startParam(0.0), m_endParam(0.0), m_reversed(false)
{
}

REllipse::REllipse(const RVector &center, const RVector &majorPoint,
                   double ratio, double startParam, double endParam,
                   bool reversed)
    : m_center(center), m_majorPoint(majorPoint), m_ratio(ratio),
      m_startParam(startParam), m_endParam(endParam), m_reversed(reversed)
{

    correctMajorMinor();
}

REllipse::~REllipse()
{
}

REllipse REllipse::createFromQuadratic(double a, double b, double c)
{
    const double d = a - b;
    const double s = hypot(d, c);

    if (s > a + b)
        return REllipse();

    REllipse ellipse;
    if (a >= b) {
        ellipse.setMajorPoint(RVector::createPolar(1.0, atan2(d + s, -c)) /
                              sqrt(0.5 * (a + b - s)));
    }
    else {
        ellipse.setMajorPoint(RVector::createPolar(1.0, atan2(-c, s - d)) /
                              sqrt(0.5 * (a + b - s)));
    }
    ellipse.setRatio(sqrt((a + b - s) / (a + b + s)));
    return ellipse;
}

REllipse REllipse::createInscribed(const RVector &p1, const RVector &p2,
                                   const RVector &p3, const RVector &p4,
                                   const RVector &centerHint)
{
    std::vector<RLine> quad;
    quad.push_back(RLine(p1, p2));
    quad.push_back(RLine(p2, p3));
    quad.push_back(RLine(p3, p4));
    quad.push_back(RLine(p4, p1));

    // center of original square projected, intersection of diagonal
    RVector centerProjection;
    {
        auto &&deigonal1 =
            RLine(quad[0].getStartPoint(), quad[1].getEndPoint());
        auto &&deigonal2 =
            RLine(quad[1].getStartPoint(), quad[2].getEndPoint();
        auto sol = deigonal1.getIntersectionPoints(deigonal2);
        if (sol.size() == 0) // this should not happen
            return REllipse();

        centerProjection = sol.at(0);
    }

    // holds the tangential points on edges, in the order of edges: 1 3 2 0
    std::vector<RVector> tangent;
    int parallel = 0;
    int parallel_index = 0;
    for (int i = 0; i <= 1; ++i) {
        auto sol1 = quad[i].getIntersectionPoints(quad[(i + 1) % 4]);
        RVector direction;
        if (sol1.size() == 0) {
            direction = quad[i].getEndPoint() - quad[i].getStartPoint();
            ++parallel;
            parallel_index = i;
        }
        else {
            direction = sol1.at(0) - centerProjection;
        }

        auto l = RLine(centerProjection, centerProjection + direction);
        for (int k = 1; k <= 3; k += 2) {
            auto sol2 = l.getIntersectionPoints(quad[(i + k) % 4]);
            if (sol2.size() > 0)
                tangent.push_back(sol2.at(0));
        }
    }

    if (tangent.size() < 3)
        return REllipse();

    // find ellipse center by projection
    RVector ellipseCenter;
    {
        auto cl0 =
            RLine(quad[1].getEndPoint(), (tangent[0] + tangent[2]) * 0.5);
        auto cl1 =
            RLine(quad[2].getEndPoint(), (tangent[1] + tangent[2]) * 0.5);
        auto sol = cl0.getIntersectionPoints(cl1, false);
        if (sol.size() == 0) {
            return REllipse();
        }
        ellipseCenter = sol.at(0);
    }

    if (parallel == 1) {
        // trapezoid
        auto &&l0 = quad[parallel_index];
        auto &&l1 = quad[(parallel_index + 2) % 4];
        RVector centerPoint = (l0.getMiddlePoint() + l1.getMiddlePoint()) * 0.5;
        // not symmetric, no inscribed ellipse
        if (fabs(centerPoint.getDistanceTo(l0.getStartPoint()) -
                 centerPoint.getDistanceTo(l0.getEndPoint())) >
            RS::PointTolerance)
            return REllipse();

        // symmetric
        double d = l0.getDistanceTo(centerPoint);
        double l = ((l0.getLength() + l1.getLength())) * 0.25;
        double k = 4. * d / fabs(l0.getLength() - l1.getLength());
        double theta = d / (l * k);
        if (theta >= 1. || d < RS::PointTolerance) {

            return REllipse();
        }
        theta = asin(theta);

        // major axis
        double a = d / (k * tan(theta));
        auto tmpRet = REllipse(RVector(0.0, 0.0), RVector(a, 0.0), d / a, 0,
                                    2 * M_PI, false);
        tmpRet.rotate(l0.getAngle());
        tmpRet.setCenter(centerPoint);
        return tmpRet;
    }

    std::vector<double> dn(3);
    RVector angleVector;

    for (size_t i = 0; i < tangent.size(); i++) {
        // relative to ellipse center
        tangent[i] -= ellipseCenter;
    }
    std::vector<std::vector<double>> mt;
    mt.clear();
    const double symTolerance = 20. * RS::PointTolerance;
    for (const RVector &vp : tangent) {
        // form the linear equation need to remove duplicated {x^2, xy, y^2}
        // terms due to symmetry (x => -x, y=> -y) i.e. rotation of 180 degrees
        // around ellipse center
        std::vector<double> mtRow;
        mtRow.push_back(vp.x * vp.x);
        mtRow.push_back(vp.x * vp.y);
        mtRow.push_back(vp.y * vp.y);
        const double l = hypot(hypot(mtRow[0], mtRow[1]), mtRow[2]);
        bool addRow(true);
        for (const auto &v : mt) {
            const RVector dv(v[0] - mtRow[0], v[1] - mtRow[1]);
            if (dv.getMagnitude() < symTolerance * l) {
                // symmetric
                addRow = false;
                break;
            }
        }
        if (addRow) {
            mtRow.push_back(1.);
            mt.push_back(mtRow);
        }
    }
    switch (mt.size()) {
    case 2: {
        // the quadrilateral is a parallelogram fixme, need to handle degenerate
        // case better double angle(center.angleTo(tangent[0]));
        RVector majorP(tangent[0]);
        double dx(majorP.getMagnitude());
        if (dx < RS::PointTolerance)
            return REllipse();
        // refuse to return zero size ellipse
        angleVector.set(majorP.x / dx, -majorP.y / dx);
        for (size_t i = 0; i < tangent.size(); i++)
            tangent[i].rotate(angleVector);

        RVector minorP(tangent[2]);
        double dy2(minorP.getSquaredMagnitude());
        // refuse to return zero size ellipse
        if (fabs(minorP.y) < RS::PointTolerance || dy2 < RS::PointTolerance)
            return REllipse();
        double ia2 = 1. / (dx * dx);
        double ib2 = 1. / (minorP.y * minorP.y);
        dn[0] = ia2;
        dn[1] = -2. * ia2 * minorP.x / minorP.y;
        dn[2] = ib2 * ia2 * minorP.x * minorP.x + ib2;
    } break;
    case 4:
        mt.pop_back();
        // only 3 points needed to form the qudratic form
        if (!RMath::linearSolver(mt, dn))
            return REllipse();
        break;
    default:
        return REllipse(); // invalid quadrilateral
    }

    auto ellipseRet = createFromQuadratic(dn[0], dn[1], dn[2]);
    if (!ellipseRet.isValid())
        return REllipse();
    ellipseRet.setCenter(ellipseCenter);

    // need to rotate back, for the parallelogram case
    if (angleVector.valid) {
        angleVector.y *= -1.;
        ellipseRet.setCenter(
            ellipseRet.getCenter().rotate(ellipseCenter, angleVector));
        ellipseRet.setMajorPoint(ellipseRet.getCenter().rotate(angleVector));
    }
    return ellipseRet;
}

REllipse REllipse::createFrom4Points(const RVector &p1, const RVector &p2,
                                     const RVector &p3, const RVector &p4)
{
    std::vector<RVector> sol = {p1, p2, p3, p4};
    std::vector<std::vector<double>> mt;
    std::vector<double> dn;
    int mSize(4);
    mt.resize(mSize);
    for (int i = 0; i < mSize;
         i++) { // form the linear equation, c0 x^2 + c1 x + c2 y^2 + c3 y = 1
        mt[i].resize(mSize + 1);
        mt[i][0] = sol[i].x * sol[i].x;
        mt[i][1] = sol[i].x;
        mt[i][2] = sol[i].y * sol[i].y;
        mt[i][3] = sol[i].y;
        mt[i][4] = 1.;
    }
    if (!RMath::linearSolver(mt, dn))
        return REllipse();
    double d(1. + 0.25 * (dn[1] * dn[1] / dn[0] + dn[3] * dn[3] / dn[2]));
    if (fabs(dn[0]) < RS::PointTolerance || fabs(dn[2]) < RS::PointTolerance ||
        d / dn[0] < RS::PointTolerance || d / dn[2] < RS::PointTolerance) {
        // ellipse not defined
        return REllipse();
    }

    RVector center(-0.5 * dn[1] / dn[0], -0.5 * dn[3] / dn[2]); // center
    d = sqrt(d / dn[0]);
    RVector majorP(d, 0.); // major point
    double ratio = sqrt(dn[0] / dn[2]);

    return REllipse(center, majorP, ratio, 0, 2 * M_PI, false);
}

RS::ShapeType REllipse::getShapeType() const
{
    return RS::Ellipse;
}

REllipse *REllipse::clone() const
{
    return new REllipse(*this);
}

bool REllipse::isDirected() const
{
    return true;
}

bool REllipse::isValid() const
{
    return m_center.isValid() && m_majorPoint.isValid() &&
           !RMath::isNaN(m_ratio) && !RMath::isNaN(m_startParam) &&
           !RMath::isNaN(m_endParam);
}

std::vector<RVector> REllipse::getFoci() const
{
    RVector vp(getMajorPoint() * sqrt(1.0 - getRatio() * getRatio()));
    return {getCenter() + vp, getCenter() - vp};
}

void REllipse::moveStartPoint(const RVector &pos, bool changeAngleOnly)
{
    if (changeAngleOnly) {
        m_startParam = getParamTo(pos);
    }
    else {
        RVector ep = getEndPoint();
        double distOri = ep.getDistanceTo(getStartPoint());
        double angleOri = ep.getAngleTo(getStartPoint());
        if (distOri < RS::PointTolerance) {
            return;
        }

        double distNew = ep.getDistanceTo(pos);
        double angleNew = ep.getAngleTo(pos);
        double factor = distNew / distOri;
        if (factor < RS::PointTolerance) {
            return;
        }
        double angleDelta = angleNew - angleOri;

        m_center.scale(factor, ep);
        m_center.rotate(angleDelta, ep);
        m_majorPoint.scale(factor);
        m_majorPoint.rotate(angleDelta);
    }
}

void REllipse::moveEndPoint(const RVector &pos, bool changeAngleOnly)
{
    if (changeAngleOnly) {
        m_endParam = getParamTo(pos);
    }
    else {
        RVector sp = getStartPoint();
        double distOri = sp.getDistanceTo(getEndPoint());
        double angleOri = sp.getAngleTo(getEndPoint());
        if (distOri < RS::PointTolerance) {
            return;
        }

        double distNew = sp.getDistanceTo(pos);
        double angleNew = sp.getAngleTo(pos);
        double factor = distNew / distOri;
        if (factor < RS::PointTolerance) {
            return;
        }
        double angleDelta = angleNew - angleOri;

        m_center.scale(factor, sp);
        m_center.rotate(angleDelta, sp);
        m_majorPoint.scale(factor);
        m_majorPoint.rotate(angleDelta);
    }
}

double REllipse::getAngleAt(double distance, RS::From from) const
{
    return 0.0;
}

double REllipse::getAngleAtPoint(const RVector &pos) const
{
    RVector posNormalized = pos;
    posNormalized.move(-getCenter());
    posNormalized.rotate(-getAngle());

    double angle;
    if (RMath::fuzzyCompare(posNormalized.y, 0.0)) {
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

    if (m_reversed) {
        angle += M_PI;
    }

    if (posNormalized.y < 0) {
        angle += M_PI;
    }

    angle += getAngle();

    return RMath::getNormalizedAngle(angle);
}

double REllipse::getParamTo(const RVector &pos) const
{
    RVector m = pos;
    m.rotate(-m_majorPoint.getAngle(), m_center);
    RVector v = m - m_center;
    v.scale(RVector(1.0, 1.0 / m_ratio));
    return v.getAngle();
}

double REllipse::getRadiusAt(double param) const
{
    RVector v(cos(param) * getMajorRadius(), sin(param) * getMinorRadius());
    return v.getMagnitude();
}

RVector REllipse::getPointAt(double param) const
{
    RVector v(cos(param) * getMajorRadius(), sin(param) * getMinorRadius());
    v.rotate(getAngle());
    v.move(m_center);
    return v;
}

RVector REllipse::getMiddlePoint() const
{
    double a;
    a = getStartParam() + getSweep() / 2.0;
    return getPointAt(a);
}

RVector REllipse::getPointOnShape() const
{
    double sp = m_startParam;
    double ep = m_endParam;
    if (isReversed()) {
        if (sp < ep) {
            sp += M_PI * 2;
        }
    }
    else {
        if (ep < sp) {
            ep += M_PI * 2;
        }
    }
    double mp = (sp + ep) / 2.0;
    return getPointAt(mp);
}

RVector REllipse::getCenter() const
{
    return m_center;
}

void REllipse::setCenter(const RVector &vector)
{
    m_center = vector;
}

RVector REllipse::getMajorPoint() const
{
    return m_majorPoint;
}

void REllipse::setMajorPoint(const RVector &p)
{
    m_majorPoint = p;
    correctMajorMinor();
}

RVector REllipse::getMinorPoint() const
{
    double angle = RMath::getNormalizedAngle(getAngle() + M_PI / 2.0);
    RVector ret;
    ret.setPolar(getMinorRadius(), angle);
    return ret;
}

void REllipse::setMinorPoint(const RVector &p)
{
    double angle = RMath::getNormalizedAngle(p.getAngle() - M_PI / 2.0);
    m_majorPoint.setPolar(getMajorRadius(), angle);
    setRatio(p.getMagnitude() / getMajorRadius());
}

bool REllipse::switchMajorMinor()
{
    if (fabs(m_ratio) < RS::PointTolerance) {
        return false;
    }
    RVector vp_start = getStartPoint();
    RVector vp_end = getStartPoint();
    RVector vp = getMajorPoint();
    setMajorPoint(RVector(-m_ratio * vp.y, m_ratio * vp.x));
    setRatio(1.0 / m_ratio);
    setStartParam(getParamTo(vp_start));
    setEndParam(getParamTo(vp_end));
    return true;
}

double REllipse::getRatio() const
{
    return m_ratio;
}

void REllipse::setRatio(double r)
{
    m_ratio = r;
    correctMajorMinor();
}

double REllipse::getStartParam() const
{
    return m_startParam;
}

void REllipse::setStartParam(double a)
{
    m_startParam = a;
}

double REllipse::getEndParam() const
{
    return m_endParam;
}

void REllipse::setEndParam(double a)
{
    m_endParam = a;
}

double REllipse::getStartAngle() const
{
    return RMath::getNormalizedAngle(m_center.getAngleTo(getStartPoint()) -
                                     getAngle());
}

void REllipse::setStartAngle(double a)
{
    double p = angleToParam(a);
    if (RMath::isNaN(p)) {
        return;
    }
    m_startParam = p;
}

double REllipse::getEndAngle() const
{
    return RMath::getNormalizedAngle(m_center.getAngleTo(getEndPoint()) -
                                     getAngle());
}

void REllipse::setEndAngle(double a)
{
    double p = angleToParam(a);
    if (RMath::isNaN(p)) {
        return;
    }
    m_endParam = p;
}

double REllipse::angleToParam(double a) const
{
    double p;
    if (fabs(a - 2 * M_PI) < RS::AngleTolerance) {
        p = 2 * M_PI;
    }
    else if (fabs(a) < RS::AngleTolerance) {
        p = 0.0;
    }
    else {
        REllipse normEllipse = *this;
        normEllipse.move(-m_center);
        normEllipse.rotate(-getAngle());
        normEllipse.setStartParam(0.0);
        normEllipse.setEndParam(2 * M_PI);

        RLine line(RVector(0, 0),
                   RVector::createPolar(getMajorRadius() * 2, a));
        std::vector<RVector> r =
            RShape::getIntersectionPoints(line, normEllipse, true);
        if (r.size() != 1) {
            return RNANDOUBLE;
        }

        p = acos(r[0].x / getMajorRadius());
    }

    if (RMath::getNormalizedAngle(a) > M_PI) {
        p = 2 * M_PI - p;
    }

    return p;
}

double REllipse::getAngleLength(bool allowForZeroLength) const
{
    double ret = 0.0;

    if (isReversed()) {
        if (m_startParam < m_endParam) {
            ret = m_startParam + 2 * M_PI - m_endParam;
        }
        else {
            ret = m_startParam - m_endParam;
        }
    }
    else {
        if (m_endParam < m_startParam) {
            ret = m_endParam + 2 * M_PI - m_startParam;
        }
        else {
            ret = m_endParam - m_startParam;
        }
    }

    // full ellipse or zero length ellipse arc:
    if (!allowForZeroLength) {
        if (fabs(ret) < RS::AngleTolerance) {
            ret = 2 * M_PI;
        }
    }
    else {
        if (ret > 2 * M_PI - RS::AngleTolerance) {
            ret = 0.0;
        }
    }

    return ret;
}

bool REllipse::isAngleWithinArc(double a) const
{
    if (isFullEllipse()) {
        return true;
    }
    return RMath::isAngleBetween(a, getStartAngle(), getEndAngle(), m_reversed);
}

bool REllipse::isParamWithinArc(double a) const
{
    if (isFullEllipse()) {
        return true;
    }
    return RMath::isAngleBetween(a, getStartParam(), getEndParam(), m_reversed);
}

RVector REllipse::getStartPoint() const
{
    RVector p(m_center.x + cos(m_startParam) * getMajorRadius(),
              m_center.y + sin(m_startParam) * getMinorRadius());
    p.rotate(getAngle(), m_center);
    return p;
}

RVector REllipse::getEndPoint() const
{
    RVector p(m_center.x + cos(m_endParam) * getMajorRadius(),
              m_center.y + sin(m_endParam) * getMinorRadius());
    p.rotate(getAngle(), m_center);
    return p;
}

double REllipse::getMajorRadius() const
{
    return m_majorPoint.getMagnitude();
}

double REllipse::getMinorRadius() const
{
    return m_majorPoint.getMagnitude() * m_ratio;
}

double REllipse::getAngle() const
{
    return m_majorPoint.getAngle();
}

void REllipse::setAngle(double a)
{
    m_majorPoint = RVector::createPolar(m_majorPoint.getMagnitude(), a);
}

bool REllipse::isFullEllipse() const
{
    double a1 = RMath::getNormalizedAngle(m_startParam);
    double a2 = RMath::getNormalizedAngle(m_endParam);
    return (a1 < RS::AngleTolerance && a2 > 2 * M_PI - RS::AngleTolerance) ||
           (fabs(a1 - a2) < RS::AngleTolerance);
}

bool REllipse::isCircular() const
{
    return getRatio() > (1.0 - 0.001);
}

double REllipse::getLength() const
{
    double a1, a2;

    if (isFullEllipse()) {
        a1 = 0.0;
        a2 = 2 * M_PI;

        double a = getMajorRadius();
        double b = getMinorRadius();
        if (RMath::fuzzyCompare((a + b), 0.0)) {
            return 0.0;
        }
        double h = pow((a - b) / (a + b), 2);

        return M_PI * (a + b) *
               ((135168 - 85760 * h - 5568 * h * h + 3867 * h * h * h) /
                (135168 - 119552 * h + 22208 * h * h - 345 * h * h * h));
    }
    else {
        a1 = RMath::getNormalizedAngle(m_startParam);
        a2 = RMath::getNormalizedAngle(m_endParam);
    }

    if (m_reversed) {
        double t = a1;
        a1 = a2;
        a2 = t;
    }

    if (RMath::fuzzyCompare(a2, 0.0)) {
        a2 = 2 * M_PI;
    }

    if (fabs(a1 - a2) < RS::AngleTolerance) {
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

    return RNANDOUBLE;
}

double REllipse::getSimpsonLength(double a1, double a2) const
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

bool REllipse::contains(const RVector &p) const
{
    RVector pt = p;
    pt.move(-m_center);
    pt.rotate(-getAngle());
    double rx = getMajorRadius();
    double ry = getMinorRadius();
    return (pt.x * pt.x) / (rx * rx) + (pt.y * pt.y) / (ry * ry) <= 1.0;
}

bool REllipse::isReversed() const
{
    return m_reversed;
}

void REllipse::setReversed(bool r)
{
    m_reversed = r;
}

double REllipse::getDirection1() const
{
    return getAngleAtPoint(getStartPoint());
}

double REllipse::getDirection2() const
{
    return RMath::getNormalizedAngle(getAngleAtPoint(getEndPoint()) + M_PI);
}

RS::Side REllipse::getSideOfPoint(const RVector &point) const
{
    if (contains(point)) {
        if (!m_reversed) {
            return RS::RightHand;
        }
        else {
            return RS::LeftHand;
        }
    }
    else {
        if (!m_reversed) {
            return RS::LeftHand;
        }
        else {
            return RS::RightHand;
        }
    }
}

RBox REllipse::getBoundingBox() const
{
    double radius1 = getMajorRadius();
    double radius2 = getMinorRadius();
    double angle = getAngle();
    double a1 = ((!isReversed()) ? m_startParam : m_endParam);
    double a2 = ((!isReversed()) ? m_endParam : m_startParam);
    RVector startPoint = getStartPoint();
    RVector endPoint = getEndPoint();

    double minX = std::min(startPoint.x, endPoint.x);
    double minY = std::min(startPoint.y, endPoint.y);
    double maxX = std::max(startPoint.x, endPoint.x);
    double maxY = std::max(startPoint.y, endPoint.y);

    // kind of a brute force. TODO: exact calculation
    RVector vp;
    double a = a1;
    do {
        vp.set(m_center.x + radius1 * cos(a), m_center.y + radius2 * sin(a));
        vp.rotate(angle, m_center);

        minX = std::min(minX, vp.x);
        minY = std::min(minY, vp.y);
        maxX = std::max(maxX, vp.x);
        maxY = std::max(maxY, vp.y);

        a += 0.03;
    } while (RMath::isAngleBetween(a, a1, a2, false) && a < 4 * M_PI);

    return RBox(RVector(minX, minY), RVector(maxX, maxY));
}

std::vector<RVector> REllipse::getEndPoints() const
{
    std::vector<RVector> ret;
    ret.push_back(getStartPoint());
    ret.push_back(getEndPoint());
    return ret;
}

std::vector<RVector> REllipse::getMiddlePoints() const
{
    std::vector<RVector> ret;
    return ret;
}

std::vector<RVector> REllipse::getCenterPoints() const
{
    std::vector<RVector> ret;
    ret.push_back(getCenter());
    return ret;
}

std::vector<RVector> REllipse::getPointsWithDistanceToEnd(double distance,
                                                          int from) const
{
    std::vector<RVector> ret;
    return ret;
}

std::vector<RVector> REllipse::getPointCloud(double segmentLength) const
{
    RPolyline pl = approximateWithArcs(64);
    return pl.getPointCloud(segmentLength);
}

RVector REllipse::getVectorTo(const RVector &point, bool limited,
                              double strictRange) const
{
    RVector ret = RVector::invalid;

    double ang = getAngle();
    // double dDistance = RMAXDOUBLE;
    bool swap = false;
    bool majorSwap = false;

    RVector normalized = (point - m_center).rotate(-ang);

    // special case: point in line with major axis:
    if (fabs(normalized.getAngle()) < RS::AngleTolerance ||
        fabs(normalized.getAngle()) > 2 * M_PI - RS::AngleTolerance) {
        ret = RVector(getMajorRadius(), 0.0);
        // dDistance = ret.distanceTo(normalized);
    }

    else if (fabs(normalized.getAngle() - M_PI) < RS::AngleTolerance) {
        ret = RVector(-getMajorRadius(), 0.0);
        // dDistance = ret.distanceTo(normalized);
    }
    else {
        double dU = normalized.x;
        double dV = normalized.y;
        double dA = getMajorRadius();
        double dB = getMinorRadius();
        double dEpsilon = 1.0e-8;
        // iteration maximum
        int iMax = 32;
        double rdX = 0.0;
        double rdY = 0.0;

        if (dA < dB) {
            double dum = dA;
            dA = dB;
            dB = dum;
            dum = dU;
            dU = dV;
            dV = dum;
            majorSwap = true;
        }

        if (dV < 0.0) {
            dV *= -1.0;
            swap = true;
        }

        // initial guess:
        double dT = dB * (dV - dB);

        // newton's method:
        int i;
        for (i = 0; i < iMax; i++) {
            double dTpASqr = dT + dA * dA;
            double dTpBSqr = dT + dB * dB;
            double dInvTpASqr = 1.0 / dTpASqr;
            double dInvTpBSqr = 1.0 / dTpBSqr;
            double dXDivA = dA * dU * dInvTpASqr;
            double dYDivB = dB * dV * dInvTpBSqr;
            double dXDivASqr = dXDivA * dXDivA;
            double dYDivBSqr = dYDivB * dYDivB;
            double dF = dXDivASqr + dYDivBSqr - 1.0;
            if (fabs(dF) < dEpsilon) {
                // f(t0) is very close to zero:
                rdX = dXDivA * dA;
                rdY = dYDivB * dB;
                break;
            }
            double dFDer =
                2.0 * (dXDivASqr * dInvTpASqr + dYDivBSqr * dInvTpBSqr);

            double dRatio = dF / dFDer;

            if (fabs(dRatio) < dEpsilon) {
                // t1-t0 is very close to zero:
                rdX = dXDivA * dA;
                rdY = dYDivB * dB;
                break;
            }
            dT += dRatio;
        }

        if (i == iMax) {
            // failed to converge:
            // dDistance = RMAXDOUBLE;
            ret = RVector::invalid;
        }
        else {
            // double dDelta0 = rdX - dU;
            // double dDelta1 = rdY - dV;
            // dDistance = sqrt(dDelta0*dDelta0 + dDelta1*dDelta1);
            ret = RVector(rdX, rdY);
        }
    }

    if (ret.isValid()) {
        if (swap) {
            ret.y *= -1.0;
        }
        if (majorSwap) {
            double dum = ret.x;
            ret.x = ret.y;
            ret.y = dum;
        }
        ret = (ret.rotate(ang) + m_center);

        if (limited) {
            double a1 = m_center.getAngleTo(getStartPoint());
            double a2 = m_center.getAngleTo(getEndPoint());
            double a = m_center.getAngleTo(ret);
            if (!RMath::isAngleBetween(a, a1, a2, m_reversed)) {
                ret = RVector::invalid;
            }
        }
    }

    return point - ret;
}

bool REllipse::move(const RVector &offset)
{
    if (!offset.isValid() || offset.getMagnitude() < RS::PointTolerance) {
        return false;
    }
    m_center += offset;
    return true;
}

bool REllipse::rotate(double rotation, const RVector &c)
{
    if (fabs(rotation) < RS::AngleTolerance) {
        return false;
    }

    m_center.rotate(rotation, c);
    m_majorPoint.rotate(rotation);

    return true;
}

std::vector<RVector> REllipse::getBoxCorners()
{
    std::vector<RVector> ret;

    RVector minorPoint = getMinorPoint();
    ret.push_back(m_center + m_majorPoint + minorPoint);
    ret.push_back(m_center + m_majorPoint - minorPoint);
    ret.push_back(m_center - m_majorPoint - minorPoint);
    ret.push_back(m_center - m_majorPoint + minorPoint);

    return ret;
}

bool REllipse::scale(const RVector &scaleFactors, const RVector &c)
{
    if (fabs(fabs(scaleFactors.x) - fabs(scaleFactors.y)) >
        RS::PointTolerance) {
        return false;
    }

    // RVector oldMinorPoint = getMinorPoint();

    // negative scaling: mirroring and scaling
    if (scaleFactors.x < 0.0) {
        mirror(RLine(m_center, m_center + RVector(0.0, 1.0)));
    }
    if (scaleFactors.y < 0.0) {
        mirror(RLine(m_center, m_center + RVector(1.0, 0.0)));
    }

    m_center.scale(scaleFactors, c);

    RVector f = RVector(fabs(scaleFactors.x), fabs(scaleFactors.y));
    m_majorPoint.scale(f);

    return true;
}

bool REllipse::mirror(const RLine &axis)
{
    RVector mp = m_center + m_majorPoint;
    RVector sp = getStartPoint();
    RVector ep = getEndPoint();

    m_center.mirror(axis.getStartPoint(), axis.getEndPoint());
    mp.mirror(axis.getStartPoint(), axis.getEndPoint());

    m_majorPoint = mp - m_center;

    if (!isFullEllipse()) {
        m_reversed = (!m_reversed);

        sp.mirror(axis.getStartPoint(), axis.getEndPoint());
        setStartParam(getParamTo(sp));

        ep.mirror(axis.getStartPoint(), axis.getEndPoint());
        setEndParam(getParamTo(ep));
    }

    return true;
}

bool REllipse::reverse()
{
    std::swap(m_startParam, m_endParam);
    m_reversed = !m_reversed;
    return true;
}

RS::Ending REllipse::getTrimEnd(const RVector &trimPoint,
                                const RVector &clickPoint)
{
    double paramToClickPoint = getParamTo(clickPoint);
    double paramToTrimPoint = getParamTo(trimPoint);

    if (RMath::getAngleDifference(paramToTrimPoint, paramToClickPoint) > M_PI) {
        return RS::EndingStart;
    }
    else {
        return RS::EndingEnd;
    }
}

bool REllipse::trimStartPoint(const RVector &trimPoint,
                              const RVector &clickPoint, bool extend)
{
    setStartParam(getParamTo(trimPoint));
    return true;
}

bool REllipse::trimEndPoint(const RVector &trimPoint, const RVector &clickPoint,
                            bool extend)
{
    setEndParam(getParamTo(trimPoint));
    return true;
}

bool REllipse::trimStartPoint(double trimDist)
{
    RVector p = getPointWithDistanceToStart(trimDist);
    return trimStartPoint(p);
}

bool REllipse::trimEndPoint(double trimDist)
{
    RVector p = getPointWithDistanceToStart(trimDist);
    return trimEndPoint(p);
}

void REllipse::correctMajorMinor()
{
    if (m_ratio > 1.0) {
        RVector mp = getMinorPoint();
        m_ratio = 1.0 / m_ratio;
        setMajorPoint(mp);
        m_startParam = RMath::getNormalizedAngle(m_startParam - M_PI / 2.0);
        m_endParam = RMath::getNormalizedAngle(m_endParam - M_PI / 2.0);
    }
}

double REllipse::getSweep() const
{
    double ret = 0.0;

    if (m_reversed) {
        if (m_startParam <= m_endParam) {
            ret = -(m_startParam + 2 * M_PI - m_endParam);
        }
        else {
            ret = -(m_startParam - m_endParam);
        }
    }
    else {
        if (m_endParam <= m_startParam) {
            ret = m_endParam + 2 * M_PI - m_startParam;
        }
        else {
            ret = m_endParam - m_startParam;
        }
    }

    return ret;
}

std::vector<RLine> REllipse::getTangents(const RVector &point) const
{
    std::vector<RLine> ret;

    if (getDistanceTo(point, false) < RS::PointTolerance) {
        // point is on ellipse:
        return ret;
    }

    // point is at center (prevents recursion when swapping ellipse minor /
    // major):
    if (point.getDistanceTo(getCenter()) < RS::PointTolerance) {
        return ret;
    }

    // swap ellipse minor / major if point is on minor axis
    // 20120928: and not also on major axis (prevent recursion):
    RLine minorAxis(getCenter(), getCenter() + getMinorPoint());
    RLine majorAxis(getCenter(), getCenter() + getMajorPoint());
    if (minorAxis.isOnShape(point, false) &&
        !majorAxis.isOnShape(point, false)) {
        REllipse e2 = *this;
        e2.m_majorPoint = getMinorPoint();
        e2.m_ratio = 1.0 / m_ratio;
        return e2.getTangents(point);
    }

    double a = getMajorRadius(); // the length of the major axis / 2
    double b = getMinorRadius(); // the length of the minor axis / 2

    // rotate and move point:
    RVector point2 = point;
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
    if (RMath::isNaN(t)) {
        return ret;
    }

    yt1 = (t - bf) / (af * 2.0);
    xt1 = e - d * yt1;
    yt2 = (-t - bf) / (af * 2.0);
    xt2 = e - d * yt2;

    RVector s1(xt1, yt1);
    s1.rotate(getAngle());
    s1.move(getCenter());

    RVector s2(xt2, yt2);
    s2.rotate(getAngle());
    s2.move(getCenter());

    if (s1.isValid()) {
        ret.push_back(RLine(point, s1));
    }

    if (s2.isValid()) {
        ret.push_back(RLine(point, s2));
    }

    return ret;
}

RVector REllipse::getTangentPoint(const RLine &line) const
{
    RLine lineNeutral = line;

    // translate line to ellipse's center
    lineNeutral.move(getCenter().getNegated());

    // rotate line points (inverse rotation of the ellipse)
    lineNeutral.rotate(-getAngle());

    // calculate slope and y-intercept of the transformed line
    // check for vertical line:
    if (lineNeutral.isVertical()) {
        // for vertical line, check if it passes through ellipse's major axis
        if (RMath::fuzzyCompare(lineNeutral.getStartPoint().x,
                                getMajorRadius())) {
            return getCenter() + getMajorPoint();
        }

        if (RMath::fuzzyCompare(lineNeutral.getStartPoint().x,
                                -getMajorRadius())) {
            return getCenter() - getMajorPoint();
        }

        return RVector::invalid;
    }

    // check if the transformed line is tangent to the axis-aligned ellipse:
    // slope of line:
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
    if (RMath::fuzzyCompare(discriminant, 0.0, 0.001)) {
        double x = -B / (2 * A);
        double y = m * x + c;

        RVector ret = RVector(x, y);
        ret.rotate(getAngle());
        ret.move(getCenter());
        return ret;
    }

    return RVector::invalid;
}

std::vector<RSpline> REllipse::approximateWithSplines() const
{
    return std::vector<RSpline>();
}

RPolyline REllipse::approximateWithArcs(int segments) const
{
    return RPolyline();
}

std::vector<std::shared_ptr<RShape>>
REllipse::getOffsetShapes(double distance, int number, RS::Side side,
                          const RVector &position)
{
    std::vector<std::shared_ptr<RShape>> ret;
    REllipse *ellipse = dynamic_cast<REllipse *>(clone());
    if (ellipse == NULL) {
        return ret;
    }

    RVector center = ellipse->getCenter();

    if (ellipse->isReversed()) {
        ellipse->reverse();
    }

    std::vector<bool> insides;
    if (position.isValid()) {
        double ang = center.getAngleTo(position) - ellipse->getAngle();
        double t = ellipse->angleToParam(ang);
        RVector p = ellipse->getPointAt(t);
        insides.push_back(center.getDistanceTo(position) <
                          center.getDistanceTo(p));
    }
    else {
        if (side == RS::BothSides) {
            insides.push_back(true);
            insides.push_back(false);
        }
        else {
            if (side == RS::LeftHand) {
                insides.push_back(true);
            }
            else {
                insides.push_back(false);
            }
        }
    }

    double a = ellipse->getMajorRadius();
    double b = ellipse->getMinorRadius();

    for (int i = 0; i < insides.size(); i++) {
        bool inside = insides[i];
        double d = distance;

        if (inside) {
            d *= -1;
        }

        for (int n = 1; n <= number; ++n) {
            RPolyline *pl = NULL;
            pl = new RPolyline();

            double endParam = ellipse->getEndParam();
            double startParam = ellipse->getStartParam();
            if (RMath::fuzzyCompare(endParam, 0.0)) {
                endParam = 2 * M_PI;
            }

            if (endParam < startParam) {
                endParam += 2 * M_PI;
            }

            double k = d * n;
            double tMax = endParam + 0.1;
            if (ellipse->isFullEllipse()) {
                tMax = endParam;
            }

            for (double t = startParam; t < tMax; t += 0.1) {
                if (t > endParam) {
                    t = endParam;
                }

                double root =
                    sqrt(a * a * pow(sin(t), 2) + b * b * pow(cos(t), 2));
                double x = (a + (b * k) / root) * cos(t);
                double y = (b + (a * k) / root) * sin(t);
                RVector v(x, y);
                v.rotate(ellipse->getAngle());
                v.move(center);

                pl->appendVertex(v);
            }

            if (ellipse->isFullEllipse()) {
                // no ellipse proxy: offset curve is polyline:
                pl->setClosed(true);
            }

            ret.push_back(std::shared_ptr<RShape>(pl));
        }
    }

    return ret;
}

std::vector<std::shared_ptr<RShape>>
REllipse::splitAt(const std::vector<RVector> &points) const
{
    if (points.size() == 0) {
        return RShape::splitAt(points);
    }

    std::vector<std::shared_ptr<RShape>> ret;

    if (m_reversed) {
        REllipse ellipse = *this;
        ellipse.reverse();
        ret = ellipse.splitAt(points);
        return RShapePrivate::getReversedShapeList(ret);
    }

    RVector startPoint = getStartPoint();
    RVector endPoint = getEndPoint();

    std::vector<RVector> sortedPoints = RVector::getSortedByAngle(
        points, m_center, m_center.getAngleTo(startPoint));

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

        REllipse *seg = clone();
        seg->setStartParam(seg->getParamTo(sortedPoints[i]));
        seg->setEndParam(seg->getParamTo(sortedPoints[i + 1]));
        ret.push_back(std::shared_ptr<RShape>(seg));
    }

    return ret;
}