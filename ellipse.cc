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

using namespace cada;

REllipseProxy *Ellipse::ellipseProxy = NULL;

/**
 * Creates an ellipse shape with invalid
 */
Ellipse::Ellipse()
    : center(Vec3d::invalid), majorPoint(Vec3d::invalid), ratio(0.0),
      startParam(0.0), endParam(0.0), reversed(false)
{
}

Ellipse::Ellipse(const Vec3d &center, const Vec3d &majorPoint, double ratio,
                 double startParam, double endParam, bool reversed)
    : center(center), majorPoint(majorPoint), ratio(ratio),
      startParam(startParam), endParam(endParam), reversed(reversed)
{

    correctMajorMinor();
}

Ellipse::~Ellipse()
{
}

/**
 * Produces an ellipse inscribed in the quadrilateral defined by the four given
 * ordered vertices (Vec3d).
 *
 * \param centerHint Hint for the position of the center (e.g. mouse cursor for
 * interactive tools) or invalid for maximum area solution.
 * \return List of ellipse [0] and center line [1].
 */
Ellipse Ellipse::createInscribed(const Vec3d &p1, const Vec3d &p2,
                                 const Vec3d &p3, const Vec3d &p4,
                                 const Vec3d &centerHint)
{
    Ellipse ret;

    if (Ellipse::hasProxy()) {
        ret = Ellipse::getEllipseProxy()->createInscribed(p1, p2, p3, p4,
                                                          centerHint);
    }

    return ret;
}

Ellipse Ellipse::createFrom4Points(const Vec3d &p1, const Vec3d &p2,
                                   const Vec3d &p3, const Vec3d &p4)
{
    Ellipse ret;

    if (Ellipse::hasProxy()) {
        ret = Ellipse::getEllipseProxy()->createFrom4Points(p1, p2, p3, p4);
    }

    return ret;
}

bool Ellipse::isValid() const
{
    return center.isValid() && majorPoint.isValid() && !Math::isNaN(ratio) &&
           !Math::isNaN(startParam) && !Math::isNaN(endParam);
}

std::vector<Vec3d> Ellipse::getFoci() const
{
    Vec3d vp(getMajorPoint() * sqrt(1.0 - getRatio() * getRatio()));
    return std::vector<Vec3d>() << getCenter() + vp << getCenter() - vp;
}

void Ellipse::moveStartPoint(const Vec3d &pos, bool changeAngleOnly)
{
    if (changeAngleOnly) {
        startParam = getParamTo(pos);
    }
    else {
        Vec3d ep = getEndPoint();
        double distOri = ep.getDistanceTo(getStartPoint());
        double angleOri = ep.getAngleTo(getStartPoint());
        if (distOri < NS::PointTolerance) {
            return;
        }

        double distNew = ep.getDistanceTo(pos);
        double angleNew = ep.getAngleTo(pos);
        double factor = distNew / distOri;
        if (factor < NS::PointTolerance) {
            return;
        }
        double angleDelta = angleNew - angleOri;

        center.scale(factor, ep);
        center.rotate(angleDelta, ep);
        majorPoint.scale(factor);
        majorPoint.rotate(angleDelta);
    }
}

void Ellipse::moveEndPoint(const Vec3d &pos, bool changeAngleOnly)
{
    if (changeAngleOnly) {
        endParam = getParamTo(pos);
    }
    else {
        Vec3d sp = getStartPoint();
        double distOri = sp.getDistanceTo(getEndPoint());
        double angleOri = sp.getAngleTo(getEndPoint());
        if (distOri < NS::PointTolerance) {
            return;
        }

        double distNew = sp.getDistanceTo(pos);
        double angleNew = sp.getAngleTo(pos);
        double factor = distNew / distOri;
        if (factor < NS::PointTolerance) {
            return;
        }
        double angleDelta = angleNew - angleOri;

        center.scale(factor, sp);
        center.rotate(angleDelta, sp);
        majorPoint.scale(factor);
        majorPoint.rotate(angleDelta);
    }
}

double Ellipse::getAngleAt(double distance, NS::From from) const
{
    // Q_UNUSED(distance)
    // Q_UNUSED(from)

    // TODO: getPointWithDistanceToStart not implemented for ellipses:
    //    Vec3d pos;
    //    if (from==NS::FromStart) {
    //        pos = getPointWithDistanceToStart(distance);
    //    }
    //    else {
    //        pos = getPointWithDistanceToEnd(distance);
    //    }

    //    return getAngleAtPoint(pos);

    return 0.0;
}

double Ellipse::getAngleAtPoint(const Vec3d &pos) const
{
    Vec3d posNormalized = pos;
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
double Ellipse::getParamTo(const Vec3d &pos) const
{
    Vec3d m = pos;
    m.rotate(-majorPoint.getAngle(), center);
    Vec3d v = m - center;
    v.scale(Vec3d(1.0, 1.0 / ratio));
    return v.getAngle();
}

/**
 * \return Radius of ellipse at given ellipse angle.
 */
double Ellipse::getRadiusAt(double param) const
{
    Vec3d v(cos(param) * getMajorRadius(), sin(param) * getMinorRadius());
    return v.getMagnitude();
}

/**
 * \return Point on ellipse at given ellipse angle.
 */
Vec3d Ellipse::getPointAt(double param) const
{
    Vec3d v(cos(param) * getMajorRadius(), sin(param) * getMinorRadius());
    v.rotate(getAngle());
    v.move(center);
    return v;
}

Vec3d Ellipse::getMiddlePoint() const
{
    double a;
    a = getStartParam() + getSweep() / 2.0;
    return getPointAt(a);
}

Vec3d Ellipse::getPointOnShape() const
{
    double sp = startParam;
    double ep = endParam;
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

Vec3d Ellipse::getCenter() const
{
    return center;
}

void Ellipse::setCenter(const Vec3d &vector)
{
    center = vector;
}

/**
 * \return Major point relative to the center point.
 */
Vec3d Ellipse::getMajorPoint() const
{
    return majorPoint;
}

/**
 * Sets the major point relative to the center point.
 */
void Ellipse::setMajorPoint(const Vec3d &p)
{
    majorPoint = p;
    correctMajorMinor();
}

/**
 * \return Minor point relative to the center point.
 */
Vec3d Ellipse::getMinorPoint() const
{
    double angle = Math::getNormalizedAngle(getAngle() + M_PI / 2.0);
    Vec3d ret;
    ret.setPolar(getMinorRadius(), angle);
    return ret;
}

/**
 * Sets the minor point relative to the center point.
 */
void Ellipse::setMinorPoint(const Vec3d &p)
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
    Vec3d vp_start = getStartPoint();
    Vec3d vp_end = getStartPoint();
    Vec3d vp = getMajorPoint();
    setMajorPoint(Vec3d(-ratio * vp.y, ratio * vp.x));
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

/**
 * \return Start angle: the angle from the ellipse
 * arc center to the ellipse arc start point.
 */
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

/**
 * \return End angle: the angle from the ellipse
 * arc center to the ellipse arc end point.
 */
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

        Line line(Vec3d(0, 0), Vec3d::createPolar(getMajorRadius() * 2, a));
        std::vector<Vec3d> r =
            Shape::getIntersectionPoints(line, normEllipse, true);
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

/**
 * \return Angle length in rad.
 */
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

Vec3d Ellipse::getStartPoint() const
{
    Vec3d p(center.x + cos(startParam) * getMajorRadius(),
            center.y + sin(startParam) * getMinorRadius());
    p.rotate(getAngle(), center);
    return p;
}

Vec3d Ellipse::getEndPoint() const
{
    Vec3d p(center.x + cos(endParam) * getMajorRadius(),
            center.y + sin(endParam) * getMinorRadius());
    p.rotate(getAngle(), center);
    return p;
}

/**
 * \return The major radius of this ellipse.
 */
double Ellipse::getMajorRadius() const
{
    return majorPoint.getMagnitude();
}

/**
 * \return The minor radius of this ellipse.
 */
double Ellipse::getMinorRadius() const
{
    return majorPoint.getMagnitude() * ratio;
}

/**
 * \return The rotation angle of this ellipse.
 */
double Ellipse::getAngle() const
{
    return majorPoint.getAngle();
}

/**
 * Sets the rotation angle of this ellipse without changing the major radius.
 */
void Ellipse::setAngle(double a)
{
    majorPoint = Vec3d::createPolar(majorPoint.getMagnitude(), a);
}

bool Ellipse::isFullEllipse() const
{
    double a1 = Math::getNormalizedAngle(startParam);
    double a2 = Math::getNormalizedAngle(endParam);
    return (a1 < NS::AngleTolerance && a2 > 2 * M_PI - NS::AngleTolerance) ||
           (fabs(a1 - a2) < NS::AngleTolerance);
}

/**
 * \return True if minor radius and major radius are equal, i.e. ratio is 1.0.
 */
bool Ellipse::isCircular() const
{
    return getRatio() > (1.0 - 0.001);
}

/**
 * \return Approximation for ellipse arc length.
 */
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

/**
 * \return Length of the ellipse segment from angle a1 to angle a2.
 */
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

bool Ellipse::contains(const Vec3d &p) const
{
    Vec3d pt = p;
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

//    std::vector<Vec3d> points = normal.getPointsWithDistanceToEnd(distance,
//    from); if (points.size()!=1) {
//        return std::numeric_limits<double>::quiet_NaN();
//    }

//    Vec3d p = points[0];

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

double Ellipse::getDirection1() const
{
    return getAngleAtPoint(getStartPoint());
    //    if (!reversed) {
    //        return Math::getNormalizedAngle(getAngle() +
    //        startParam+M_PI/2.0);
    //    }
    //    else {
    //        return Math::getNormalizedAngle(getAngle() +
    //        startParam-M_PI/2.0);
    //    }
}

double Ellipse::getDirection2() const
{
    return Math::getNormalizedAngle(getAngleAtPoint(getEndPoint()) + M_PI);
    //    if (!reversed) {
    //        return Math::getNormalizedAngle(getAngle() + endParam-M_PI/2.0);
    //    }
    //    else {
    //        return Math::getNormalizedAngle(getAngle() + endParam+M_PI/2.0);
    //    }
}

NS::Side Ellipse::getSideOfPoint(const Vec3d &point) const
{
    if (contains(point)) {
        if (!reversed) {
            return NS::RightHand;
        }
        else {
            return NS::LeftHand;
        }
    }
    else {
        if (!reversed) {
            return NS::LeftHand;
        }
        else {
            return NS::RightHand;
        }
    }
}

BBox Ellipse::getBoundingBox() const
{
    double radius1 = getMajorRadius();
    double radius2 = getMinorRadius();
    double angle = getAngle();
    double a1 = ((!isReversed()) ? startParam : endParam);
    double a2 = ((!isReversed()) ? endParam : startParam);
    Vec3d startPoint = getStartPoint();
    Vec3d endPoint = getEndPoint();

    double minX = qMin(startPoint.x, endPoint.x);
    double minY = qMin(startPoint.y, endPoint.y);
    double maxX = qMax(startPoint.x, endPoint.x);
    double maxY = qMax(startPoint.y, endPoint.y);

    // kind of a brute force. TODO: exact calculation
    Vec3d vp;
    double a = a1;
    do {
        vp.set(center.x + radius1 * cos(a), center.y + radius2 * sin(a));
        vp.rotate(angle, center);

        minX = qMin(minX, vp.x);
        minY = qMin(minY, vp.y);
        maxX = qMax(maxX, vp.x);
        maxY = qMax(maxY, vp.y);

        a += 0.03;
    } while (Math::isAngleBetween(a, a1, a2, false) && a < 4 * M_PI);

    return BBox(Vec3d(minX, minY), Vec3d(maxX, maxY));
}

std::vector<Vec3d> Ellipse::getEndPoints() const
{
    std::vector<Vec3d> ret;
    ret.push_back(getStartPoint());
    ret.push_back(getEndPoint());
    return ret;
}

/**
 * \todo implement
 */
std::vector<Vec3d> Ellipse::getMiddlePoints() const
{
    std::vector<Vec3d> ret;
    // ret.push_back(getMiddlePoint());
    return ret;
}

std::vector<Vec3d> Ellipse::getCenterPoints() const
{
    std::vector<Vec3d> ret;
    ret.push_back(getCenter());
    return ret;
}

/**
 * \todo implement
 */
std::vector<Vec3d> Ellipse::getPointsWithDistanceToEnd(double distance,
                                                       int from) const
{
    // Q_UNUSED(distance)
    // Q_UNUSED(from)

    std::vector<Vec3d> ret;
    return ret;
}

std::vector<Vec3d> Ellipse::getPointCloud(double segmentLength) const
{
    Polyline pl = approximateWithArcs(64);
    return pl.getPointCloud(segmentLength);
}

Vec3d Ellipse::getVectorTo(const Vec3d &point, bool limited,
                           double strictRange) const
{
    // Q_UNUSED(strictRange)

    Vec3d ret = Vec3d::invalid;

    double ang = getAngle();
    // double dDistance = RMAXDOUBLE;
    bool swap = false;
    bool majorSwap = false;

    Vec3d normalized = (point - center).get2D().rotate(-ang);

    // special case: point in line with major axis:
    if (fabs(normalized.getAngle()) < NS::AngleTolerance ||
        fabs(normalized.getAngle()) > 2 * M_PI - NS::AngleTolerance) {
        ret = Vec3d(getMajorRadius(), 0.0);
        // dDistance = ret.distanceTo(normalized);
    }

    else if (fabs(normalized.getAngle() - M_PI) < NS::AngleTolerance) {
        ret = Vec3d(-getMajorRadius(), 0.0);
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
            ret = Vec3d::invalid;
        }
        else {
            // double dDelta0 = rdX - dU;
            // double dDelta1 = rdY - dV;
            // dDistance = sqrt(dDelta0*dDelta0 + dDelta1*dDelta1);
            ret = Vec3d(rdX, rdY);
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
        ret = (ret.rotate(ang) + center);

        if (limited) {
            double a1 = center.getAngleTo(getStartPoint());
            double a2 = center.getAngleTo(getEndPoint());
            double a = center.getAngleTo(ret);
            if (!Math::isAngleBetween(a, a1, a2, reversed)) {
                ret = Vec3d::invalid;
            }
        }
    }

    /*
    if (dist!=NULL) {
        if (ret.valid) {
            *dist = dDistance;
        } else {
            *dist = RS_MAXDOUBLE;
        }
    }

    if (entity!=NULL) {
        if (ret.valid) {
            *entity = this;
        }
        else {
            *entity = NULL;
        }
    }
    */

    return point - ret;
}

bool Ellipse::move(const Vec3d &offset)
{
    if (!offset.isValid() || offset.getMagnitude() < NS::PointTolerance) {
        return false;
    }
    center += offset;
    return true;
}

bool Ellipse::rotate(double rotation, const Vec3d &c)
{
    if (fabs(rotation) < NS::AngleTolerance) {
        return false;
    }

    center.rotate(rotation, c);
    majorPoint.rotate(rotation);

    return true;
}

std::vector<Vec3d> Ellipse::getBoxCorners()
{
    std::vector<Vec3d> ret;

    Vec3d minorPoint = getMinorPoint();
    ret.push_back(center + majorPoint + minorPoint);
    ret.push_back(center + majorPoint - minorPoint);
    ret.push_back(center - majorPoint - minorPoint);
    ret.push_back(center - majorPoint + minorPoint);

    return ret;
}

bool Ellipse::scale(const Vec3d &scaleFactors, const Vec3d &c)
{
    if (fabs(fabs(scaleFactors.x) - fabs(scaleFactors.y)) >
        NS::PointTolerance) {
        qWarning("Ellipse::scale: scaling with different factors in X/Y not "
                 "supported for ellipses at this point");
        return false;
    }

    // Vec3d oldMinorPoint = getMinorPoint();

    // negative scaling: mirroring and scaling
    if (scaleFactors.x < 0.0) {
        mirror(Line(center, center + Vec3d(0.0, 1.0)));
    }
    if (scaleFactors.y < 0.0) {
        mirror(Line(center, center + Vec3d(1.0, 0.0)));
    }

    center.scale(scaleFactors, c);

    // oldMinorPoint.scale(scaleFactors);

    Vec3d f =
        Vec3d(fabs(scaleFactors.x), fabs(scaleFactors.y), fabs(scaleFactors.z));
    majorPoint.scale(f);

    //    if (fabs(majorPoint.getMagnitude()) > 1.0e-4) {
    //        ratio = oldMinorPoint.getMagnitude() / majorPoint.getMagnitude();
    //    }

    return true;

    //    std::vector<Vec3d> box = getBoxCorners();
    //    Vec3d::scaleList(box, scaleFactors, c);
    //    // TODO:
    //    Ellipse e = Ellipse::createInscribed(box);
    //    //*this = e;

    //    return true;
}

bool Ellipse::mirror(const Line &axis)
{
    Vec3d mp = center + majorPoint;
    Vec3d sp = getStartPoint();
    Vec3d ep = getEndPoint();

    center.mirror(axis);
    mp.mirror(axis);

    majorPoint = mp - center;

    if (!isFullEllipse()) {
        reversed = (!reversed);

        sp.mirror(axis);
        setStartParam(getParamTo(sp));

        ep.mirror(axis);
        setEndParam(getParamTo(ep));
    }

    return true;
}

bool Ellipse::reverse()
{
    double a = startParam;
    startParam = endParam;
    endParam = a;
    reversed = !reversed;
    return true;
}

NS::Ending Ellipse::getTrimEnd(const Vec3d &trimPoint, const Vec3d &clickPoint)
{
    double paramToClickPoint = getParamTo(clickPoint);
    double paramToTrimPoint = getParamTo(trimPoint);

    if (Math::getAngleDifference(paramToTrimPoint, paramToClickPoint) > M_PI) {
        return NS::EndingStart;
    }
    else {
        return NS::EndingEnd;
    }
}

bool Ellipse::trimStartPoint(const Vec3d &trimPoint, const Vec3d &clickPoint,
                             bool extend)
{
    // Q_UNUSED(clickPoint)
    // Q_UNUSED(extend)
    setStartParam(getParamTo(trimPoint));
    return true;
}

bool Ellipse::trimEndPoint(const Vec3d &trimPoint, const Vec3d &clickPoint,
                           bool extend)
{
    // Q_UNUSED(clickPoint)
    // Q_UNUSED(extend)
    setEndParam(getParamTo(trimPoint));
    return true;
}

void Ellipse::correctMajorMinor()
{
    if (ratio > 1.0) {
        Vec3d mp = getMinorPoint();
        ratio = 1.0 / ratio;
        setMajorPoint(mp);
        startParam = Math::getNormalizedAngle(startParam - M_PI / 2.0);
        endParam = Math::getNormalizedAngle(endParam - M_PI / 2.0);
    }
}

/**
 * \return Arc sweep in rad. The sweep is the angle covered by this arc.
 * Positive for ccw, negative for cw.
 */
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

std::vector<Line> Ellipse::getTangents(const Vec3d &point) const
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
    Vec3d point2 = point;
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

    Vec3d s1(xt1, yt1);
    s1.rotate(getAngle());
    s1.move(getCenter());

    Vec3d s2(xt2, yt2);
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

/**
 * \return Tangent point of the given line to this tangent or an invalid vector
 * if the line is not a tangent
 */
Vec3d Ellipse::getTangentPoint(const Line &line) const
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

        return Vec3d::invalid;
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

        Vec3d ret = Vec3d(x, y);
        ret.rotate(getAngle());
        ret.move(getCenter());
        return ret;
    }

    return Vec3d::invalid;
}

std::vector<BSpline> Ellipse::approximateWithSplines() const
{
    if (Ellipse::hasProxy()) {
        return Ellipse::getEllipseProxy()->approximateWithSplines(*this);
    }
    return std::vector<BSpline>();
}

Polyline Ellipse::approximateWithArcs(int segments) const
{
    if (Ellipse::hasProxy()) {
        return Ellipse::getEllipseProxy()->approximateWithArcs(*this, segments);
    }
    return Polyline();
}

/**
 * \return Array of spline shapes representing the parallel curves to this
 * ellipse shape.
 */
std::vector<std::shared_ptr<Shape>>
Ellipse::getOffsetShapes(double distance, int number, NS::Side side,
                         const Vec3d &position)
{
    errorCode = 0;
    std::vector<std::shared_ptr<Shape>> ret;
    Ellipse *ellipse = dynamic_cast<Ellipse *>(clone());
    if (ellipse == NULL) {
        return ret;
    }

    Vec3d center = ellipse->getCenter();

    if (ellipse->isReversed()) {
        ellipse->reverse();
    }

    std::vector<bool> insides;
    if (position.isValid()) {
        double ang = center.getAngleTo(position) - ellipse->getAngle();
        double t = ellipse->angleToParam(ang);
        Vec3d p = ellipse->getPointAt(t);
        insides.push_back(center.getDistanceTo(position) <
                          center.getDistanceTo(p));
    }
    else {
        if (side == NS::BothSides) {
            insides.push_back(true);
            insides.push_back(false);
        }
        else {
            if (side == NS::LeftHand) {
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
            BSpline *spl = NULL;
            Polyline *pl = NULL;
            if (BSpline::hasProxy()) {
                spl = new BSpline();
            }
            else {
                pl = new Polyline();
            }

            double endParam = ellipse->getEndParam();
            double startParam = ellipse->getStartParam();
            if (Math::fuzzyCompare(endParam, 0.0)) {
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
                Vec3d v(x, y);
                v.rotate(ellipse->getAngle());
                v.move(center);
                if (spl != NULL) {
                    spl->appendFitPoint(v);
                }
                else {
                    pl->appendVertex(v);
                }
            }

            if (ellipse->isFullEllipse()) {
                if (spl != NULL) {
                    spl->setPeriodic(true);
                }
                else {
                    // no ellipse proxy: offset curve is polyline:
                    pl->setClosed(true);
                }
            }

            if (spl != NULL) {
                ret.push_back(std::shared_ptr<Shape>(spl));
            }
            else {
                ret.push_back(std::shared_ptr<Shape>(pl));
            }
        }
    }

    return ret;
}

std::vector<std::shared_ptr<Shape>>
Ellipse::splitAt(const std::vector<Vec3d> &points) const
{
    if (points.size() == 0) {
        return Shape::splitAt(points);
    }

    std::vector<std::shared_ptr<Shape>> ret;

    if (reversed) {
        Ellipse ellipse = *this;
        ellipse.reverse();
        ret = ellipse.splitAt(points);
        return Shape::getReversedShapeList(ret);
    }

    Vec3d startPoint = getStartPoint();
    Vec3d endPoint = getEndPoint();

    std::vector<Vec3d> sortedPoints =
        Vec3d::getSortedByAngle(points, center, center.getAngleTo(startPoint));

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

        Ellipse *seg = clone();
        seg->setStartParam(seg->getParamTo(sortedPoints[i]));
        seg->setEndParam(seg->getParamTo(sortedPoints[i + 1]));
        ret.push_back(std::shared_ptr<Shape>(seg));
    }

    return ret;
}
