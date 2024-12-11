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

#include <cada2d/RBox.h>
#include <cada2d/RLine.h>
#include <cada2d/RMath.h>
#include <cada2d/RPolyline.h>
#include <cada2d/RVector.h>

#include <algorithm>

class RVectorDistanceSort {
public:
    static bool lessThan(const RVector &v1, const RVector &v2);
    static RVector v;
};

class RVectorLeftRightTopBottomSort {
public:
    static bool lessThan(const RVector &v1, const RVector &v2);
};

class RVectorAngleSort {
public:
    static bool lessThan(const RVector &v1, const RVector &v2);
    static RVector center;
    static double angle;
};

const RVector RVector::invalid = RVector(0, 0, false);
const RVector RVector::nullVector = RVector(0, 0, true);
const RVector RVector::nanVector = RVector(RNANDOUBLE, RNANDOUBLE, true);

RVector RVectorDistanceSort::v;
RVector RVectorAngleSort::center;
double RVectorAngleSort::angle = 0.0;

RVector::RVector(double vx, double vy, bool valid_in) : x(vx), y(vy)
{

    valid = valid_in && RMath::isNormal(x) && RMath::isNormal(y);
}

RVector::~RVector()
{
}

bool RVector::isValid() const
{
    return valid;
}

bool RVector::isZero() const
{
    return fabs(x) < RS::PointTolerance && fabs(y) < RS::PointTolerance;
}

bool RVector::isSane() const
{
    return isValid() && RMath::isSane(x) && RMath::isSane(y);
}

bool RVector::isNaN() const
{
    return RMath::isNaN(x) || RMath::isNaN(y);
}

void RVector::setX(double x)
{
    this->x = x;
}

double RVector::getX()
{
    return x;
}

void RVector::setY(double y)
{
    this->y = y;
}

double RVector::getY()
{
    return y;
}

void RVector::setPolar(double radius, double angle)
{
    x = radius * cos(angle);
    y = radius * sin(angle);
    valid = RMath::isNormal(radius) && RMath::isNormal(angle);
}

void RVector::setAngle(double a)
{
    double m = getMagnitude();
    setPolar(m, a);
}

double RVector::getAngle() const
{
    double ret = 0.0;
    double m = getMagnitude();

    if (m > 1.0e-6) {
        double dp = getDotProduct(*this, RVector(1.0, 0.0));
        if (dp / m >= 1.0) {
            ret = 0.0;
        }
        else if (dp / m < -1.0) {
            ret = M_PI;
        }
        else {
            ret = acos(dp / m);
        }
        if (y < 0.0) {
            ret = 2 * M_PI - ret;
        }
    }
    return ret;
}

double RVector::getAngleToPlaneXY() const
{
    RVector n(0, 0, 1);

    if (getMagnitude() < 1.0e-4) {
        return M_PI / 2;
    }
    else if ((getDotProduct(*this, n) / (getMagnitude() * 1)) > 1.0) {
        return 0.0;
    }
    else {
        return M_PI / 2 - acos(getDotProduct(*this, n) / (getMagnitude() * 1));
    }
}

double RVector::getAngleTo(const RVector &v) const
{
    if (!valid || !v.valid) {
        return RNANDOUBLE;
    }
    else {
        return (v - *this).getAngle();
    }
}

void RVector::setMagnitude(double m)
{
    double a = getAngle();
    setPolar(m, a);
}

double RVector::getMagnitude() const
{
    if (!valid) {
        return RNANDOUBLE;
    }
    // Note that the z coordinate is also needed for 2d
    //   (due to definition of crossP())
    return sqrt(x * x + y * y);
}

double RVector::getSquaredMagnitude() const
{
    if (!valid) {
        return RNANDOUBLE;
    }

    return x * x + y * y;
}

RVector RVector::getLerp(const RVector &v, double t) const
{
    return RVector(x + (v.x - x) * t, y + (v.y - y) * t);
}

RVector RVector::getUnitVector() const
{
    return *this / getMagnitude();
}

bool RVector::isInside(const RBox &b) const
{
    RVector bMin = b.getMinimum();
    RVector bMax = b.getMaximum();

    return (x >= bMin.x && x <= bMax.x && y >= bMin.y && y <= bMax.y);
}

bool RVector::equalsFuzzy(const RVector &v, double tol) const
{
    return (std::fabs(x - v.x) < tol && std::fabs(y - v.y) < tol &&
            valid == v.valid);
}

double RVector::getDistanceTo(const RVector &v) const
{
    if (!valid || !v.valid) {
        return RNANDOUBLE;
    }
    else {
        return (*this - v).getMagnitude();
    }
}

RVector RVector::move(const RVector &offset)
{
    *this += offset;
    return *this;
}

void RVector::moveList(std::vector<RVector> &list, const RVector &offset)
{
    for (int i = 0; i < list.size(); i++) {
        list[i].move(offset);
    }
}

RVector RVector::rotate(double rotation)
{
    if (!valid) {
        return *this;
    }

    double r = getMagnitude();
    double a = getAngle() + rotation;

    x = cos(a) * r;
    y = sin(a) * r;

    return *this;
}

RVector RVector::rotate(double rotation, const RVector &center)
{
    *this = center + (*this - center).rotate(rotation);
    return *this;
}

RVector RVector::getRotated(double rotation, const RVector &center) const
{
    RVector ret = *this;
    ret.rotate(rotation, center);
    return ret;
}

RVector RVector::scale(double factor, const RVector &center)
{
    return scale(RVector(factor, factor, factor), center);
}

RVector RVector::scale(const RVector &factors, const RVector &center)
{
    if (center == RVector()) {
        x *= factors.x;
        y *= factors.y;
        return *this;
    }

    *this = center + (*this - center).scale(factors);
    return *this;
}

RVector RVector::getScaled(const RVector &factors, const RVector &center) const
{
    RVector ret = *this;
    ret.scale(factors, center);
    return ret;
}

RVector RVector::mirror(const RVector &axis1, const RVector &axis2)
{

    double phi1 = axis1.getAngleTo(*this);
    double phi2 = axis1.getAngleTo(axis2) - phi1;
    double r1 = axis1.getDistanceTo(*this);
    double r2 = axis2.getDistanceTo(*this);

    if (r1 < 1.0e-6 || r2 < 1.0e-6) {
        // point touches one axis point
    }
    else {
        setPolar(r1, phi1 + 2 * phi2);
        (*this) += axis1;
    }

    return *this;
}

RVector RVector::flipHorizontal()
{
    return mirror(RVector(0, 0, 0), RVector(0, 1, 0));
}

RVector RVector::flipVertical()
{
    return mirror(RVector(0, 0, 0), RVector(1, 0, 0));
}

RVector RVector::stretch(const RPolyline &area, const RVector &offset)
{
    if (area.contains(*this, true)) {
        return move(offset);
    }
    return *this;
}

RVector RVector::operator+(const RVector &v) const
{
    return RVector(x + v.x, y + v.y, valid && v.valid);
}

RVector RVector::operator-(const RVector &v) const
{
    return RVector(x - v.x, y - v.y, valid && v.valid);
}

RVector RVector::operator*(double s) const
{
    return RVector(x * s, y * s, valid);
}

RVector RVector::operator/(double s) const
{
    return RVector(x / s, y / s, valid);
}

RVector RVector::operator-() const
{
    return getNegated();
}

RVector RVector::getNegated() const
{
    return RVector(-x, -y, valid);
}

RVector RVector::getAbsolute() const
{
    return RVector(fabs(x), fabs(y));
}

RVector RVector::normalize()
{
    *this = getNormalized();
    return *this;
}

RVector RVector::getNormalized() const
{
    double l = getMagnitude();
    if (l < RS::PointTolerance) {
        return RVector::invalid;
    }
    return *this / l;
}

double RVector::getDotProduct(const RVector &v1, const RVector &v2)
{
    return v1.x * v2.x + v1.y * v2.y;
}

void RVector::operator+=(const RVector &v)
{
    x += v.x;
    y += v.y;
    valid = valid && v.valid;
}

void RVector::operator-=(const RVector &v)
{
    x -= v.x;
    y -= v.y;
    valid = valid && v.valid;
}

void RVector::operator*=(double s)
{
    x *= s;
    y *= s;
}

void RVector::operator/=(double s)
{
    x /= s;
    y /= s;
}

bool RVector::operator==(const RVector &v) const
{
    if (valid == true && v.valid == true) {
        return x == v.x && y == v.y;
    }
    else if (valid == false && v.valid == false) {
        return true;
    }
    return false;
}

bool RVector::containsFuzzy(const std::vector<RVector> &vectors,
                            const RVector &v, double tol)
{
    return findFirstFuzzy(vectors, v, tol) != -1;
}

int RVector::findFirstFuzzy(const std::vector<RVector> &vectors,
                            const RVector &v, double tol)
{
    for (int i = 0; i < vectors.size(); i++) {
        if (v.equalsFuzzy(vectors[i], tol)) {
            return i;
        }
    }

    return -1;
}

RVector RVector::getMinimum(const std::vector<RVector> &vectors)
{
    if (vectors.size() == 0) {
        return RVector();
    }

    RVector ret = vectors[0];

    std::vector<RVector>::const_iterator it = vectors.begin();
    it++;
    for (; it != vectors.end(); it++) {
        ret = getMinimum(ret, *it);
    }

    return ret;
}

RVector RVector::getMaximum(const std::vector<RVector> &vectors)
{
    if (vectors.size() == 0) {
        return RVector();
    }

    RVector ret = vectors[0];

    std::vector<RVector>::const_iterator it = vectors.begin();
    it++;
    for (; it != vectors.end(); it++) {
        ret = getMaximum(ret, *it);
    }

    return ret;
}

RVector RVector::getMinimumX(const std::vector<RVector> &vectors)
{
    if (vectors.size() == 0) {
        return RVector();
    }

    RVector ret = vectors[0];

    for (int i = 0; i < vectors.size(); i++) {
        if (vectors[i].x < ret.x) {
            ret = vectors[i];
        }
    }

    return ret;
}

RVector RVector::getMaximumX(const std::vector<RVector> &vectors)
{
    if (vectors.size() == 0) {
        return RVector();
    }

    RVector ret = vectors[0];

    for (int i = 0; i < vectors.size(); i++) {
        if (vectors[i].x > ret.x) {
            ret = vectors[i];
        }
    }

    return ret;
}

RVector RVector::getMinimumY(const std::vector<RVector> &vectors)
{
    if (vectors.size() == 0) {
        return RVector();
    }

    RVector ret = vectors[0];

    for (int i = 0; i < vectors.size(); i++) {
        if (vectors[i].y < ret.y) {
            ret = vectors[i];
        }
    }

    return ret;
}

RVector RVector::getMaximumY(const std::vector<RVector> &vectors)
{
    if (vectors.size() == 0) {
        return RVector();
    }

    RVector ret = vectors[0];

    for (int i = 0; i < vectors.size(); i++) {
        if (vectors[i].y > ret.y) {
            ret = vectors[i];
        }
    }

    return ret;
}

RVector RVector::getMinimum(const RVector &v1, const RVector &v2)
{
    return RVector(std::min(v1.x, v2.x), std::min(v1.y, v2.y),
                   v1.valid && v2.valid);
}

RVector RVector::getMaximum(const RVector &v1, const RVector &v2)
{
    return RVector(std::max(v1.x, v2.x), std::max(v1.y, v2.y),
                   v1.valid && v2.valid);
}

RVector RVector::getAverage(const RVector &v1, const RVector &v2)
{
    return (v1 + v2) / 2.0;
}

RVector RVector::getAverage(const std::vector<RVector> &vectors)
{
    RVector sum = RVector::nullVector;
    for (int i = 0; i < vectors.size(); i++) {
        sum += vectors[i];
    }
    return sum / vectors.size();
}

RVector RVector::getFloor() const
{
    return RVector(floorl(x), floorl(y), valid);
}

RVector RVector::getCeil() const
{
    return RVector(ceill(x), ceill(y), valid);
}

double RVector::getCrossProduct(const RVector &v1, const RVector &v2)
{
    return v1.x * v2.y - v1.y * v2.x;
}

RVector RVector::getDividedComponents(const RVector &v) const
{
    return RVector(x / v.x, y / v.y, valid);
}

RVector RVector::getMultipliedComponents(const RVector &v) const
{
    return RVector(x * v.x, y * v.y, valid);
}

RVector RVector::getClosest(const std::vector<RVector> &list) const
{
    int index = getClosestIndex(list);
    if (index == -1) {
        return RVector::invalid;
    }
    return list[index];
}

double RVector::getClosestDistance(const std::vector<RVector> &list, int counts)
{
    double ret = RMAXDOUBLE;
    int i = list.size();
    if (counts < i) {
        i = counts;
    }
    if (i < 1) {
        return ret;
    }
    for (int j = 0; j < i; j++) {
        double d = getDistanceTo(list[j]);
        if (d < ret) {
            ret = d;
        }
    }
    return ret;
}

std::vector<RVector>
RVector::getSortedByDistance(const std::vector<RVector> &list, const RVector &v)
{
    RVectorDistanceSort::v = v;
    std::vector<RVector> ret = list;
    std::sort(ret.begin(), ret.end(), RVectorDistanceSort::lessThan);
    return ret;
}

std::vector<RVector>
RVector::getSortedLeftRightTopBottom(const std::vector<RVector> &list)
{
    std::vector<RVector> ret = list;
    std::sort(ret.begin(), ret.end(), RVectorLeftRightTopBottomSort::lessThan);
    return ret;
}

std::vector<RVector> RVector::getSortedByAngle(const std::vector<RVector> &list,
                                               const RVector &center,
                                               double angle)
{
    RVectorAngleSort::center = center;
    RVectorAngleSort::angle = angle;
    std::vector<RVector> ret = list;
    std::sort(ret.begin(), ret.end(), RVectorAngleSort::lessThan);
    return ret;
}

RVector operator*(double s, const RVector &v)
{
    return v * s;
}

bool RVectorDistanceSort::lessThan(const RVector &v1, const RVector &v2)
{
    return v.getDistanceTo(v1) < v.getDistanceTo(v2);
}

bool RVectorLeftRightTopBottomSort::lessThan(const RVector &v1,
                                             const RVector &v2)
{
    return v1.y > v2.y || (v1.y == v2.y && v1.x < v2.x);
}

bool RVectorAngleSort::lessThan(const RVector &v1, const RVector &v2)
{
    double a1 = center.getAngleTo(v1);
    double a2 = center.getAngleTo(v2);

    double diff1 = RMath::getAngleDifference(angle, a1);
    if (RMath::fuzzyAngleCompare(diff1, M_PI * 2)) {
        diff1 = 0.0;
    }
    double diff2 = RMath::getAngleDifference(angle, a2);
    if (RMath::fuzzyAngleCompare(diff2, M_PI * 2)) {
        diff2 = 0.0;
    }
    return diff1 < diff2;
}
