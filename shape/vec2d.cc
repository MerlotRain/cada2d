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

#include <assert.h>
#include <math.h>
#include <stdexcept>

namespace cada {

class RVectorAngleSort {
public:
    static bool lessThan(const Vec2d &v1, const Vec2d &v2);
    static Vec2d center;
    static double angle;
};

class RVectorDistanceSort {
public:
    static bool lessThan(const Vec2d &v1, const Vec2d &v2);
    static Vec2d v;
};

Vec2d RVectorDistanceSort::v;
Vec2d RVectorAngleSort::center;
double RVectorAngleSort::angle = 0.0;

const Vec2d Vec2d::invalid = Vec2d(0, 0, false);
const Vec2d Vec2d::nullVector = Vec2d(0, 0, true);
const Vec2d Vec2d::nanVector =
    Vec2d(std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::quiet_NaN(), true);

Vec2d::Vec2d() : x(0.0), y(0.0), valid(true)
{
}

Vec2d::Vec2d(double vx, double vy, bool valid_in)
    : x(vx), y(vy)
{

    valid = valid_in && Math::isNormal(x) && Math::isNormal(y);
}

Vec2d::Vec2d(const std::vector<double> &tuples)
{
    if (tuples.size() > 0) {
        x = tuples[0];
    }
    if (tuples.size() > 1) {
        y = tuples[1];
    }
    valid = true;
}

Vec2d::~Vec2d()
{
}
void Vec2d::set(double vx, double vy, double vz = 0.0)
{
    x = vx;
    y = vy;
    valid = true;
}

bool Vec2d::isValid() const
{
    return valid;
}

bool Vec2d::isZero() const
{
    return fabs(x) < NS::PointTolerance && fabs(y) < NS::PointTolerance;
}

bool Vec2d::isSane() const
{
    return isValid() && Math::isSane(x) && Math::isSane(y);
}

bool Vec2d::isNaN() const
{
    return Math::isNaN(x) || Math::isNaN(y);
}

void Vec2d::setPolar(double radius, double angle)
{
    x = radius * cos(angle);
    y = radius * sin(angle);
    valid = Math::isNormal(radius) && Math::isNormal(angle);
}

void Vec2d::setAngle(double a)
{
    double m = getMagnitude();
    setPolar(m, a);
}

double Vec2d::getAngle() const
{
    double ret = 0.0;
    double m = getMagnitude();

    if (m > 1.0e-6) {
        double dp = getDotProduct(*this, Vec2d(1.0, 0.0));
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

double Vec2d::getAngleTo(const Vec2d &v) const
{
    if (!valid || !v.valid) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    else {
        return (v - *this).getAngle();
    }
}

void Vec2d::setMagnitude(double m)
{
    double a = getAngle();
    setPolar(m, a);
}

double Vec2d::getMagnitude() const
{
    if (!valid) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return sqrt(x * x + y * y);
}

double Vec2d::getSquaredMagnitude() const
{
    if (!valid) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    return x * x + y * y;
}

Vec2d Vec2d::getLerp(const Vec2d &v, double t) const
{
    return Vec2d(x + (v.x - x) * t, y + (v.y - y) * t);
}

Vec2d Vec2d::getUnitVector() const
{
    return *this / getMagnitude();
}

bool Vec2d::equalsFuzzy(const Vec2d &v, double tol) const
{
    return (std::fabs(x - v.x) < tol && std::fabs(y - v.y) < tol &&
            valid == v.valid);
}

double Vec2d::getDistanceTo(const Vec2d &v) const
{
    if (!valid || !v.valid) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    else {
        return (*this - v).getMagnitude();
    }
}

Vec2d Vec2d::move(const Vec2d &offset)
{
    *this += offset;
    return *this;
}

Vec2d Vec2d::rotate(double rotation)
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

Vec2d Vec2d::rotate(double rotation, const Vec2d &center)
{
    *this = center + (*this - center).rotate(rotation);
    return *this;
}

Vec2d Vec2d::getRotated(double rotation, const Vec2d &center) const
{
    Vec2d ret = *this;
    ret.rotate(rotation, center);
    return ret;
}

Vec2d Vec2d::scale(double factor, const Vec2d &center)
{
    return scale(Vec2d(factor, factor, factor), center);
}

Vec2d Vec2d::scale(const Vec2d &factors, const Vec2d &center)
{
    if (center == Vec2d()) {
        x *= factors.x;
        y *= factors.y;
        return *this;
    }

    *this = center + (*this - center).scale(factors);
    return *this;
}

Vec2d Vec2d::getScaled(const Vec2d &factors, const Vec2d &center) const
{
    Vec2d ret = *this;
    ret.scale(factors, center);
    return ret;
}

Vec2d Vec2d::mirror(const Line &axis)
{
    double phi1 = axis.startPoint.getAngleTo(*this);
    double phi2 = axis.startPoint.getAngleTo(axis.endPoint) - phi1;
    double r1 = axis.startPoint.getDistanceTo(*this);
    double r2 = axis.endPoint.getDistanceTo(*this);

    if (r1 < 1.0e-6 || r2 < 1.0e-6) {
        // point touches one axis point
    }
    else {
        setPolar(r1, phi1 + 2 * phi2);
        (*this) += axis.startPoint;
    }

    return *this;
}

Vec2d Vec2d::getMirrored(const Line &axis) const
{
    Vec2d ret = *this;
    ret.mirror(axis);
    return ret;
}

Vec2d Vec2d::mirror(const Vec2d &axis1, const Vec2d &axis2)
{
    return mirror(Line(axis1, axis2));
}

Vec2d Vec2d::flipHorizontal()
{
    return mirror(Vec2d(0, 0, 0), Vec2d(0, 1, 0));
}

Vec2d Vec2d::flipVertical()
{
    return mirror(Vec2d(0, 0, 0), Vec2d(1, 0, 0));
}

Vec2d Vec2d::operator+(const Vec2d &v) const
{
    return Vec2d(x + v.x, y + v.y, valid && v.valid);
}

Vec2d Vec2d::operator-(const Vec2d &v) const
{
    return Vec2d(x - v.x, y - v.y, valid && v.valid);
}

Vec2d Vec2d::operator*(double s) const
{
    return Vec2d(x * s, y * s, valid);
}

Vec2d Vec2d::operator/(double s) const
{
    return Vec2d(x / s, y / s, valid);
}

Vec2d Vec2d::operator-() const
{
    return getNegated();
}

Vec2d Vec2d::getNegated() const
{
    return Vec2d(-x, -y, valid);
}

Vec2d Vec2d::getAbsolute() const
{
    return Vec2d(fabs(x), fabs(y));
}

double Vec2d::dot(const Vec2d &other) const
{
    return Vec2d::getDotProduct(*this, other);
}

Vec2d Vec2d::normalize()
{
    *this = getNormalized();
    return *this;
}

Vec2d Vec2d::getNormalized() const
{
    double l = getMagnitude();
    if (l < NS::PointTolerance) {
        return Vec2d::invalid;
    }
    return *this / l;
}

double Vec2d::getDotProduct(const Vec2d &v1, const Vec2d &v2)
{
    return v1.x * v2.x + v1.y * v2.y;
}

void Vec2d::operator+=(const Vec2d &v)
{
    x += v.x;
    y += v.y;
    valid = valid && v.valid;
}

void Vec2d::operator-=(const Vec2d &v)
{
    x -= v.x;
    y -= v.y;
    valid = valid && v.valid;
}

void Vec2d::operator*=(double s)
{
    x *= s;
    y *= s;
}

void Vec2d::operator/=(double s)
{
    x /= s;
    y /= s;
}

bool Vec2d::operator==(const Vec2d &v) const
{
    if (valid == true && v.valid == true) {
        return x == v.x && y == v.y;
    }
    else if (valid == false && v.valid == false) {
        return true;
    }
    return false;
}

bool Vec2d::containsFuzzy(const std::vector<Vec2d> &vectors, const Vec2d &v,
                          double tol)
{
    return findFirstFuzzy(vectors, v, tol) != -1;
}

int Vec2d::findFirstFuzzy(const std::vector<Vec2d> &vectors, const Vec2d &v,
                          double tol)
{
    for (int i = 0; i < vectors.size(); i++) {
        if (v.equalsFuzzy(vectors[i], tol)) {
            return i;
        }
    }

    return -1;
}

Vec2d Vec2d::getMinimum(const std::vector<Vec2d> &vectors)
{
    if (vectors.size() == 0) {
        return Vec2d();
    }

    Vec2d ret = vectors[0];

    std::vector<Vec2d>::const_iterator it = vectors.begin();
    it++;
    for (; it != vectors.end(); it++) {
        ret = getMinimum(ret, *it);
    }

    return ret;
}

Vec2d Vec2d::getMaximum(const std::vector<Vec2d> &vectors)
{
    if (vectors.size() == 0) {
        return Vec2d();
    }

    Vec2d ret = vectors[0];

    std::vector<Vec2d>::const_iterator it = vectors.begin();
    it++;
    for (; it != vectors.end(); it++) {
        ret = getMaximum(ret, *it);
    }

    return ret;
}

Vec2d Vec2d::getMinimum(const Vec2d &v1, const Vec2d &v2)
{
    return Vec2d(std::min(v1.x, v2.x), std::min(v1.y, v2.y), v1.valid && v2.valid);
}

Vec2d Vec2d::getMaximum(const Vec2d &v1, const Vec2d &v2)
{
    return Vec2d(std::max(v1.x, v2.x), std::max(v1.y, v2.y), v1.valid && v2.valid);
}

Vec2d Vec2d::getAverage(const Vec2d &v1, const Vec2d &v2)
{
    return (v1 + v2) / 2.0;
}

Vec2d Vec2d::getAverage(const std::vector<Vec2d> &vectors)
{
    Vec2d sum = Vec2d::nullVector;
    for (int i = 0; i < vectors.size(); i++) {
        sum += vectors[i];
    }
    return sum / vectors.size();
}

double Vec2d::getCrossProduct(const Vec2d &v1, const Vec2d &v2)
{
    return v1.x * v2.y - v1.y * v2.x;
}

Vec2d Vec2d::getDividedComponents(const Vec2d &v) const
{
    return Vec2d(x / v.x, y / v.y, valid);
}

Vec2d Vec2d::getMultipliedComponents(const Vec2d &v) const
{
    return Vec2d(x * v.x, y * v.y, valid);
}

Vec2d Vec2d::getClosest(const std::vector<Vec2d> &list) const
{
    int index = getClosestIndex(list);
    if (index == -1) {
        return Vec2d::invalid;
    }
    return list[index];
}

double Vec2d::getClosestDistance(const std::vector<Vec2d> &list, int counts)
{
    double ret = DBL_MAX;
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

int Vec2d::getClosestIndex(const std::vector<Vec2d> &list) const
{
    double minDist = DBL_MAX;
    int index = -1;

    for (int i = 0; i < list.size(); ++i) {
        if (list[i].valid) {
            double dist = getDistanceTo(list[i]);
            if (dist < minDist) {
                minDist = dist;
                index = i;
            }
        }
    }

    return index;
}

Vec2d Vec2d::createPolar(double radius, double angle)
{
    Vec2d ret;
    ret.setPolar(radius, angle);
    return ret;
}

std::vector<Vec2d> Vec2d::getSortedByDistance(const std::vector<Vec2d> &list,
                                              const Vec2d &v)
{
    RVectorDistanceSort::v = v;
    std::vector<Vec2d> ret = list;
    std::sort(ret.begin(), ret.end(), RVectorDistanceSort::lessThan);
    return ret;
}

std::vector<Vec2d> Vec2d::getSortedByAngle(const std::vector<Vec2d> &list,
                                           const Vec2d &center, double angle)
{
    RVectorAngleSort::center = center;
    RVectorAngleSort::angle = angle;
    std::vector<Vec2d> ret = list;
    std::sort(ret.begin(), ret.end(), RVectorAngleSort::lessThan);
    return ret;
}

bool RVectorDistanceSort::lessThan(const Vec2d &v1, const Vec2d &v2)
{
    return v.getDistanceTo(v1) < v.getDistanceTo(v2);
}

bool RVectorAngleSort::lessThan(const Vec2d &v1, const Vec2d &v2)
{
    double a1 = center.getAngleTo(v1);
    double a2 = center.getAngleTo(v2);

    double diff1 = Math::getAngleDifference(angle, a1);
    if (Math::fuzzyAngleCompare(diff1, M_PI * 2)) {
        diff1 = 0.0;
    }
    double diff2 = Math::getAngleDifference(angle, a2);
    if (Math::fuzzyAngleCompare(diff2, M_PI * 2)) {
        diff2 = 0.0;
    }
    return diff1 < diff2;
}

} // namespace cada