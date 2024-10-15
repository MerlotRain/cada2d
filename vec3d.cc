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
    static bool lessThan(const Vec3d &v1, const Vec3d &v2);
    static Vec3d center;
    static double angle;
};

class RVectorDistanceSort {
public:
    static bool lessThan(const Vec3d &v1, const Vec3d &v2);
    static Vec3d v;
};

class RVectorLeftRightTopBottomSort {
public:
    static bool lessThan(const Vec3d &v1, const Vec3d &v2);
};

Vec3d RVectorDistanceSort::v;
Vec3d RVectorAngleSort::center;
double RVectorAngleSort::angle = 0.0;

const Vec3d Vec3d::invalid = Vec3d(0, 0, 0, false);
const Vec3d Vec3d::nullVector = Vec3d(0, 0, 0, true);
const Vec3d Vec3d::nanVector =
    Vec3d(std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::quiet_NaN(), true);

Vec3d::Vec3d() : x(0.0), y(0.0), z(0.0), valid(true)
{
}

Vec3d::Vec3d(double vx, double vy, double vz, bool valid_in)
    : x(vx), y(vy), z(vz)
{

    valid =
        valid_in && Math::isNormal(x) && Math::isNormal(y) && Math::isNormal(z);
}

Vec3d::Vec3d(const std::vector<double> &tuples)
{
    if (tuples.size() > 0) {
        x = tuples[0];
    }
    if (tuples.size() > 1) {
        y = tuples[1];
    }
    if (tuples.size() > 2) {
        z = tuples[2];
    }
    valid = true;
}

Vec3d::~Vec3d()
{
}
void Vec3d::set(double vx, double vy, double vz = 0.0)
{
    x = vx;
    y = vy;
    z = vz;
    valid = true;
}

bool Vec3d::isValid() const
{
    return valid;
}

bool Vec3d::isZero() const
{
    return fabs(x) < NS::PointTolerance && fabs(y) < NS::PointTolerance &&
           fabs(z) < NS::PointTolerance;
}

bool Vec3d::isSane() const
{
    return isValid() && Math::isSane(x) && Math::isSane(y) && Math::isSane(z);
}

bool Vec3d::isNaN() const
{
    return Math::isNaN(x) || Math::isNaN(y) || Math::isNaN(z);
}

void Vec3d::setPolar(double radius, double angle)
{
    x = radius * cos(angle);
    y = radius * sin(angle);
    z = 0.0;
    valid = Math::isNormal(radius) && Math::isNormal(angle);
}

Vec3d Vec3d::get2D() const
{
    return Vec3d(x, y);
}

void Vec3d::setAngle(double a)
{
    double m = getMagnitude();
    setPolar(m, a);
}

double Vec3d::getAngle() const
{
    double ret = 0.0;
    double m = getMagnitude2D();

    if (m > 1.0e-6) {
        double dp = getDotProduct(*this, Vec3d(1.0, 0.0));
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

double Vec3d::getAngleToPlaneXY() const
{
    Vec3d n(0, 0, 1);

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

double Vec3d::getAngleTo(const Vec3d &v) const
{
    if (!valid || !v.valid) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    else {
        return (v.get2D() - get2D()).getAngle();
    }
}

void Vec3d::setMagnitude2D(double m)
{
    double a = getAngle();
    setPolar(m, a);
}

double Vec3d::getMagnitude() const
{
    if (!valid) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return sqrt(x * x + y * y + z * z);
}

double Vec3d::getMagnitude2D() const
{
    if (!valid) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return sqrt(x * x + y * y);
}

double Vec3d::getSquaredMagnitude() const
{
    if (!valid) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    return x * x + y * y + z * z;
}

Vec3d Vec3d::getLerp(const Vec3d &v, double t) const
{
    return Vec3d(x + (v.x - x) * t, y + (v.y - y) * t, z + (v.z - z) * t);
}

Vec3d Vec3d::getUnitVector() const
{
    return *this / getMagnitude();
}

bool Vec3d::isInside(const BBox &b) const
{
    Vec3d bMin = b.getMinimum();
    Vec3d bMax = b.getMaximum();

    return (x >= bMin.x && x <= bMax.x && y >= bMin.y && y <= bMax.y &&
            z >= bMin.z && z <= bMax.z);
}

bool Vec3d::equalsFuzzy(const Vec3d &v, double tol) const
{
    return (std::fabs(x - v.x) < tol && std::fabs(y - v.y) < tol &&
            std::fabs(z - v.z) < tol && valid == v.valid);
}

bool Vec3d::equalsFuzzy2D(const Vec3d &v, double tol) const
{
    return (std::fabs(x - v.x) < tol && std::fabs(y - v.y) < tol &&
            valid == v.valid);
}

double Vec3d::getDistanceTo(const Vec3d &v) const
{
    if (!valid || !v.valid) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    else {
        return (*this - v).getMagnitude();
    }
}

double Vec3d::getDistanceTo2D(const Vec3d &v) const
{
    if (!valid || !v.valid) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    else {
        return (*this - v).getMagnitude2D();
    }
}

bool Vec3d::isInWindow(const Vec3d &firstCorner, const Vec3d &secondCorner)
{

    double minX = std::min(firstCorner.x, secondCorner.x);
    double maxX = std::max(firstCorner.x, secondCorner.x);
    double minY = std::min(firstCorner.y, secondCorner.y);
    double maxY = std::max(firstCorner.y, secondCorner.y);

    return (x >= minX && x <= maxX && y >= minY && y <= maxY);
}

Vec3d Vec3d::move(const Vec3d &offset)
{
    *this += offset;
    return *this;
}

void Vec3d::moveList(std::vector<Vec3d> &list, const Vec3d &offset)
{
    for (int i = 0; i < list.size(); i++) {
        list[i].move(offset);
    }
}

Vec3d Vec3d::rotate(double rotation)
{
    if (!valid) {
        return *this;
    }

    double r = getMagnitude2D();
    double a = getAngle() + rotation;

    x = cos(a) * r;
    y = sin(a) * r;

    return *this;
}

Vec3d Vec3d::rotate(double rotation, const Vec3d &center)
{
    *this = center + (*this - center).rotate(rotation);
    return *this;
}

Vec3d Vec3d::getRotated(double rotation, const Vec3d &center) const
{
    Vec3d ret = *this;
    ret.rotate(rotation, center);
    return ret;
}

void Vec3d::rotateList(std::vector<Vec3d> &list, double rotation)
{
    for (int i = 0; i < list.size(); i++) {
        list[i].rotate(rotation);
    }
}

void Vec3d::rotateList(std::vector<Vec3d> &list, double rotation,
                       const Vec3d &center)
{
    for (int i = 0; i < list.size(); i++) {
        list[i].rotate(rotation, center);
    }
}

Vec3d Vec3d::scale(double factor, const Vec3d &center)
{
    return scale(Vec3d(factor, factor, factor), center);
}

Vec3d Vec3d::scale(const Vec3d &factors, const Vec3d &center)
{
    if (center == Vec3d()) {
        x *= factors.x;
        y *= factors.y;
        z *= factors.z;
        return *this;
    }

    *this = center + (*this - center).scale(factors);
    return *this;
}

Vec3d Vec3d::getScaled(const Vec3d &factors, const Vec3d &center) const
{
    Vec3d ret = *this;
    ret.scale(factors, center);
    return ret;
}

void Vec3d::scaleList(std::vector<Vec3d> &list, double factor,
                      const Vec3d &center)
{
    for (int i = 0; i < list.size(); i++) {
        list[i].scale(factor, center);
    }
}

void Vec3d::scaleList(std::vector<Vec3d> &list, const Vec3d &factors,
                      const Vec3d &center)
{
    for (int i = 0; i < list.size(); i++) {
        list[i].scale(factors, center);
    }
}

Vec3d Vec3d::mirror(const Line &axis)
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

Vec3d Vec3d::getMirrored(const Line &axis) const
{
    Vec3d ret = *this;
    ret.mirror(axis);
    return ret;
}

Vec3d Vec3d::mirror(const Vec3d &axis1, const Vec3d &axis2)
{
    return mirror(Line(axis1, axis2));
}

Vec3d Vec3d::flipHorizontal()
{
    return mirror(Vec3d(0, 0, 0), Vec3d(0, 1, 0));
}

Vec3d Vec3d::flipVertical()
{
    return mirror(Vec3d(0, 0, 0), Vec3d(1, 0, 0));
}

Vec3d Vec3d::stretch(const Polyline &area, const Vec3d &offset)
{
    if (area.contains(*this, true)) {
        return move(offset);
    }
    return *this;
}

Vec3d Vec3d::operator+(const Vec3d &v) const
{
    return Vec3d(x + v.x, y + v.y, z + v.z, valid && v.valid);
}

Vec3d Vec3d::operator-(const Vec3d &v) const
{
    return Vec3d(x - v.x, y - v.y, z - v.z, valid && v.valid);
}

Vec3d Vec3d::operator*(double s) const
{
    return Vec3d(x * s, y * s, z * s, valid);
}

Vec3d Vec3d::operator/(double s) const
{
    return Vec3d(x / s, y / s, z / s, valid);
}

Vec3d Vec3d::operator-() const
{
    return getNegated();
}

Vec3d Vec3d::getNegated() const
{
    return Vec3d(-x, -y, -z, valid);
}

Vec3d Vec3d::getAbsolute() const
{
    return Vec3d(fabs(x), fabs(y), fabs(z));
}

double Vec3d::dot(const Vec3d &other) const
{
    return Vec3d::getDotProduct(*this, other);
}

Vec3d Vec3d::normalize()
{
    *this = getNormalized();
    return *this;
}

Vec3d Vec3d::getNormalized() const
{
    double l = getMagnitude();
    if (l < NS::PointTolerance) {
        return Vec3d::invalid;
    }
    return *this / l;
}

double Vec3d::getDotProduct(const Vec3d &v1, const Vec3d &v2)
{
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

void Vec3d::operator+=(const Vec3d &v)
{
    x += v.x;
    y += v.y;
    z += v.z;
    valid = valid && v.valid;
}

void Vec3d::operator-=(const Vec3d &v)
{
    x -= v.x;
    y -= v.y;
    z -= v.z;
    valid = valid && v.valid;
}

void Vec3d::operator*=(double s)
{
    x *= s;
    y *= s;
    z *= s;
}

void Vec3d::operator/=(double s)
{
    x /= s;
    y /= s;
    z /= s;
}

bool Vec3d::operator==(const Vec3d &v) const
{
    if (valid == true && v.valid == true) {
        return x == v.x && y == v.y && z == v.z;
    }
    else if (valid == false && v.valid == false) {
        return true;
    }
    return false;
}

bool Vec3d::containsFuzzy(const std::vector<Vec3d> &vectors, const Vec3d &v,
                          double tol)
{
    return findFirstFuzzy(vectors, v, tol) != -1;
}

int Vec3d::findFirstFuzzy(const std::vector<Vec3d> &vectors, const Vec3d &v,
                          double tol)
{
    for (int i = 0; i < vectors.size(); i++) {
        if (v.equalsFuzzy(vectors[i], tol)) {
            return i;
        }
    }

    return -1;
}

Vec3d Vec3d::getMinimum(const std::vector<Vec3d> &vectors)
{
    if (vectors.size() == 0) {
        return Vec3d();
    }

    Vec3d ret = vectors[0];

    std::vector<Vec3d>::const_iterator it = vectors.begin();
    it++;
    for (; it != vectors.end(); it++) {
        ret = getMinimum(ret, *it);
    }

    return ret;
}

Vec3d Vec3d::getMaximum(const std::vector<Vec3d> &vectors)
{
    if (vectors.size() == 0) {
        return Vec3d();
    }

    Vec3d ret = vectors[0];

    std::vector<Vec3d>::const_iterator it = vectors.begin();
    it++;
    for (; it != vectors.end(); it++) {
        ret = getMaximum(ret, *it);
    }

    return ret;
}

Vec3d Vec3d::getMinimumX(const std::vector<Vec3d> &vectors)
{
    if (vectors.size() == 0) {
        return Vec3d();
    }

    Vec3d ret = vectors[0];

    for (int i = 0; i < vectors.size(); i++) {
        if (vectors[i].x < ret.x) {
            ret = vectors[i];
        }
    }

    return ret;
}

Vec3d Vec3d::getMaximumX(const std::vector<Vec3d> &vectors)
{
    if (vectors.size() == 0) {
        return Vec3d();
    }

    Vec3d ret = vectors[0];

    for (int i = 0; i < vectors.size(); i++) {
        if (vectors[i].x > ret.x) {
            ret = vectors[i];
        }
    }

    return ret;
}

Vec3d Vec3d::getMinimumY(const std::vector<Vec3d> &vectors)
{
    if (vectors.size() == 0) {
        return Vec3d();
    }

    Vec3d ret = vectors[0];

    for (int i = 0; i < vectors.size(); i++) {
        if (vectors[i].y < ret.y) {
            ret = vectors[i];
        }
    }

    return ret;
}

Vec3d Vec3d::getMaximumY(const std::vector<Vec3d> &vectors)
{
    if (vectors.size() == 0) {
        return Vec3d();
    }

    Vec3d ret = vectors[0];

    for (int i = 0; i < vectors.size(); i++) {
        if (vectors[i].y > ret.y) {
            ret = vectors[i];
        }
    }

    return ret;
}

Vec3d Vec3d::getMinimum(const Vec3d &v1, const Vec3d &v2)
{
    return Vec3d(std::min(v1.x, v2.x), std::min(v1.y, v2.y),
                 std::min(v1.z, v2.z), v1.valid && v2.valid);
}

Vec3d Vec3d::getMaximum(const Vec3d &v1, const Vec3d &v2)
{
    return Vec3d(std::max(v1.x, v2.x), std::max(v1.y, v2.y),
                 std::max(v1.z, v2.z), v1.valid && v2.valid);
}

Vec3d Vec3d::getAverage(const Vec3d &v1, const Vec3d &v2)
{
    return (v1 + v2) / 2.0;
}

Vec3d Vec3d::getAverage(const std::vector<Vec3d> &vectors)
{
    Vec3d sum = Vec3d::nullVector;
    for (int i = 0; i < vectors.size(); i++) {
        sum += vectors[i];
    }
    return sum / vectors.size();
}

std::vector<Vec3d> Vec3d::getUnion(const std::vector<Vec3d> &vectorsA,
                                   const std::vector<Vec3d> &vectorsB,
                                   double tol)
{
    std::vector<Vec3d> ret;
    for (int i = 0; i < vectorsA.size(); i++) {
        if (Vec3d::containsFuzzy(vectorsB, vectorsA[i], tol)) {
            ret.push_back(vectorsA[i]);
        }
    }
    return ret;
}

std::vector<Vec3d> Vec3d::getUnique(const std::vector<Vec3d> &vectors,
                                    double tol)
{
    std::vector<Vec3d> ret;
    for (int i = 0; i < vectors.size(); i++) {
        if (!Vec3d::containsFuzzy(ret, vectors[i], tol)) {
            ret.push_back(vectors[i]);
        }
    }
    return ret;
}

std::vector<double> Vec3d::getXList(const std::vector<Vec3d> &vectors)
{
    std::vector<double> ret;
    std::vector<Vec3d>::const_iterator it;
    for (it = vectors.cbegin(); it != vectors.cend(); ++it) {
        ret.push_back((*it).x);
    }
    return ret;
}

std::vector<double> Vec3d::getYList(const std::vector<Vec3d> &vectors)
{
    std::vector<double> ret;
    std::vector<Vec3d>::const_iterator it;
    for (it = vectors.cbegin(); it != vectors.cend(); ++it) {
        ret.push_back((*it).y);
    }
    return ret;
}

std::vector<double> Vec3d::getZList(const std::vector<Vec3d> &vectors)
{
    std::vector<double> ret;
    std::vector<Vec3d>::const_iterator it;
    for (it = vectors.cbegin(); it != vectors.cend(); ++it) {
        ret.push_back((*it).z);
    }
    return ret;
}

Vec3d Vec3d::getFloor() const
{
    return Vec3d(floorl(x), floorl(y), floorl(z), valid);
}

Vec3d Vec3d::getCeil() const
{
    return Vec3d(ceill(x), ceill(y), ceill(z), valid);
}

bool Vec3d::lteXY(const Vec3d &v) const
{
    return Vec3d::lessThanEqualXY(*this, v);
}
bool Vec3d::gteXY(const Vec3d &v) const
{
    return Vec3d::greaterThanEqualXY(*this, v);
}

Vec3d Vec3d::getCrossProduct(const Vec3d &v1, const Vec3d &v2)
{
    return Vec3d(v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z,
                 v1.x * v2.y - v1.y * v2.x, v1.valid && v2.valid);
}

Vec3d Vec3d::getDividedComponents(const Vec3d &v) const
{
    return Vec3d(x / v.x, y / v.y, z / v.z, valid);
}

Vec3d Vec3d::getMultipliedComponents(const Vec3d &v) const
{
    return Vec3d(x * v.x, y * v.y, z * v.z, valid);
}

Vec3d Vec3d::getClosest(const std::vector<Vec3d> &list) const
{
    int index = getClosestIndex(list);
    if (index == -1) {
        return Vec3d::invalid;
    }
    return list[index].get2D();
}

Vec3d Vec3d::getClosest2D(const std::vector<Vec3d> &list) const
{
    int index = getClosestIndex2D(list);
    if (index == -1) {
        return Vec3d::invalid;
    }
    return list[index];
}

double Vec3d::getClosestDistance(const std::vector<Vec3d> &list, int counts)
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

int Vec3d::getClosestIndex(const std::vector<Vec3d> &list, bool ignoreZ) const
{
    double minDist = DBL_MAX;
    int index = -1;

    for (int i = 0; i < list.size(); ++i) {
        if (list[i].valid) {
            double dist;
            if (ignoreZ) {
                dist = getDistanceTo2D(list[i]);
            }
            else {
                dist = getDistanceTo(list[i]);
            }
            if (dist < minDist) {
                minDist = dist;
                index = i;
            }
        }
    }

    return index;
}

int Vec3d::getClosestIndex2D(const std::vector<Vec3d> &list) const
{
    return getClosestIndex(list, true);
}

Vec3d Vec3d::createPolar(double radius, double angle)
{
    Vec3d ret;
    ret.setPolar(radius, angle);
    return ret;
}

bool Vec3d::lessThanX(const Vec3d &v1, const Vec3d &v2)
{
    return v1.x < v2.x;
}

bool Vec3d::greaterThanX(const Vec3d &v1, const Vec3d &v2)
{
    return v1.x > v2.x;
}

bool Vec3d::lessThanY(const Vec3d &v1, const Vec3d &v2)
{
    return v1.y < v2.y;
}

bool Vec3d::greaterThanY(const Vec3d &v1, const Vec3d &v2)
{
    return v1.y > v2.y;
}

bool Vec3d::lessThanEqualXY(const Vec3d &v1, const Vec3d &v2)
{
    return v1.x <= v2.x && v2.y <= v2.y;
}

bool Vec3d::greaterThanEqualXY(const Vec3d &v1, const Vec3d &v2)
{
    return v1.x >= v2.x && v2.y >= v2.y;
}

std::vector<Vec3d> Vec3d::getSortedByDistance(const std::vector<Vec3d> &list,
                                              const Vec3d &v)
{
    RVectorDistanceSort::v = v;
    std::vector<Vec3d> ret = list;
    std::sort(ret.begin(), ret.end(), RVectorDistanceSort::lessThan);
    return ret;
}

std::vector<Vec3d>
Vec3d::getSortedLeftRightTopBottom(const std::vector<Vec3d> &list)
{
    std::vector<Vec3d> ret = list;
    std::sort(ret.begin(), ret.end(), RVectorLeftRightTopBottomSort::lessThan);
    return ret;
}

std::vector<Vec3d> Vec3d::getSortedByAngle(const std::vector<Vec3d> &list,
                                           const Vec3d &center, double angle)
{
    RVectorAngleSort::center = center;
    RVectorAngleSort::angle = angle;
    std::vector<Vec3d> ret = list;
    std::sort(ret.begin(), ret.end(), RVectorAngleSort::lessThan);
    return ret;
}

bool RVectorDistanceSort::lessThan(const Vec3d &v1, const Vec3d &v2)
{
    return v.getDistanceTo(v1) < v.getDistanceTo(v2);
}

bool RVectorLeftRightTopBottomSort::lessThan(const Vec3d &v1, const Vec3d &v2)
{
    return v1.y > v2.y || (v1.y == v2.y && v1.x < v2.x);
}

bool RVectorAngleSort::lessThan(const Vec3d &v1, const Vec3d &v2)
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