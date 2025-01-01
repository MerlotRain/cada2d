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

#ifndef RVECTOR_H
#define RVECTOR_H

#include <cada2d/RS.h>
#include <cada2d/exports.h>
#include <vector>

class RBox;
class RLine;
class RPolyline;

#define RDEFAULT_RVECTOR RVector()

class CADA_API RVector
{

public:
    double x;
    double y;
    bool valid;

    inline RVector() : x(0.0), y(0.0), valid(true) {}
    RVector(double vx, double vy, bool valid_in = true);
    ~RVector();
    inline void set(double vx, double vy)
    {
        x = vx;
        y = vy;
        valid = true;
    }
    void setPolar(double radius, double angle);

    bool isValid() const;
    bool isZero() const;
    bool isSane() const;
    bool isNaN() const;

    bool isInside(const RBox &b) const;

    bool equalsFuzzy(const RVector &v, double tol = RS::PointTolerance) const;
    double getDistanceTo(const RVector &v) const;
    void setAngle(double a);
    double getAngle() const;
    double getAngleToPlaneXY() const;
    double getAngleTo(const RVector &v) const;
    void setMagnitude(double m);
    double getMagnitude() const;
    double getSquaredMagnitude() const;
    RVector getLerp(const RVector &v, double t) const;
    RVector getUnitVector() const;
    void setX(double x);
    double getX() const;
    void setY(double y);
    double getY() const;

    RVector move(const RVector &offset);
    static void moveList(std::vector<RVector> &list, const RVector &offset);

    RVector rotate(double rotation);
    RVector rotate(const RVector angleVector);
    RVector rotate(double rotation, const RVector &center);
    RVector rotate(const RVector &center, const RVector &angleVector);
    RVector getRotated(double rotation, const RVector &center) const;
    RVector scale(double factor, const RVector &center = nullVector);
    RVector scale(const RVector &factors, const RVector &center = nullVector);
    RVector getScaled(const RVector &factors, const RVector &center) const;
    RVector mirror(const RVector &axis1, const RVector &axis2);
    RVector flipHorizontal();
    RVector flipVertical();
    RVector stretch(const RPolyline &area, const RVector &offset);

    RVector getDividedComponents(const RVector &v) const;
    RVector getMultipliedComponents(const RVector &v) const;

    RVector getClosest(const std::vector<RVector> &list) const;
    double getClosestDistance(const std::vector<RVector> &list, int counts);
    int getClosestIndex(const std::vector<RVector> &list) const;

    RVector operator+(const RVector &v) const;
    RVector operator-(const RVector &v) const;
    RVector operator*(double s) const;
    RVector operator/(double s) const;
    RVector operator-() const;
    RVector getNegated() const;
    RVector getAbsolute() const;

    double dot(const RVector &other) const
    {
        return RVector::getDotProduct(*this, other);
    }

    RVector normalize();
    RVector getNormalized() const;

    void operator+=(const RVector &v);
    void operator-=(const RVector &v);
    void operator*=(double s);
    void operator/=(double s);

    bool operator==(const RVector &v) const;
    bool operator!=(const RVector &v) const { return !operator==(v); }

    RVector getFloor() const;
    RVector getCeil() const;

    static bool containsFuzzy(const std::vector<RVector> &vectors,
                              const RVector &v,
                              double tol = RS::PointTolerance);
    static int findFirstFuzzy(const std::vector<RVector> &vectors,
                              const RVector &v,
                              double tol = RS::PointTolerance);

    static RVector getMinimum(const std::vector<RVector> &vectors);
    static RVector getMaximum(const std::vector<RVector> &vectors);

    static RVector getMinimumX(const std::vector<RVector> &vectors);
    static RVector getMaximumX(const std::vector<RVector> &vectors);
    static RVector getMinimumY(const std::vector<RVector> &vectors);
    static RVector getMaximumY(const std::vector<RVector> &vectors);

    static RVector getMinimum(const RVector &v1, const RVector &v2);
    static RVector getMaximum(const RVector &v1, const RVector &v2);

    static RVector getAverage(const RVector &v1, const RVector &v2);
    static RVector getAverage(const std::vector<RVector> &vectors);

    static double getCrossProduct(const RVector &v1, const RVector &v2);
    static double getDotProduct(const RVector &v1, const RVector &v2);
    static RVector createPolar(double radius, double angle)
    {
        RVector ret;
        ret.setPolar(radius, angle);
        return ret;
    }

    static std::vector<RVector>
    getSortedByDistance(const std::vector<RVector> &list, const RVector &v);

    static std::vector<RVector>
    getSortedLeftRightTopBottom(const std::vector<RVector> &list);

    static std::vector<RVector>
    getSortedByAngle(const std::vector<RVector> &list, const RVector &center,
                     double angle);

    static const RVector invalid;
    static const RVector nullVector;
    static const RVector nanVector;
};

RVector operator*(double s, const RVector &v);

#endif