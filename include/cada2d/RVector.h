/**
 * Copyright (c) 2011-2018 by Andrew Mustun. All rights reserved.
 *
 * This file is part of the QCAD project.
 *
 * QCAD is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * QCAD is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with QCAD.
 */

#ifndef RVECTOR_H
#define RVECTOR_H

#include <vector>
#include "RS.h"

class RBox;
class RLine;
class RPolyline;

#define RDEFAULT_RVECTOR RVector()

struct RVector {
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
    double getX();
    void setY(double y);
    double getY();

    RVector move(const RVector &offset);
    static void moveList(std::vector<RVector> &list, const RVector &offset);

    RVector rotate(double rotation);
    RVector rotate(double rotation, const RVector &center);
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