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

#ifndef RBOX_H
#define RBOX_H

#include "RShape.h"
#include "RVector.h"

class RLine;
class RPolyline;

#define RDEFAULT_RBOX RBox()

struct RBox {

    RVector c1;
    RVector c2;

    RBox();
    RBox(double x1, double y1, double x2, double y2);
    RBox(const RVector &c1, const RVector &c2);
    RBox(const RVector &center, double range);
    RBox(const RVector &center, double width, double height);

    bool isValid() const;
    bool isSane() const;

    bool equalsFuzzy(const RBox &b, double tol = RS::PointTolerance) const;

    double getWidth() const;
    double getHeight() const;
    RVector getSize() const;
    double getArea() const;
    RVector getCenter() const;
    RVector getMinimum() const;
    RVector getMaximum() const;
    bool isOutside(const RBox &other) const;
    bool isOutsideXY(const RBox &other) const;
    bool contains(const RBox &other) const;

    bool contains(const RVector &v) const;
    bool containsBox(const RBox &other) const { return contains(other); }
    bool containsPoint(const RVector &v) const { return contains(v); }
    bool intersects(const RBox &other) const;
    bool intersectsWith(const RShape &shape, bool limited = true) const;

    void growToInclude(const RBox &other);
    void growToIncludeBoxes(const std::vector<RBox> &others);
    void growToIncludeBox(const RBox &other);

    void growToInclude(const RVector &v);
    void growToIncludePoint(const RVector &v);

    RVector getCorner1() const;
    void setCorner1(const RVector &v);
    RVector getCorner2() const;
    void setCorner2(const RVector &v);

    std::vector<RVector> getCorners() const;
    std::vector<RLine> getLines() const;
    RPolyline getPolyline() const;

    RBox &grow(double offset);
    RBox &growXY(double offset);
    RBox &growXY(double offsetX, double offsetY);

    void move(const RVector &offset);
    bool scaleByReference(const RVector &referencePoint,
                          const RVector &targetPoint,
                          bool keepAspectRatio = false,
                          bool fromCenter = false);

    bool operator==(const RBox &other) const;
    bool operator!=(const RBox &other) const { return !operator==(other); }
};

#endif
