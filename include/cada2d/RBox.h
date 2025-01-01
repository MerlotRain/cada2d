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

#ifndef RBOX_H
#define RBOX_H

#include <cada2d/RShape.h>
#include <cada2d/RVector.h>

class RLine;
class RPolyline;

#define RDEFAULT_RBOX RBox()

class CADA_API RBox
{

public:
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
