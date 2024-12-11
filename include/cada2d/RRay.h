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

#ifndef RRAY_H
#define RRAY_H

#include <cada2d/RXLine.h>
#include <cada2d/RS.h>
#include <cada2d/RShape.h>
#include <cada2d/RVector.h>

class RBox;
class RPolyline;

class CADA_API RRay : public RXLine {
public:
    RRay();
    RRay(const RLine &line);
    RRay(const RVector &basePoint, const RVector &directionVector);
    RRay(const RVector &basePoint, double angle, double distance);
    ~RRay();

    RS::ShapeType getShapeType() const override;

    RRay *clone() const override;

    bool trimEndPoint(const RVector &trimPoint,
                      const RVector &clickPoint = RVector::invalid,
                      bool extend = false) override;
    std::vector<RVector> getPointsWithDistanceToEnd(double distance,
                                                    int from) const override;

    bool reverse() override;
    RLine getClippedLine(const RBox &box) const override;
    RVector getVectorTo(const RVector &point, bool limited = true,
                        double strictRange = RMAXDOUBLE) const override;

    bool stretch(const RPolyline &area, const RVector &offset) override;

    std::vector<std::shared_ptr<RShape>>
    splitAt(const std::vector<RVector> &points) const override;
};

#endif
