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

#ifndef RPOINT_H
#define RPOINT_H

#include <cada2d/RBox.h>
#include <cada2d/RShape.h>
#include <cada2d/RVector.h>

class RLine;

class CADA_API RPoint : public RShape
{
public:
    RPoint();
    RPoint(double x, double y);
    RPoint(const RVector &position);
    ~RPoint();

    RS::ShapeType getShapeType() const override;
    std::shared_ptr<RShape> clone() const override;

    RVector getPosition() const;
    void setPosition(const RVector &p);

    RBox getBoundingBox() const override;
    double getLength() const override;

    std::vector<RVector> getEndPoints() const override;
    std::vector<RVector> getMiddlePoints() const override;
    std::vector<RVector> getCenterPoints() const override;
    std::vector<RVector>
    getPointsWithDistanceToEnd(double distance,
                               int from = RS::FromAny) const override;
    std::vector<RVector> getPointCloud(double segmentLength) const override;

    double getAngleAt(double distance,
                      RS::From from = RS::FromStart) const override;

    RVector getVectorTo(const RVector &point, bool limited = true,
                        double strictRange = RMAXDOUBLE) const override;

    bool move(const RVector &offset) override;
    bool rotate(double rotation, const RVector &center = RVector()) override;
    bool scale(const RVector &scaleFactors,
               const RVector &center = RVector()) override;
    bool mirror(const RLine &axis) override;
    bool flipHorizontal() override;
    bool flipVertical() override;

private:
    RVector m_position;
};

#endif
