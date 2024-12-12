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

#ifndef RCIRCLE_H
#define RCIRCLE_H

#include <cada2d/RArc.h>
#include <cada2d/RShape.h>
#include <cada2d/RVector.h>

class RBox;
class RLine;

class CADA_API RCircle : public RShape {
public:
    RCircle();
    RCircle(double cx, double cy, const double radius);
    RCircle(const RVector &center, const double radius);
    ~RCircle();

    RS::ShapeType getShapeType() const override;
    RCircle *clone() const override;

    static RCircle createFrom2Points(const RVector &p1, const RVector &p2);
    static RCircle createFrom3Points(const RVector &p1, const RVector &p2,
                                     const RVector &p3);

    RArc toArc(double startAngle = 0.0) const;

    bool isValid() const override;

    RBox getBoundingBox() const override;
    double getLength() const override;

    std::vector<RVector> getEndPoints() const override;
    std::vector<RVector> getMiddlePoints() const override;
    std::vector<RVector> getCenterPoints() const override;
    std::vector<RVector> getArcReferencePoints() const override;
    std::vector<RVector>
    getPointsWithDistanceToEnd(double distance,
                               int from = RS::FromAny) const override;
    std::vector<RVector> getPointCloud(double segmentLength) const override;

    double getAngleAt(double distance,
                      RS::From from = RS::FromStart) const override;
    RVector getPointAtAngle(double a) const;

    RVector getVectorTo(const RVector &point, bool limited = true,
                        double strictRange = RMAXDOUBLE) const override;

    RVector getPointOnShape() const override;

    RVector getCenter() const;
    void setCenter(const RVector &vector);
    double getRadius() const;
    void setRadius(double radius);

    double getDiameter() const;
    void setDiameter(double d);
    double getCircumference() const;
    void setCircumference(double c);
    double getArea() const;
    void setArea(double a);

    bool contains(const RVector &p) const;

    bool move(const RVector &offset) override;
    bool rotate(double rotation, const RVector &center = RVector()) override;
    bool scale(const RVector &scaleFactors,
               const RVector &center = RVector()) override;
    bool mirror(const RLine &axis) override;
    bool flipHorizontal() override;
    bool flipVertical() override;

    std::vector<RLine> getTangents(const RVector &point) const;

    std::vector<std::shared_ptr<RShape>>
    getOffsetShapes(double distance, int number, RS::Side side, RS::JoinType join,
                    const RVector &position = RVector::invalid) override;

    std::vector<std::shared_ptr<RShape>>
    splitAt(const std::vector<RVector> &points) const override;

private:
    RVector m_center;
    double m_radius;
};

#endif
