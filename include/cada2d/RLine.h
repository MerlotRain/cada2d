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

#ifndef RLINE_H
#define RLINE_H

#include <cada2d/RS.h>
#include <cada2d/RShape.h>
#include <cada2d/RVector.h>

class RBox;
class RPolyline;

class CADA_API RLine : public RShape {
public:
    RLine();
    RLine(double x1, double y1, double x2, double y2);
    RLine(const RVector &startPoint, const RVector &endPoint);
    RLine(const RVector &startPoint, double angle, double distance);

    RS::ShapeType getShapeType() const override;
    RLine *clone() const override;

    bool isDirected() const override;
    bool isValid() const override;

    RBox getBoundingBox() const override;

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

    RVector getStartPoint() const override;
    void setStartPoint(const RVector &vector);
    RVector getEndPoint() const override;
    void setEndPoint(const RVector &vector);

    RVector getMiddlePoint() const override;

    double getLength() const override;
    double getAngle() const;

    void setLength(double l, bool fromStart = true);
    void setAngle(double a);

    bool isParallel(const RLine &line) const;
    bool isCollinear(const RLine &line) const;

    bool isVertical(double tolerance = RS::PointTolerance) const;
    bool isHorizontal(double tolerance = RS::PointTolerance) const;

    double getDirection1() const override;
    double getDirection2() const override;

    RS::Side getSideOfPoint(const RVector &point) const override;

    void clipToXY(const RBox &box);

    bool move(const RVector &offset) override;
    bool rotate(double rotation, const RVector &center = RVector()) override;
    bool scale(const RVector &scaleFactors,
               const RVector &center = RVector()) override;
    bool mirror(const RLine &axis) override;
    bool flipHorizontal() override;
    bool flipVertical() override;
    bool reverse() override;
    bool stretch(const RPolyline &area, const RVector &offset) override;

    bool moveTo(const RVector &dest);

    RS::Ending getTrimEnd(const RVector &trimPoint,
                          const RVector &clickPoint) override;
    bool trimStartPoint(const RVector &trimPoint,
                        const RVector &clickPoint = RVector::invalid,
                        bool extend = false) override;
    bool trimEndPoint(const RVector &trimPoint,
                      const RVector &clickPoint = RVector::invalid,
                      bool extend = false) override;
    bool trimStartPoint(double trimDist) override;
    bool trimEndPoint(double trimDist) override;
    double getDistanceFromStart(const RVector &p) const override;

    std::vector<std::shared_ptr<RShape>>
    getOffsetShapes(double distance, int number, RS::Side side, RS::JoinType join,
                    const RVector &position = RVector::invalid) override;

    std::vector<std::shared_ptr<RShape>>
    splitAt(const std::vector<RVector> &points) const override;

private:
    RVector m_startPoint;
    RVector m_endPoint;
};

#endif
