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

#ifndef RXLINE_H
#define RXLINE_H

#include "RLine.h"
#include "RS.h"
#include "RShape.h"
#include "RVector.h"

class RBox;
class RPolyline;

class CADA_API RXLine : public RShape {
public:
    RXLine();
    RXLine(const RLine &line);
    RXLine(const RVector &basePoint, const RVector &directionVector);
    RXLine(const RVector &basePoint, double angle, double distance);
    ~RXLine();

    RS::ShapeType getShapeType() const override;

    RLine getLineShape() const;
    RXLine *clone() const override;

    bool isDirected() const override;

    RBox getBoundingBox() const;

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

    RVector getBasePoint() const;
    void setBasePoint(const RVector &vector);
    RVector getSecondPoint() const;
    void setSecondPoint(const RVector &vector);
    RVector getDirectionVector() const;
    void setDirectionVector(const RVector &vector);

    RVector getMiddlePoint() const override;

    double getLength() const;
    void setLength(double l);
    double getAngle() const;
    void setAngle(double a);

    double getDirection1() const override;
    double getDirection2() const override;

    RS::Side getSideOfPoint(const RVector &point) const override;

    RVector getStartPoint() const override;
    RVector getEndPoint() const override;

    bool trimStartPoint(const RVector &trimPoint,
                        const RVector &clickPoint = RVector::invalid,
                        bool extend = false) override;
    bool trimEndPoint(const RVector &trimPoint,
                      const RVector &clickPoint = RVector::invalid,
                      bool extend = false) override;
    bool trimStartPoint(double trimDist) override;
    bool trimEndPoint(double trimDist) override;
    RS::Ending getTrimEnd(const RVector &trimPoint,
                          const RVector &clickPoint) override;
    double getDistanceFromStart(const RVector &p) const override;

    virtual RLine getClippedLine(const RBox &box) const;

    bool move(const RVector &offset) override;
    bool rotate(double rotation, const RVector &center = RVector()) override;
    bool scale(const RVector &scaleFactors,
               const RVector &center = RVector()) override;
    bool mirror(const RLine &axis) override;
    bool reverse() override;
    bool stretch(const RPolyline &area, const RVector &offset) override;

    std::vector<std::shared_ptr<RShape>>
    getOffsetShapes(double distance, int number, RS::Side side,
                    const RVector &position = RVector::invalid) override;

    std::vector<std::shared_ptr<RShape>>
    splitAt(const std::vector<RVector> &points) const override;

protected:
    RVector m_basePoint;
    RVector m_directionVector;
};

#endif
