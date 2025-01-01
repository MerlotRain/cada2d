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

#ifndef RARC_H
#define RARC_H

#include <cada2d/RShape.h>
#include <cada2d/RVector.h>

class RBox;
class RLine;
class RPolyline;

class CADA_API RArc : public RShape
{
public:
    RArc();
    RArc(double cx, double cy, double radius, double startAngle,
         double endAngle, bool reversed = false);
    RArc(const RVector &center, double radius, double startAngle,
         double endAngle, bool reversed = false);

    RS::ShapeType getShapeType() const override;
    std::shared_ptr<RShape> clone() const override;
    bool isDirected() const override;
    bool isValid() const override;
    bool isFullCircle(double tolerance = RS::AngleTolerance) const;

    static RArc createFrom3Points(const RVector &startPoint,
                                  const RVector &point,
                                  const RVector &endPoint);
    static RArc createFrom2PBulge(const RVector &startPoint,
                                  const RVector &endPoint, double bulge);
    static RArc createTangential(const RVector &startPoint, const RVector &pos,
                                 double direction, double radius);
    static std::vector<RArc> createBiarc(const RVector &startPoint,
                                         double startDirection,
                                         const RVector &endPoint,
                                         double endDirection,
                                         bool secondTry = false);

    RBox getBoundingBox() const override;

    std::vector<RVector> getEndPoints() const override;
    std::vector<RVector> getMiddlePoints() const override;
    std::vector<RVector> getCenterPoints() const override;
    std::vector<RVector> getArcReferencePoints() const;
    std::vector<RVector>
    getPointsWithDistanceToEnd(double distance,
                               int from = RS::FromAny) const override;
    std::vector<RVector> getPointCloud(double segmentLength) const override;

    RVector getVectorTo(const RVector &point, bool limited = true,
                        double strictRange = RMAXDOUBLE) const override;

    RVector getCenter() const;
    void setCenter(const RVector &vector);
    double getRadius() const;
    void setRadius(double radius);
    double getStartAngle() const;
    void setStartAngle(double startAngle);
    double getEndAngle() const;
    void setEndAngle(double endAngle);
    bool isReversed() const;
    void setReversed(bool reversed);
    double getAngleLength(bool allowForZeroLength = false) const;
    bool isAngleWithinArc(double a) const;

    double getDiameter() const;
    void setDiameter(double d);
    void setLength(double l);
    double getArea() const;
    void setArea(double a);
    double getChordArea() const;

    double getDirection1() const override;
    double getDirection2() const override;

    RS::Side getSideOfPoint(const RVector &point) const override;

    double getSweep() const;
    void setSweep(double s);
    double getLength() const;

    RVector getStartPoint() const override;
    RVector getEndPoint() const override;
    RVector getPointAtAngle(double a) const;
    double getAngleAt(double distance,
                      RS::From from = RS::FromStart) const override;
    RVector getMiddlePoint() const override;

    void moveStartPoint(const RVector &pos, bool keepRadius = true);
    void moveEndPoint(const RVector &pos, bool keepRadius = true);
    void moveMiddlePoint(const RVector &pos);
    double getBulge() const;

    bool move(const RVector &offset) override;
    bool rotate(double rotation, const RVector &center = RVector()) override;
    bool scale(const RVector &scaleFactors,
               const RVector &center = RVector()) override;
    bool mirror(const RLine &axis) override;
    bool reverse() override;
    bool stretch(const RPolyline &area, const RVector &offset) override;

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

    RPolyline approximateWithLines(double segmentLength,
                                   double angle = 0.0) const;
    RPolyline approximateWithLinesTan(double segmentLength,
                                      double angle = 0.0) const;

    std::vector<RLine> getTangents(const RVector &point) const;

    std::vector<std::shared_ptr<RShape>>
    getOffsetShapes(double distance, int number, RS::Side side,
                    RS::JoinType join,
                    const RVector &position = RVector::invalid) override;

    std::vector<std::shared_ptr<RShape>>
    splitAt(const std::vector<RVector> &points) const override;

    std::vector<RArc> splitAtQuadrantLines() const;

private:
    RVector m_center;
    double m_radius;
    double m_startAngle;
    double m_endAngle;
    bool m_reversed;
};

#endif
