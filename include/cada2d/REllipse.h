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

#ifndef RELLIPSE_H
#define RELLIPSE_H

#include <cada2d/RShape.h>
#include <cada2d/RSpline.h>
#include <cada2d/RLine.h>
#include <cada2d/RVector.h>

class RBox;

class CADA_API REllipse : public RShape {
public:
    REllipse();
    REllipse(const RVector &center, const RVector &majorPoint, double ratio,
             double startParam, double endParam, bool reversed);
    ~REllipse();

    static REllipse
    createInscribed(const RVector &p1, const RVector &p2, const RVector &p3,
                    const RVector &p4,
                    const RVector &centerHint = RVector::invalid);
    static REllipse createFrom4Points(const RVector &p1, const RVector &p2,
                                      const RVector &p3, const RVector &p4);

    RS::ShapeType getShapeType() const override;
    REllipse *clone() const override;
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

    RVector getVectorTo(const RVector &point, bool limited = true,
                        double strictRange = RMAXDOUBLE) const override;

    void moveStartPoint(const RVector &pos, bool changeAngleOnly = false);
    void moveEndPoint(const RVector &pos, bool changeAngleOnly = false);

    std::vector<RVector> getFoci() const;

    RVector getCenter() const;
    void setCenter(const RVector &vector);
    RVector getMajorPoint() const;
    RVector getMinorPoint() const;
    void setMajorPoint(const RVector &vector);
    void setMinorPoint(const RVector &p);
    bool switchMajorMinor();
    double getRatio() const;
    void setRatio(double radius);

    double getStartParam() const;
    void setStartParam(double startParam);

    double getEndParam() const;
    void setEndParam(double endParam);

    double getStartAngle() const;
    void setStartAngle(double a);

    double angleToParam(double a) const;

    double getEndAngle() const;
    void setEndAngle(double a);

    double getAngleLength(bool allowForZeroLength = false) const;

    bool isAngleWithinArc(double a) const;
    bool isParamWithinArc(double a) const;

    bool isReversed() const;
    void setReversed(bool reversed);

    double getDirection1() const override;
    double getDirection2() const override;

    RS::Side getSideOfPoint(const RVector &point) const override;

    RVector getStartPoint() const override;
    RVector getEndPoint() const override;
    double getMajorRadius() const;
    double getMinorRadius() const;
    double getAngle() const;
    void setAngle(double a);
    bool isFullEllipse() const;
    bool isCircular() const;
    double getLength() const override;
    double getSimpsonLength(double f1, double f2) const;

    bool contains(const RVector &p) const;

    double getAngleAt(double distance,
                      RS::From from = RS::FromStart) const override;

    double getAngleAtPoint(const RVector &pos) const override;
    double getParamTo(const RVector &pos) const;
    double getRadiusAt(double param) const;
    RVector getPointAt(double param) const;
    RVector getMiddlePoint() const override;

    RVector getPointOnShape() const override;

    bool move(const RVector &offset) override;
    bool rotate(double rotation, const RVector &center = RVector()) override;
    bool scale(const RVector &scaleFactors,
               const RVector &center = RVector()) override;
    bool mirror(const RLine &axis) override;

    bool reverse() override;

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

    void correctMajorMinor();
    double getSweep() const;

    std::vector<RVector> getBoxCorners();

    std::vector<RLine> getTangents(const RVector &point) const;
    RVector getTangentPoint(const RLine &line) const;

    std::vector<RSpline> approximateWithSplines() const;
    RPolyline approximateWithArcs(int segments) const;

    std::vector<std::shared_ptr<RShape>>
    getOffsetShapes(double distance, int number, RS::Side side,
                    const RVector &position = RVector::invalid) override;
    std::vector<std::shared_ptr<RShape>>
    splitAt(const std::vector<RVector> &points) const override;

private:
    RVector m_center;
    RVector m_majorPoint;
    double m_ratio;
    double m_startParam;
    double m_endParam;
    bool m_reversed;
};

#endif
