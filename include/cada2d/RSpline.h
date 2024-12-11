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

#ifndef RSPLINE_H
#define RSPLINE_H

#include <cada2d/RArc.h>
#include <cada2d/RBox.h>
#include <cada2d/RShape.h>
#include <cada2d/RVector.h>
#include <cada2d/RPolyline.h>

class RLine;

#ifndef R_NO_OPENNURBS
#include "opennurbs/opennurbs.h"
#endif

#ifndef RDEFAULT_MIN1
#define RDEFAULT_MIN1 -1
#endif

class CADA_API RSpline : public RShape {
public:
    RSpline();
    RSpline(const RSpline &other);
    RSpline(const std::vector<RVector> &controlPoints, int degree);
    ~RSpline();

    void copySpline(const RSpline &other);

    RS::ShapeType getShapeType() const override;
    RSpline *clone() const override;
    bool isDirected() const override;

    static std::vector<RSpline> createSplinesFromArc(const RArc &arc);
    static RSpline createBezierFromSmallArc(double r, double a1, double a2);

    bool isInterpolated() const override;

    void appendControlPoint(const RVector &point);
    void appendControlPoints(const std::vector<RVector> &points);
    void removeLastControlPoint();
    void setControlPoints(const std::vector<RVector> &points);
    std::vector<RVector> getControlPoints() const;
    std::vector<RVector> getControlPointsWrapped() const;
    int countControlPoints() const;
    RVector getControlPointAt(int i) const;

    void appendFitPoint(const RVector &point);
    void prependFitPoint(const RVector &point);
    void insertFitPointAt(const RVector &point);
    void insertFitPointAt(double t, const RVector &point);
    void removeFitPointAt(const RVector &point);
    void removeFirstFitPoint();
    void removeLastFitPoint();
    void setFitPoints(const std::vector<RVector> &points);
    std::vector<RVector> getFitPoints() const;
    int countFitPoints() const;
    bool hasFitPoints() const;
    RVector getFitPointAt(int i) const;

    std::vector<double> getKnotVector() const;
    std::vector<double> getActualKnotVector() const;
    void setKnotVector(const std::vector<double> &knots);
    void appendKnot(double k);
    std::vector<double> getWeights() const;
    void setWeights(std::vector<double> &w);

    void setDegree(int d);
    int getDegree() const;

    int getOrder() const;

    void setPeriodic(bool on);

    bool isClosed() const;
    bool isGeometricallyClosed(double tolerance = RS::PointTolerance) const;
    bool isPeriodic() const;

    double getDirection1() const override;
    double getDirection2() const override;

    RS::Side getSideOfPoint(const RVector &point) const override;

    RVector getStartPoint() const override;
    RVector getEndPoint() const override;

    void setStartPoint(const RVector &v);
    void setEndPoint(const RVector &v);

    void setTangents(const RVector &start, const RVector &end);
    void unsetTangents();

    void setTangentAtStart(const RVector &t);
    RVector getTangentAtStart() const;
    void unsetTangentAtStart();
    void setTangentAtEnd(const RVector &t);
    RVector getTangentAtEnd() const;
    void unsetTangentAtEnd();

    void updateTangentsPeriodic();

    RBox getBoundingBox() const override;

    double getLength() const override;
    RVector getPointAt(double t) const;
    RVector getPointAtDistance(double distance) const;
    double getAngleAt(double distance,
                      RS::From from = RS::FromStart) const override;

    std::vector<RVector> getEndPoints() const override;
    RVector getMiddlePoint() const override;
    std::vector<RVector> getMiddlePoints() const override;
    std::vector<RVector> getCenterPoints() const override;
    std::vector<RVector>
    getPointsWithDistanceToEnd(double distance,
                               int from = RS::FromAny) const override;
    std::vector<RVector> getPointCloud(double segmentLength) const override;

    RVector getVectorTo(const RVector &point, bool limited = true,
                        double strictRange = RMAXDOUBLE) const override;
    bool
    isOnShape(const RVector &point, bool limited = true,
              double tolerance = RDEFAULT_TOLERANCE_1E_MIN4) const override;

    bool move(const RVector &offset) override;
    bool rotate(double rotation, const RVector &center = RVector()) override;
    bool scale(const RVector &scaleFactors,
               const RVector &center = RVector()) override;
    bool mirror(const RLine &axis) override;
    bool flipHorizontal() override;
    bool flipVertical() override;
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

    std::vector<RSpline>
    splitAtPoints(const std::vector<RVector> &points) const;
    std::vector<RSpline> splitAtParams(const std::vector<double> &params) const;

    RPolyline toPolyline(int segments) const;
    RPolyline approximateWithArcs(double tolerance,
                                  double radiusLimit = RDEFAULT_MIN1) const;

    std::vector<std::shared_ptr<RShape>>
    getExploded(int segments = RDEFAULT_MIN1) const;
    std::vector<std::shared_ptr<RShape>> getExplodedBezier(int segments) const;
    std::vector<std::shared_ptr<RShape>>
    getExplodedWithSegmentLength(double segmentLength) const;

    std::vector<RSpline> getBezierSegments(const RBox &queryBox = RBox()) const;

    bool isValid() const override;
    double getTDelta() const;
    double getTMin() const;
    double getTMax() const;
    double getTAtPoint(const RVector &point) const;
    double getTAtDistance(double distance) const;
    double getDistanceAtT(double t) const;
    std::vector<RSpline> getSegments(const std::vector<RVector> &points) const;

    std::vector<RVector> getDiscontinuities() const;
    RSpline simplify(double tolerance);

    void updateFromControlPoints() const;
    void updateFromFitPoints() const;
    void update() const;

    std::vector<std::shared_ptr<RShape>>
    splitAt(const std::vector<RVector> &points) const override;

    bool isDirty() const { return m_dirty; }

    std::vector<RVector> getSelfIntersectionPoints(
        double tolerance = RS::PointTolerance) const override;

protected:
    void appendToExploded(const RLine &line) const;
    // void appendToExploded(std::vector<std::shared_ptr<RShape> >& list) const;
    void invalidate() const;
    void updateInternal() const;
    void updateBoundingBox() const;

private:
    mutable std::vector<RVector> m_controlPoints;
    mutable std::vector<double> m_knotVector;
    mutable std::vector<double> m_weights;
    std::vector<RVector> m_fitPoints;
    mutable int m_degree;
    mutable RVector m_tangentStart;
    mutable RVector m_tangentEnd;
    mutable bool m_periodic;

    mutable bool m_dirty;
    mutable bool m_updateInProgress;

private:
#ifndef R_NO_OPENNURBS
    mutable ON_NurbsCurve m_curve;
#endif
    mutable RBox m_boundingBox;
    mutable std::vector<std::shared_ptr<RShape>> m_exploded;
    // cached length:
    mutable double m_length;
};

#endif
