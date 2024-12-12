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

#ifndef RPOLYLINE_H
#define RPOLYLINE_H

#include <cada2d/RShape.h>
#include <cada2d/RVector.h>

class RBox;
class RLine;

#ifndef RDEFAULT_MIN1
#define RDEFAULT_MIN1 -1
#endif

class CADA_API RPolyline : public RShape {
public:
    RPolyline();
    RPolyline(const std::vector<RVector> &vertices, bool closed);
    RPolyline(const std::vector<std::shared_ptr<RShape>> &segments);
    ~RPolyline();

    RS::ShapeType getShapeType() const override;
    RPolyline *clone() const override;
    bool isDirected() const override;

    void clear();
    void normalize(double tolerance = RS::PointTolerance);

    bool prependShape(const RShape &shape);
    bool appendShape(const RShape &shape, bool prepend = false);
    bool appendShapeAuto(const RShape &shape);
    bool appendShapeTrim(const RShape &shape);
    bool closeTrim();

    void appendVertex(const RVector &vertex, double bulge = 0.0,
                      double w1 = 0.0, double w2 = 0.0);
    void appendVertex(double x, double y, double bulge = 0.0, double w1 = 0.0,
                      double w2 = 0.0);
    void prependVertex(const RVector &vertex, double bulge = 0.0,
                       double w1 = 0.0, double w2 = 0.0);
    void insertVertex(int index, const RVector &vertex,
                      double bulgeBefore = 0.0, double bulgeAfter = 0.0);
    void insertVertexAt(const RVector &point);
    RVector insertVertexAtDistance(double dist);
    void removeFirstVertex();
    void removeLastVertex();
    void removeVertex(int index);
    void removeVerticesAfter(int index);
    void removeVerticesBefore(int index);

    bool isEmpty() const { return countVertices() == 0; }

    void setVertices(const std::vector<RVector> &vertices);
    std::vector<RVector> getVertices() const;
    void setVertexAt(int i, const RVector &v);
    void moveVertexAt(int i, const RVector &offset);
    RVector getVertexAt(int i) const;
    int getVertexIndex(const RVector &v,
                       double tolerance = RS::PointTolerance) const;
    RVector getLastVertex() const;
    int countVertices() const;

    void setBulges(const std::vector<double> &b);
    std::vector<double> getBulges() const;
    double getBulgeAt(int i) const;
    void setBulgeAt(int i, double b);
    bool hasArcSegments() const;

    std::vector<double> getVertexAngles() const;
    double
    getVertexAngle(int i,
                   RS::Orientation orientation = RS::UnknownOrientation) const;

    void setGlobalWidth(double w);
    void setStartWidthAt(int i, double w);
    double getStartWidthAt(int i) const;
    void setEndWidthAt(int i, double w);
    double getEndWidthAt(int i) const;
    bool hasWidths() const;
    void setStartWidths(const std::vector<double> &sw);
    std::vector<double> getStartWidths() const;
    void setEndWidths(const std::vector<double> &ew);
    std::vector<double> getEndWidths() const;

    void setClosed(bool on);
    bool isClosed() const;
    bool isGeometricallyClosed(double tolerance = RS::PointTolerance) const;
    bool autoClose(double tolerance = RS::PointTolerance)
    {
        return toLogicallyClosed(tolerance);
    }
    bool toLogicallyClosed(double tolerance = RS::PointTolerance);
    bool toLogicallyOpen();

    std::vector<RVector> getSelfIntersectionPoints(
        double tolerance = RS::PointTolerance) const override;

    RS::Orientation getOrientation(bool implicitelyClosed = false) const;
    bool setOrientation(RS::Orientation orientation);

    RPolyline convertArcToLineSegments(int segments) const;
    RPolyline convertArcToLineSegmentsLength(double segmentLength) const;

    bool contains(const RVector &point, bool borderIsInside = false,
                  double tolerance = RS::PointTolerance) const;
    bool containsShape(const RShape &shape) const;

    RVector getPointInside() const;

    RVector getStartPoint() const override;
    RVector getEndPoint() const override;
    RVector getMiddlePoint() const override;

    void moveStartPoint(const RVector &pos);
    void moveEndPoint(const RVector &pos);

    void moveSegmentAt(int i, const RVector &offset);

    double getDirection1() const override;
    double getDirection2() const override;

    RS::Side getSideOfPoint(const RVector &point) const override;

    RBox getBoundingBox() const override;

    double getArea() const;

    double getLength() const override;

    double getDistanceFromStart(const RVector &p) const override;
    std::vector<double> getDistancesFromStart(const RVector &p) const override;
    double getLengthTo(const RVector &p, bool limited = true) const;
    double getSegmentsLength(int fromIndex, int toIndex) const;

    std::vector<RVector> getEndPoints() const override;
    std::vector<RVector> getMiddlePoints() const override;
    std::vector<RVector> getCenterPoints() const override;
    RVector getPointAtPercent(double p) const override;
    std::vector<RVector>
    getPointsWithDistanceToEnd(double distance,
                               int from = RS::FromAny) const override;
    std::vector<RVector> getPointCloud(double segmentLength) const override;

    double getAngleAt(double distance,
                      RS::From from = RS::FromStart) const override;

    RVector getVectorTo(const RVector &point, bool limited = true,
                        double strictRange = RMAXDOUBLE) const override;
    double getDistanceTo(const RVector &point, bool limited = true,
                         double strictRange = RMAXDOUBLE) const override;

    int getClosestSegment(const RVector &point) const;
    int getClosestVertex(const RVector &point) const;

    bool move(const RVector &offset) override;
    bool rotate(double rotation, const RVector &center = RVector()) override;
    bool scale(double scaleFactor, const RVector &center = RVector()) override;
    bool scale(const RVector &scaleFactors,
               const RVector &center = RVector()) override;
    bool mirror(const RLine &axis) override;
    bool reverse() override;
    RPolyline getReversed() const;
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

    std::vector<std::shared_ptr<RShape>>
    getExploded(int segments = RDEFAULT_MIN1) const;
    std::vector<RPolyline> getOutline() const;
    std::vector<std::pair<RPolyline, RPolyline>> getLeftRightOutline() const;
    std::vector<RPolyline> getLeftOutline() const;
    std::vector<RPolyline> getRightOutline() const;
    bool isInterpolated() const override;
    int countSegments() const;
    std::shared_ptr<RShape> getSegmentAt(int i) const;
    bool isArcSegmentAt(int i) const;
    std::shared_ptr<RShape> getLastSegment() const;
    std::shared_ptr<RShape> getFirstSegment() const;

    static bool isStraight(double bulge);

    bool simplify(double tolerance = RS::PointTolerance);
    std::vector<RVector>
    verifyTangency(double toleranceMin = RS::AngleTolerance,
                   double toleranceMax = M_PI_4);

    void stripWidths();
    void setMinimumWidth(double w);

    int getSegmentAtDist(double dist);
    bool relocateStartPoint(const RVector &p);
    bool relocateStartPoint(double dist);
    bool convertToClosed();
    bool convertToOpen();

    RPolyline modifyPolylineCorner(const RShape &trimmedShape1,
                                   RS::Ending ending1, int segmentIndex1,
                                   const RShape &trimmedShape2,
                                   RS::Ending ending2, int segmentIndex2,
                                   const RShape *cornerShape = NULL) const;

    bool isConcave() const;
    std::vector<RVector> getConvexVertices(bool convex = true) const;
    std::vector<RVector> getConcaveVertices() const;

    RVector getCentroid() const;

    std::vector<RPolyline> splitAtDiscontinuities(double tolerance) const;
    std::vector<RPolyline> splitAtSegmentTypeChange() const;

    double getBaseAngle() const;
    double getWidth() const;
    bool setWidth(double v);
    double getHeight() const;
    bool setHeight(double v);

    RPolyline roundAllCorners(double radius) const;
    RPolyline getPolygon(double segmentLength) const;
    RPolyline getPolygonHull(double angle, double tolerance,
                             bool inner = false) const;

    std::vector<std::shared_ptr<RShape>>
    getOffsetShapes(double distance, int number, RS::Side side,
                    RS::JoinType join,
                    const RVector &position = RVector::invalid) override;

protected:
    bool isLineSegment(int i) const;

protected:
    std::vector<RVector> m_vertices;
    std::vector<double> m_bulges;
    std::vector<double> m_endWidths;
    std::vector<double> m_startWidths;
    
    bool m_closed;
};

#endif
