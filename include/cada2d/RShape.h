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

#ifndef RSHAPE_H
#define RSHAPE_H

#include <cada2d/exports.h>
#include <cada2d/RS.h>
#include <cada2d/RMath.h>
#include <cada2d/RVector.h>
#include <cada2d/RBox.h>

#include <memory>

class RArc;
class RBox;
class RCircle;
class REllipse;
class RExplodable;
class RLine;
class RPolyline;
class RSpline;

#ifndef RDEFAULT_TOLERANCE_1E_MIN4
#define RDEFAULT_TOLERANCE_1E_MIN4 1.0e-4
#endif

class CADA_API RShape {
public:
    RShape();
    virtual ~RShape();
    virtual bool isValid() const;

    virtual RS::ShapeType getShapeType() const;
    virtual RShape *clone() const = 0;
    virtual std::shared_ptr<RShape> cloneToSharedPointer() const
    {
        return std::shared_ptr<RShape>(clone());
    }

    virtual bool isInterpolated() const;

    virtual RVector
    getClosestPointOnShape(const RVector &p, bool limited = true,
                           double strictRange = RMAXDOUBLE) const;
    virtual RBox getBoundingBox() const = 0;
    virtual double getLength() const = 0;
    virtual bool equals(const RShape &other,
                        double tolerance = RS::PointTolerance) const;
    virtual RVector getVectorTo(const RVector &point, bool limited = true,
                                double strictRange = RMAXDOUBLE) const = 0;

    virtual double getDistanceTo(const RVector &point, bool limited = true,
                                 double strictRange = RMAXDOUBLE) const;
    virtual double getMaxDistanceTo(const std::vector<RVector> &points,
                                    bool limited = true,
                                    double strictRange = RMAXDOUBLE) const;
    virtual bool isOnShape(const RVector &point, bool limited = true,
                           double tolerance = RDEFAULT_TOLERANCE_1E_MIN4) const;
    virtual std::vector<RVector>
    filterOnShape(const std::vector<RVector> &pointList, bool limited = true,
                  double tolerance = RDEFAULT_TOLERANCE_1E_MIN4) const;
    virtual RVector getVectorFromEndpointTo(const RVector &point) const;
    virtual std::vector<RVector> getEndPoints() const = 0;
    virtual std::vector<RVector> getMiddlePoints() const = 0;
    virtual std::vector<RVector> getCenterPoints() const = 0;
    virtual std::vector<RVector> getArcReferencePoints() const
    {
        return std::vector<RVector>();
    }

    virtual RVector getPointOnShape() const;

    virtual std::vector<RVector> getPointCloud(double segmentLength) const = 0;
    virtual std::vector<RVector>
    getPointsWithDistanceToEnd(double distance,
                               int from = RS::FromAny) const = 0;

    virtual RVector getPointWithDistanceToStart(double distance) const;
    virtual RVector getPointWithDistanceToEnd(double distance) const;
    virtual double getAngleAt(double distance,
                              RS::From from = RS::FromStart) const;
    virtual double getAngleAtPoint(const RVector &pos) const;
    virtual RVector getPointAtPercent(double p) const;
    virtual double getAngleAtPercent(double p) const;

    virtual bool intersectsWith(const RShape &other, bool limited = true) const;

    std::vector<RVector> getIntersectionPoints(const RShape &other,
                                               bool limited = true,
                                               bool same = false,
                                               bool force = false) const;

    virtual std::vector<RVector>
    getSelfIntersectionPoints(double tolerance = RS::PointTolerance) const;
    virtual bool isDirected() const;
    virtual double getDirection1() const;
    virtual double getDirection2() const;

    virtual RS::Side getSideOfPoint(const RVector &point) const;

    virtual RVector getStartPoint() const;
    virtual RVector getEndPoint() const;
    virtual RVector getMiddlePoint() const;

    virtual bool reverse();

    virtual bool trimStartPoint(const RVector &trimPoint,
                                const RVector &clickPoint = RVector::invalid,
                                bool extend = false);

    virtual bool trimStartPoint(double trimDist);
    virtual bool trimEndPoint(const RVector &trimPoint,
                              const RVector &clickPoint = RVector::invalid,
                              bool extend = false);
    virtual bool trimEndPoint(double trimDist);
    virtual RS::Ending getTrimEnd(const RVector &trimPoint,
                                  const RVector &clickPoint);
    virtual double getDistanceFromStart(const RVector &p) const;
    virtual std::vector<double> getDistancesFromStart(const RVector &p) const;
    static std::vector<RVector> getIntersectionPoints(const RShape &shape1,
                                                      const RShape &shape2,
                                                      bool limited = true,
                                                      bool same = false,
                                                      bool force = false);

    virtual bool move(const RVector &offset) = 0;
    virtual bool rotate(double rotation, const RVector &center = RVector()) = 0;
    virtual bool scale(double scaleFactor, const RVector &center = RVector());
    virtual bool scale(const RVector &scaleFactors,
                       const RVector &center = RVector()) = 0;
    virtual bool mirror(const RLine &axis) = 0;
    virtual bool flipHorizontal();
    virtual bool flipVertical();
    virtual bool stretch(const RBox &area, const RVector &offset);
    virtual bool stretch(const RPolyline &area, const RVector &offset);

    virtual std::vector<std::shared_ptr<RShape>>
    getOffsetShapes(double distance, int number, RS::Side side,
                    const RVector &position = RVector::invalid);

    virtual std::vector<std::shared_ptr<RShape>>
    splitAt(const std::vector<RVector> &points) const;
};

#endif /* RSHAPE_H */
