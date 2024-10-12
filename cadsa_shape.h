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

#ifndef CADSA_SHAPE_H
#define CADSA_SHAPE_H

#include "cadsa.h"
#include <vector>
#include <float.h>

namespace cadsa {

class Vec2d {
    double mX;
    double mY;
    bool mValid;

public:
    static const Vec2d invalid;
    static const Vec2d nullVec2d;
    static const Vec2d nanVec2d;

public:
    Vec2d();
    Vec2d(double x, double y, bool valid = true);
    ~Vec2d() = default;

    double getX() const;
    double getY() const;
    bool isValid() const;
    void setX(double x);
    void setY(double y);

    bool isValid() const;
    bool isZero() const;
    bool isSane() const;
    bool isNaN() const;

    void set(double x, double y);
    void setPolar(double radius, double angle);
    void setAngle(double a);
    double getAngle() const;
    double getAngleTo(const Vec2d &v) const;
    double getMagnitude() const;
    double getDistanceTo(const Vec2d &v) const;
    bool equalsFuzzy(const Vec2d& v, double tol = NS::PointTolerance) const;
    Vec2d getLerp(const Vec2d& v, double t) const;
    Vec2d getUnitVector() const;

public:
    Vec2d &move(const Vec2d &offset);
    Vec2d &rotate(double rotation);
    Vec2d &rotate(double rotation, const Vec2d &center);
    Vec2d scale(double factor, const Vec2d& center = nullVec2d);
    Vec2d scale(const Vec2d& factors, const Vec2d& center = nullVec2d);
    Vec2d getScaled(const Vec2d& factors, const Vec2d& center) const;

public:
    Vec2d operator+(const Vec2d &rhs) const;
    Vec2d operator-(const Vec2d &rhs) const;
    Vec2d operator*(double s) const;
    Vec2d operator/(double s) const;
    Vec2d operator-() const;
    Vec2d &operator+=(const Vec2d &rhs);
    Vec2d &operator-=(const Vec2d &rhs);
    Vec2d &operator*=(double s);
    Vec2d &operator/=(double s);

    bool operator==(const Vec2d &rhs) const;
    bool operator!=(const Vec2d &rhs) const;

public:
    static double getDotProduct(const Vec2d &v1, const Vec2d &v2);
    static Vec2d createPolar(double radius, double angle);
    static Vec2d getMinimum(const Vec2d& v1, const Vec2d &v2);
    static Vec2d getMaximum(const Vec2d& v1, const Vec2d &v2);
    static Vec2d getAverage(const Vec2d& v1, const Vec2d& v2);
    static Vec2d getAverage(const std::vector<Vec2d>& vectors);
    static std::vector<Vec2d> getUnion(const std::vector<Vec2d>& vectorsA, const std::vector<Vec2d>& vectorsB, double tol = NS::PointTolerance);
    static std::vector<Vec2d> getUnique(const std::vector<Vec2d>& vectors, double tol = NS::PointTolerance);
};

class BBox {
    Vec2d c1;
    Vec2d c2;

public:
    BBox();
    BBox(double x1, double y1, double x2, double y2);
    BBox(const Vec2d &c1, const Vec2d &c2);
    BBox(const Vec2d &center, double range);
    BBox(const Vec2d &center, double width, double height);

    bool isValid() const;
    bool isSane() const;

    bool equalsFuzzy(const BBox &b, double tol = DBL_EPSILON);
    double getWidth() const;
    double getHeight() const;
    double getArea() const;
    Vec2d getSize() const;
    Vec2d getCenter() const;
    Vec2d getMinimum() const;
    Vec2d getMaximum() const;

    bool isOutside(const BBox& other) const;
    bool contains(const BBox& other) const;
    bool contains(const Vec2d& v) const;
    bool intersects(const BBox& other) const;
    bool intersectsWith(const RShape& shape, bool limited = true) const;

    void growToInclude(const BBox& other);
    void growToIncludeBoxes(const std::vector<BBox>& others);
    void growToInclude(const Vec2d& v);

    Vec2d getCorner1() const;
    void setCorner1(const Vec2d& v);
    Vec2d getCorner2() const;
    void setCorner2(const Vec2d& v);

    std::vector<Vec2d> getCorners() const;

    BBox& grow(double offset);
    BBox& growXY(double offset);
    BBox& growXY(double offsetX, double offsetY);

    BBox& move(const Vec2d& offset);
    bool scaleByReference(const Vec2d& referencePoint, const Vec2d& targetPoint, bool keepAspectRatio = false, bool fromCenter = false);

    bool operator ==(const BBox& other) const;
    bool operator !=(const BBox& other) const;
};



class Shape {

    static double ellipse2tr(double x, double y, double AA, double BB,
                        double CC, double DD, double EE, double FF);

protected:
    ShapeType mShapeType;
    mutable BBox mBBox;

public:

    enum ShapeType {
        CADA_POINT,
        CADA_LINE,
        CADA_ARC,
        CADA_CIRCLE,
        CADA_ELLIPSE,
        CADA_XLINE,
        CADA_RAY,
        CADA_POLYLINE,
        CADA_SPLINE,
        CADA_TRIANGLE
    };

    Shape();
    virtual ~Shape();

    virtual ShapeType shapeType() const = 0;
    virtual Shape *clone() = 0;

    virtual Vec2d getStartPoint() const;
    virtual Vec2d getEndPoint() const;
    virtual Vec2d getMiddlePoint() const;

    virtual bool equals(Shape *other, double tol = DBL_EPSILON);
    virtual std::vector<Vec2d> getEndPoints() const = 0;
    virtual std::vector<Vec2d> getMiddlePoints() const = 0;
    virtual std::vector<Vec2d> getCenterPoints() const = 0;
    virtual bool isIntersects(Shape* shp, bool limited = true) const;
    virtual std::vector<Vec2d> intersectionPoints(Shape* shp, bool limited = true, bool same = false, bool force = false) const;
    virtual std::vector<Vec2d> selfIntersectionPoints(double tol = DBL_EPSILON) const;
    virtual Side getSideOfPoint(const Vec2d& pt) const;
    virtual BBox getBoundingBox() const = 0;

public:
    virtual bool isInterpolated() const;
    virtual Vec2d getClosestPointOnShape(const Vec2d& p, bool limited = true, double strictRange = DBL_MAX) const;
    virtual double getLength() const = 0;
    virtual Vec2d getVectorTo(const Vec2d& point, bool limited = true, double strictRange = DBL_MAX) const = 0;

    virtual double getDistanceTo(const Vec2d& point, bool limited = true, double strictRange = DBL_MAX) const;
    virtual double getMaxDistanceTo(const std::vector<Vec2d>& points, bool limited = true, double strictRange = DBL_MAX) const;
    virtual bool isOnShape(const Vec2d& point, bool limited = true, double tolerance = DBL_EPSILON) const;
    virtual std::vector<Vec2d> filterOnShape(const std::vector<Vec2d>& pointList, bool limited = true, double tolerance = DBL_EPSILON) const;
    virtual Vec2d getVectorFromEndpointTo(const Vec2d& point) const;
    virtual std::vector<Vec2d> getArcRefPoints() const;
    virtual Vec2d getPointOnShape() const;
    virtual std::vector<Vec2d> getPointsWithDistanceToEnd(double distance, int from = NS::FROM_ANY) const = 0;
    virtual Vec2d getPointWithDistanceToStart(double distance) const;
    virtual Vec2d getPointWithDistanceToEnd(double distance) const;
    virtual double getAngleAt(double distance, NS::From from = NS::FromStart) const;
    virtual double getAngleAtPoint(const Vec2d& pos) const;

    virtual Vec2d getPointAtPercent(double p) const;
    virtual double getAngleAtPercent(double p) const;

    virtual bool intersectsWith(const Shape& other, bool limited = true) const;
    std::vector<Vec2d> getIntersectionPoints(const Shape& other, bool limited = true, bool same = false, bool force = false) const;
    virtual std::vector<Vec2d> getSelfIntersectionPoints(double tolerance=NS::PointTolerance) const;

    virtual bool isDirected() const;
    virtual double getDirection1() const;
    virtual double getDirection2() const;

    virtual bool reverse();

    virtual bool trimStartPoint(const Vec2d& trimPoint, const Vec2d& clickPoint = Vec2d::invalid, bool extend = false);
    virtual bool trimStartPoint(double trimDist);
    virtual bool trimEndPoint(const Vec2d& trimPoint, const Vec2d& clickPoint = Vec2d::invalid, bool extend = false);
    virtual bool trimEndPoint(double trimDist);
    virtual NS::Ending getTrimEnd(const Vec2d& trimPoint, const Vec2d& clickPoint);
    virtual double getDistanceFromStart(const Vec2d& p) const;
    virtual std::vector<double> getDistancesFromStart(const Vec2d& p) const;

    virtual bool move(const Vec2d& offset) = 0;
    virtual bool rotate(double rotation, const Vec2d& center) = 0;
    virtual bool scale(double scaleFactor, const Vec2d& center);
    virtual bool scale(const Vec2d& scaleFactors, const Vec2d& center) = 0;
    virtual bool mirror(const Line& axis) = 0;
    virtual bool flipHorizontal();
    virtual bool flipVertical();
    virtual bool stretch(const BBox& area, const Vec2d& offset);
    virtual bool stretch(const Polyline* area, const Vec2d& offset);
    virtual Shape * getTransformed(const Mat& transform) const = 0;
    static std::vector<Polyline*> getPolylines(const std::vector<Shape * >& shapes);
    static std::vector<Shape * > getOrderedShapes(const std::vector<Shape * >& shapes);
    static bool order(std::vector<std::vector<Shape * > >& boundary);
    virtual std::vector<Shape *> getOffsetShapes(double distance, int number, NS::Side side, const Vec2d& position = Vec2d::invalid);
    static std::vector<Shape *> getOffsetLines(const Shape& shape, double distance, int number, NS::Side side, const Vec2d& position = Vec2d::invalid);
    static std::vector<Shape *> getOffsetArcs(const Shape& shape, double distance, int number, NS::Side side, const Vec2d& position = Vec2d::invalid);
    static std::vector<Shape *> getReversedShapeList(const std::vector<Shape * >& shapes);
    virtual std::vector<Shape *> splitAt(const std::vector<Vec2d>& points) const;
};




class Point : public Shape {
    Vec2d mp;

public:
    explicit Point(const Vec2d &v);
    Point(double x, double y);
    ~Point() = default;

    ShapeType shapeType() const override;
    Shape *clone() override;
    Vec2d getPosition() const;
    void setPosition(const Vec2d& p);

    virtual BBox getBoundingBox() const;
    virtual double getLength() const;

    virtual std::vector<Vec2d> getEndPoints() const;
    virtual std::vector<Vec2d> getMiddlePoints() const;
    virtual std::vector<Vec2d> getCenterPoints() const;
    virtual std::vector<Vec2d> getPointsWithDistanceToEnd(
        double distance, int from = NS::FromAny) const;
    virtual std::vector<Vec2d> getPointCloud(double segmentLength) const;

    virtual double getAngleAt(double distance, NS::From from = NS::FromStart) const;

    virtual Vec2d getVectorTo(const Vec2d& point,
            bool limited = true, double strictRange = DBL_MAX) const;

    virtual bool move(const Vec2d& offset);
    virtual bool rotate(double rotation, const Vec2d& center);
    virtual bool scale(const Vec2d& scaleFactors, const Vec2d& center);
    virtual bool mirror(const RLine& axis);
    virtual bool flipHorizontal();
    virtual bool flipVertical();

    virtual Shape *getTransformed(const Mat& transform) const;
};



class Line : public Shape {
    Vec2d mBegin;
    Vec2d mEnd;

public:
    Line();
    Line(double x1, double y1, double x2, double y2);
    Line(const Vec2d &begin, const Vec2d &end);
    Line(const Vec2d& startPoint, double angle, double distance);

    ShapeType shapeType() const override;
    Shape *clone() override;
    virtual bool isDirected() const;

    virtual bool isValid() const;

    virtual BBox getBoundingBox() const;

    virtual std::vector<Vec2d> getEndPoints() const;
    virtual std::vector<Vec2d> getMiddlePoints() const;
    virtual std::vector<Vec2d> getCenterPoints() const;
    virtual std::vector<Vec2d> getPointsWithDistanceToEnd(
        double distance, int from = NS::FromAny) const;
    virtual std::vector<Vec2d> getPointCloud(double segmentLength) const;

    virtual double getAngleAt(double distance, NS::From from = NS::FromStart) const;

    virtual Vec2d getVectorTo(const Vec2d& point,
            bool limited = true, double strictRange = DBL_MAX) const;

    virtual Vec2d getStartPoint() const;
    void setStartPoint(const Vec2d& vector);
    virtual Vec2d getEndPoint() const;
    void setEndPoint(const Vec2d& vector);

    virtual Vec2d getMiddlePoint() const;
    
    double getLength() const;
    double getAngle() const;

    void setLength(double l, bool fromStart = true);
    void setAngle(double a);

    bool isParallel(const RLine& line) const;

    bool isVertical(double tolerance = NS::PointTolerance) const;
    bool isHorizontal(double tolerance = NS::PointTolerance) const;

    virtual double getDirection1() const;
    virtual double getDirection2() const;

    virtual NS::Side getSideOfPoint(const Vec2d& point) const;

    void clipToXY(const BBox& box);

    virtual bool move(const Vec2d& offset);
    virtual bool rotate(double rotation, const Vec2d& center);
    virtual bool scale(const Vec2d& scaleFactors, const Vec2d& center);
    virtual bool mirror(const RLine& axis);
    virtual bool flipHorizontal();
    virtual bool flipVertical();
    virtual bool reverse();
    virtual bool stretch(const RPolyline& area, const Vec2d& offset);

    virtual bool moveTo(const Vec2d& dest);

    virtual Shape *getTransformed(const Mat& transform) const;

    virtual NS::Ending getTrimEnd(const Vec2d& trimPoint, const Vec2d& clickPoint);
    virtual bool trimStartPoint(const Vec2d& trimPoint, const Vec2d& clickPoint = Vec2d::invalid, bool extend = false);
    virtual bool trimEndPoint(const Vec2d& trimPoint, const Vec2d& clickPoint = Vec2d::invalid, bool extend = false);
    virtual bool trimStartPoint(double trimDist) {
        return RShape::trimStartPoint(trimDist);
    }
    virtual bool trimEndPoint(double trimDist) {
        return RShape::trimEndPoint(trimDist);
    }
    virtual double getDistanceFromStart(const Vec2d& p) const;

    virtual std::vector<Shape *> getOffsetShapes(double distance, int number, NS::Side side, const Vec2d& position = Vec2d::invalid) {
        return RShape::getOffsetLines(*this, distance, number, side, position);
    }

    virtual std::vector<Shape *> splitAt(const std::vector<Vec2d>& points) const;

};

class Circle : public Shape {
    Vec2d mCenter;
    double mRadius;

public:
    Circle();
    Circle(double cx, double cy, double radius);


    static RCircle createFrom2Points(const Vec2d& p1, const Vec2d& p2);
    static RCircle createFrom3Points(const Vec2d& p1, const Vec2d& p2, const Vec2d& p3);

    RArc toArc(double startAngle=0.0) const;

    virtual bool isValid() const;

    virtual BBox getBoundingBox() const;
    virtual double getLength() const;

    virtual std::vector<Vec2d> getEndPoints() const;
    virtual std::vector<Vec2d> getMiddlePoints() const;
    virtual std::vector<Vec2d> getCenterPoints() const;
    virtual std::vector<Vec2d> getArcReferencePoints() const;
    virtual std::vector<Vec2d> getPointsWithDistanceToEnd(
        double distance, int from = NS::FromAny) const;
    virtual std::vector<Vec2d> getPointCloud(double segmentLength) const;

    virtual double getAngleAt(double distance, NS::From from = NS::FromStart) const;
    Vec2d getPointAtAngle(double a) const;

    virtual Vec2d getVectorTo(const Vec2d& point,
            bool limited = true, double strictRange = DBL_MAX) const;

    Vec2d getCenter() const;
    void setCenter(const Vec2d& vector);
    double getRadius() const;
    void setRadius(double radius);

    double getDiameter() const;
    void setDiameter(double d);
    double getCircumference() const;
    void setCircumference(double c);
    double getArea() const;
    void setArea(double a);

    bool contains(const Vec2d& p) const;
    //bool touchesCircleInternally(const RCircle& other) const;

    virtual bool move(const Vec2d& offset);
    virtual bool rotate(double rotation, const Vec2d& center);
    virtual bool scale(const Vec2d& scaleFactors, const Vec2d& center);
    virtual bool mirror(const RLine& axis);
    virtual bool flipHorizontal();
    virtual bool flipVertical();

    virtual Shape *getTransformed(const Mat& transform) const;

    std::vector<RLine> getTangents(const Vec2d& point) const;

    virtual std::vector<Shape *> getOffsetShapes(double distance, int number, NS::Side side, const Vec2d& position = Vec2d::invalid) {
        return RShape::getOffsetArcs(*this, distance, number, side, position);
    }

    virtual std::vector<Shape *> splitAt(const std::vector<Vec2d>& points) const;
};

class Arc : public Shape {
    Vec2d mCenter;
    double mRadius;
    double mStartAngle;
    double mEndAngle;
    bool mReversed;

    bool isFullCircle(double tol = DBL_EPSILON) const;

public:
    Arc();
    Arc(const Vec2d &center, double radius, double startAngle, double endAngle,
        bool reversed = false);

    double getSweep() const;

    ShapeType shapeType() const override;
    Shape *clone() override;
    virtual bool isDirected() const;

    virtual bool isValid() const;
    bool isFullCircle(double tolerance = NS::AngleTolerance) const;

    static RArc createFrom3Points(const Vec2d& startPoint,
                                  const Vec2d& point,
                                  const Vec2d& endPoint);
    static RArc createFrom2PBulge(const Vec2d& startPoint,
                                  const Vec2d& endPoint,
                                  double bulge);
    static RArc createTangential(const Vec2d& startPoint,
                                 const Vec2d& pos,
                                 double direction,
                                 double radius);
    static std::vector<RArc> createBiarc(const Vec2d& startPoint, double startDirection,
                                   const Vec2d& endPoint, double endDirection, bool secondTry = false);

    virtual BBox getBoundingBox() const;

    virtual std::vector<Vec2d> getEndPoints() const;
    virtual std::vector<Vec2d> getMiddlePoints() const;
    virtual std::vector<Vec2d> getCenterPoints() const;
    virtual std::vector<Vec2d> getArcReferencePoints() const;
    virtual std::vector<Vec2d> getPointsWithDistanceToEnd(
        double distance, int from = NS::FromAny) const;
    virtual std::vector<Vec2d> getPointCloud(double segmentLength) const;

    virtual Vec2d getVectorTo(const Vec2d& point,
            bool limited = true, double strictRange = DBL_MAX) const;

    Vec2d getCenter() const;
    void setCenter(const Vec2d& vector);
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

    virtual double getDirection1() const;
    virtual double getDirection2() const;

    virtual NS::Side getSideOfPoint(const Vec2d& point) const;

    double getSweep() const;
    void setSweep(double s);
    double getLength() const;

    virtual Vec2d getStartPoint() const;
    virtual Vec2d getEndPoint() const;
    Vec2d getPointAtAngle(double a) const;
    virtual double getAngleAt(double distance, NS::From from = NS::FromStart) const;
    virtual Vec2d getMiddlePoint() const;

    void moveStartPoint(const Vec2d& pos, bool keepRadius = true);
    void moveEndPoint(const Vec2d& pos, bool keepRadius = true);
    void moveMiddlePoint(const Vec2d& pos);
    double getBulge() const;

    virtual bool move(const Vec2d& offset);
    virtual bool rotate(double rotation, const Vec2d& center);
    virtual bool scale(const Vec2d& scaleFactors, const Vec2d& center);
    virtual bool mirror(const RLine& axis);
    virtual bool reverse();
    virtual bool stretch(const RPolyline& area, const Vec2d& offset);

    virtual Shape *getTransformed(const Mat& transform) const;

    virtual NS::Ending getTrimEnd(const Vec2d& trimPoint, const Vec2d& clickPoint);
    virtual bool trimStartPoint(const Vec2d& trimPoint, const Vec2d& clickPoint = Vec2d::invalid, bool extend = false);
    virtual bool trimEndPoint(const Vec2d& trimPoint, const Vec2d& clickPoint = Vec2d::invalid, bool extend = false);
    virtual bool trimStartPoint(double trimDist) {
        return RShape::trimStartPoint(trimDist);
    }
    virtual bool trimEndPoint(double trimDist) {
        return RShape::trimEndPoint(trimDist);
    }
    virtual double getDistanceFromStart(const Vec2d& p) const;

    RPolyline approximateWithLines(double segmentLength, double angle = 0.0) const;
    RPolyline approximateWithLinesTan(double segmentLength, double angle = 0.0) const;

    std::vector<RLine> getTangents(const Vec2d& point) const;

    virtual std::vector<Shape *> getOffsetShapes(double distance, int number, NS::Side side, const Vec2d& position = Vec2d::invalid) {
        return RShape::getOffsetArcs(*this, distance, number, side, position);
    }

    virtual std::vector<Shape *> splitAt(const std::vector<Vec2d>& points) const;

    std::vector<RArc> splitAtQuadrantLines() const;
};

class Ellipse : public Shape {
    Vec2d mCenter;
    Vec2d mMajorPoint;
    double mRatio;
    double mStartParam;
    double mEndParam;
    bool mReversed;

public:
    Ellipse();
    Ellipse(const Vec2d &center, const Vec2d &majorPoint, double ratio, double startParam, double endParam, bool reversed);

    ShapeType shapeType() const override;
    Shape *clone() override;
    virtual bool isDirected() const;

    virtual bool isValid() const;

    virtual BBox getBoundingBox() const;

    virtual std::vector<Vec2d> getEndPoints() const;
    virtual std::vector<Vec2d> getMiddlePoints() const;
    virtual std::vector<Vec2d> getCenterPoints() const;
    virtual std::vector<Vec2d> getPointsWithDistanceToEnd(
        double distance, int from = NS::FromAny) const;
    virtual std::vector<Vec2d> getPointCloud(double segmentLength) const;

    virtual Vec2d getVectorTo(const Vec2d& point,
            bool limited = true, double strictRange = DBL_MAX) const;

    void moveStartPoint(const Vec2d& pos, bool changeAngleOnly=false);
    void moveEndPoint(const Vec2d& pos, bool changeAngleOnly=false);

    std::vector<Vec2d> getFoci() const;

    Vec2d getCenter() const;
    void setCenter(const Vec2d& vector);
    Vec2d getMajorPoint() const;
    Vec2d getMinorPoint() const;
    void setMajorPoint(const Vec2d& vector);
    void setMinorPoint(const Vec2d &p);
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

    virtual double getDirection1() const;
    virtual double getDirection2() const;

    virtual NS::Side getSideOfPoint(const Vec2d& point) const;

    virtual Vec2d getStartPoint() const;
    virtual Vec2d getEndPoint() const;
    double getMajorRadius() const;
    double getMinorRadius() const;
    double getAngle() const;
    void setAngle(double a);
    bool isFullEllipse() const;
    bool isCircular() const;
    double getLength() const;
    double getSimpsonLength(double f1, double f2) const;

    bool contains(const Vec2d& p) const;

    virtual double getAngleAt(double distance, NS::From from = NS::FromStart) const;

    double getAngleAtPoint(const Vec2d& pos) const;
    double getParamTo(const Vec2d& pos) const;
    double getRadiusAt(double param) const;
    Vec2d getPointAt(double param) const;
    Vec2d getMiddlePoint() const;

    virtual Vec2d getPointOnShape() const;

    virtual bool move(const Vec2d& offset);
    virtual bool rotate(double rotation, const Vec2d& center);
    virtual bool scale(const Vec2d& scaleFactors, const Vec2d& center);
    virtual bool mirror(const RLine& axis);

    virtual bool reverse();

    virtual Shape *getTransformed(const Mat& transform) const;

    virtual NS::Ending getTrimEnd(const Vec2d& trimPoint, const Vec2d& clickPoint);
    virtual bool trimStartPoint(const Vec2d& trimPoint, const Vec2d& clickPoint = Vec2d::invalid, bool extend = false);
    virtual bool trimEndPoint(const Vec2d& trimPoint, const Vec2d& clickPoint = Vec2d::invalid, bool extend = false);
    virtual bool trimStartPoint(double trimDist);
    virtual bool trimEndPoint(double trimDist);

    void correctMajorMinor();
    double getSweep() const;

    std::vector<Vec2d> getBoxCorners();

    std::vector<RLine> getTangents(const Vec2d& point) const;
    Vec2d getTangentPoint(const RLine& line) const;

    std::vector<Shape *> approximateWithSplines() const;
    RPolyline approximateWithArcs(int segments) const;

    virtual std::vector<Shape *> getOffsetShapes(double distance, int number, NS::Side side, const Vec2d& position = Vec2d::invalid);
    virtual std::vector<Shape *> splitAt(const std::vector<Vec2d>& points) const;
};

class Polyline : public Shape {
    std::vector<Vec2d> mVertices;
    std::vector<double> mBulges;
    std::vector<double> mEndWidths;
    std::vector<double> mStartWidths;
    bool mClosed;

    bool isStraight(double bulge) const;

public:
    Polyline();
    RPolyline(const std::vector<Vec2d>& vertices, bool closed);
    RPolyline(const std::vector<Shape *>& segments);
    ~Polyline();

    virtual bool isDirected() const;
    bool isFlat() const;

    void clear();
    void normalize(double tolerance = NS::PointTolerance);

    bool prependShape(const RShape& shape);
    bool appendShape(const RShape& shape, bool prepend = false);
    bool appendShapeAuto(const RShape& shape);
    bool appendShapeTrim(const RShape& shape);
    bool closeTrim();

    void appendVertex(const Vec2d& vertex, double bulge = 0.0, double w1 = 0.0, double w2 = 0.0);
    void appendVertex(double x, double y, double bulge = 0.0, double w1 = 0.0, double w2 = 0.0);
    void prependVertex(const Vec2d& vertex, double bulge = 0.0, double w1 = 0.0, double w2 = 0.0);
    void insertVertex(int index, const Vec2d& vertex, double bulgeBefore = 0.0, double bulgeAfter = 0.0);
    void insertVertexAt(const Vec2d& point);
    Vec2d insertVertexAtDistance(double dist);
    void removeFirstVertex();
    void removeLastVertex();
    void removeVertex(int index);
    void removeVerticesAfter(int index);
    void removeVerticesBefore(int index);

    bool isEmpty() const;

    void setVertices(const std::vector<Vec2d>& vertices);
    std::vector<Vec2d> getVertices() const;
    void setVertexAt(int i, const Vec2d& v);
    void moveVertexAt(int i, const Vec2d& offset);
    Vec2d getVertexAt(int i) const;
    int getVertexIndex(const Vec2d& v, double tolerance=NS::PointTolerance) const;
    Vec2d getLastVertex() const;
    int countVertices() const;

    void setBulges(const std::vector<double>& b);
    std::vector<double> getBulges() const;
    double getBulgeAt(int i) const;
    void setBulgeAt(int i, double b);
    bool hasArcSegments() const;

    std::vector<double> getVertexAngles() const;
    double getVertexAngle(int i, NS::Orientation orientation = NS::UnknownOrientation) const;

    void setGlobalWidth(double w);
    void setStartWidthAt(int i, double w);
    double getStartWidthAt(int i) const;
    void setEndWidthAt(int i, double w);
    double getEndWidthAt(int i) const;
    bool hasWidths() const;
    void setStartWidths(const std::vector<double>& sw);
    std::vector<double> getStartWidths() const;
    void setEndWidths(const std::vector<double>& ew);
    std::vector<double> getEndWidths() const;

    void setClosed(bool on);
    bool isClosed() const;
    bool isGeometricallyClosed(double tolerance=NS::PointTolerance) const;
    bool autoClose(double tolerance=NS::PointTolerance);
    bool toLogicallyClosed(double tolerance=NS::PointTolerance);
    bool toLogicallyOpen();

    std::vector<Vec2d> getSelfIntersectionPoints(double tolerance=NS::PointTolerance) const;

    NS::Orientation getOrientation(bool implicitelyClosed = false) const;
    bool setOrientation(NS::Orientation orientation);

    RPolyline convertArcToLineSegments(int segments) const;
    RPolyline convertArcToLineSegmentsLength(double segmentLength) const;

    virtual bool contains(const Vec2d& point, bool borderIsInside=false, double tolerance=NS::PointTolerance) const;
    bool containsShape(const RShape& shape) const;

    Vec2d getPointInside() const;

    virtual Vec2d getStartPoint() const;
    virtual Vec2d getEndPoint() const;
    virtual Vec2d getMiddlePoint() const;

    void moveStartPoint(const Vec2d& pos);
    void moveEndPoint(const Vec2d& pos);

    void moveSegmentAt(int i, const Vec2d& offset);

    virtual double getDirection1() const;
    virtual double getDirection2() const;

    virtual NS::Side getSideOfPoint(const Vec2d& point) const;

    virtual BBox getBoundingBox() const;

    double getArea() const;

    virtual double getLength() const;

    virtual double getDistanceFromStart(const Vec2d& p) const;
    virtual std::vector<double> getDistancesFromStart(const Vec2d& p) const;
    double getLengthTo(const Vec2d& p, bool limited = true) const;
    double getSegmentsLength(int fromIndex, int toIndex) const;

    virtual std::vector<Vec2d> getEndPoints() const;
    virtual std::vector<Vec2d> getMiddlePoints() const;
    virtual std::vector<Vec2d> getCenterPoints() const;
    virtual Vec2d getPointAtPercent(double p) const;
    virtual std::vector<Vec2d> getPointsWithDistanceToEnd(double distance, int from = NS::FromAny) const;
    virtual std::vector<Vec2d> getPointCloud(double segmentLength) const;

    virtual double getAngleAt(double distance, NS::From from = NS::FromStart) const;

    virtual Vec2d getVectorTo(const Vec2d& point, bool limited = true, double strictRange = DBL_MAX) const;
    virtual double getDistanceTo(const Vec2d& point, bool limited = true, double strictRange = DBL_MAX) const;

    int getClosestSegment(const Vec2d& point) const;
    int getClosestVertex(const Vec2d& point) const;

    virtual bool move(const Vec2d& offset);
    virtual bool rotate(double rotation, const Vec2d& center);
    virtual bool scale(double scaleFactor, const Vec2d& center);
    virtual bool scale(const Vec2d& scaleFactors, const Vec2d& center);
    virtual bool mirror(const RLine& axis);
    virtual bool reverse();
    virtual RPolyline getReversed() const;
    virtual bool stretch(const RPolyline& area, const Vec2d& offset);

    virtual Shape *getTransformed(const Mat& transform) const;

    virtual NS::Ending getTrimEnd(const Vec2d& trimPoint, const Vec2d& clickPoint);
    virtual bool trimStartPoint(const Vec2d& trimPoint, const Vec2d& clickPoint = Vec2d::invalid, bool extend = false);
    virtual bool trimEndPoint(const Vec2d& trimPoint, const Vec2d& clickPoint = Vec2d::invalid, bool extend = false);
    virtual bool trimStartPoint(double trimDist);
    virtual bool trimEndPoint(double trimDist);

    virtual std::vector<Shape *> getExploded(int segments = -1) const;
    std::vector<RPolyline> getOutline() const;
    std::vector<QPair<RPolyline, RPolyline> > getLeftRightOutline() const;
    std::vector<RPolyline> getLeftOutline() const;
    std::vector<RPolyline> getRightOutline() const;
    virtual bool isInterpolated() const;
    int countSegments() const;
    Shape *getSegmentAt(int i) const;
    bool isArcSegmentAt(int i) const;
    Shape *getLastSegment() const;
    Shape *getFirstSegment() const;

    static bool isStraight(double bulge);

    RPainterPath toPainterPath(bool addOriginalShapes = false) const;

    bool simplify(double tolerance = NS::PointTolerance);
    std::vector<Vec2d> verifyTangency(double toleranceMin = NS::AngleTolerance, double toleranceMax = M_PI_4);

    void stripWidths();
    void setMinimumWidth(double w);

    int getSegmentAtDist(double dist);
    bool relocateStartPoint(const Vec2d& p);
    bool relocateStartPoint(double dist);
    bool convertToClosed();
    bool convertToOpen();

    RPolyline modifyPolylineCorner(
            const RShape& trimmedShape1, NS::Ending ending1, int segmentIndex1,
            const RShape& trimmedShape2, NS::Ending ending2, int segmentIndex2,
            const RShape* cornerShape = NULL) const;

    bool isConcave() const;
    std::vector<Vec2d> getConvexVertices(bool convex = true) const;
    std::vector<Vec2d> getConcaveVertices() const;

    Vec2d getCentroid() const;

    std::vector<RPolyline> splitAtDiscontinuities(double tolerance) const;
    std::vector<RPolyline> splitAtSegmentTypeChange() const;

    double getBaseAngle() const;
    double getWidth() const;
    bool setWidth(double v);
    double getHeight() const;
    bool setHeight(double v);

    std::vector<RPolyline> morph(const RPolyline& target, int steps, NS::Easing easing = NS::Linear, bool zLinear = true, double customFactor = RNANDOUBLE) const;
    RPolyline roundAllCorners(double radius) const;
    RPolyline getPolygon(double segmentLength) const;
    RPolyline getPolygonHull(double angle, double tolerance, bool inner = false) const;
};

class XLine : public Shape {
    Vec2d mBasePoint;
    Vec2d mDirectionVec;

public:
    XLine();
    XLine(Line* line);
    XLine(const Vec2d &base, const Vec2d &dir);
    XLine(const Vec2d &base, double angle, double distance);

    ShapeType shapeType() const override;
    Shape *clone() override;

    virtual bool isDirected() const;
    BBox getBoundingBox() const;

    virtual std::vector<Vec2d> getEndPoints() const;
    virtual std::vector<Vec2d> getMiddlePoints() const;
    virtual std::vector<Vec2d> getCenterPoints() const;
    virtual std::vector<Vec2d> getPointsWithDistanceToEnd(
        double distance, int from = NS::FromAny) const;
    virtual std::vector<Vec2d> getPointCloud(double segmentLength) const;
    virtual double getAngleAt(double distance, NS::From from = NS::FromStart) const;
    virtual Vec2d getVectorTo(const Vec2d& point, bool limited = true, double strictRange = DBL_MAX) const;

    Vec2d getBasePoint() const;
    void setBasePoint(const Vec2d& vector);
    Vec2d getSecondPoint() const;
    void setSecondPoint(const Vec2d& vector);
    Vec2d getDirectionVector() const;
    void setDirectionVector(const Vec2d& vector);

    virtual Vec2d getMiddlePoint() const;
    
    double getLength() const;
    void setLength(double l);
    double getAngle() const;
    void setAngle(double a);

    virtual double getDirection1() const;
    virtual double getDirection2() const;

    virtual NS::Side getSideOfPoint(const Vec2d& point) const;

    virtual Vec2d getStartPoint() const;
    virtual Vec2d getEndPoint() const;

    virtual bool trimStartPoint(const Vec2d& trimPoint, const Vec2d& clickPoint = Vec2d::invalid, bool extend = false);
    virtual bool trimEndPoint(const Vec2d& trimPoint, const Vec2d& clickPoint = Vec2d::invalid, bool extend = false);
    virtual bool trimStartPoint(double trimDist);
    virtual bool trimEndPoint(double trimDist);
    virtual NS::Ending getTrimEnd(const Vec2d& trimPoint, const Vec2d& clickPoint);
    virtual double getDistanceFromStart(const Vec2d& p) const;

    virtual RLine getClippedLine(const BBox& box) const;

    virtual bool move(const Vec2d& offset);
    virtual bool rotate(double rotation, const Vec2d& center);
    virtual bool scale(const Vec2d& scaleFactors, const Vec2d& center);
    virtual bool mirror(const RLine& axis);
    virtual bool reverse();
    virtual bool stretch(const RPolyline& area, const Vec2d& offset);

    virtual Shape *getTransformed(const Mat& transform) const;
    virtual std::vector<Shape *> getOffsetShapes(double distance, int number, NS::Side side, const Vec2d& position = Vec2d::invalid);
    virtual std::vector<Shape *> splitAt(const std::vector<Vec2d>& points) const;

};

class Ray : public XLine 
{
public:
    Ray();
    Ray(const RLine& line);
    Ray(const Vec2d& basePoint, const Vec2d& directionVector);
    Ray(const Vec2d& basePoint, double angle, double distance);
    virtual ~Ray();

    virtual RShape::Type getShapeType() const;
    virtual Shape* clone() const;

    virtual bool trimEndPoint(const Vec2d& trimPoint, const Vec2d& clickPoint = Vec2d::invalid, bool extend = false);
    virtual std::vector<Vec2d> getPointsWithDistanceToEnd(double distance, int from) const;
    virtual bool reverse();
    virtual RLine getClippedLine(const BBox& box) const;
    virtual Vec2d getVectorTo(const Vec2d& point, bool limited = true, double strictRange = DBL_MAX) const;
    virtual bool stretch(const RPolyline& area, const Vec2d& offset);
    virtual std::vector<Shape *> splitAt(const std::vector<Vec2d>& points) const;
};

class BSpline : public Shape {
    std::vector<Vec2d> mControlPoints;
    std::vector<double> mKnots;
    std::vector<double> mWeights;
    mutable int mDegree;

    std::vector<Vec2d> mFitPoints;

    mutable Vec2d mTangentStart;
    mutable Vec2d mTangentEnd;
    mutable bool mPeriodic;
    mutable bool mDirty;
    mutable bool mUpdateInProgress;

protected:
    mutable BBox mBoundingBox;
    mutable std::vector<Shape *> mExploded;
    mutable double mLength;

    void appendToExploded(const RLine& line) const;
    void invalidate() const;
    void updateInternal() const;
    void updateBoundingBox() const;

public:
    BSpline();
    BSpline(const std::vector<Vec2d>& controlPoints, int degree);

    virtual RShape::Type getShapeType() const;

    virtual Shape* clone() const;
    virtual bool isDirected() const;

    static std::vector<BSpline *> createSplinesFromArc(const RArc& arc);
    static BSpline * createBezierFromSmallArc(double r, double a1, double a2);

    virtual bool isInterpolated() const;
    void appendControlPoint(const Vec2d& point);
    void appendControlPoints(const std::vector<Vec2d>& points);
    void removeLastControlPoint();
    void setControlPoints(const std::vector<Vec2d>& points);
    std::vector<Vec2d> getControlPoints() const;
    std::vector<Vec2d> getControlPointsWrapped() const;
    int countControlPoints() const;
    Vec2d getControlPointAt(int i) const;

    void appendFitPoint(const Vec2d& point);
    void prependFitPoint(const Vec2d& point);
    void insertFitPointAt(const Vec2d& point);
    void insertFitPointAt(double t, const Vec2d& point);
    void removeFitPointAt(const Vec2d& point);
    void removeFirstFitPoint();
    void removeLastFitPoint();
    void setFitPoints(const std::vector<Vec2d>& points);
    std::vector<Vec2d> getFitPoints() const;
    int countFitPoints() const;
    bool hasFitPoints() const;
    Vec2d getFitPointAt(int i) const;

    std::vector<double> getKnotVector() const;
    std::vector<double> getActualKnotVector() const;
    void setKnotVector(const std::vector<double>& knots);
    void appendKnot(double k);
    std::vector<double> getWeights() const;
    void setWeights(std::vector<double>& w);

    void setDegree(int d);
    int getDegree() const;

    int getOrder() const;

    void setPeriodic(bool on);

    bool isClosed() const;
    bool isGeometricallyClosed(double tolerance=NS::PointTolerance) const;
    bool isPeriodic() const;

    virtual double getDirection1() const;
    virtual double getDirection2() const;

    virtual NS::Side getSideOfPoint(const Vec2d& point) const;

    virtual Vec2d getStartPoint() const;
    virtual Vec2d getEndPoint() const;

    void setStartPoint(const Vec2d& v);
    void setEndPoint(const Vec2d& v);

    void setTangents(const Vec2d& start, const Vec2d& end);
    void unsetTangents();

    void setTangentAtStart(const Vec2d& t);
    Vec2d getTangentAtStart() const;
    void unsetTangentAtStart();
    void setTangentAtEnd(const Vec2d& t);
    Vec2d getTangentAtEnd() const;
    void unsetTangentAtEnd();

    void updateTangentsPeriodic();

    virtual BBox getBoundingBox() const;

    virtual double getLength() const;
    Vec2d getPointAt(double t) const;
    Vec2d getPointAtDistance(double distance) const;
    virtual double getAngleAt(double distance, NS::From from = NS::FromStart) const;

    virtual std::vector<Vec2d> getEndPoints() const;
    virtual Vec2d getMiddlePoint() const;
    virtual std::vector<Vec2d> getMiddlePoints() const;
    virtual std::vector<Vec2d> getCenterPoints() const;
    virtual std::vector<Vec2d> getPointsWithDistanceToEnd(
        double distance, int from = NS::FromAny) const;
    virtual std::vector<Vec2d> getPointCloud(double segmentLength) const;
    virtual Vec2d getVectorTo(const Vec2d& point, bool limited = true, double strictRange = DBL_MAX) const;
    virtual bool isOnShape(const Vec2d& point, bool limited = true, double tolerance = DBL_EPSILON) const;

    virtual bool move(const Vec2d& offset);
    virtual bool rotate(double rotation, const Vec2d& center);
    virtual bool scale(const Vec2d& scaleFactors, const Vec2d& center);
    virtual bool mirror(const RLine& axis);
    virtual bool flipHorizontal();
    virtual bool flipVertical();
    virtual bool reverse();
    virtual bool stretch(const RPolyline& area, const Vec2d& offset);

    Shape *getTransformed(const Mat& transform) const;

    virtual NS::Ending getTrimEnd(const Vec2d& trimPoint, const Vec2d& clickPoint);
    virtual bool trimStartPoint(const Vec2d& trimPoint, const Vec2d& clickPoint = Vec2d::invalid, bool extend = false);
    virtual bool trimEndPoint(const Vec2d& trimPoint, const Vec2d& clickPoint = Vec2d::invalid, bool extend = false);
    virtual bool trimStartPoint(double trimDist) {
        return RShape::trimStartPoint(trimDist);
    }
    virtual bool trimEndPoint(double trimDist) {
        return RShape::trimEndPoint(trimDist);
    }
    virtual double getDistanceFromStart(const Vec2d& p) const;

    std::vector<Shape *> splitAtPoints(const std::vector<Vec2d>& points) const;
    std::vector<Shape *> splitAtParams(const std::vector<double>& params) const;

    RPolyline toPolyline(int segments) const;
    RPolyline approximateWithArcs(double tolerance, double radiusLimit=-1) const;

    virtual std::vector<Shape *> getExploded(int segments = -1) const;
    std::vector<Shape *> getExplodedBezier(int segments) const;
    std::vector<Shape *> getExplodedWithSegmentLength(double segmentLength) const;

    std::vector<Shape *> getBezierSegments(const BBox& queryBox = BBox()) const;

    virtual bool isValid() const;
    double getTDelta() const;
    double getTMin() const;
    double getTMax() const;
    double getTAtPoint(const Vec2d& point) const;
    double getTAtDistance(double distance) const;
    double getDistanceAtT(double t) const;
    std::vector<Shape *> getSegments(const std::vector<Vec2d>& points) const;

    std::vector<Vec2d> getDiscontinuities() const;
    Shape * simplify(double tolerance);

    void updateFromControlPoints() const;
    void updateFromFitPoints() const;
    void update() const;

    virtual std::vector<Shape *> splitAt(const std::vector<Vec2d>& points) const;
    bool isDirty() const;
    std::vector<Vec2d> getSelfIntersectionPoints(double tolerance=NS::PointTolerance) const;
};

class Triangle : public Shape {
    Vec2d mCorner[3];

public:
    Triangle();
    Triangle(const Vec2d &p1, const Vec2d &p2, const Vec2d &p3);

    ShapeType shapeType() const override;
    Shape *clone() override;

    RPolyline getPolyline() const;
    NS::Orientation getOrientation() const;
    virtual bool reverse();

    static RTriangle createArrow(const Vec2d& position, double direction, double size);

    virtual BBox getBoundingBox() const;
    virtual double getLength() const;
    double getArea() const;
    Vec2d getCorner(int i) const;
    void setCorner(int i, const Vec2d& p);
    void setCorners(const Vec2d& c1, const Vec2d& c2, const Vec2d& c3);

    virtual std::vector<Vec2d> getEndPoints() const;
    virtual std::vector<Vec2d> getMiddlePoints() const;
    virtual std::vector<Vec2d> getCenterPoints() const;
    virtual std::vector<Vec2d> getPointsWithDistanceToEnd(
        double distance, int from = NS::FromAny) const;
    virtual std::vector<Vec2d> getPointCloud(double segmentLength) const;

    virtual double getDistanceTo(const Vec2d& point, bool limited = true, double strictRange = DBL_MAX) const;
    virtual Vec2d getVectorTo(const Vec2d& point, bool limited = true, double strictRange = DBL_MAX) const;
    virtual Vec2d getNormal() const;
    bool isPointInTriangle(const Vec2d& p, bool treatAsQuadrant = false) const;
    bool isPointInQuadrant(const Vec2d& p) const;

    double getD() const;

    virtual std::vector<Shape *> getExploded(int segments) const;

    virtual bool move(const Vec2d& offset);
    virtual bool rotate(double rotation, const Vec2d& center);
    virtual bool scale(const Vec2d& scaleFactors, const Vec2d& center);
    virtual bool mirror(const RLine& axis);
    virtual bool flipHorizontal();
    virtual bool flipVertical();
};



class Mat
{
public:
    enum TransformationType {
        TxNone      = 0x00,
        TxTranslate = 0x01,
        TxScale     = 0x02,
        TxRotate    = 0x04,
        TxShear     = 0x08,
        TxProject   = 0x10
    };

    Mat();
    Mat(double h11, double h12, double h13, double h21, double h22, double h23, double h31, double h32, double h33);
    Mat(double h11, double h12, double h21, double h22, double dx, double dy);

    Mat &operator=(Mat &&other) = default;
    Mat &operator=(const Mat &) = default;
    Mat(Mat &&other) = default;
    Mat(const Mat &other) = default;

    bool isAffine() const;
    bool isIdentity() const;
    bool isInvertible() const;
    bool isScaling() const;
    bool isRotating() const;
    bool isTranslating() const;

    TransformationType type() const;

     double determinant() const;

    double m11() const;
    double m12() const;
    double m13() const;
    double m21() const;
    double m22() const;
    double m23() const;
    double m31() const;
    double m32() const;
    double m33() const;
    double dx() const;
    double dy() const;

    void setMatrix(double m11, double m12, double m13, double m21, double m22, double m23, double m31, double m32, double m33);
    Mat inverted(bool *invertible = nullptr) const;
    Mat adjoint() const;
    Mat transposed() const;
    Mat &translate(double dx, double dy);
    Mat &scale(double sx, double sy);
    Mat &shear(double sh, double sv);
    Mat &rotate(double a, Qt::Axis axis, double distanceToPlane);
    Mat &rotateRadians(double a, Qt::Axis axis, double distanceToPlane);

    static bool squareToQuad(const QPolygonF &square, Mat &result);
    static bool quadToSquare(const QPolygonF &quad, Mat &result);
    static bool quadToQuad(const QPolygonF &one, const QPolygonF &two, Mat &result);

    bool operator==(const Mat &) const;
    bool operator!=(const Mat &) const;
    Mat &operator*=(const Mat &);
    Mat operator*(const Mat &o) const;

    void reset();
    QPointF      map(const QPointF &p) const;
    QLineF       map(const QLineF &l) const;
    QPolygonF    map(const QPolygonF &a) const;
    QRegion      map(const QRegion &r) const;
    QPolygon     mapToPolygon(const BBox &r) const;
    BBox mapRect(const BBox &) const;
    void map(int x, int y, int *tx, int *ty) const;
    void map(double x, double y, double *tx, double *ty) const;

    Mat &operator*=(double div);
    Mat &operator/=(double div);
    Mat &operator+=(double div);
    Mat &operator-=(double div);

    static Mat fromTranslate(double dx, double dy);
    static Mat fromScale(double dx, double dy);

private:
    TransformationType inline_type() const;
    void do_map(double x, double y, double &nx, double &ny) const;

    double m_matrix[3][3];
    mutable uint m_type : 5;
    mutable uint m_dirty : 5;
};

} // namespace cadsa

#endif