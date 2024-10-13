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

#ifndef CADA_SHAPE_H
#define CADA_SHAPE_H

#include "cada_ns.h"
#include "cada_math.h"
#include <vector>

namespace cada {

class Line;
class Polyline;
class BBox;

class Vec3d {
    double x;
    double y;
    double z;
    bool valid;

public:
    Vec3d();
    Vec3d(double vx, double vy, double vz = 0.0, bool valid_in = true);
    Vec3d(const std::vector<double> &tuples);
    ~Vec3d();

    void set(double vx, double vy, double vz = 0.0);
    void setPolar(double radius, double angle);
    Vec3d get2D() const;

    bool isValid() const;
    bool isZero() const;
    bool isSane() const;
    bool isNaN() const;

    bool isInside(const BBox &b) const;

    bool equalsFuzzy(const Vec3d &v, double tol = NS::PointTolerance) const;
    bool equalsFuzzy2D(const Vec3d &v, double tol = NS::PointTolerance) const;
    double getDistanceTo(const Vec3d &v) const;
    double getDistanceTo2D(const Vec3d &v) const;
    void setAngle(double a);
    double getAngle() const;
    double getAngleToPlaneXY() const;
    double getAngleTo(const Vec3d &v) const;
    void setMagnitude2D(double m);
    double getMagnitude() const;
    double getSquaredMagnitude() const;
    double getMagnitude2D() const;
    Vec3d getLerp(const Vec3d &v, double t) const;
    Vec3d getUnitVector() const;
    void setX(double x);
    double getX() const;
    double &getX() { return x; }
    void setY(double y);
    double getY() const;
    double &getY() { return y; }
    void setZ(double z);
    double getZ() const;
    double &getZ() { return z; }

    bool isInWindow(const Vec3d &firstCorner, const Vec3d &secondCorner);

    Vec3d move(const Vec3d &offset);
    static void moveList(std::vector<Vec3d> &list, const Vec3d &offset);

    Vec3d rotate(double rotation);
    Vec3d rotate(double rotation, const Vec3d &center);
    Vec3d getRotated(double rotation, const Vec3d &center) const;
    static void rotateList(std::vector<Vec3d> &list, double rotation);
    static void rotateList(std::vector<Vec3d> &list, double rotation,
                           const Vec3d &center);

    Vec3d scale(double factor, const Vec3d &center = nullVector);
    Vec3d scale(const Vec3d &factors, const Vec3d &center = nullVector);
    Vec3d getScaled(const Vec3d &factors, const Vec3d &center) const;
    static void scaleList(std::vector<Vec3d> &list, double factor,
                          const Vec3d &center = nullVector);
    static void scaleList(std::vector<Vec3d> &list, const Vec3d &factors,
                          const Vec3d &center = nullVector);

    Vec3d mirror(const Line &axis);
    Vec3d getMirrored(const Line &axis) const;
    Vec3d mirror(const Vec3d &axis1, const Vec3d &axis2);
    Vec3d flipHorizontal();
    Vec3d flipVertical();
    Vec3d stretch(const Polyline &area, const Vec3d &offset);

    Vec3d getDividedComponents(const Vec3d &v) const;
    Vec3d getMultipliedComponents(const Vec3d &v) const;

    Vec3d getClosest(const std::vector<Vec3d> &list) const;
    Vec3d getClosest2D(const std::vector<Vec3d> &list) const;
    double getClosestDistance(const std::vector<Vec3d> &list, int counts);
    int getClosestIndex(const std::vector<Vec3d> &list,
                        bool ignoreZ = false) const;
    int getClosestIndex2D(const std::vector<Vec3d> &list) const;

    Vec3d operator+(const Vec3d &v) const;
    Vec3d operator-(const Vec3d &v) const;
    Vec3d operator*(double s) const;
    Vec3d operator/(double s) const;
    Vec3d operator-() const;
    Vec3d getNegated() const;
    Vec3d getAbsolute() const;

    void operator+=(const Vec3d &v);
    void operator-=(const Vec3d &v);
    void operator*=(double s);
    void operator/=(double s);

    bool operator==(const Vec3d &v) const;
    bool operator!=(const Vec3d &v) const { return !operator==(v); }

    double dot(const Vec3d &other) const;
    Vec3d normalize();
    Vec3d getNormalized() const;

    Vec3d getFloor() const;
    Vec3d getCeil() const;

    bool lteXY(const Vec3d &v) const;
    bool gteXY(const Vec3d &v) const;

    static bool containsFuzzy(const std::vector<Vec3d> &vectors, const Vec3d &v,
                              double tol = NS::PointTolerance);
    static int findFirstFuzzy(const std::vector<Vec3d> &vectors, const Vec3d &v,
                              double tol = NS::PointTolerance);

    static Vec3d getMinimum(const std::vector<Vec3d> &vectors);
    static Vec3d getMaximum(const std::vector<Vec3d> &vectors);

    static Vec3d getMinimumX(const std::vector<Vec3d> &vectors);
    static Vec3d getMaximumX(const std::vector<Vec3d> &vectors);
    static Vec3d getMinimumY(const std::vector<Vec3d> &vectors);
    static Vec3d getMaximumY(const std::vector<Vec3d> &vectors);

    static Vec3d getMinimum(const Vec3d &v1, const Vec3d &v2);
    static Vec3d getMaximum(const Vec3d &v1, const Vec3d &v2);

    static Vec3d getAverage(const Vec3d &v1, const Vec3d &v2);
    static Vec3d getAverage(const std::vector<Vec3d> &vectors);

    static std::vector<Vec3d> getUnion(const std::vector<Vec3d> &vectorsA,
                                       const std::vector<Vec3d> &vectorsB,
                                       double tol = NS::PointTolerance);
    static std::vector<Vec3d> getUnique(const std::vector<Vec3d> &vectors,
                                        double tol = NS::PointTolerance);

    static std::vector<double> getXList(const std::vector<Vec3d> &vectors);
    static std::vector<double> getYList(const std::vector<Vec3d> &vectors);
    static std::vector<double> getZList(const std::vector<Vec3d> &vectors);

    static Vec3d getCrossProduct(const Vec3d &v1, const Vec3d &v2);
    static double getDotProduct(const Vec3d &v1, const Vec3d &v2);
    static Vec3d createPolar(double radius, double angle);

    static bool lessThanX(const Vec3d &v1, const Vec3d &v2);
    static bool greaterThanX(const Vec3d &v1, const Vec3d &v2);
    static bool lessThanY(const Vec3d &v1, const Vec3d &v2);
    static bool greaterThanY(const Vec3d &v1, const Vec3d &v2);
    static bool lessThanEqualXY(const Vec3d &v1, const Vec3d &v2);
    static bool greaterThanEqualXY(const Vec3d &v1, const Vec3d &v2);

    static std::vector<Vec3d>
    getSortedByDistance(const std::vector<Vec3d> &list, const Vec3d &v);
    static std::vector<Vec3d>
    getSortedLeftRightTopBottom(const std::vector<Vec3d> &list);
    static std::vector<Vec3d> getSortedByAngle(const std::vector<Vec3d> &list,
                                               const Vec3d &center,
                                               double angle);

public:
    static const Vec3d invalid;
    static const Vec3d nullVector;
    static const Vec3d nanVector;
};

class BBox {
    Vec3d c1;
    Vec3d c2;

public:
    BBox();
    BBox(double x1, double y1, double x2, double y2);
    BBox(const Vec3d &c1, const Vec3d &c2);
    BBox(const Vec3d &center, double range);
    BBox(const Vec3d &center, double width, double height);

    bool isValid() const;
    bool isSane() const;

    bool equalsFuzzy(const BBox &b, double tol = NS::PointTolerance) const;
    bool equalsFuzzy2D(const BBox &b, double tol = NS::PointTolerance) const;

    BBox get2D() const { return BBox(c1.get2D(), c2.get2D()); }

    double getWidth() const;
    double getHeight() const;
    Vec3d getSize() const;
    double getArea() const;
    Vec3d getCenter() const;
    Vec3d getMinimum() const;
    Vec3d getMaximum() const;
    bool isOutside(const BBox &other) const;
    bool isOutsideXY(const BBox &other) const;
    bool contains(const BBox &other) const;
    bool contains(const Vec3d &v) const;
    bool intersects(const BBox &other) const;
    bool intersectsWith(const Shape &shape, bool limited = true) const;

    void growToInclude(const BBox &other);
    void growToIncludeBoxes(const std::vector<BBox> &others);

    void growToInclude(const Vec3d &v);

    Vec3d getCorner1() const;
    void setCorner1(const Vec3d &v);
    Vec3d getCorner2() const;
    void setCorner2(const Vec3d &v);

    std::vector<Vec3d> getCorners() const;
    std::vector<Vec3d> getCorners2d() const;
    std::vector<Line> getLines2d() const;
    Polyline getPolyline2d() const;
    std::vector<Triangle> getTriangles() const;

    BBox &grow(double offset);
    BBox &growXY(double offset);
    BBox &growXY(double offsetX, double offsetY);

    void move(const Vec3d &offset);
    bool scaleByReference(const Vec3d &referencePoint, const Vec3d &targetPoint,
                          bool keepAspectRatio = false,
                          bool fromCenter = false);

    bool operator==(const BBox &other) const;
    bool operator!=(const BBox &other) const { return !operator==(other); }
};

class Object {
public:
    Object() {}
    virtual ~Object() {}

private:
    Object &operator=(const Object &) = delete;
    Object(const Object &) = delete;
};

class Shape : public Object {
public:
    Shape() {}
    virtual ~Shape() {}

    virtual bool isValid() const { return true; }
    virtual NS::ShapeType getShapeType() const { return NS::Unkonwn; }
    virtual Shape *clone() const = 0;
    virtual bool isInterpolated() const { return false; }

    virtual Vec3d getClosestPointOnShape(const Vec3d &p, bool limited = true,
                                         double strictRange = DBL_MAX) const;

    virtual BBox getBoundingBox() const = 0;
    virtual void to2D() { setZ(0.0); }

    virtual void setZ(double z) = 0;
    virtual double getLength() const = 0;
    virtual bool equals(const Shape &other,
                        double tolerance = NS::PointTolerance) const;
    virtual Vec3d getVectorTo(const Vec3d &point, bool limited = true,
                              double strictRange = DBL_MAX) const = 0;

    virtual double getDistanceTo(const Vec3d &point, bool limited = true,
                                 double strictRange = DBL_MAX) const;
    virtual double getMaxDistanceTo(const std::vector<Vec3d> &points,
                                    bool limited = true,
                                    double strictRange = DBL_MAX) const;
    virtual bool isOnShape(const Vec3d &point, bool limited = true,
                           double tolerance = DBL_EPSILON) const;
    virtual std::vector<Vec3d>
    filterOnShape(const std::vector<Vec3d> &pointList, bool limited = true,
                  double tolerance = DBL_EPSILON) const;
    virtual Vec3d getVectorFromEndpointTo(const Vec3d &point) const;
    virtual std::vector<Vec3d> getEndPoints() const = 0;
    virtual std::vector<Vec3d> getMiddlePoints() const = 0;
    virtual std::vector<Vec3d> getCenterPoints() const = 0;
    virtual std::vector<Vec3d> getArcReferencePoints() const;
    virtual Vec3d getPointOnShape() const;
    virtual std::vector<Vec3d> getPointCloud(double segmentLength) const = 0;
    virtual std::vector<Vec3d>
    getPointsWithDistanceToEnd(double distance,
                               int from = NS::FromAny) const = 0;

    virtual Vec3d getPointWithDistanceToStart(double distance) const;
    virtual Vec3d getPointWithDistanceToEnd(double distance) const;
    virtual double getAngleAt(double distance,
                              NS::From from = NS::FromStart) const;
    virtual double getAngleAtPoint(const Vec3d &pos) const;

    virtual Vec3d getPointAtPercent(double p) const;
    virtual double getAngleAtPercent(double p) const;

    virtual bool intersectsWith(const Shape &other, bool limited = true) const;

    std::vector<Vec3d> getIntersectionPoints(const Shape &other,
                                             bool limited = true,
                                             bool same = false,
                                             bool force = false) const;

    virtual std::vector<Vec3d>
    getSelfIntersectionPoints(double tolerance = NS::PointTolerance) const;

    virtual bool isDirected() const;
    virtual double getDirection1() const;
    virtual double getDirection2() const;

    virtual NS::Side getSideOfPoint(const Vec3d &point) const;
    virtual Vec3d getStartPoint() const { return Vec3d::invalid; }
    virtual Vec3d getEndPoint() const { return Vec3d::invalid; }
    virtual Vec3d getMiddlePoint() const { return Vec3d::invalid; }

    virtual bool reverse() { return false; }

    virtual bool trimStartPoint(const Vec3d &trimPoint,
                                const Vec3d &clickPoint = Vec3d::invalid,
                                bool extend = false);
    virtual bool trimStartPoint(double trimDist);
    virtual bool trimEndPoint(const Vec3d &trimPoint,
                              const Vec3d &clickPoint = Vec3d::invalid,
                              bool extend = false);

    virtual bool trimEndPoint(double trimDist);
    virtual NS::Ending getTrimEnd(const Vec3d &trimPoint,
                                  const Vec3d &clickPoint);
    virtual double getDistanceFromStart(const Vec3d &p) const;
    virtual std::vector<double> getDistancesFromStart(const Vec3d &p) const;
    static std::vector<Vec3d> getIntersectionPoints(const Shape &shape1,
                                                    const Shape &shape2,
                                                    bool limited = true,
                                                    bool same = false,
                                                    bool force = false);

    virtual bool move(const Vec3d &offset) = 0;
    virtual bool rotate(double rotation, const Vec3d &center = Vec3d()) = 0;
    virtual bool scale(double scaleFactor, const Vec3d &center = Vec3d());
    virtual bool scale(const Vec3d &scaleFactors,
                       const Vec3d &center = Vec3d()) = 0;
    virtual bool mirror(const Line &axis) = 0;
    virtual bool flipHorizontal();
    virtual bool flipVertical();
    virtual bool stretch(const BBox &area, const Vec3d &offset);
    virtual bool stretch(const Polyline &area, const Vec3d &offset);

    static std::vector<Polyline>
    getPolylines(const std::vector<QSharedPointer<Shape>> &shapes);
    static std::vector<QSharedPointer<Shape>>
    getOrderedShapes(const std::vector<QSharedPointer<Shape>> &shapes);
    static bool
    order(std::vector<std::vector<QSharedPointer<Shape>>> &boundary);

    virtual std::vector<QSharedPointer<Shape>>
    getOffsetShapes(double distance, int number, NS::Side side,
                    const Vec3d &position = Vec3d::invalid);

    static std::vector<QSharedPointer<Shape>>
    getOffsetLines(const Shape &shape, double distance, int number,
                   NS::Side side, const Vec3d &position = Vec3d::invalid);
    static std::vector<QSharedPointer<Shape>>
    getOffsetArcs(const Shape &shape, double distance, int number,
                  NS::Side side, const Vec3d &position = Vec3d::invalid);

    static std::vector<QSharedPointer<Shape>>
    getReversedShapeList(const std::vector<QSharedPointer<Shape>> &shapes);

    virtual std::vector<QSharedPointer<Shape>>
    splitAt(const std::vector<Vec3d> &points) const;

    static std::vector<QSharedPointer<Shape>>
    trim(const Shape &trimShape, const Vec3d &trimClickPos,
         const Shape &limitingShape, const Vec3d &limitingClickPos,
         bool trimBoth, bool samePolyline);

    static std::vector<QSharedPointer<Shape>>
    roundCorners(const std::vector<QSharedPointer<Shape>> &shapes,
                 double radius);

    static std::vector<QSharedPointer<Shape>>
    roundShapes(const QSharedPointer<Shape> shape1, const Vec3d &clickPos1,
                const QSharedPointer<Shape> shape2, const Vec3d &clickPos2,
                bool trim, bool samePolyline, double radius, const Vec3d &pos);

    static QSharedPointer<Shape> xLineToRay(QSharedPointer<Shape> shape);
    static QSharedPointer<Shape> rayToLine(QSharedPointer<Shape> shape);

    static QSharedPointer<Shape> scaleArc(const Shape &shape,
                                          const Vec3d &scaleFactors,
                                          const Vec3d &center = Vec3d());

    static QSharedPointer<Shape>
    transformArc(const Shape &shape, RShapeTransformation &transformation);
    static QSharedPointer<Shape>
    ellipseToArcCircleEllipse(const Ellipse &ellipse);

    virtual std::vector<QSharedPointer<Shape>>
    roundAllCorners(const std::vector<QSharedPointer<Shape>> &shapes,
                    double radius) = 0;

private:
    static double ellipse2tr(double x, double y, double AA, double BB,
                             double CC, double DD, double EE, double FF);
};

class Point : public Shape {
public:
    Point();
    Point(double x, double y);
    Point(const Vec3d &position);
    virtual ~Point();

    virtual NS::ShapeType getShapeType() const { return Point; }

    virtual Point *clone() const { return new Point(*this); }

    virtual void setZ(double z);

    Vec3d getPosition() const { return position; }

    void setPosition(const Vec3d &p) { position = p; }

    virtual BBox getBoundingBox() const;
    virtual double getLength() const;

    virtual std::vector<Vec3d> getEndPoints() const;
    virtual std::vector<Vec3d> getMiddlePoints() const;
    virtual std::vector<Vec3d> getCenterPoints() const;
    virtual std::vector<Vec3d>
    getPointsWithDistanceToEnd(double distance, int from = NS::FromAny) const;
    virtual std::vector<Vec3d> getPointCloud(double segmentLength) const;

    virtual double getAngleAt(double distance,
                              NS::From from = NS::FromStart) const;

    virtual Vec3d getVectorTo(const Vec3d &point, bool limited = true,
                              double strictRange = DBL_MAX) const;

    virtual bool move(const Vec3d &offset);
    virtual bool rotate(double rotation, const Vec3d &center = Vec3d());
    virtual bool scale(const Vec3d &scaleFactors,
                       const Vec3d &center = Vec3d());
    virtual bool mirror(const Line &axis);
    virtual bool flipHorizontal();
    virtual bool flipVertical();

public:
    /**
     * \getter{getPosition}
     * \setter{setPosition}
     */
    Vec3d position;
};

class Line : public Shape {
public:
    Line();
    Line(double x1, double y1, double x2, double y2);
    Line(const Vec3d &startPoint, const Vec3d &endPoint);
    Line(const Vec3d &startPoint, double angle, double distance);

    virtual NS::ShapeType getShapeType() const { return Line; }

    virtual Line *clone() const { return new Line(*this); }

    virtual bool isDirected() const { return true; }

    virtual void setZ(double z);

    virtual bool isValid() const;

    virtual BBox getBoundingBox() const;

    virtual std::vector<Vec3d> getEndPoints() const;
    virtual std::vector<Vec3d> getMiddlePoints() const;
    virtual std::vector<Vec3d> getCenterPoints() const;
    virtual std::vector<Vec3d>
    getPointsWithDistanceToEnd(double distance, int from = NS::FromAny) const;
    virtual std::vector<Vec3d> getPointCloud(double segmentLength) const;

    virtual double getAngleAt(double distance,
                              NS::From from = NS::FromStart) const;

    virtual Vec3d getVectorTo(const Vec3d &point, bool limited = true,
                              double strictRange = DBL_MAX) const;

    virtual Vec3d getStartPoint() const;
    void setStartPoint(const Vec3d &vector);
    virtual Vec3d getEndPoint() const;
    void setEndPoint(const Vec3d &vector);

    virtual Vec3d getMiddlePoint() const;

    double getLength() const;
    double getAngle() const;

    void setLength(double l, bool fromStart = true);
    void setAngle(double a);

    bool isParallel(const Line &line) const;

    bool isVertical(double tolerance = NS::PointTolerance) const;
    bool isHorizontal(double tolerance = NS::PointTolerance) const;

    virtual double getDirection1() const;
    virtual double getDirection2() const;

    virtual NS::Side getSideOfPoint(const Vec3d &point) const;

    void clipToXY(const BBox &box);

    virtual bool move(const Vec3d &offset);
    virtual bool rotate(double rotation, const Vec3d &center = Vec3d());
    virtual bool scale(const Vec3d &scaleFactors,
                       const Vec3d &center = Vec3d());
    virtual bool mirror(const Line &axis);
    virtual bool flipHorizontal();
    virtual bool flipVertical();
    virtual bool reverse();
    virtual bool stretch(const Polyline &area, const Vec3d &offset);

    virtual bool moveTo(const Vec3d &dest);

    virtual NS::Ending getTrimEnd(const Vec3d &trimPoint,
                                  const Vec3d &clickPoint);
    virtual bool trimStartPoint(const Vec3d &trimPoint,
                                const Vec3d &clickPoint = Vec3d::invalid,
                                bool extend = false);
    virtual bool trimEndPoint(const Vec3d &trimPoint,
                              const Vec3d &clickPoint = Vec3d::invalid,
                              bool extend = false);
    virtual bool trimStartPoint(double trimDist)
    {
        return Shape::trimStartPoint(trimDist);
    }
    virtual bool trimEndPoint(double trimDist)
    {
        return Shape::trimEndPoint(trimDist);
    }
    virtual double getDistanceFromStart(const Vec3d &p) const;

    virtual std::vector<QSharedPointer<Shape>>
    getOffsetShapes(double distance, int number, NS::Side side,
                    const Vec3d &position = Vec3d::invalid)
    {
        return Shape::getOffsetLines(*this, distance, number, side, position);
    }

    virtual std::vector<QSharedPointer<Shape>>
    splitAt(const std::vector<Vec3d> &points) const;

public:
    /**
     * \getter{getStartPoint}
     * \setter{setStartPoint}
     */
    Vec3d startPoint;
    /**
     * \getter{getEndPoint}
     * \setter{setEndPoint}
     */
    Vec3d endPoint;
};

class Circle : public Shape {
public:
    Circle();
    Circle(double cx, double cy, const double radius);
    Circle(const Vec3d &center, const double radius);
    virtual ~Circle();

    virtual NS::ShapeType getShapeType() const { return Circle; }

    virtual Circle *clone() const { return new Circle(*this); }

    static Circle createFrom2Points(const Vec3d &p1, const Vec3d &p2);
    static Circle createFrom3Points(const Vec3d &p1, const Vec3d &p2,
                                    const Vec3d &p3);

    Arc toArc(double startAngle = 0.0) const;

    virtual bool isValid() const { return center.isValid(); }

    virtual void setZ(double z);

    virtual BBox getBoundingBox() const;
    virtual double getLength() const;

    virtual std::vector<Vec3d> getEndPoints() const;
    virtual std::vector<Vec3d> getMiddlePoints() const;
    virtual std::vector<Vec3d> getCenterPoints() const;
    virtual std::vector<Vec3d> getArcReferencePoints() const;
    virtual std::vector<Vec3d>
    getPointsWithDistanceToEnd(double distance, int from = NS::FromAny) const;
    virtual std::vector<Vec3d> getPointCloud(double segmentLength) const;

    virtual double getAngleAt(double distance,
                              NS::From from = NS::FromStart) const;
    Vec3d getPointAtAngle(double a) const;

    virtual Vec3d getVectorTo(const Vec3d &point, bool limited = true,
                              double strictRange = DBL_MAX) const;

    Vec3d getCenter() const;
    void setCenter(const Vec3d &vector);
    double getRadius() const;
    void setRadius(double radius);

    double getDiameter() const;
    void setDiameter(double d);
    double getCircumference() const;
    void setCircumference(double c);
    double getArea() const;
    void setArea(double a);

    bool contains(const Vec3d &p) const;

    virtual bool move(const Vec3d &offset);
    virtual bool rotate(double rotation, const Vec3d &center = Vec3d());
    virtual bool scale(const Vec3d &scaleFactors,
                       const Vec3d &center = Vec3d());
    virtual bool mirror(const Line &axis);
    virtual bool flipHorizontal();
    virtual bool flipVertical();

    std::vector<Line> getTangents(const Vec3d &point) const;

    virtual std::vector<QSharedPointer<Shape>>
    getOffsetShapes(double distance, int number, NS::Side side,
                    const Vec3d &position = Vec3d::invalid)
    {
        return Shape::getOffsetArcs(*this, distance, number, side, position);
    }

    virtual std::vector<QSharedPointer<Shape>>
    splitAt(const std::vector<Vec3d> &points) const;

public:
    /**
     * \getter{getCenter}
     * \setter{setCenter}
     */
    Vec3d center;
    /**
     * \getter{getRadius}
     * \setter{setRadius}
     */
    double radius;
};

class Arc : public Shape {
public:
    Arc();
    Arc(double cx, double cy, double radius, double startAngle, double endAngle,
        bool reversed = false);
    Arc(const Vec3d &center, double radius, double startAngle, double endAngle,
        bool reversed = false);

    virtual NS::ShapeType getShapeType() const { return Arc; }

    virtual Arc *clone() const { return new Arc(*this); }

    virtual bool isDirected() const { return true; }

    virtual void setZ(double z);

    virtual bool isValid() const;
    bool isFullCircle(double tolerance = NS::AngleTolerance) const;

    static Arc createFrom3Points(const Vec3d &startPoint, const Vec3d &point,
                                 const Vec3d &endPoint);
    static Arc createFrom2PBulge(const Vec3d &startPoint, const Vec3d &endPoint,
                                 double bulge);
    static Arc createTangential(const Vec3d &startPoint, const Vec3d &pos,
                                double direction, double radius);
    static std::vector<Arc> createBiarc(const Vec3d &startPoint,
                                        double startDirection,
                                        const Vec3d &endPoint,
                                        double endDirection,
                                        bool secondTry = false);

    virtual BBox getBoundingBox() const;

    virtual std::vector<Vec3d> getEndPoints() const;
    virtual std::vector<Vec3d> getMiddlePoints() const;
    virtual std::vector<Vec3d> getCenterPoints() const;
    virtual std::vector<Vec3d> getArcReferencePoints() const;
    virtual std::vector<Vec3d>
    getPointsWithDistanceToEnd(double distance, int from = NS::FromAny) const;
    virtual std::vector<Vec3d> getPointCloud(double segmentLength) const;

    virtual Vec3d getVectorTo(const Vec3d &point, bool limited = true,
                              double strictRange = DBL_MAX) const;

    Vec3d getCenter() const;
    void setCenter(const Vec3d &vector);
    double getRadius() const;
    void setRadius(double radius);
    double getStartAngle() const;
    void setStartAngle(double startAngle);
    double getEndAngle() const;
    void setEndAngle(double endAngle);
    bool isReversed() const;
    void setReversed(bool reversed);
    double getAngleLength(bool allowForZeroLength = false) const;
    bool isAngleWithinArc(double a) const
    {
        return Math::isAngleBetween(a, startAngle, endAngle, reversed);
    }

    double getDiameter() const;
    void setDiameter(double d);
    void setLength(double l);
    double getArea() const;
    void setArea(double a);
    double getChordArea() const;

    virtual double getDirection1() const;
    virtual double getDirection2() const;

    virtual NS::Side getSideOfPoint(const Vec3d &point) const;

    double getSweep() const;
    void setSweep(double s);
    double getLength() const;

    virtual Vec3d getStartPoint() const;
    virtual Vec3d getEndPoint() const;
    Vec3d getPointAtAngle(double a) const;
    virtual double getAngleAt(double distance,
                              NS::From from = NS::FromStart) const;
    virtual Vec3d getMiddlePoint() const;

    void moveStartPoint(const Vec3d &pos, bool keepRadius = true);
    void moveEndPoint(const Vec3d &pos, bool keepRadius = true);
    void moveMiddlePoint(const Vec3d &pos);
    double getBulge() const;

    virtual bool move(const Vec3d &offset);
    virtual bool rotate(double rotation, const Vec3d &center = Vec3d());
    virtual bool scale(const Vec3d &scaleFactors,
                       const Vec3d &center = Vec3d());
    virtual bool mirror(const Line &axis);
    virtual bool reverse();
    virtual bool stretch(const Polyline &area, const Vec3d &offset);

    virtual NS::Ending getTrimEnd(const Vec3d &trimPoint,
                                  const Vec3d &clickPoint);
    virtual bool trimStartPoint(const Vec3d &trimPoint,
                                const Vec3d &clickPoint = Vec3d::invalid,
                                bool extend = false);
    virtual bool trimEndPoint(const Vec3d &trimPoint,
                              const Vec3d &clickPoint = Vec3d::invalid,
                              bool extend = false);
    virtual bool trimStartPoint(double trimDist)
    {
        return Shape::trimStartPoint(trimDist);
    }
    virtual bool trimEndPoint(double trimDist)
    {
        return Shape::trimEndPoint(trimDist);
    }
    virtual double getDistanceFromStart(const Vec3d &p) const;

    Polyline approximateWithLines(double segmentLength,
                                  double angle = 0.0) const;
    Polyline approximateWithLinesTan(double segmentLength,
                                     double angle = 0.0) const;

    std::vector<Line> getTangents(const Vec3d &point) const;

    virtual std::vector<QSharedPointer<Shape>>
    getOffsetShapes(double distance, int number, NS::Side side,
                    const Vec3d &position = Vec3d::invalid)
    {
        return Shape::getOffsetArcs(*this, distance, number, side, position);
    }

    virtual std::vector<QSharedPointer<Shape>>
    splitAt(const std::vector<Vec3d> &points) const;

    std::vector<Arc> splitAtQuadrantLines() const;

public:
    /**
     * \getter{getCenter}
     * \setter{setCenter}
     */
    Vec3d center;
    /**
     * \getter{getRadius}
     * \setter{setRadius}
     */
    double radius;
    /**
     * \getter{getStartAngle}
     * \setter{setStartAngle}
     */
    double startAngle;
    /**
     * \getter{getEndAngle}
     * \setter{setEndAngle}
     */
    double endAngle;
    /**
     * \getter{isReversed}
     * \setter{setReversed}
     */
    bool reversed;
};

class Ellipse : public Shape {
public:
    Ellipse();
    Ellipse(const Vec3d &center, const Vec3d &majorPoint, double ratio,
            double startParam, double endParam, bool reversed);
    virtual ~Ellipse();

    static Ellipse createInscribed(const Vec3d &p1, const Vec3d &p2,
                                   const Vec3d &p3, const Vec3d &p4,
                                   const Vec3d &centerHint = Vec3d::invalid);
    static Ellipse createFrom4Points(const Vec3d &p1, const Vec3d &p2,
                                     const Vec3d &p3, const Vec3d &p4);

    virtual NS::ShapeType getShapeType() const { return Ellipse; }

    virtual Ellipse *clone() const { return new Ellipse(*this); }

    virtual bool isDirected() const { return true; }

    virtual bool isValid() const;

    virtual void setZ(double z);

    virtual BBox getBoundingBox() const;

    virtual std::vector<Vec3d> getEndPoints() const;
    virtual std::vector<Vec3d> getMiddlePoints() const;
    virtual std::vector<Vec3d> getCenterPoints() const;
    virtual std::vector<Vec3d>
    getPointsWithDistanceToEnd(double distance, int from = NS::FromAny) const;
    virtual std::vector<Vec3d> getPointCloud(double segmentLength) const;

    virtual Vec3d getVectorTo(const Vec3d &point, bool limited = true,
                              double strictRange = DBL_MAX) const;

    void moveStartPoint(const Vec3d &pos, bool changeAngleOnly = false);
    void moveEndPoint(const Vec3d &pos, bool changeAngleOnly = false);

    std::vector<Vec3d> getFoci() const;

    Vec3d getCenter() const;
    void setCenter(const Vec3d &vector);
    Vec3d getMajorPoint() const;
    Vec3d getMinorPoint() const;
    void setMajorPoint(const Vec3d &vector);
    void setMinorPoint(const Vec3d &p);
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

    bool isAngleWithinArc(double a) const
    {
        if (isFullEllipse()) {
            return true;
        }
        return Math::isAngleBetween(a, getStartAngle(), getEndAngle(),
                                    reversed);
    }
    bool isParamWithinArc(double a) const
    {
        if (isFullEllipse()) {
            return true;
        }
        return Math::isAngleBetween(a, getStartParam(), getEndParam(),
                                    reversed);
    }

    bool isReversed() const;
    void setReversed(bool reversed);

    virtual double getDirection1() const;
    virtual double getDirection2() const;

    virtual NS::Side getSideOfPoint(const Vec3d &point) const;

    virtual Vec3d getStartPoint() const;
    virtual Vec3d getEndPoint() const;
    double getMajorRadius() const;
    double getMinorRadius() const;
    double getAngle() const;
    void setAngle(double a);
    bool isFullEllipse() const;
    bool isCircular() const;
    double getLength() const;
    double getSimpsonLength(double f1, double f2) const;

    bool contains(const Vec3d &p) const;

    virtual double getAngleAt(double distance,
                              NS::From from = NS::FromStart) const;

    double getAngleAtPoint(const Vec3d &pos) const;
    double getParamTo(const Vec3d &pos) const;
    double getRadiusAt(double param) const;
    Vec3d getPointAt(double param) const;
    Vec3d getMiddlePoint() const;

    virtual Vec3d getPointOnShape() const;

    virtual bool move(const Vec3d &offset);
    virtual bool rotate(double rotation, const Vec3d &center = Vec3d());
    virtual bool scale(const Vec3d &scaleFactors,
                       const Vec3d &center = Vec3d());
    virtual bool mirror(const Line &axis);

    virtual bool reverse();

    virtual NS::Ending getTrimEnd(const Vec3d &trimPoint,
                                  const Vec3d &clickPoint);
    virtual bool trimStartPoint(const Vec3d &trimPoint,
                                const Vec3d &clickPoint = Vec3d::invalid,
                                bool extend = false);
    virtual bool trimEndPoint(const Vec3d &trimPoint,
                              const Vec3d &clickPoint = Vec3d::invalid,
                              bool extend = false);
    virtual bool trimStartPoint(double trimDist)
    {
        return Shape::trimStartPoint(trimDist);
    }
    virtual bool trimEndPoint(double trimDist)
    {
        return Shape::trimEndPoint(trimDist);
    }

    void correctMajorMinor();
    double getSweep() const;

    std::vector<Vec3d> getBoxCorners();

    std::vector<Line> getTangents(const Vec3d &point) const;
    Vec3d getTangentPoint(const Line &line) const;

    std::vector<BSpline> approximateWithSplines() const;
    Polyline approximateWithArcs(int segments) const;

    virtual std::vector<QSharedPointer<Shape>>
    getOffsetShapes(double distance, int number, NS::Side side,
                    const Vec3d &position = Vec3d::invalid);
    virtual std::vector<QSharedPointer<Shape>>
    splitAt(const std::vector<Vec3d> &points) const;

public:
    /**
     * \getter{getCenter}
     * \setter{setCenter}
     */
    Vec3d center;
    /**
     * \getter{getMajorPoint}
     * \setter{setMajorPoint}
     */
    Vec3d majorPoint;
    /**
     * \getter{getRatio}
     * \setter{setRatio}
     */
    double ratio;
    /**
     * \getter{getStartParam}
     * \setter{setStartParam}
     */
    double startParam;
    /**
     * \getter{getEndParam}
     * \setter{setEndParam}
     */
    double endParam;
    /**
     * \getter{isReversed}
     * \setter{setReversed}
     */
    bool reversed;

private:
    static REllipseProxy *ellipseProxy;
};

class Polyline : public Shape {
public:
    Polyline();
    Polyline(const std::vector<Vec3d> &vertices, bool closed);
    Polyline(const std::vector<QSharedPointer<Shape>> &segments);
    virtual ~Polyline();

    virtual NS::ShapeType getShapeType() const { return Polyline; }

    virtual Polyline *clone() const { return new Polyline(*this); }

    virtual bool isDirected() const { return true; }

    virtual void setZ(double z);
    bool isFlat() const;

    void clear();
    void normalize(double tolerance = NS::PointTolerance);

    bool prependShape(const Shape &shape);
    bool appendShape(const Shape &shape, bool prepend = false);
    bool appendShapeAuto(const Shape &shape);
    bool appendShapeTrim(const Shape &shape);
    bool closeTrim();

    void appendVertex(const Vec3d &vertex, double bulge = 0.0, double w1 = 0.0,
                      double w2 = 0.0);
    void appendVertex(double x, double y, double bulge = 0.0, double w1 = 0.0,
                      double w2 = 0.0);
    void prependVertex(const Vec3d &vertex, double bulge = 0.0, double w1 = 0.0,
                       double w2 = 0.0);
    void insertVertex(int index, const Vec3d &vertex, double bulgeBefore = 0.0,
                      double bulgeAfter = 0.0);
    void insertVertexAt(const Vec3d &point);
    Vec3d insertVertexAtDistance(double dist);
    void removeFirstVertex();
    void removeLastVertex();
    void removeVertex(int index);
    void removeVerticesAfter(int index);
    void removeVerticesBefore(int index);

    bool isEmpty() const { return countVertices() == 0; }

    void setVertices(const std::vector<Vec3d> &vertices);
    std::vector<Vec3d> getVertices() const;
    void setVertexAt(int i, const Vec3d &v);
    void moveVertexAt(int i, const Vec3d &offset);
    Vec3d getVertexAt(int i) const;
    int getVertexIndex(const Vec3d &v,
                       double tolerance = NS::PointTolerance) const;
    Vec3d getLastVertex() const;
    int countVertices() const;

    void setBulges(const std::vector<double> &b);
    std::vector<double> getBulges() const;
    double getBulgeAt(int i) const;
    void setBulgeAt(int i, double b);
    bool hasArcSegments() const;

    std::vector<double> getVertexAngles() const;
    double
    getVertexAngle(int i,
                   NS::Orientation orientation = NS::UnknownOrientation) const;

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
    bool isGeometricallyClosed(double tolerance = NS::PointTolerance) const;
    bool autoClose(double tolerance = NS::PointTolerance)
    {
        return toLogicallyClosed(tolerance);
    }
    bool toLogicallyClosed(double tolerance = NS::PointTolerance);
    bool toLogicallyOpen();

    std::vector<Vec3d>
    getSelfIntersectionPoints(double tolerance = NS::PointTolerance) const;

    NS::Orientation getOrientation(bool implicitelyClosed = false) const;
    bool setOrientation(NS::Orientation orientation);

    Polyline convertArcToLineSegments(int segments) const;
    Polyline convertArcToLineSegmentsLength(double segmentLength) const;

    virtual bool contains(const Vec3d &point, bool borderIsInside = false,
                          double tolerance = NS::PointTolerance) const;
    bool containsShape(const Shape &shape) const;

    Vec3d getPointInside() const;

    virtual Vec3d getStartPoint() const;
    virtual Vec3d getEndPoint() const;
    virtual Vec3d getMiddlePoint() const;

    void moveStartPoint(const Vec3d &pos);
    void moveEndPoint(const Vec3d &pos);

    void moveSegmentAt(int i, const Vec3d &offset);

    virtual double getDirection1() const;
    virtual double getDirection2() const;

    virtual NS::Side getSideOfPoint(const Vec3d &point) const;

    virtual BBox getBoundingBox() const;

    double getArea() const;

    virtual double getLength() const;

    virtual double getDistanceFromStart(const Vec3d &p) const
    {
        std::vector<double> res = getDistancesFromStart(p);
        if (res.isEmpty()) {
            return DBL_MAX;
        }
        return res.first();
    }
    virtual std::vector<double> getDistancesFromStart(const Vec3d &p) const;
    double getLengthTo(const Vec3d &p, bool limited = true) const;
    double getSegmentsLength(int fromIndex, int toIndex) const;

    virtual std::vector<Vec3d> getEndPoints() const;
    virtual std::vector<Vec3d> getMiddlePoints() const;
    virtual std::vector<Vec3d> getCenterPoints() const;
    virtual Vec3d getPointAtPercent(double p) const;
    virtual std::vector<Vec3d>
    getPointsWithDistanceToEnd(double distance, int from = NS::FromAny) const;
    virtual std::vector<Vec3d> getPointCloud(double segmentLength) const;

    virtual double getAngleAt(double distance,
                              NS::From from = NS::FromStart) const;

    virtual Vec3d getVectorTo(const Vec3d &point, bool limited = true,
                              double strictRange = DBL_MAX) const;
    virtual double getDistanceTo(const Vec3d &point, bool limited = true,
                                 double strictRange = DBL_MAX) const;

    int getClosestSegment(const Vec3d &point) const;
    int getClosestVertex(const Vec3d &point) const;

    virtual bool move(const Vec3d &offset);
    virtual bool rotate(double rotation, const Vec3d &center = Vec3d());
    virtual bool scale(double scaleFactor, const Vec3d &center = Vec3d());
    virtual bool scale(const Vec3d &scaleFactors,
                       const Vec3d &center = Vec3d());
    virtual bool mirror(const Line &axis);
    virtual bool reverse();
    virtual Polyline getReversed() const;
    virtual bool stretch(const Polyline &area, const Vec3d &offset);

    virtual NS::Ending getTrimEnd(const Vec3d &trimPoint,
                                  const Vec3d &clickPoint);
    virtual bool trimStartPoint(const Vec3d &trimPoint,
                                const Vec3d &clickPoint = Vec3d::invalid,
                                bool extend = false);
    virtual bool trimEndPoint(const Vec3d &trimPoint,
                              const Vec3d &clickPoint = Vec3d::invalid,
                              bool extend = false);
    virtual bool trimStartPoint(double trimDist);
    virtual bool trimEndPoint(double trimDist);

    virtual std::vector<QSharedPointer<Shape>>
    getExploded(int segments = RDEFAULT_MIN1) const;
    std::vector<Polyline> getOutline() const;
    std::vector<QPair<Polyline, Polyline>> getLeftRightOutline() const;
    std::vector<Polyline> getLeftOutline() const
    {
        std::vector<QPair<Polyline, Polyline>> lr = getLeftRightOutline();
        std::vector<Polyline> ret;
        for (int i = 0; i < lr.length(); i++) {
            ret.append(lr[i].first);
        }
        return ret;
    }
    std::vector<Polyline> getRightOutline() const
    {
        std::vector<QPair<Polyline, Polyline>> lr = getLeftRightOutline();
        std::vector<Polyline> ret;
        for (int i = 0; i < lr.length(); i++) {
            ret.append(lr[i].second);
        }
        return ret;
    }
    virtual bool isInterpolated() const { return false; }
    int countSegments() const;
    QSharedPointer<Shape> getSegmentAt(int i) const;
    bool isArcSegmentAt(int i) const;
    QSharedPointer<Shape> getLastSegment() const;
    QSharedPointer<Shape> getFirstSegment() const;

    static bool isStraight(double bulge);

    RPainterPath toPainterPath(bool addOriginalShapes = false) const;

    bool simplify(double tolerance = NS::PointTolerance);
    std::vector<Vec3d> verifyTangency(double toleranceMin = NS::AngleTolerance,
                                      double toleranceMax = M_PI_4);

    void stripWidths();
    void setMinimumWidth(double w);

    int getSegmentAtDist(double dist);
    bool relocateStartPoint(const Vec3d &p);
    bool relocateStartPoint(double dist);
    bool convertToClosed();
    bool convertToOpen();

    Polyline modifyPolylineCorner(const Shape &trimmedShape1,
                                  NS::Ending ending1, int segmentIndex1,
                                  const Shape &trimmedShape2,
                                  NS::Ending ending2, int segmentIndex2,
                                  const Shape *cornerShape = NULL) const;

    bool isConcave() const;
    std::vector<Vec3d> getConvexVertices(bool convex = true) const;
    std::vector<Vec3d> getConcaveVertices() const;

    Vec3d getCentroid() const;

    std::vector<Polyline> splitAtDiscontinuities(double tolerance) const;
    std::vector<Polyline> splitAtSegmentTypeChange() const;

    double getBaseAngle() const;
    double getWidth() const;
    bool setWidth(double v);
    double getHeight() const;
    bool setHeight(double v);

    std::vector<Polyline> morph(const Polyline &target, int steps,
                                NS::Easing easing = NS::Linear,
                                bool zLinear = true,
                                double customFactor = RNANDOUBLE) const;
    Polyline roundAllCorners(double radius) const;
    Polyline getPolygon(double segmentLength) const;
    Polyline getPolygonHull(double angle, double tolerance,
                            bool inner = false) const;

protected:
    bool isLineSegment(int i) const;

protected:
    /**
     * \getter{getVertices}
     * \setter{setVertices}
     */
    std::vector<Vec3d> vertices;

    std::vector<double> bulges;

    std::vector<double> endWidths;
    std::vector<double> startWidths;

    /**
     * \getter{isClosed}
     * \setter{setClosed}
     */
    bool closed;
};

class XLine : public Shape {
public:
    XLine();
    XLine(const Line &line);
    XLine(const Vec3d &basePoint, const Vec3d &directionVector);
    XLine(const Vec3d &basePoint, double angle, double distance);
    virtual ~XLine();

    virtual NS::ShapeType getShapeType() const { return XLine; }

    Line getLineShape() const
    {
        return Line(basePoint, basePoint + directionVector);
    }

    virtual XLine *clone() const { return new XLine(*this); }

    virtual bool isDirected() const { return true; }

    BBox getBoundingBox() const;

    virtual void setZ(double z);

    virtual std::vector<Vec3d> getEndPoints() const;
    virtual std::vector<Vec3d> getMiddlePoints() const;
    virtual std::vector<Vec3d> getCenterPoints() const;
    virtual std::vector<Vec3d>
    getPointsWithDistanceToEnd(double distance, int from = NS::FromAny) const;
    virtual std::vector<Vec3d> getPointCloud(double segmentLength) const;

    virtual double getAngleAt(double distance,
                              NS::From from = NS::FromStart) const;

    virtual Vec3d getVectorTo(const Vec3d &point, bool limited = true,
                              double strictRange = DBL_MAX) const;

    Vec3d getBasePoint() const;
    void setBasePoint(const Vec3d &vector);
    Vec3d getSecondPoint() const;
    void setSecondPoint(const Vec3d &vector);
    Vec3d getDirectionVector() const;
    void setDirectionVector(const Vec3d &vector);

    virtual Vec3d getMiddlePoint() const;

    double getLength() const;
    void setLength(double l);
    double getAngle() const;
    void setAngle(double a);

    virtual double getDirection1() const;
    virtual double getDirection2() const;

    virtual NS::Side getSideOfPoint(const Vec3d &point) const;

    virtual Vec3d getStartPoint() const;
    virtual Vec3d getEndPoint() const;

    virtual bool trimStartPoint(const Vec3d &trimPoint,
                                const Vec3d &clickPoint = Vec3d::invalid,
                                bool extend = false);
    virtual bool trimEndPoint(const Vec3d &trimPoint,
                              const Vec3d &clickPoint = Vec3d::invalid,
                              bool extend = false);
    virtual bool trimStartPoint(double trimDist)
    {
        return Shape::trimStartPoint(trimDist);
    }
    virtual bool trimEndPoint(double trimDist)
    {
        return Shape::trimEndPoint(trimDist);
    }
    virtual NS::Ending getTrimEnd(const Vec3d &trimPoint,
                                  const Vec3d &clickPoint);
    virtual double getDistanceFromStart(const Vec3d &p) const;

    virtual Line getClippedLine(const BBox &box) const;

    virtual bool move(const Vec3d &offset);
    virtual bool rotate(double rotation, const Vec3d &center = Vec3d());
    virtual bool scale(const Vec3d &scaleFactors,
                       const Vec3d &center = Vec3d());
    virtual bool mirror(const Line &axis);
    virtual bool reverse();
    virtual bool stretch(const Polyline &area, const Vec3d &offset);

    virtual std::vector<QSharedPointer<Shape>>
    getOffsetShapes(double distance, int number, NS::Side side,
                    const Vec3d &position = Vec3d::invalid)
    {
        return Shape::getOffsetLines(*this, distance, number, side, position);
    }

    virtual std::vector<QSharedPointer<Shape>>
    splitAt(const std::vector<Vec3d> &points) const;

public:
    /**
     * \getter{getBasePoint}
     * \setter{setBasePoint}
     */
    Vec3d basePoint;
    /**
     * \getter{getDirectionVector}
     * \setter{setDirectionVector}
     */
    Vec3d directionVector;
};

class Ray : public XLine {
public:
    Ray();
    Ray(const Line &line);
    Ray(const Vec3d &basePoint, const Vec3d &directionVector);
    Ray(const Vec3d &basePoint, double angle, double distance);
    virtual ~Ray();

    virtual NS::ShapeType getShapeType() const { return Ray; }

    virtual Ray *clone() const { return new Ray(*this); }

    virtual bool trimEndPoint(const Vec3d &trimPoint,
                              const Vec3d &clickPoint = Vec3d::invalid,
                              bool extend = false);
    virtual std::vector<Vec3d> getPointsWithDistanceToEnd(double distance,
                                                          int from) const;

    virtual bool reverse();
    virtual Line getClippedLine(const BBox &box) const;
    virtual Vec3d getVectorTo(const Vec3d &point, bool limited = true,
                              double strictRange = DBL_MAX) const;

    virtual bool stretch(const Polyline &area, const Vec3d &offset);

    virtual std::vector<QSharedPointer<Shape>>
    splitAt(const std::vector<Vec3d> &points) const;
};

class BSpline : public Shape {
public:
    BSpline();
    BSpline(const BSpline &other);
    BSpline(const std::vector<Vec3d> &controlPoints, int degree);

    BSpline &operator=(const BSpline &other);

    virtual NS::ShapeType getShapeType() const { return Spline; }

    virtual BSpline *clone() const { return new BSpline(*this); }

    virtual bool isDirected() const { return true; }

    void copySpline(const BSpline &other);

    static std::vector<BSpline> createSplinesFromArc(const Arc &arc);
    static BSpline createBezierFromSmallArc(double r, double a1, double a2);

    virtual void setZ(double z);

    virtual bool isInterpolated() const { return true; }

    void appendControlPoint(const Vec3d &point);
    void appendControlPoints(const std::vector<Vec3d> &points);
    void removeLastControlPoint();
    void setControlPoints(const std::vector<Vec3d> &points);
    std::vector<Vec3d> getControlPoints() const;
    std::vector<Vec3d> getControlPointsWrapped() const;
    int countControlPoints() const;
    Vec3d getControlPointAt(int i) const;

    void appendFitPoint(const Vec3d &point);
    void prependFitPoint(const Vec3d &point);
    void insertFitPointAt(const Vec3d &point);
    void insertFitPointAt(double t, const Vec3d &point);
    void removeFitPointAt(const Vec3d &point);
    void removeFirstFitPoint();
    void removeLastFitPoint();
    void setFitPoints(const std::vector<Vec3d> &points);
    std::vector<Vec3d> getFitPoints() const;
    int countFitPoints() const;
    bool hasFitPoints() const;
    Vec3d getFitPointAt(int i) const;

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
    // bool isClosedPeriodic() const;

    bool isClosed() const;
    bool isGeometricallyClosed(double tolerance = NS::PointTolerance) const;
    bool isPeriodic() const;

    virtual double getDirection1() const;
    virtual double getDirection2() const;

    virtual NS::Side getSideOfPoint(const Vec3d &point) const;

    virtual Vec3d getStartPoint() const;
    virtual Vec3d getEndPoint() const;

    void setStartPoint(const Vec3d &v);
    void setEndPoint(const Vec3d &v);

    void setTangents(const Vec3d &start, const Vec3d &end);
    void unsetTangents();

    void setTangentAtStart(const Vec3d &t);
    Vec3d getTangentAtStart() const;
    void unsetTangentAtStart();
    void setTangentAtEnd(const Vec3d &t);
    Vec3d getTangentAtEnd() const;
    void unsetTangentAtEnd();

    void updateTangentsPeriodic();

    virtual BBox getBoundingBox() const;

    virtual double getLength() const;
    Vec3d getPointAt(double t) const;
    Vec3d getPointAtDistance(double distance) const;
    virtual double getAngleAt(double distance,
                              NS::From from = NS::FromStart) const;

    virtual std::vector<Vec3d> getEndPoints() const;
    virtual Vec3d getMiddlePoint() const;
    virtual std::vector<Vec3d> getMiddlePoints() const;
    virtual std::vector<Vec3d> getCenterPoints() const;
    virtual std::vector<Vec3d>
    getPointsWithDistanceToEnd(double distance, int from = NS::FromAny) const;
    virtual std::vector<Vec3d> getPointCloud(double segmentLength) const;

    virtual Vec3d getVectorTo(const Vec3d &point, bool limited = true,
                              double strictRange = DBL_MAX) const;
    virtual bool isOnShape(const Vec3d &point, bool limited = true,
                           double tolerance = DBL_EPSILON) const;

    virtual bool move(const Vec3d &offset);
    virtual bool rotate(double rotation, const Vec3d &center = Vec3d());
    virtual bool scale(const Vec3d &scaleFactors,
                       const Vec3d &center = Vec3d());
    virtual bool mirror(const Line &axis);
    virtual bool flipHorizontal();
    virtual bool flipVertical();
    virtual bool reverse();
    virtual bool stretch(const Polyline &area, const Vec3d &offset);

    virtual NS::Ending getTrimEnd(const Vec3d &trimPoint,
                                  const Vec3d &clickPoint);
    virtual bool trimStartPoint(const Vec3d &trimPoint,
                                const Vec3d &clickPoint = Vec3d::invalid,
                                bool extend = false);
    virtual bool trimEndPoint(const Vec3d &trimPoint,
                              const Vec3d &clickPoint = Vec3d::invalid,
                              bool extend = false);
    virtual bool trimStartPoint(double trimDist)
    {
        return Shape::trimStartPoint(trimDist);
    }
    virtual bool trimEndPoint(double trimDist)
    {
        return Shape::trimEndPoint(trimDist);
    }
    virtual double getDistanceFromStart(const Vec3d &p) const;

    std::vector<BSpline> splitAtPoints(const std::vector<Vec3d> &points) const;
    std::vector<BSpline> splitAtParams(const std::vector<double> &params) const;

    Polyline toPolyline(int segments) const;
    Polyline approximateWithArcs(double tolerance,
                                 double radiusLimit = RDEFAULT_MIN1) const;

    virtual std::vector<QSharedPointer<Shape>>
    getExploded(int segments = RDEFAULT_MIN1) const;
    std::vector<QSharedPointer<Shape>> getExplodedBezier(int segments) const;
    std::vector<QSharedPointer<Shape>>
    getExplodedWithSegmentLength(double segmentLength) const;

    std::vector<BSpline>
    getBezierSegments(const BBox &queryBox = RDEFAULT_RBOX) const;

    virtual bool isValid() const;
    double getTDelta() const;
    double getTMin() const;
    double getTMax() const;
    double getTAtPoint(const Vec3d &point) const;
    double getTAtDistance(double distance) const;
    double getDistanceAtT(double t) const;
    std::vector<BSpline> getSegments(const std::vector<Vec3d> &points) const;

    std::vector<Vec3d> getDiscontinuities() const;
    BSpline simplify(double tolerance);

    void updateFromControlPoints() const;
    void updateFromFitPoints() const;
    void update() const;

    virtual std::vector<QSharedPointer<Shape>>
    splitAt(const std::vector<Vec3d> &points) const;

    bool isDirty() const { return dirty; }

    std::vector<Vec3d>
    getSelfIntersectionPoints(double tolerance = NS::PointTolerance) const;

protected:
    void appendToExploded(const Line &line) const;
    // void appendToExploded(std::vector<QSharedPointer<Shape> >& list) const;
    void invalidate() const;
    void updateInternal() const;
    void updateBoundingBox() const;

    virtual void print(QDebug dbg) const;

public:
    // members are mutable, so the spline can update itself from fit points

    /**
     * \getter{getControlPoints}
     * \setter{setControlPoints}
     */
    mutable std::vector<Vec3d> controlPoints;

    /**
     * \getter{getKnotVector}
     */
    mutable std::vector<double> knotVector;

    /**
     * \getter{getWeights}
     * \setter{setWeights}
     */
    mutable std::vector<double> weights;

    /**
     * \getter{getFitPoints}
     * \setter{setFitPoints}
     */
    std::vector<Vec3d> fitPoints;

    /**
     * \getter{getDegree}
     * \setter{setDegree}
     */
    mutable int degree;

    /**
     * Unit vector start tangent.
     */
    mutable Vec3d tangentStart;

    /**
     * Unit vector end tangent.
     */
    mutable Vec3d tangentEnd;

    /**
     * Closed periodic flag.
     */
    mutable bool periodic;

    mutable bool dirty;
    mutable bool updateInProgress;

private:
    mutable BBox boundingBox;
    mutable std::vector<QSharedPointer<Shape>> exploded;
    // cached length:
    mutable double length;

    static RSplineProxy *splineProxy;
};

class Triangle : public Shape {
public:
    Triangle();
    Triangle(const Vec3d &p1, const Vec3d &p2, const Vec3d &p3);
    virtual ~Triangle();

    virtual NS::ShapeType getShapeType() const { return Triangle; }

    virtual Triangle *clone() const { return new Triangle(*this); }

    virtual void setZ(double z);

    Polyline getPolyline() const;
    NS::Orientation getOrientation() const;
    virtual bool reverse();

    static Triangle createArrow(const Vec3d &position, double direction,
                                double size);

    virtual BBox getBoundingBox() const;
    virtual double getLength() const;
    double getArea() const;
    Vec3d getCorner(int i) const;
    void setCorner(int i, const Vec3d &p);
    void setCorners(const Vec3d &c1, const Vec3d &c2, const Vec3d &c3);

    virtual std::vector<Vec3d> getEndPoints() const;
    virtual std::vector<Vec3d> getMiddlePoints() const;
    virtual std::vector<Vec3d> getCenterPoints() const;
    virtual std::vector<Vec3d>
    getPointsWithDistanceToEnd(double distance, int from = NS::FromAny) const;
    virtual std::vector<Vec3d> getPointCloud(double segmentLength) const;

    virtual double getDistanceTo(const Vec3d &point, bool limited = true,
                                 double strictRange = DBL_MAX) const;
    virtual Vec3d getVectorTo(const Vec3d &point, bool limited = true,
                              double strictRange = DBL_MAX) const;
    virtual Vec3d getNormal() const;

    bool isPointInTriangle(const Vec3d &p, bool treatAsQuadrant = false) const;
    bool isPointInQuadrant(const Vec3d &p) const;

    double getD() const;

    virtual std::vector<QSharedPointer<Shape>>
    getExploded(int segments = RDEFAULT_MIN1) const;

    virtual bool move(const Vec3d &offset)
    {
        corner[0].move(offset);
        corner[1].move(offset);
        corner[2].move(offset);
        return true;
    }
    virtual bool rotate(double rotation, const Vec3d &center = Vec3d())
    {
        corner[0].rotate(rotation, center);
        corner[1].rotate(rotation, center);
        corner[2].rotate(rotation, center);
        return true;
    }
    virtual bool scale(const Vec3d &scaleFactors, const Vec3d &center = Vec3d())
    {
        corner[0].scale(scaleFactors, center);
        corner[1].scale(scaleFactors, center);
        corner[2].scale(scaleFactors, center);
        return true;
    }
    virtual bool mirror(const Line &axis)
    {
        corner[0].mirror(axis);
        corner[1].mirror(axis);
        corner[2].mirror(axis);
        return true;
    }
    virtual bool flipHorizontal()
    {
        corner[0].flipHorizontal();
        corner[1].flipHorizontal();
        corner[2].flipHorizontal();
        return true;
    }
    virtual bool flipVertical()
    {
        corner[0].flipVertical();
        corner[1].flipVertical();
        corner[2].flipVertical();
        return true;
    }

public:
    Vec3d corner[3];
};

} // namespace cada

#endif