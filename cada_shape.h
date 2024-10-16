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
#include <memory>
#include <array>

namespace cada {

class BBox;

class Shape;
class Point;
class Line;
class Circle;
class Arc;
class Ellipse;
class Polyline;
class XLine;
class Ray;
class BSpline;
class Triangle;

struct Vec2d {
    double x;
    double y;
    double z;
    bool valid;

    Vec2d();
    Vec2d(double vx, double vy, double vz = 0.0, bool valid_in = true);
    Vec2d(const std::vector<double> &tuples);
    ~Vec2d();

    void set(double vx, double vy, double vz = 0.0);
    void setPolar(double radius, double angle);
    Vec2d get2D() const;

    bool isValid() const;
    bool isZero() const;
    bool isSane() const;
    bool isNaN() const;

    bool isInside(const BBox &b) const;

    bool equalsFuzzy(const Vec2d &v, double tol = DBL_EPSILON) const;
    bool equalsFuzzy2D(const Vec2d &v, double tol = DBL_EPSILON) const;
    double getDistanceTo(const Vec2d &v) const;
    double getDistanceTo2D(const Vec2d &v) const;
    void setAngle(double a);
    double getAngle() const;
    double getAngleToPlaneXY() const;
    double getAngleTo(const Vec2d &v) const;
    void setMagnitude2D(double m);
    double getMagnitude() const;
    double getSquaredMagnitude() const;
    double getMagnitude2D() const;
    Vec2d getLerp(const Vec2d &v, double t) const;
    Vec2d getUnitVector() const;

    Vec2d move(const Vec2d &offset);

    Vec2d rotate(double rotation);
    Vec2d rotate(double rotation, const Vec2d &center);
    Vec2d getRotated(double rotation, const Vec2d &center) const;

    Vec2d scale(double factor, const Vec2d &center = nullVector);
    Vec2d scale(const Vec2d &factors, const Vec2d &center = nullVector);
    Vec2d getScaled(const Vec2d &factors, const Vec2d &center) const;

    Vec2d mirror(const Line &axis);
    Vec2d getMirrored(const Line &axis) const;
    Vec2d mirror(const Vec2d &axis1, const Vec2d &axis2);
    Vec2d flipHorizontal();
    Vec2d flipVertical();
    Vec2d stretch(const Polyline &area, const Vec2d &offset);

    Vec2d getDividedComponents(const Vec2d &v) const;
    Vec2d getMultipliedComponents(const Vec2d &v) const;

    Vec2d getClosest(const std::vector<Vec2d> &list) const;
    Vec2d getClosest2D(const std::vector<Vec2d> &list) const;
    double getClosestDistance(const std::vector<Vec2d> &list, int counts);
    int getClosestIndex(const std::vector<Vec2d> &list,
                        bool ignoreZ = false) const;
    int getClosestIndex2D(const std::vector<Vec2d> &list) const;

    Vec2d operator+(const Vec2d &v) const;
    Vec2d operator-(const Vec2d &v) const;
    Vec2d operator*(double s) const;
    Vec2d operator/(double s) const;
    Vec2d operator-() const;
    Vec2d getNegated() const;
    Vec2d getAbsolute() const;

    void operator+=(const Vec2d &v);
    void operator-=(const Vec2d &v);
    void operator*=(double s);
    void operator/=(double s);

    bool operator==(const Vec2d &v) const;
    bool operator!=(const Vec2d &v) const { return !operator==(v); }

    double dot(const Vec2d &other) const;
    Vec2d normalize();
    Vec2d getNormalized() const;

    static bool containsFuzzy(const std::vector<Vec2d> &vectors, const Vec2d &v,
                              double tol = DBL_EPSILON);
    static int findFirstFuzzy(const std::vector<Vec2d> &vectors, const Vec2d &v,
                              double tol = DBL_EPSILON);

    static Vec2d getMinimum(const std::vector<Vec2d> &vectors);
    static Vec2d getMaximum(const std::vector<Vec2d> &vectors);

    static Vec2d getMinimum(const Vec2d &v1, const Vec2d &v2);
    static Vec2d getMaximum(const Vec2d &v1, const Vec2d &v2);

    static Vec2d getAverage(const Vec2d &v1, const Vec2d &v2);
    static Vec2d getAverage(const std::vector<Vec2d> &vectors);

    static Vec2d getCrossProduct(const Vec2d &v1, const Vec2d &v2);
    static double getDotProduct(const Vec2d &v1, const Vec2d &v2);
    static Vec2d createPolar(double radius, double angle);


    static std::vector<Vec2d>
    getSortedByDistance(const std::vector<Vec2d> &list, const Vec2d &v);
    static std::vector<Vec2d> getSortedByAngle(const std::vector<Vec2d> &list,
                                               const Vec2d &center,
                                               double angle);

    static const Vec2d invalid;
    static const Vec2d nullVector;
    static const Vec2d nanVector;
};


struct Vec3d {
    double x, y, z;

    Vec3d(double x, double y, double z);
    Vec3d operator-(const Vec3d &rhs) const;
    Vec3d operator*(double s) const;
    Vec3d operator+(const Vec3d &rhs) const;
};

struct BBox {
    Vec2d c1;
    Vec2d c2;

    BBox();
    BBox(double x1, double y1, double x2, double y2);
    BBox(const Vec2d &c1, const Vec2d &c2);
    BBox(const Vec2d &center, double range);
    BBox(const Vec2d &center, double width, double height);

    bool isValid() const;
    bool isSane() const;

    bool equalsFuzzy(const BBox &b, double tol = DBL_EPSILON) const;
    bool equalsFuzzy2D(const BBox &b, double tol = DBL_EPSILON) const;

    BBox get2D() const;

    double getWidth() const;
    double getHeight() const;
    Vec2d getSize() const;
    double getArea() const;
    Vec2d getCenter() const;
    Vec2d getMinimum() const;
    Vec2d getMaximum() const;
    bool isOutside(const BBox &other) const;
    bool isOutsideXY(const BBox &other) const;
    bool contains(const BBox &other) const;
    bool contains(const Vec2d &v) const;
    bool intersects(const BBox &other) const;
    bool intersectsWith(const Shape &shape, bool limited = true) const;

    void growToInclude(const BBox &other);
    void growToIncludeBoxes(const std::vector<BBox> &others);

    void growToInclude(const Vec2d &v);

    Vec2d getCorner1() const;
    void setCorner1(const Vec2d &v);
    Vec2d getCorner2() const;
    void setCorner2(const Vec2d &v);

    std::vector<Vec2d> getCorners() const;
    std::vector<Vec2d> getCorners2d() const;
    std::vector<Line> getLines2d() const;
    Polyline getPolyline2d() const;

    BBox &grow(double offset);
    BBox &growXY(double offset);
    BBox &growXY(double offsetX, double offsetY);

    void move(const Vec2d &offset);
    bool scaleByReference(const Vec2d &referencePoint, const Vec2d &targetPoint,
                          bool keepAspectRatio = false,
                          bool fromCenter = false);

    bool operator==(const BBox &other) const;
    bool operator!=(const BBox &other) const { return !operator==(other); }
};

class Plane
{
    std::array<double, 4> mRep;

public:
    Plane(const Vec3d& p1, const Vec3d &p2, const Vec3d& p3);
    Plane(const double& a, const double& b, const double& c, const double& d);

    double a() const;
    double b() const;
    double c() const;
    double d() const;

    Vec3d point() const;
    Vec3d base1() const;
    Vec3d base2() const;
    Vec3d orthogonalVector() const;
    bool on(const Vec3d& p) const;
    Vec3d projection(const Vec3d &p) const;
    Vec2d to2d(const Vec3d& p) const;
    Vec3d to3d(const Vec3d& p) const;
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

    virtual bool isValid() const;
    virtual NS::ShapeType getShapeType() const;
    virtual Shape *clone() const = 0;
    virtual bool isInterpolated() const;
    virtual BBox getBoundingBox() const = 0;
    virtual double getLength() const = 0;
    virtual Vec2d getVectorTo(const Vec2d &point, bool limited = true,
                              double strictRange = DBL_MAX) const = 0;

    virtual Vec2d getClosestPointOnShape(const Vec2d &p, bool limited = true,
                                         double strictRange = DBL_MAX) const;

    virtual bool equals(const Shape &other,
                        double tolerance = DBL_EPSILON) const;
    virtual double getDistanceTo(const Vec2d &point, bool limited = true,
                                 double strictRange = DBL_MAX) const;
    virtual double getMaxDistanceTo(const std::vector<Vec2d> &points,
                                    bool limited = true,
                                    double strictRange = DBL_MAX) const;
    virtual bool isOnShape(const Vec2d &point, bool limited = true,
                           double tolerance = DBL_EPSILON) const;
    virtual std::vector<Vec2d>
    filterOnShape(const std::vector<Vec2d> &pointList, bool limited = true,
                  double tolerance = DBL_EPSILON) const;
    virtual Vec2d getVectorFromEndpointTo(const Vec2d &point) const;
    virtual std::vector<Vec2d> getEndPoints() const = 0;
    virtual std::vector<Vec2d> getMiddlePoints() const = 0;
    virtual std::vector<Vec2d> getCenterPoints() const = 0;
    virtual std::vector<Vec2d> getPointCloud(double segmentLength) const = 0;
    virtual std::vector<Vec2d>
    getPointsWithDistanceToEnd(double distance,
                               int from = NS::FromAny) const = 0;

    virtual std::vector<Vec2d> getArcRefPoints() const;
    virtual Vec2d getPointOnShape() const;

    virtual Vec2d getPointWithDistanceToStart(double distance) const;
    virtual Vec2d getPointWithDistanceToEnd(double distance) const;
    virtual double getAngleAt(double distance,
                              NS::From from = NS::FromStart) const;
    virtual double getAngleAtPoint(const Vec2d &pos) const;

    virtual Vec2d getPointAtPercent(double p) const;
    virtual double getAngleAtPercent(double p) const;

    virtual bool intersectsWith(const Shape &other, bool limited = true) const;

    std::vector<Vec2d> getIntersectionPoints(const Shape &other,
                                             bool limited = true,
                                             bool same = false,
                                             bool force = false) const;

    virtual std::vector<Vec2d>
    getSelfIntersectionPoints(double tolerance = DBL_EPSILON) const;

    virtual bool isDirected() const;
    virtual double getDirection1() const;
    virtual double getDirection2() const;

    virtual NS::Side getSideOfPoint(const Vec2d &point) const;
    virtual Vec2d getStartPoint() const;
    virtual Vec2d getEndPoint() const;
    virtual Vec2d getMiddlePoint() const;

    virtual bool reverse();

    virtual bool trimStartPoint(const Vec2d &trimPoint,
                                const Vec2d &clickPoint = Vec2d::invalid,
                                bool extend = false);
    virtual bool trimStartPoint(double trimDist);
    virtual bool trimEndPoint(const Vec2d &trimPoint,
                              const Vec2d &clickPoint = Vec2d::invalid,
                              bool extend = false);

    virtual bool trimEndPoint(double trimDist);
    virtual NS::Ending getTrimEnd(const Vec2d &trimPoint,
                                  const Vec2d &clickPoint);
    virtual double getDistanceFromStart(const Vec2d &p) const;
    virtual std::vector<double> getDistancesFromStart(const Vec2d &p) const;
    static std::vector<Vec2d> getIntersectionPoints(const Shape &shape1,
                                                    const Shape &shape2,
                                                    bool limited = true,
                                                    bool same = false,
                                                    bool force = false);

    virtual bool move(const Vec2d &offset) = 0;
    virtual bool rotate(double rotation, const Vec2d &center = Vec2d()) = 0;
    virtual bool scale(double scaleFactor, const Vec2d &center = Vec2d());
    virtual bool scale(const Vec2d &scaleFactors,
                       const Vec2d &center = Vec2d()) = 0;
    virtual bool mirror(const Line &axis) = 0;
    virtual bool flipHorizontal();
    virtual bool flipVertical();
    virtual bool stretch(const BBox &area, const Vec2d &offset);
    virtual bool stretch(const Polyline &area, const Vec2d &offset);

    static std::vector<Polyline>
    getPolylines(const std::vector<std::shared_ptr<Shape>> &shapes);
    static std::vector<std::shared_ptr<Shape>>
    getOrderedShapes(const std::vector<std::shared_ptr<Shape>> &shapes);
    static bool
    order(std::vector<std::vector<std::shared_ptr<Shape>>> &boundary);

    virtual std::vector<std::shared_ptr<Shape>>
    getOffsetShapes(double distance, int number, NS::Side side,
                    const Vec2d &position = Vec2d::invalid);

    static std::vector<std::shared_ptr<Shape>>
    getOffsetLines(const Shape &shape, double distance, int number,
                   NS::Side side, const Vec2d &position = Vec2d::invalid);
    static std::vector<std::shared_ptr<Shape>>
    getOffsetArcs(const Shape &shape, double distance, int number,
                  NS::Side side, const Vec2d &position = Vec2d::invalid);

    static std::vector<std::shared_ptr<Shape>>
    getReversedShapeList(const std::vector<std::shared_ptr<Shape>> &shapes);

    virtual std::vector<std::shared_ptr<Shape>>
    splitAt(const std::vector<Vec2d> &points) const;

    static std::vector<std::shared_ptr<Shape>>
    trim(const Shape &trimShape, const Vec2d &trimClickPos,
         const Shape &limitingShape, const Vec2d &limitingClickPos,
         bool trimBoth, bool samePolyline);

    static std::vector<std::shared_ptr<Shape>>
    roundCorners(const std::vector<std::shared_ptr<Shape>> &shapes,
                 double radius);

    static std::vector<std::shared_ptr<Shape>>
    roundShapes(const std::shared_ptr<Shape> shape1, const Vec2d &clickPos1,
                const std::shared_ptr<Shape> shape2, const Vec2d &clickPos2,
                bool trim, bool samePolyline, double radius, const Vec2d &pos);

    static std::shared_ptr<Shape> xLineToRay(std::shared_ptr<Shape> shape);
    static std::shared_ptr<Shape> rayToLine(std::shared_ptr<Shape> shape);

    static std::shared_ptr<Shape> scaleArc(const Shape &shape,
                                           const Vec2d &scaleFactors,
                                           const Vec2d &center = Vec2d());

    static std::shared_ptr<Shape>
    ellipseToArcCircleEllipse(const Ellipse &ellipse);

    virtual std::vector<std::shared_ptr<Shape>>
    roundAllCorners(const std::vector<std::shared_ptr<Shape>> &shapes,
                    double radius) = 0;

private:
    static double ellipse2tr(double x, double y, double AA, double BB,
                             double CC, double DD, double EE, double FF);
};

class Point : public Shape {
public:
    Point();
    Point(double x, double y);
    Point(const Vec2d &position);
    virtual ~Point();

    virtual NS::ShapeType getShapeType() const;
    virtual Point *clone() const;

    Vec2d getPosition() const;
    void setPosition(const Vec2d &p);

    virtual BBox getBoundingBox() const;
    virtual double getLength() const;

    virtual std::vector<Vec2d> getEndPoints() const;
    virtual std::vector<Vec2d> getMiddlePoints() const;
    virtual std::vector<Vec2d> getCenterPoints() const;
    virtual std::vector<Vec2d>
    getPointsWithDistanceToEnd(double distance, int from = NS::FromAny) const;
    virtual std::vector<Vec2d> getPointCloud(double segmentLength) const;

    virtual double getAngleAt(double distance,
                              NS::From from = NS::FromStart) const;

    virtual Vec2d getVectorTo(const Vec2d &point, bool limited = true,
                              double strictRange = DBL_MAX) const;

    virtual bool move(const Vec2d &offset);
    virtual bool rotate(double rotation, const Vec2d &center = Vec2d());
    virtual bool scale(const Vec2d &scaleFactors,
                       const Vec2d &center = Vec2d());
    virtual bool mirror(const Line &axis);
    virtual bool flipHorizontal();
    virtual bool flipVertical();

private:
    Vec2d position;
};

class Line : public Shape {
public:
    Line();
    Line(double x1, double y1, double x2, double y2);
    Line(const Vec2d &startPoint, const Vec2d &endPoint);
    Line(const Vec2d &startPoint, double angle, double distance);

    virtual NS::ShapeType getShapeType() const;
    virtual Line *clone() const;

    virtual bool isDirected() const;

    virtual bool isValid() const;

    virtual BBox getBoundingBox() const;

    virtual std::vector<Vec2d> getEndPoints() const;
    virtual std::vector<Vec2d> getMiddlePoints() const;
    virtual std::vector<Vec2d> getCenterPoints() const;
    virtual std::vector<Vec2d>
    getPointsWithDistanceToEnd(double distance, int from = NS::FromAny) const;
    virtual std::vector<Vec2d> getPointCloud(double segmentLength) const;

    virtual double getAngleAt(double distance,
                              NS::From from = NS::FromStart) const;

    virtual Vec2d getVectorTo(const Vec2d &point, bool limited = true,
                              double strictRange = DBL_MAX) const;

    virtual Vec2d getStartPoint() const;
    void setStartPoint(const Vec2d &vector);
    virtual Vec2d getEndPoint() const;
    void setEndPoint(const Vec2d &vector);

    virtual Vec2d getMiddlePoint() const;

    double getLength() const;
    double getAngle() const;

    void setLength(double l, bool fromStart = true);
    void setAngle(double a);

    bool isParallel(const Line &line) const;

    bool isVertical(double tolerance = DBL_EPSILON) const;
    bool isHorizontal(double tolerance = DBL_EPSILON) const;

    virtual double getDirection1() const;
    virtual double getDirection2() const;

    virtual NS::Side getSideOfPoint(const Vec2d &point) const;

    void clipToXY(const BBox &box);

    virtual bool move(const Vec2d &offset);
    virtual bool rotate(double rotation, const Vec2d &center = Vec2d());
    virtual bool scale(const Vec2d &scaleFactors,
                       const Vec2d &center = Vec2d());
    virtual bool mirror(const Line &axis);
    virtual bool flipHorizontal();
    virtual bool flipVertical();
    virtual bool reverse();
    virtual bool stretch(const Polyline &area, const Vec2d &offset);

    virtual bool moveTo(const Vec2d &dest);

    virtual NS::Ending getTrimEnd(const Vec2d &trimPoint,
                                  const Vec2d &clickPoint);
    virtual bool trimStartPoint(const Vec2d &trimPoint,
                                const Vec2d &clickPoint = Vec2d::invalid,
                                bool extend = false);
    virtual bool trimEndPoint(const Vec2d &trimPoint,
                              const Vec2d &clickPoint = Vec2d::invalid,
                              bool extend = false);
    virtual bool trimStartPoint(double trimDist);
    virtual bool trimEndPoint(double trimDist);
    virtual double getDistanceFromStart(const Vec2d &p) const;

    virtual std::vector<std::shared_ptr<Shape>>
    getOffsetShapes(double distance, int number, NS::Side side,
                    const Vec2d &position = Vec2d::invalid);

    virtual std::vector<std::shared_ptr<Shape>>
    splitAt(const std::vector<Vec2d> &points) const;

private:
    Vec2d startPoint;
    Vec2d endPoint;
};

class Circle : public Shape {
public:
    Circle();
    Circle(double cx, double cy, const double radius);
    Circle(const Vec2d &center, const double radius);
    virtual ~Circle();

    static Circle createFrom2Points(const Vec2d &p1, const Vec2d &p2);
    static Circle createFrom3Points(const Vec2d &p1, const Vec2d &p2,
                                    const Vec2d &p3);

    virtual NS::ShapeType getShapeType() const;
    virtual Circle *clone() const;

    static Circle createFrom2Points(const Vec2d &p1, const Vec2d &p2);
    static Circle createFrom3Points(const Vec2d &p1, const Vec2d &p2,
                                    const Vec2d &p3);

    Arc toArc(double startAngle = 0.0) const;

    virtual bool isValid() const;

    virtual BBox getBoundingBox() const;
    virtual double getLength() const;

    virtual std::vector<Vec2d> getEndPoints() const;
    virtual std::vector<Vec2d> getMiddlePoints() const;
    virtual std::vector<Vec2d> getCenterPoints() const;
    virtual std::vector<Vec2d> getArcRefPoints() const;
    virtual std::vector<Vec2d>
    getPointsWithDistanceToEnd(double distance, int from = NS::FromAny) const;
    virtual std::vector<Vec2d> getPointCloud(double segmentLength) const;

    virtual double getAngleAt(double distance,
                              NS::From from = NS::FromStart) const;
    Vec2d getPointAtAngle(double a) const;

    virtual Vec2d getVectorTo(const Vec2d &point, bool limited = true,
                              double strictRange = DBL_MAX) const;

    Vec2d getCenter() const;
    void setCenter(const Vec2d &vector);
    double getRadius() const;
    void setRadius(double radius);

    double getDiameter() const;
    void setDiameter(double d);
    double getCircumference() const;
    void setCircumference(double c);
    double getArea() const;
    void setArea(double a);

    bool contains(const Vec2d &p) const;

    virtual bool move(const Vec2d &offset);
    virtual bool rotate(double rotation, const Vec2d &center = Vec2d());
    virtual bool scale(const Vec2d &scaleFactors,
                       const Vec2d &center = Vec2d());
    virtual bool mirror(const Line &axis);
    virtual bool flipHorizontal();
    virtual bool flipVertical();

    std::vector<Line> getTangents(const Vec2d &point) const;

    virtual std::vector<std::shared_ptr<Shape>>
    getOffsetShapes(double distance, int number, NS::Side side,
                    const Vec2d &position = Vec2d::invalid);

    virtual std::vector<std::shared_ptr<Shape>>
    splitAt(const std::vector<Vec2d> &points) const;

private:
    Vec2d center;
    double radius;
};

class Arc : public Shape {
public:
    Arc();
    Arc(double cx, double cy, double radius, double startAngle, double endAngle,
        bool reversed = false);
    Arc(const Vec2d &center, double radius, double startAngle, double endAngle,
        bool reversed = false);

    virtual NS::ShapeType getShapeType() const;
    virtual Arc *clone() const;

    virtual bool isDirected() const;

    virtual bool isValid() const;
    bool isFullCircle(double tolerance = NS::AngleTolerance) const;

    static Arc createFrom3Points(const Vec2d &startPoint, const Vec2d &point,
                                 const Vec2d &endPoint);
    static Arc createFrom2PBulge(const Vec2d &startPoint, const Vec2d &endPoint,
                                 double bulge);
    static Arc createTangential(const Vec2d &startPoint, const Vec2d &pos,
                                double direction, double radius);
    static std::vector<Arc> createBiarc(const Vec2d &startPoint,
                                        double startDirection,
                                        const Vec2d &endPoint,
                                        double endDirection,
                                        bool secondTry = false);

    virtual BBox getBoundingBox() const;

    virtual std::vector<Vec2d> getEndPoints() const;
    virtual std::vector<Vec2d> getMiddlePoints() const;
    virtual std::vector<Vec2d> getCenterPoints() const;
    virtual std::vector<Vec2d> getArcRefPoints() const;
    virtual std::vector<Vec2d>
    getPointsWithDistanceToEnd(double distance, int from = NS::FromAny) const;
    virtual std::vector<Vec2d> getPointCloud(double segmentLength) const;

    virtual Vec2d getVectorTo(const Vec2d &point, bool limited = true,
                              double strictRange = DBL_MAX) const;

    Vec2d getCenter() const;
    void setCenter(const Vec2d &vector);
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

    virtual NS::Side getSideOfPoint(const Vec2d &point) const;

    double getSweep() const;
    void setSweep(double s);
    double getLength() const;

    virtual Vec2d getStartPoint() const;
    virtual Vec2d getEndPoint() const;
    Vec2d getPointAtAngle(double a) const;
    virtual double getAngleAt(double distance,
                              NS::From from = NS::FromStart) const;
    virtual Vec2d getMiddlePoint() const;

    void moveStartPoint(const Vec2d &pos, bool keepRadius = true);
    void moveEndPoint(const Vec2d &pos, bool keepRadius = true);
    void moveMiddlePoint(const Vec2d &pos);
    double getBulge() const;

    virtual bool move(const Vec2d &offset);
    virtual bool rotate(double rotation, const Vec2d &center = Vec2d());
    virtual bool scale(const Vec2d &scaleFactors,
                       const Vec2d &center = Vec2d());
    virtual bool mirror(const Line &axis);
    virtual bool reverse();
    virtual bool stretch(const Polyline &area, const Vec2d &offset);

    virtual NS::Ending getTrimEnd(const Vec2d &trimPoint,
                                  const Vec2d &clickPoint);
    virtual bool trimStartPoint(const Vec2d &trimPoint,
                                const Vec2d &clickPoint = Vec2d::invalid,
                                bool extend = false);
    virtual bool trimEndPoint(const Vec2d &trimPoint,
                              const Vec2d &clickPoint = Vec2d::invalid,
                              bool extend = false);
    virtual bool trimStartPoint(double trimDist);
    virtual bool trimEndPoint(double trimDist);
    virtual double getDistanceFromStart(const Vec2d &p) const;

    Polyline approximateWithLines(double segmentLength,
                                  double angle = 0.0) const;
    Polyline approximateWithLinesTan(double segmentLength,
                                     double angle = 0.0) const;

    std::vector<Line> getTangents(const Vec2d &point) const;

    virtual std::vector<std::shared_ptr<Shape>>
    getOffsetShapes(double distance, int number, NS::Side side,
                    const Vec2d &position = Vec2d::invalid);

    virtual std::vector<std::shared_ptr<Shape>>
    splitAt(const std::vector<Vec2d> &points) const;

    std::vector<Arc> splitAtQuadrantLines() const;

private:
    Vec2d center;
    double radius;
    double startAngle;
    double endAngle;
    bool reversed;
};

class Ellipse : public Shape {
public:
    Ellipse();
    Ellipse(const Vec2d &center, const Vec2d &majorPoint, double ratio,
            double startParam, double endParam, bool reversed);
    virtual ~Ellipse();

    static Ellipse createInscribed(const Vec2d &p1, const Vec2d &p2,
                                   const Vec2d &p3, const Vec2d &p4,
                                   const Vec2d &centerHint = Vec2d::invalid);
    static Ellipse createFrom4Points(const Vec2d &p1, const Vec2d &p2,
                                     const Vec2d &p3, const Vec2d &p4);

    virtual NS::ShapeType getShapeType() const;
    virtual Ellipse *clone() const;

    virtual bool isDirected() const;
    virtual bool isValid() const;

    virtual BBox getBoundingBox() const;

    virtual std::vector<Vec2d> getEndPoints() const;
    virtual std::vector<Vec2d> getMiddlePoints() const;
    virtual std::vector<Vec2d> getCenterPoints() const;
    virtual std::vector<Vec2d>
    getPointsWithDistanceToEnd(double distance, int from = NS::FromAny) const;
    virtual std::vector<Vec2d> getPointCloud(double segmentLength) const;

    virtual Vec2d getVectorTo(const Vec2d &point, bool limited = true,
                              double strictRange = DBL_MAX) const;

    void moveStartPoint(const Vec2d &pos, bool changeAngleOnly = false);
    void moveEndPoint(const Vec2d &pos, bool changeAngleOnly = false);

    std::vector<Vec2d> getFoci() const;

    Vec2d getCenter() const;
    void setCenter(const Vec2d &vector);
    Vec2d getMajorPoint() const;
    Vec2d getMinorPoint() const;
    void setMajorPoint(const Vec2d &vector);
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

    virtual NS::Side getSideOfPoint(const Vec2d &point) const;

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

    bool contains(const Vec2d &p) const;

    virtual double getAngleAt(double distance,
                              NS::From from = NS::FromStart) const;

    double getAngleAtPoint(const Vec2d &pos) const;
    double getParamTo(const Vec2d &pos) const;
    double getRadiusAt(double param) const;
    Vec2d getPointAt(double param) const;
    Vec2d getMiddlePoint() const;

    virtual Vec2d getPointOnShape() const;

    virtual bool move(const Vec2d &offset);
    virtual bool rotate(double rotation, const Vec2d &center = Vec2d());
    virtual bool scale(const Vec2d &scaleFactors,
                       const Vec2d &center = Vec2d());
    virtual bool mirror(const Line &axis);

    virtual bool reverse();

    virtual NS::Ending getTrimEnd(const Vec2d &trimPoint,
                                  const Vec2d &clickPoint);
    virtual bool trimStartPoint(const Vec2d &trimPoint,
                                const Vec2d &clickPoint = Vec2d::invalid,
                                bool extend = false);
    virtual bool trimEndPoint(const Vec2d &trimPoint,
                              const Vec2d &clickPoint = Vec2d::invalid,
                              bool extend = false);
    virtual bool trimStartPoint(double trimDist);
    virtual bool trimEndPoint(double trimDist);

    void correctMajorMinor();
    double getSweep() const;

    std::vector<Vec2d> getBoxCorners();

    std::vector<Line> getTangents(const Vec2d &point) const;
    Vec2d getTangentPoint(const Line &line) const;

    std::vector<BSpline> approximateWithSplines() const;
    Polyline approximateWithArcs(int segments) const;

    virtual std::vector<std::shared_ptr<Shape>>
    getOffsetShapes(double distance, int number, NS::Side side,
                    const Vec2d &position = Vec2d::invalid);
    virtual std::vector<std::shared_ptr<Shape>>
    splitAt(const std::vector<Vec2d> &points) const;

private:
    Vec2d center;
    Vec2d majorPoint;
    double ratio;
    double startParam;
    double endParam;
    bool reversed;
};

class Polyline : public Shape {
public:
    Polyline();
    Polyline(const std::vector<Vec2d> &vertices, bool closed);
    Polyline(const std::vector<std::shared_ptr<Shape>> &segments);
    virtual ~Polyline();

    virtual NS::ShapeType getShapeType() const;

    virtual Polyline *clone() const;

    virtual bool isDirected() const;
    bool isFlat() const;

    void clear();
    void normalize(double tolerance = DBL_EPSILON);

    bool prependShape(const Shape &shape);
    bool appendShape(const Shape &shape, bool prepend = false);
    bool appendShapeAuto(const Shape &shape);
    bool appendShapeTrim(const Shape &shape);
    bool closeTrim();

    void appendVertex(const Vec2d &vertex, double bulge = 0.0, double w1 = 0.0,
                      double w2 = 0.0);
    void appendVertex(double x, double y, double bulge = 0.0, double w1 = 0.0,
                      double w2 = 0.0);
    void prependVertex(const Vec2d &vertex, double bulge = 0.0, double w1 = 0.0,
                       double w2 = 0.0);
    void insertVertex(int index, const Vec2d &vertex, double bulgeBefore = 0.0,
                      double bulgeAfter = 0.0);
    void insertVertexAt(const Vec2d &point);
    Vec2d insertVertexAtDistance(double dist);
    void removeFirstVertex();
    void removeLastVertex();
    void removeVertex(int index);
    void removeVerticesAfter(int index);
    void removeVerticesBefore(int index);

    bool isEmpty() const;

    void setVertices(const std::vector<Vec2d> &vertices);
    std::vector<Vec2d> getVertices() const;
    void setVertexAt(int i, const Vec2d &v);
    void moveVertexAt(int i, const Vec2d &offset);
    Vec2d getVertexAt(int i) const;
    int getVertexIndex(const Vec2d &v,
                       double tolerance = DBL_EPSILON) const;
    Vec2d getLastVertex() const;
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
    bool isGeometricallyClosed(double tolerance = DBL_EPSILON) const;
    bool autoClose(double tolerance = DBL_EPSILON);
    bool toLogicallyClosed(double tolerance = DBL_EPSILON);
    bool toLogicallyOpen();

    std::vector<Vec2d>
    getSelfIntersectionPoints(double tolerance = DBL_EPSILON) const;

    NS::Orientation getOrientation(bool implicitelyClosed = false) const;
    bool setOrientation(NS::Orientation orientation);

    Polyline convertArcToLineSegments(int segments) const;
    Polyline convertArcToLineSegmentsLength(double segmentLength) const;

    virtual bool contains(const Vec2d &point, bool borderIsInside = false,
                          double tolerance = DBL_EPSILON) const;
    bool containsShape(const Shape &shape) const;

    Vec2d getPointInside() const;

    virtual Vec2d getStartPoint() const;
    virtual Vec2d getEndPoint() const;
    virtual Vec2d getMiddlePoint() const;

    void moveStartPoint(const Vec2d &pos);
    void moveEndPoint(const Vec2d &pos);

    void moveSegmentAt(int i, const Vec2d &offset);

    virtual double getDirection1() const;
    virtual double getDirection2() const;

    virtual NS::Side getSideOfPoint(const Vec2d &point) const;

    virtual BBox getBoundingBox() const;

    double getArea() const;

    virtual double getLength() const;

    virtual double getDistanceFromStart(const Vec2d &p) const;
    virtual std::vector<double> getDistancesFromStart(const Vec2d &p) const;
    double getLengthTo(const Vec2d &p, bool limited = true) const;
    double getSegmentsLength(int fromIndex, int toIndex) const;

    virtual std::vector<Vec2d> getEndPoints() const;
    virtual std::vector<Vec2d> getMiddlePoints() const;
    virtual std::vector<Vec2d> getCenterPoints() const;
    virtual Vec2d getPointAtPercent(double p) const;
    virtual std::vector<Vec2d>
    getPointsWithDistanceToEnd(double distance, int from = NS::FromAny) const;
    virtual std::vector<Vec2d> getPointCloud(double segmentLength) const;

    virtual double getAngleAt(double distance,
                              NS::From from = NS::FromStart) const;

    virtual Vec2d getVectorTo(const Vec2d &point, bool limited = true,
                              double strictRange = DBL_MAX) const;
    virtual double getDistanceTo(const Vec2d &point, bool limited = true,
                                 double strictRange = DBL_MAX) const;

    int getClosestSegment(const Vec2d &point) const;
    int getClosestVertex(const Vec2d &point) const;
    virtual bool move(const Vec2d &offset);
    virtual bool rotate(double rotation, const Vec2d &center = Vec2d());
    virtual bool scale(double scaleFactor, const Vec2d &center = Vec2d());
    virtual bool scale(const Vec2d &scaleFactors,
                       const Vec2d &center = Vec2d());
    virtual bool mirror(const Line &axis);
    virtual bool reverse();
    virtual Polyline getReversed() const;
    virtual bool stretch(const Polyline &area, const Vec2d &offset);

    virtual NS::Ending getTrimEnd(const Vec2d &trimPoint,
                                  const Vec2d &clickPoint);
    virtual bool trimStartPoint(const Vec2d &trimPoint,
                                const Vec2d &clickPoint = Vec2d::invalid,
                                bool extend = false);
    virtual bool trimEndPoint(const Vec2d &trimPoint,
                              const Vec2d &clickPoint = Vec2d::invalid,
                              bool extend = false);
    virtual bool trimStartPoint(double trimDist);
    virtual bool trimEndPoint(double trimDist);

    virtual std::vector<std::shared_ptr<Shape>>
    getExploded(int segments = -1) const;
    std::vector<Polyline> getOutline() const;
    std::vector<std::pair<Polyline, Polyline>> getLeftRightOutline() const;
    std::vector<Polyline> getLeftOutline() const;
    std::vector<Polyline> getRightOutline() const;
    virtual bool isInterpolated() const;
    int countSegments() const;
    std::shared_ptr<Shape> getSegmentAt(int i) const;
    bool isArcSegmentAt(int i) const;
    std::shared_ptr<Shape> getLastSegment() const;
    std::shared_ptr<Shape> getFirstSegment() const;

    bool isStraight(double bulge) const;

    bool simplify(double tolerance = DBL_EPSILON);
    std::vector<Vec2d> verifyTangency(double toleranceMin = NS::AngleTolerance,
                                      double toleranceMax = M_PI_4);

    void stripWidths();
    void setMinimumWidth(double w);

    int getSegmentAtDist(double dist);
    bool relocateStartPoint(const Vec2d &p);
    bool relocateStartPoint(double dist);
    bool convertToClosed();
    bool convertToOpen();

    Polyline modifyPolylineCorner(const Shape &trimmedShape1,
                                  NS::Ending ending1, int segmentIndex1,
                                  const Shape &trimmedShape2,
                                  NS::Ending ending2, int segmentIndex2,
                                  const Shape *cornerShape = NULL) const;

    bool isConcave() const;
    std::vector<Vec2d> getConvexVertices(bool convex = true) const;
    std::vector<Vec2d> getConcaveVertices() const;

    Vec2d getCentroid() const;

    std::vector<Polyline> splitAtDiscontinuities(double tolerance) const;
    std::vector<Polyline> splitAtSegmentTypeChange() const;

    double getBaseAngle() const;
    double getWidth() const;
    bool setWidth(double v);
    double getHeight() const;
    bool setHeight(double v);

    std::vector<Polyline>
    morph(const Polyline &target, int steps, NS::Easing easing = NS::Linear,
          bool zLinear = true,
          double customFactor = std::numeric_limits<double>::quiet_NaN()) const;
    Polyline roundAllCorners(double radius) const;
    Polyline getPolygon(double segmentLength) const;
    Polyline getPolygonHull(double angle, double tolerance,
                            bool inner = false) const;

protected:
    bool isLineSegment(int i) const;

protected:
    std::vector<Vec2d> vertices;
    std::vector<double> bulges;
    std::vector<double> endWidths;
    std::vector<double> startWidths;
    bool closed;
};

class XLine : public Shape {
public:
    XLine();
    XLine(const Line &line);
    XLine(const Vec2d &basePoint, const Vec2d &directionVector);
    XLine(const Vec2d &basePoint, double angle, double distance);
    virtual ~XLine();

    virtual NS::ShapeType getShapeType() const;

    Line getLineShape() const;
    virtual XLine *clone() const;
    virtual bool isDirected() const;

    BBox getBoundingBox() const;

    virtual std::vector<Vec2d> getEndPoints() const;
    virtual std::vector<Vec2d> getMiddlePoints() const;
    virtual std::vector<Vec2d> getCenterPoints() const;
    virtual std::vector<Vec2d>
    getPointsWithDistanceToEnd(double distance, int from = NS::FromAny) const;
    virtual std::vector<Vec2d> getPointCloud(double segmentLength) const;

    virtual double getAngleAt(double distance,
                              NS::From from = NS::FromStart) const;

    virtual Vec2d getVectorTo(const Vec2d &point, bool limited = true,
                              double strictRange = DBL_MAX) const;

    Vec2d getBasePoint() const;
    void setBasePoint(const Vec2d &vector);
    Vec2d getSecondPoint() const;
    void setSecondPoint(const Vec2d &vector);
    Vec2d getDirectionVector() const;
    void setDirectionVector(const Vec2d &vector);

    virtual Vec2d getMiddlePoint() const;

    double getLength() const;
    void setLength(double l);
    double getAngle() const;
    void setAngle(double a);

    virtual double getDirection1() const;
    virtual double getDirection2() const;

    virtual NS::Side getSideOfPoint(const Vec2d &point) const;

    virtual Vec2d getStartPoint() const;
    virtual Vec2d getEndPoint() const;

    virtual bool trimStartPoint(const Vec2d &trimPoint,
                                const Vec2d &clickPoint = Vec2d::invalid,
                                bool extend = false);
    virtual bool trimEndPoint(const Vec2d &trimPoint,
                              const Vec2d &clickPoint = Vec2d::invalid,
                              bool extend = false);
    virtual bool trimStartPoint(double trimDist);
    virtual bool trimEndPoint(double trimDist);
    virtual NS::Ending getTrimEnd(const Vec2d &trimPoint,
                                  const Vec2d &clickPoint);
    virtual double getDistanceFromStart(const Vec2d &p) const;

    virtual Line getClippedLine(const BBox &box) const;

    virtual bool move(const Vec2d &offset);
    virtual bool rotate(double rotation, const Vec2d &center = Vec2d());
    virtual bool scale(const Vec2d &scaleFactors,
                       const Vec2d &center = Vec2d());
    virtual bool mirror(const Line &axis);
    virtual bool reverse();
    virtual bool stretch(const Polyline &area, const Vec2d &offset);

    virtual std::vector<std::shared_ptr<Shape>>
    getOffsetShapes(double distance, int number, NS::Side side,
                    const Vec2d &position = Vec2d::invalid);

    virtual std::vector<std::shared_ptr<Shape>>
    splitAt(const std::vector<Vec2d> &points) const;

private:
    Vec2d basePoint;
    Vec2d directionVector;
};

class Ray : public XLine {
public:
    Ray();
    Ray(const Line &line);
    Ray(const Vec2d &basePoint, const Vec2d &directionVector);
    Ray(const Vec2d &basePoint, double angle, double distance);
    virtual ~Ray();

    virtual NS::ShapeType getShapeType() const;

    virtual Ray *clone() const;

    virtual bool trimEndPoint(const Vec2d &trimPoint,
                              const Vec2d &clickPoint = Vec2d::invalid,
                              bool extend = false);
    virtual std::vector<Vec2d> getPointsWithDistanceToEnd(double distance,
                                                          int from) const;
    virtual bool reverse();
    virtual Line getClippedLine(const BBox &box) const;
    virtual Vec2d getVectorTo(const Vec2d &point, bool limited = true,
                              double strictRange = DBL_MAX) const;

    virtual bool stretch(const Polyline &area, const Vec2d &offset);

    virtual std::vector<std::shared_ptr<Shape>>
    splitAt(const std::vector<Vec2d> &points) const;
};

class BSpline : public Shape {
public:
    BSpline();
    BSpline(const BSpline &other);
    BSpline(const std::vector<Vec2d> &controlPoints, int degree);

    BSpline &operator=(const BSpline &other);

    virtual NS::ShapeType getShapeType() const;

    virtual BSpline *clone() const;

    virtual bool isDirected() const;

    void copySpline(const BSpline &other);

    static std::vector<BSpline> createSplinesFromArc(const Arc &arc);
    static BSpline createBezierFromSmallArc(double r, double a1, double a2);

    virtual bool isInterpolated() const;

    void appendControlPoint(const Vec2d &point);
    void appendControlPoints(const std::vector<Vec2d> &points);
    void removeLastControlPoint();
    void setControlPoints(const std::vector<Vec2d> &points);
    std::vector<Vec2d> getControlPoints() const;
    std::vector<Vec2d> getControlPointsWrapped() const;
    int countControlPoints() const;
    Vec2d getControlPointAt(int i) const;

    void appendFitPoint(const Vec2d &point);
    void prependFitPoint(const Vec2d &point);
    void insertFitPointAt(const Vec2d &point);
    void insertFitPointAt(double t, const Vec2d &point);
    void removeFitPointAt(const Vec2d &point);
    void removeFirstFitPoint();
    void removeLastFitPoint();
    void setFitPoints(const std::vector<Vec2d> &points);
    std::vector<Vec2d> getFitPoints() const;
    int countFitPoints() const;
    bool hasFitPoints() const;
    Vec2d getFitPointAt(int i) const;

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
    bool isGeometricallyClosed(double tolerance = DBL_EPSILON) const;
    bool isPeriodic() const;

    virtual double getDirection1() const;
    virtual double getDirection2() const;

    virtual NS::Side getSideOfPoint(const Vec2d &point) const;

    virtual Vec2d getStartPoint() const;
    virtual Vec2d getEndPoint() const;

    void setStartPoint(const Vec2d &v);
    void setEndPoint(const Vec2d &v);

    void setTangents(const Vec2d &start, const Vec2d &end);
    void unsetTangents();

    void setTangentAtStart(const Vec2d &t);
    Vec2d getTangentAtStart() const;
    void unsetTangentAtStart();
    void setTangentAtEnd(const Vec2d &t);
    Vec2d getTangentAtEnd() const;
    void unsetTangentAtEnd();

    void updateTangentsPeriodic();

    virtual BBox getBoundingBox() const;

    virtual double getLength() const;
    Vec2d getPointAt(double t) const;
    Vec2d getPointAtDistance(double distance) const;
    virtual double getAngleAt(double distance,
                              NS::From from = NS::FromStart) const;

    virtual std::vector<Vec2d> getEndPoints() const;
    virtual Vec2d getMiddlePoint() const;
    virtual std::vector<Vec2d> getMiddlePoints() const;
    virtual std::vector<Vec2d> getCenterPoints() const;
    virtual std::vector<Vec2d>
    getPointsWithDistanceToEnd(double distance, int from = NS::FromAny) const;
    virtual std::vector<Vec2d> getPointCloud(double segmentLength) const;

    virtual Vec2d getVectorTo(const Vec2d &point, bool limited = true,
                              double strictRange = DBL_MAX) const;
    virtual bool isOnShape(const Vec2d &point, bool limited = true,
                           double tolerance = DBL_EPSILON) const;

    virtual bool move(const Vec2d &offset);
    virtual bool rotate(double rotation, const Vec2d &center = Vec2d());
    virtual bool scale(const Vec2d &scaleFactors,
                       const Vec2d &center = Vec2d());
    virtual bool mirror(const Line &axis);
    virtual bool flipHorizontal();
    virtual bool flipVertical();
    virtual bool reverse();
    virtual bool stretch(const Polyline &area, const Vec2d &offset);

    virtual NS::Ending getTrimEnd(const Vec2d &trimPoint,
                                  const Vec2d &clickPoint);
    virtual bool trimStartPoint(const Vec2d &trimPoint,
                                const Vec2d &clickPoint = Vec2d::invalid,
                                bool extend = false);
    virtual bool trimEndPoint(const Vec2d &trimPoint,
                              const Vec2d &clickPoint = Vec2d::invalid,
                              bool extend = false);
    virtual bool trimStartPoint(double trimDist);
    virtual bool trimEndPoint(double trimDist);
    virtual double getDistanceFromStart(const Vec2d &p) const;

    std::vector<BSpline> splitAtPoints(const std::vector<Vec2d> &points) const;
    std::vector<BSpline> splitAtParams(const std::vector<double> &params) const;

    Polyline toPolyline(int segments) const;
    Polyline approximateWithArcs(double tolerance,
                                 double radiusLimit = -1) const;

    virtual std::vector<std::shared_ptr<Shape>>
    getExploded(int segments = -1) const;
    std::vector<std::shared_ptr<Shape>> getExplodedBezier(int segments) const;
    std::vector<std::shared_ptr<Shape>>
    getExplodedWithSegmentLength(double segmentLength) const;

    std::vector<BSpline> getBezierSegments(const BBox &queryBox = BBox()) const;

    virtual bool isValid() const;
    double getTDelta() const;
    double getTMin() const;
    double getTMax() const;
    double getTAtPoint(const Vec2d &point) const;
    double getTAtDistance(double distance) const;
    double getDistanceAtT(double t) const;
    std::vector<BSpline> getSegments(const std::vector<Vec2d> &points) const;

    std::vector<Vec2d> getDiscontinuities() const;
    BSpline simplify(double tolerance);

    void updateFromControlPoints() const;
    void updateFromFitPoints() const;
    void update() const;

    virtual std::vector<std::shared_ptr<Shape>>
    splitAt(const std::vector<Vec2d> &points) const;

    bool isDirty() const;

    std::vector<Vec2d>
    getSelfIntersectionPoints(double tolerance = DBL_EPSILON) const;

protected:
    void appendToExploded(const Line &line) const;
    void invalidate() const;
    void updateInternal() const;
    void updateBoundingBox() const;

private:
    mutable std::vector<Vec2d> controlPoints;
    mutable std::vector<double> knotVector;
    mutable std::vector<double> weights;
    std::vector<Vec2d> fitPoints;
    mutable int degree;

    mutable Vec2d tangentStart;
    mutable Vec2d tangentEnd;
    mutable bool periodic;

    mutable bool dirty;
    mutable bool updateInProgress;

private:
    mutable BBox boundingBox;
    mutable std::vector<std::shared_ptr<Shape>> exploded;
    // cached length:
    mutable double length;
};

class Triangle : public Shape {
public:
    Triangle();
    Triangle(const Vec2d &p1, const Vec2d &p2, const Vec2d &p3);
    virtual ~Triangle();

    virtual NS::ShapeType getShapeType() const;

    virtual Triangle *clone() const;

    Polyline getPolyline() const;
    NS::Orientation getOrientation() const;
    virtual bool reverse();

    static Triangle createArrow(const Vec2d &position, double direction,
                                double size);

    virtual BBox getBoundingBox() const;
    virtual double getLength() const;
    double getArea() const;
    Vec2d getCorner(int i) const;
    void setCorner(int i, const Vec2d &p);
    void setCorners(const Vec2d &c1, const Vec2d &c2, const Vec2d &c3);

    virtual std::vector<Vec2d> getEndPoints() const;
    virtual std::vector<Vec2d> getMiddlePoints() const;
    virtual std::vector<Vec2d> getCenterPoints() const;
    virtual std::vector<Vec2d>
    getPointsWithDistanceToEnd(double distance, int from = NS::FromAny) const;
    virtual std::vector<Vec2d> getPointCloud(double segmentLength) const;

    virtual double getDistanceTo(const Vec2d &point, bool limited = true,
                                 double strictRange = DBL_MAX) const;
    virtual Vec2d getVectorTo(const Vec2d &point, bool limited = true,
                              double strictRange = DBL_MAX) const;
    virtual Vec2d getNormal() const;

    bool isPointInTriangle(const Vec2d &p, bool treatAsQuadrant = false) const;
    bool isPointInQuadrant(const Vec2d &p) const;

    double getD() const;

    virtual std::vector<std::shared_ptr<Shape>>
    getExploded(int segments = -1) const;

    virtual bool move(const Vec2d &offset);
    virtual bool rotate(double rotation, const Vec2d &center = Vec2d());
    virtual bool scale(const Vec2d &scaleFactors,
                       const Vec2d &center = Vec2d());
    virtual bool mirror(const Line &axis);
    virtual bool flipHorizontal();
    virtual bool flipVertical();

public:
    Vec2d corner[3];
};

} // namespace cada

#endif