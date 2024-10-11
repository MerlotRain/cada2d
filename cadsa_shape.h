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

class BBox;
class Vec2d {
    double mX;
    double mY;
    bool mValid;

public:
    Vec2d();
    Vec2d(double x, double y, bool valid = true);
    ~Vec2d() = default;

    double x() const;
    double y() const;
    bool isValid() const;
    void setX(double x);
    void setY(double y);

    void set(double x, double y);
    void setPolar(double radius, double angle);
    double getAngle() const;
    double getAngleTo(const Vec2d &v) const;
    double getMagnitude() const;
    double getDistanceTo(const Vec2d &v) const;

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
    double width() const;
    double height() const;
    double area() const;
    Vec2d size() const;
    Vec2d center() const;
    Vec2d minimum() const;
    Vec2d maximum() const;
};

enum Side {
    NO_SIDE,
    LEFT_HAND,
    RIGHT_HAND,
    BOTH_SIDES,
};

enum Ending {
    ENDING_START,
    ENDING_END,
    ENDING_NONE,
};

enum From {
    FROM_START = 0x01,
    FROM_END = 0x02,
    FROM_ANY = FROM_START | FROM_END,
    ALONG_POLYLINE = 0x04,
};

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

class Shape {
protected:
    ShapeType mShapeType;
    mutable BBox mBBox;

public:
    Shape();
    virtual ~Shape();

    virtual ShapeType shapeType() const = 0;
    virtual Shape *clone() = 0;

    virtual bool equals(Shape *other, double tol = DBL_EPSILON);
    virtual std::vector<Vec2d> getEndPoints() const = 0;
    virtual std::vector<Vec2d> getMiddlePoints() const = 0;
    virtual std::vector<Vec2d> getCenterPoints() const = 0;
};

class Point : public Shape {
    Vec2d mp;

public:
    explicit Point(const Vec2d &v);
    Point(double x, double y);
    ~Point() = default;

    ShapeType shapeType() const override;
    Shape *clone() override;
    std::vector<Vec2d> getEndPoints() const override;
    std::vector<Vec2d> getMiddlePoints() const override;
    std::vector<Vec2d> getCenterPoints() const override;
};

class Line : public Shape {
    Vec2d mBegin;
    Vec2d mEnd;

public:
    Line();
    Line(const Vec2d &begin, const Vec2d &end);

    ShapeType shapeType() const override;
    Shape *clone() override;
    std::vector<Vec2d> getEndPoints() const override;
    std::vector<Vec2d> getMiddlePoints() const override;
    std::vector<Vec2d> getCenterPoints() const override;

    Vec2d getMiddlePoint() const;
};

class Circle : public Shape {
    Vec2d mCenter;
    double mRadius;

public:
    Circle();
    Circle(double cx, double cy, double radius);

    Vec2d center() const;
    void setCenter(const Vec2d &c);
    double radius() const;
    void setRadius(double r);

    ShapeType shapeType() const override;
    Shape *clone() override;
    std::vector<Vec2d> getEndPoints() const override;
    std::vector<Vec2d> getMiddlePoints() const override;
    std::vector<Vec2d> getCenterPoints() const override;

    std::vector<Vec2d> getArcRefPoints() const;
};

class Arc : public Shape {
    Vec2d mCenter;
    double mRadius;
    double mStartAngle;
    double mEndAngle;
    bool mReversed;

public:
    Arc();
    Arc(const Vec2d &center, double radius, double startAngle, double endAngle,
        bool reversed = false);

    double getSweep() const;

    ShapeType shapeType() const override;
    Shape *clone() override;
    std::vector<Vec2d> getEndPoints() const override;
    std::vector<Vec2d> getMiddlePoints() const override;
    std::vector<Vec2d> getCenterPoints() const override;

    Vec2d getStartPoint() const;
    Vec2d getEndPoint() const;
    Vec2d getMiddlePoint() const;
    Vec2d getPointAtAngle(double a) const;
    double getAngleAt(double dis, From from = From::FROM_START) const;
    std::vector<Vec2d> getArcRefPoints() const;
    std::vector<Vec2d> getPointsWithDistanceToEnd(double distance,
                                                  int from = FROM_ANY) const;
};

class Ellipse : public Shape {
    Vec2d mCenter;
    Vec2d mMajorPoint;
    double mRatio;
    double mStartParam;
    double mEndParam;
    bool mReversed;
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

    ShapeType shapeType() const override;
    Shape *clone() override;
    std::vector<Vec2d> getEndPoints() const override;
    std::vector<Vec2d> getMiddlePoints() const override;
    std::vector<Vec2d> getCenterPoints() const override;

    Shape *getSegmentAt(int i) const;
    std::vector<Shape *> getExploded() const;
};

class XLine : public Shape {
    Vec2d mBasePoint;
    Vec2d mDirectionVec;
};

class Ray : public XLine {};

class BSpline : public Shape {
    std::vector<Vec2d> mControlPoints;
    std::vector<double> mKnots;
    std::vector<double> mWeights;
    int mDegree;

    std::vector<Vec2d> mFitPoints;
};

class Triangle : public Shape {
    Vec2d mCorner[3];
};

} // namespace cadsa

#endif