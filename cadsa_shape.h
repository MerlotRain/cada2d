#ifndef CADSA_SHAPE_H
#define CADSA_SHAPE_H

#include "cadsa.h"
#include <vector>

namespace cadsa {

class BBox;
class Vec2d
{
    double mX;
    double mY;
    bool mValid;

public:
    Vec2d();
    Vec2d(double x, double y, bool valid = true);
    ~Vec2d() = default;

    void set(double x, double y);
    void setPolar(double radius, double angle);
    double getAngle() const;
    double getAngleTo(const Vec2d & v) const;
    double getMagnitude() const;

public:
    Vec2d operator+(const Vec2d &rhs) const;
    Vec2d operator-(const Vec2d &rhs) const;
    Vec2d operator*(double s) const;
    Vec2d operator/(double s) const;
    Vec2d operator-() const;

public:
    static double getDotProduct(const Vec2d &v1, const Vec2d &v2);
};

class BBox
{
    Vec2d c1;
    Vec2d c2;

public:
    BBox();
    BBox(double x1, double y1, double x2, double y2);
    BBox(const Vec2d& c1, const Vec2d& c2);
    BBox(const Vec2d& center, double range);
    BBox(const Vec2d& center, double width, double height);

    bool isValid() const;
    bool isSane() const;

    bool equalsFuzzy(const BBox& b, double tol = DBL_EPSILON);
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

enum Ending
{
    ENDING_START,
    ENDING_END,
    ENDING_NONE,
};

enum From
{
    FROM_START,
    FROM_END,
};


enum ShapeType
{
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

class Shape
{
protected:
    ShapeType mShapeType;
    mutable BBox mBBox;

public:
    Shape();
    virtual ~Shape();

    virtual ShapeType shapeType() const = 0;
    virtual Shape* clone() = 0;

    virtual bool equals(Shape* other, double tol = DBL_EPSILON);
    virtual std::vector<Vec2d> getEndPoints() const = 0;
    virtual std::vector<Vec2d> getMiddlePoints() const = 0;
    virtual std::vector<Vec2d> getCenterPoints() const = 0;
};


class Point : public Shape
{
    Vec2d mp;

public:
    explicit Point(const Vec2d &v);
    Point(double x, double y);
    ~Point() = default;

    Shape* clone() override;
    std::vector<Vec2d> getEndPoints() const override;
    std::vector<Vec2d> getMiddlePoints() const override;
    std::vector<Vec2d> getCenterPoints() const override;
};

class Line : public Shape
{
    Vec2d mBegin;
    Vec2d mEnd;

public:
    Line();

    Shape* clone() override;
    std::vector<Vec2d> getEndPoints() const override;
    std::vector<Vec2d> getMiddlePoints() const override;
    std::vector<Vec2d> getCenterPoints() const override; 

    Vec2d getMiddlePoint() const;

};

class Circle : public Shape
{
    Vec2d mCenter;
    double mRadius;

public:
    Circle();
    Circle(double cx, double cy, double radius);

    Vec2d center() const;
    void setCenter(const Vec2d &c);
    double radius() const;
    void setRadius(double r);


    Shape* clone() override;
    std::vector<Vec2d> getEndPoints() const override;
    std::vector<Vec2d> getMiddlePoints() const override;
    std::vector<Vec2d> getCenterPoints() const override; 

    std::vector<Vec2d> getArcRefPoints() const;


};

class Arc : public Shape
{
    Vec2d mCenter;
    double mRadius;
    double mStartAngle;
    double mEndAngle;
    bool mReversed;

public:
    Arc();

    double getSweep() const;

    Shape* clone() override;
    std::vector<Vec2d> getEndPoints() const override;
    std::vector<Vec2d> getMiddlePoints() const override;
    std::vector<Vec2d> getCenterPoints() const override;

    Vec2d getStartPoint() const;
    Vec2d getEndPoint() const;
    Vec2d getMiddlePoint() const;
    Vec2d getPointAtAngle(double a) const;
    double getAngleAt(double dis, From from = From::FROM_START) const;
    std::vector<Vec2d> getArcRefPoints() const;
};

class Ellipse : public Shape
{
    Vec2d mCenter;
    Vec2d mMajorPoint;
    double mRatio;
    double mStartParam;
    double mEndParam;
    bool mReversed;
};

class Polyline : public Shape
{
    std::vector<Vec2d> mVertices;
    std::vector<double> mBulges;
    std::vector<double> mEndWidths;
    std::vector<double> mStartWidths;
    bool mClosed;

public:
    Polyline();


    Shape* clone() override;
    std::vector<Vec2d> getEndPoints() const override;
    std::vector<Vec2d> getMiddlePoints() const override;
    std::vector<Vec2d> getCenterPoints() const override;

    Shape* getSegmentAt(int i) const;
    std::vector<Shape*> getExploded() const;


};

class XLine : public Shape
{
    Vec2d mBasePoint;
    Vec2d mDirectionVec;
};

class Ray : public XLine
{

};

class BSpline : public Shape
{
    std::vector<Vec2d> mControlPoints;
    std::vector<double> mKnots;
    std::vector<double> mWeights;
    int mDegree;

    std::vector<Vec2d> mFitPoints;
    
};

class Triangle : public Shape
{
    Vec2d mCorner[3];
};


} // namespace cadsa

#endif