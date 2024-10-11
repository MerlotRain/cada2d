#include "cadsa_shape.h"

using namespace cadsa;

Circle::Circle() {}

Circle::Circle(double cx, double cy, double radius) {}

Vec2d Circle::center() const
{
    return mCenter;
}

void Circle::setCenter(const Vec2d &c)
{
    mCenter = c;
}

double Circle::radius() const
{
    return mRadius;
}

void Circle::setRadius(double r)
{
    mRadius = r;
}

Shape* Circle::clone() 
{
    Circle* pClone = new Circle();
    pClone->mRadius = mRadius;
    pClone->mRadius = mRadius;
    return pClone;
}

std::vector<Vec2d> Circle::getEndPoints() const { return std::vector<Vec2d>(); }

std::vector<Vec2d> Circle::getMiddlePoints() const { return std::vector<Vec2d>(); }

std::vector<Vec2d> Circle::getCenterPoints() const 
{
    std::vector<Vec2d> ret;
    ret.push_back(mCenter);
    return ret;
}

std::vector<Vec2d> Circle::getArcRefPoints() const
{
    std::vector<Vec2d> ret;
    ret.push_back(mCenter + Vec2d(mRadius, 0));
    ret.push_back(mCenter + Vec2d(0, mRadius));
    ret.push_back(mCenter - Vec2d(mRadius, 0));
    ret.push_back(mCenter - Vec2d(0, mRadius));
    return ret;
}