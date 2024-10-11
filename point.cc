#include "shape.h"

using namespace cadsa;

Point::Point(const Vec2d &v)
    : mp(v)
{}

Point::Point(double x, double y)
    : mp(x, y)
{}

std::vector<Vec2d> Point::getEndPoints() const
{
    std::vector<Vec2d> ret;
    ret.push_back(mp);
    return ret;
}

std::vector<Vec2d> Point::getMiddlePoints() const
{
    std::vector<Vec2d> ret;
    ret.push_back(mp);
    return ret;
}

std::vector<Vec2d> Point::getCenterPoints() const
{
    std::vector<Vec2d> ret;
    ret.push_back(mp);
    return ret;
}