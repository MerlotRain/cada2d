#include "cadsa_shape.h"

using namespace cadsa;

Shape* Line::clone() 
{
    Line* pClone = new Line();
    return pClone;
}

std::vector<Vec2d> Line::getEndPoints() const 
{
    std::vector<Vec2d> ret;
    ret.push_back(mBegin);
    ret.push_back(mEnd);
    return ret;
}

std::vector<Vec2d> Line::getMiddlePoints() const 
{
    std::vector<Vec2d> ret;
    ret.push_back(getMiddlePoint());
    return ret;
}

std::vector<Vec2d> Line::getCenterPoints() const { return getMiddlePoints(); }

Vec2d Line::getMiddlePoint() const
{
    return (mBegin + mEnd) / 2.0;
}