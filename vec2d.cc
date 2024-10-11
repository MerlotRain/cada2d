#include "vec2d.h"

#include "cada.h"

using namespace cadsa;

Vec2d::Vec2d() : mX(0.0), mY(0.0), mValid(true)
{
}

Vec2d::Vec2d(double x, double y) : mX(x), mY(y)
{
    mValid = mValid && isNormal(x) && isNormal(y);
}

void Vec2d::set(double x, double y)
{
    mX = x;
    mY = y;
    mValid = true;
}
void Vec2d::setPolar(double radius, double angle)
{
    mX = radius * cos(angle);
    mY = radius * sin(angle);
    mValid = isNormal(mX) && isNormal(mY);
}