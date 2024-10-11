#include "vec2d.h"

#include "cada.h"
#include <assert.h>

using namespace cadsa;

Vec2d::Vec2d() : mX(0.0), mY(0.0), mValid(true)
{
}

Vec2d::Vec2d(double x, double y, bool valid) : mX(x), mY(y)
{
    mValid = valid && isNormal(x) && isNormal(y);
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

double Vec2d::getAngleTo(const Vec2d & v) const
{
    if(!mValid || !v.mValid) return std::numeric_limits<double>::quiet_NaN();
    else
    {
        return (v - *this).getAngle();
    }
}

double Vec2d::getAngle() const
{
    double ret = 0.0;
    double m = getMagnitude();

    if (m > 1.0e-6)
    {
        double dp = getDotProduct(*this, Vec2d(1.0, 0.0));
        if (dp / m >= 1.0) { ret = 0.0; }
        else if (dp / m < -1.0) { ret = M_PI; }
        else { ret = acos(dp / m); }
        if (mY < 0.0) { ret = 2 * M_PI - ret; }
    }
    return ret;
}

double Vec2d::getMagnitude() const
{
    if (!mValid) { return std::numeric_limits<double>::quiet_NaN(); }
    return sqrt(mX * mX + mY * mY);
}

Vec2d Vec2d::operator+(const Vec2d &rhs) const
{
    return Vec2d(mX + rhs.mX, mY + rhs.mY, mValid && rhs.mValid);
}

Vec2d Vec2d::operator-(const Vec2d &rhs) const
{
    return Vec2d(mX - rhs.mX, mY - rhs.mY, mValid && rhs.mValid);
}

Vec2d Vec2d::operator*(double s) const
{
    return Vec2d(mX * s, mY * s, mValid);
}

Vec2d Vec2d::operator/(double s) const
{
    if(s == 0)
        throw std::invalid_argument("s can not be zero");
    return Vec2d(mX / s, mY / s, mValid);
}

Vec2d Vec2d::operator-() const
{
    return Vec2d(-mX, -mY, mValid);
}

double Vec2d::getDotProduct(const Vec2d &v1, const Vec2d &v2)
{
    return v1.mX * v2.mX + v1.mY * v2.mY;
}