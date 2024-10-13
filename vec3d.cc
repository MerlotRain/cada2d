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

#include "cada_shape.h"

#include <assert.h>
#include <math.h>
#include <stdexcept>

using namespace cada;

const Vec2d Vec2d::invalid = Vec2d(0, 0, false);
const Vec2d Vec2d::nullVec2d = Vec2d(0, 0, true);
const Vec2d Vec2d::nanVec2d =
    Vec2d(std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::quiet_NaN(), true);

Vec2d::Vec2d() : mX(0.0), mY(0.0), mValid(true)
{
}

Vec2d::Vec2d(double x, double y, bool valid) : mX(x), mY(y)
{
    mValid = valid && NS::isNormal(x) && NS::isNormal(y);
}

double Vec2d::x() const
{
    return mX;
}

double Vec2d::y() const
{
    return mY;
}

bool Vec2d::isValid() const
{
    return mValid;
}

void Vec2d::setX(double x)
{
    mX = x;
}

void Vec2d::setY(double y)
{
    mY = y;
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
    mValid = NS::isNormal(mX) && NS::isNormal(mY);
}

double Vec2d::getAngleTo(const Vec2d &v) const
{
    if (!mValid || !v.mValid)
        return std::numeric_limits<double>::quiet_NaN();
    else {
        return (v - *this).getAngle();
    }
}

double Vec2d::getAngle() const
{
    double ret = 0.0;
    double m = getMagnitude();

    if (m > 1.0e-6) {
        double dp = getDotProduct(*this, Vec2d(1.0, 0.0));
        if (dp / m >= 1.0) {
            ret = 0.0;
        }
        else if (dp / m < -1.0) {
            ret = M_PI;
        }
        else {
            ret = acos(dp / m);
        }
        if (mY < 0.0) {
            ret = 2 * M_PI - ret;
        }
    }
    return ret;
}

double Vec2d::getMagnitude() const
{
    if (!mValid) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return sqrt(mX * mX + mY * mY);
}

double Vec2d::getDistanceTo(const Vec2d &v) const
{
    if (!mValid || !v.mValid) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    else {
        return (*this - v).getMagnitude();
    }
}

Vec2d& Vec2d::move(const Vec2d &offset)
{
    *this += offset;
    return *this;
}

Vec2d& Vec2d::rotate(double rotation)
{
    if(!mValid)
        return *this;

    double r = getMagnitude();
    double a = getAngle() + rotation;

    mX = cos(a) * r;
    mY = sin(a) * r;
    return *this;
}

Vec2d& Vec2d::rotate(double rotation, const Vec2d &center)
{
    *this = center + (*this - center).rotate(rotation);
    return *this;
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
    if (s == 0)
        throw std::invalid_argument("s can not be zero");
    return Vec2d(mX / s, mY / s, mValid);
}

Vec2d Vec2d::operator-() const
{
    return Vec2d(-mX, -mY, mValid);
}

Vec2d &Vec2d::operator+=(const Vec2d &rhs)
{
    mX += rhs.mX;
    mY += rhs.mY;
    mValid = mValid && rhs.mValid;
    return *this;
}

Vec2d &Vec2d::operator-=(const Vec2d &rhs)
{
    mX -= rhs.mX;
    mY -= rhs.mY;
    mValid = mValid && rhs.mValid;
    return *this;
}

Vec2d &Vec2d::operator*=(double s)
{
    mX *= s;
    mY *= s;
    return *this;
}

Vec2d &Vec2d::operator/=(double s)
{
    if (s == 0)
        throw std::invalid_argument("s can not be zero");
    mX /= s;
    mY /= s;
    return *this;
}

bool Vec2d::operator==(const Vec2d &rhs) const
{
    return mX == rhs.mX && mY == rhs.mY && mValid == rhs.mValid;
}

bool Vec2d::operator!=(const Vec2d &rhs) const
{
    return !(*this == rhs);
}

double Vec2d::getDotProduct(const Vec2d &v1, const Vec2d &v2)
{
    return v1.mX * v2.mX + v1.mY * v2.mY;
}

Vec2d Vec2d::createPolar(double radius, double angle)
{
    Vec2d ret;
    ret.setPolar(radius, angle);
    return ret;
}

Vec2d Vec2d::getMinimum(const Vec2d& v1, const Vec2d &v2)
{
    return Vec2d(min(v1.x(), v2.x()), min(v1.y(), v2.y()), v1.isValid() && v2.isValid());
}

Vec2d Vec2d::getMaximum(const Vec2d& v1, const Vec2d &v2)
{
    return Vec2d(max(v1.x(), v2.x()), max(v1.y(), v2.y()), v1.isValid() && v2.isValid());
}