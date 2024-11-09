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
#include <numeric>
#include <limits>
#include <sstream>
#include <iomanip>

namespace cada {
namespace shape {

Circle::Circle() : mCenter(Vec2d::invalid), mRadius(0.0)
{
}

Circle::Circle(double cx, double cy, const double radius)
    : mCenter(cx, cy), mRadius(radius)
{
}

Circle::Circle(const Vec2d &center, const double radius)
    : mCenter(center), mRadius(radius)
{
}

bool Circle::isValid() const
{
    return mCenter.isValid();
}

NS::ShapeType Circle::getShapeType() const
{
    return NS::Circle;
}

Circle *Circle::cloneImpl() const
{
    Circle *pClone = new Circle();
    pClone->mCenter = mCenter;
    pClone->mCenter = mCenter;
    return pClone;
}

std::unique_ptr<Arc> Circle::toArc(double startAngle) const
{
    return ShapeFactory::instance()->createArc(
        getCenter(), getRadius(), startAngle, startAngle + 2 * M_PI, false);
}

Vec2d Circle::getCenter() const
{
    return mCenter;
}

void Circle::setCenter(const Vec2d &vector)
{
    mCenter = vector;
}

double Circle::getRadius() const
{
    return mRadius;
}

void Circle::setRadius(double r)
{
    mRadius = r;
}

double Circle::getDiameter() const
{
    return 2 * mRadius;
}

void Circle::setDiameter(double d)
{
    mRadius = d / 2.0;
}

double Circle::getCircumference() const
{
    return mRadius * 2 * M_PI;
}

void Circle::setCircumference(double c)
{
    mRadius = c / M_PI / 2.0;
}

double Circle::getArea() const
{
    return mRadius * mRadius * M_PI;
}

void Circle::setArea(double a)
{
    mRadius = sqrt(fabs(a) / M_PI);
}

bool Circle::contains(const Vec2d &p) const
{
    return p.getDistanceTo(mCenter) < mRadius;
}

std::vector<Vec2d> Circle::getEndPoints() const
{
    std::vector<Vec2d> ret;
    return ret;
}

std::vector<Vec2d> Circle::getMiddlePoints() const
{
    std::vector<Vec2d> ret;
    return ret;
}

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
std::vector<std::unique_ptr<Line>> Circle::getTangents(const Vec2d &point) const
{
    std::vector<std::unique_ptr<Line>> ret;

    Vec2d thalesCenter = (point + getCenter()) / 2;
    double thalesRadius = point.getDistanceTo(thalesCenter);

    if (thalesRadius < getRadius() / 2.0) {
        return ret;
    }

    Circle thalesCircle(thalesCenter, thalesRadius);

    std::vector<Vec2d> ips = thalesCircle.getIntersectionPoints(this, false);

    if (ips.size() > 0) {
        ret.push_back(ShapeFactory::instance()->createLine(point, ips[0]));
        if (ips.size() > 1) {
            ret.push_back(ShapeFactory::instance()->createLine(point, ips[1]));
        }
    }

    return ret;
}

std::string Circle::to_string() const
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision(6);
    ss << "Circle: ";
    ss << "center: " << mCenter.to_string() << ", ";
    ss << "radius: " << mRadius;
    return ss.str();
}

} // namespace shape
} // namespace cada