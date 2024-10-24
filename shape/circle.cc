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

Circle Circle::createFrom2Points(const Vec2d &p1, const Vec2d &p2)
{
    Vec2d center = (p1 + p2) / 2.0;
    double radius = p1.getDistanceTo(p2) / 2.0;
    return Circle(center, radius);
}

Circle Circle::createFrom3Points(const Vec2d &p1, const Vec2d &p2,
                                 const Vec2d &p3)
{
    Vec2d mp1 = Vec2d::getAverage(p1, p2);
    double a1 = p1.getAngleTo(p2) + M_PI / 2.0;
    Vec2d dir1 = Vec2d::createPolar(1.0, a1);
    Vec2d mp2 = Vec2d::getAverage(p2, p3);
    double a2 = p2.getAngleTo(p3) + M_PI / 2.0;
    Vec2d dir2 = Vec2d::createPolar(1.0, a2);

    Line midLine1(mp1, mp1 + dir1);
    Line midLine2(mp2, mp2 + dir2);

    std::vector<Vec2d> ips = midLine1.getIntersectionPoints(midLine2, false);
    if (ips.size() != 1) {
        return Circle();
    }

    Vec2d center = ips[0];
    double radius = center.getDistanceTo(p3);
    return Circle(center, radius);
}

bool Circle::isValid() const
{
    return mCenter.isValid();
}

NS::ShapeType Circle::getShapeType() const
{
    return NS::Circle;
}

Shape* Circle::clone() const
{
    Circle* pClone = new Circle();
    pClone->mCenter = mCenter;
    pClone->mCenter = mCenter;
    return pClone;
}

Arc Circle::toArc(double startAngle) const
{
    return Arc(getCenter(), getRadius(), startAngle, startAngle + 2 * M_PI,
               false);
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
std::vector<Line> Circle::getTangents(const Vec2d &point) const
{
    std::vector<Line> ret;

    Vec2d thalesCenter = (point + getCenter()) / 2;
    double thalesRadius = point.getDistanceTo(thalesCenter);

    if (thalesRadius < getRadius() / 2.0) {
        return ret;
    }

    Circle thalesCircle(thalesCenter, thalesRadius);

    std::vector<Vec2d> ips = thalesCircle.getIntersectionPoints(*this, false);

    if (ips.size() > 0) {
        ret.push_back(Line(point, ips[0]));
        if (ips.size() > 1) {
            ret.push_back(Line(point, ips[1]));
        }
    }

    return ret;
}

} // namespace shape
} // namespace cada