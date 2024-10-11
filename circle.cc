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

#include "cadsa_shape.h"

using namespace cadsa;

Circle::Circle()
{
}

Circle::Circle(double cx, double cy, double radius)
{
}

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

ShapeType Circle::shapeType() const
{
    return ShapeType::CADA_CIRCLE;
}

Shape *Circle::clone()
{
    Circle *pClone = new Circle();
    pClone->mRadius = mRadius;
    pClone->mRadius = mRadius;
    return pClone;
}

std::vector<Vec2d> Circle::getEndPoints() const
{
    return std::vector<Vec2d>();
}

std::vector<Vec2d> Circle::getMiddlePoints() const
{
    return std::vector<Vec2d>();
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