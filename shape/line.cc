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
#include <sstream>
#include <iomanip>

namespace cada {
namespace shape {

Line::Line() : mStartPoint(Vec2d::invalid), mEndPoint(Vec2d::invalid)
{
}

Line::Line(double x1, double y1, double x2, double y2)
    : mStartPoint(x1, y1), mEndPoint(x2, y2)
{
}

Line::Line(const Vec2d &mStartPoint, const Vec2d &mEndPoint)
    : mStartPoint(mStartPoint), mEndPoint(mEndPoint)
{
}

Line::Line(const Vec2d &mStartPoint, double angle, double distance)
    : mStartPoint(mStartPoint)
{

    mEndPoint = mStartPoint + Vec2d::createPolar(distance, angle);
}

bool Line::isValid() const
{
    return mStartPoint.isSane() && mEndPoint.isSane();
}

void Line::setLength(double l, bool fromStart)
{
    if (fromStart) {
        mEndPoint = mStartPoint + Vec2d::createPolar(l, getAngle());
    }
    else {
        mStartPoint = mEndPoint - Vec2d::createPolar(l, getAngle());
    }
}

double Line::getAngle() const
{
    return mStartPoint.getAngleTo(mEndPoint);
}

void Line::setAngle(double a)
{
    mEndPoint = mStartPoint + Vec2d::createPolar(getLength(), a);
}

bool Line::isParallel(const Line *line) const
{
    double a = getAngle();
    double oa = line->getAngle();

    return Math::isSameDirection(a, oa) || Math::isSameDirection(a, oa + M_PI);
}

bool Line::isVertical(double tolerance) const
{
    return Math::fuzzyCompare(mStartPoint.x, mEndPoint.x, tolerance);
}

bool Line::isHorizontal(double tolerance) const
{
    return Math::fuzzyCompare(mStartPoint.y, mEndPoint.y, tolerance);
}

Vec2d Line::getStartPoint() const
{
    return mStartPoint;
}

void Line::setStartPoint(const Vec2d &vector)
{
    mStartPoint = vector;
}

Vec2d Line::getEndPoint() const
{
    return mEndPoint;
}

void Line::setEndPoint(const Vec2d &vector)
{
    mEndPoint = vector;
}

NS::ShapeType Line::getShapeType() const
{
    return NS::Line;
}

Line *Line::cloneImpl() const
{
    Line *pClone = new Line();
    pClone->mStartPoint = mStartPoint;
    pClone->mEndPoint = mEndPoint;
    return pClone;
}

Vec2d Line::getMiddlePoint() const
{
    return (mStartPoint + mEndPoint) / 2.0;
}

std::vector<Vec2d> Line::getEndPoints() const
{
    std::vector<Vec2d> ret;
    ret.push_back(mStartPoint);
    ret.push_back(mEndPoint);
    return ret;
}

std::vector<Vec2d> Line::getMiddlePoints() const
{
    std::vector<Vec2d> ret;
    ret.push_back(getMiddlePoint());
    return ret;
}

std::vector<Vec2d> Line::getCenterPoints() const
{
    return getMiddlePoints();
}

std::string Line::to_string() const
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision(6);
    ss << "Line: ";
    ss << "start: " << mStartPoint.to_string() << ", ";
    ss << "end: " << mEndPoint.to_string();
    return ss.str();
}

} // namespace shape
} // namespace cada