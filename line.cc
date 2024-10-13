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

using namespace cada;

Line::Line()
{
}

Line::Line(const Vec3d &begin, const Vec3d &end) : mBegin(begin), mEnd(end)
{
}

NS::ShapeType Line::shapeType() const
{
    return NS::Line;
}

Shape *Line::clone()
{
    Line *pClone = new Line();
    return pClone;
}

std::vector<Vec3d> Line::getEndPoints() const
{
    std::vector<Vec3d> ret;
    ret.push_back(mBegin);
    ret.push_back(mEnd);
    return ret;
}

std::vector<Vec3d> Line::getMiddlePoints() const
{
    std::vector<Vec3d> ret;
    ret.push_back(getMiddlePoint());
    return ret;
}

std::vector<Vec3d> Line::getCenterPoints() const
{
    return getMiddlePoints();
}

NS::Side Line::getSideOfPoint(const Vec3d &pt) const
{
    double entityAngle = getAngle();
    double angleToCoord = mBegin.getAngleTo(pt);
    double angleDiff = Math::getAngleDifference(entityAngle, angleToCoord);

    if (angleDiff < M_PI) {
        return NS::LeftHand;
    }
    return NS::RightHand;
}

Vec3d Line::getStartPoint() const
{
    return mBegin;
}

Vec3d Line::getEndPoint() const
{
    return mEnd;
}

Vec3d Line::getMiddlePoint() const
{
    return (mBegin + mEnd) / 2.0;
}