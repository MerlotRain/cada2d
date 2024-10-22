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

namespace cada {
namespace shape {

/**
 * Creates an xline object with invalid base point and direction.
 */
XLine::XLine() : mBasePoint(Vec2d::invalid), mDirectionVector(Vec2d::invalid)
{
}

XLine::XLine(const Line &line)
    : mBasePoint(line.getStartPoint()),
      mDirectionVector(line.getEndPoint() - line.getStartPoint())
{
}

/**
 * Creates an xline object with the given base point and direction.
 */
XLine::XLine(const Vec2d &basePoint, const Vec2d &directionVector)
    : mBasePoint(basePoint), mDirectionVector(directionVector)
{
}

XLine::XLine(const Vec2d &basePoint, double angle, double distance)
    : mBasePoint(basePoint),
      mDirectionVector(Vec2d::createPolar(distance, angle))
{
}

bool XLine::isValid() const
{
    return true;
}

NS::ShapeType XLine::getShapeType() const
{
    return NS::XLine;
}

std::unique_ptr<Shape> XLine::clone() const
{
    XLine *pClone = new XLine();
    pClone->mBasePoint = mBasePoint;
    pClone->mDirectionVector = mDirectionVector;
    return std::unique_ptr<Shape>(pClone);
}

double XLine::getAngle() const
{
    return mDirectionVector.getAngle();
}

void XLine::setAngle(double a)
{
    mDirectionVector.setAngle(a);
}

void XLine::setLength(double l)
{
    return;
}

Vec2d XLine::getStartPoint() const
{
    return mBasePoint;
}

Vec2d XLine::getEndPoint() const
{
    return mBasePoint + mDirectionVector;
}

Vec2d XLine::getBasePoint() const
{
    return mBasePoint;
}

void XLine::setBasePoint(const Vec2d &vector)
{
    mBasePoint = vector;
}

Vec2d XLine::getDirectionVector() const
{
    return mDirectionVector;
}

void XLine::setDirectionVector(const Vec2d &vector)
{
    mDirectionVector = vector;
}

Vec2d XLine::getMiddlePoint() const
{
    return Vec2d::invalid;
}

std::vector<Vec2d> XLine::getEndPoints() const
{
    return std::vector<Vec2d>();
}

std::vector<Vec2d> XLine::getMiddlePoints() const
{
    return std::vector<Vec2d>();
}

std::vector<Vec2d> XLine::getCenterPoints() const
{
    return std::vector<Vec2d>();
}

} // namespace shape
} // namespace cada