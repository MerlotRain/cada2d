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

Ray::Ray() : XLine()
{
}

Ray::Ray(const Line &line) : XLine(line)
{
}

Ray::Ray(const Vec2d &basePoint, const Vec2d &directionVector)
    : XLine(basePoint, directionVector)
{
}

Ray::Ray(const Vec2d &basePoint, double angle, double distance)
    : XLine(basePoint, angle, distance)
{
}

NS::ShapeType Ray::getShapeType() const
{
    return NS::Ray;
}

std::unique_ptr<Shape> Ray::clone() const
{
    Ray *pClone = new Ray();
    pClone->mBasePoint = mBasePoint;
    pClone->mDirectionVector = mDirectionVector;
    return std::unique_ptr<Shape>(pClone);
}

} // namespace shape
} // namespace cada