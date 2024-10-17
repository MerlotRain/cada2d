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

Point::Point()
{
}

Point::Point(double x, double y) : mPosition(x, y)
{
}

Point::Point(const Vec2d &position) : mPosition(position)
{
}

Vec2d Point::getPosition() const
{
    return mPosition;
}

void Point::setPosition(const Vec2d &p)
{
    mPosition = p;
}

bool Point::isValid() const
{
    return mPosition.isValid();
}

NS::ShapeType Point::getShapeType() const
{
    return NS::Point;
}

Point *Point::clone() const
{
    Point *pClone = new Point();
    pClone->mPosition = mPosition;
    return pClone;
}

std::vector<Vec2d> Point::getEndPoints() const
{
    std::vector<Vec2d> ret;
    ret.push_back(mPosition);
    return ret;
}

std::vector<Vec2d> Point::getMiddlePoints() const
{
    std::vector<Vec2d> ret;
    ret.push_back(mPosition);
    return ret;
}

std::vector<Vec2d> Point::getCenterPoints() const
{
    std::vector<Vec2d> ret;
    ret.push_back(mPosition);
    return ret;
}

} // namespace cada