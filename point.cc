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

Point::Point(const Vec3d &v) : mp(v)
{
}

Point::Point(double x, double y) : mp(x, y)
{
}

ShapeType Point::shapeType() const
{
    return ShapeType::CADA_POINT;
}

Shape *Point::clone()
{
    return nullptr;
}

std::vector<Vec3d> Point::getEndPoints() const
{
    std::vector<Vec3d> ret;
    ret.push_back(mp);
    return ret;
}

std::vector<Vec3d> Point::getMiddlePoints() const
{
    std::vector<Vec3d> ret;
    ret.push_back(mp);
    return ret;
}

std::vector<Vec3d> Point::getCenterPoints() const
{
    std::vector<Vec3d> ret;
    ret.push_back(mp);
    return ret;
}