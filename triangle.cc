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

Triangle::Triangle() {}

Triangle::Triangle(const Vec2d &p1, const Vec2d &p2, const Vec2d &p3)
{
    mCorner[0] = p1;
    mCorner[1] = p2;
    mCorner[2] = p3;
}

ShapeType Triangle::shapeType() const { return ShapeType::CADA_TRIANGLE; }

Shape *Triangle::clone() 
{

}

std::vector<Vec2d> Triangle::getEndPoints() const 
{
    std::vector<Vec2d> ret;

    ret.push_back(mCorner[0]);
    ret.push_back(mCorner[1]);
    ret.push_back(mCorner[2]);

    return ret;
}

std::vector<Vec2d> Triangle::getMiddlePoints() const 
{
    std::vector<Vec2d> ret;

    ret.push_back((mCorner[0] + mCorner[1]) / 2.0);
    ret.push_back((mCorner[1] + mCorner[2]) / 2.0);
    ret.push_back((mCorner[2] + mCorner[0]) / 2.0);

    return ret;
}

std::vector<Vec2d> Triangle::getCenterPoints() const { return getMiddlePoints(); }

std::vector<Shape *> Triangle::getExploded() const 
{
    std::vector<Shape *> ret;
    for(int i = 0; i < 3; ++i)
    {
        ret.push_back(new Line(mCorner[i], mCorner[(i+1) % 3]));
    }
    return ret;
}

bool Triangle::move(const Vec2d &offset) 
{
    mCorner[0].move(offset);
    mCorner[1].move(offset);
    mCorner[2].move(offset);
    return true;
}

bool Triangle::rotate(double rotation, const Vec2d &center) 
{
    mCorner[0].rotate(rotation, center);
    mCorner[1].rotate(rotation, center);
    mCorner[2].rotate(rotation, center);
    return true;
}

BBox Triangle::getBoundingBox() const
{
    return BBox(Vec2d::getMinimum(Vec2d::getMinimum(mCorner[0], mCorner[1]),
                                    mCorner[2]),
                Vec2d::getMaximum(Vec2d::getMaximum(mCorner[0], mCorner[1]),
                                    mCorner[2]));
}