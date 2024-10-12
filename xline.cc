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

XLine::XLine() : mBasePoint(Vec2d::invalid), mDirectionVec(Vec2d::invalid) {}

XLine::XLine(Line* line) : mBasePoint(line->getStartPoint()), mDirectionVec(line->getEndPoint() - line->getStartPoint())
{}

XLine::XLine(const Vec2d &base, const Vec2d &dir) : mBasePoint(base), mDirectionVec(dir) {}

XLine::XLine(const Vec2d &base, double angle, double distance) {}

ShapeType XLine::shapeType() const { ShapeType::CADA_XLINE; }

Shape *XLine::clone() 
{
    XLine* pClone = new XLine();
    pClone->mBasePoint = mBasePoint;
    pClone->mDirectionVec = mDirectionVec;
    return pClone;
}

Side XLine::sideOfPoint(const Vec2d& pt) const 
{
    Line l(mBasePoint, mBasePoint + mDirectionVec);
    return l.sideOfPoint(pt);
}

bool XLine::move(const Vec2d &offset) 
{
    if(!offset.isValid() || offset.getMagnitude() < NS::PointTolerance)
    {
        return false;
    }
    mBasePoint += offset;
    return true;
}

bool XLine::rotate(double rotation, const Vec2d &center)
{
    if(fabs(rotation) < NS::AngleTolerance)
        return false;

    mBasePoint.rotate(rotation, center);
    mDirectionVec.rotate(rotation);
    return true;
}

BBox XLine::getBoundingBox() const
{
    return RBox(RVector::getMinimum(mBasePoint, mBasePoint + mDirectionVec),
                RVector::getMaximum(mBasePoint, mBasePoint + mDirectionVec));
}