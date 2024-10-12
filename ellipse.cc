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

Ellipse::Ellipse() :mCenter(Vec2d::invalid), mMajorPoint(Vec2d::invalid), mRatio(0.0),mStartParam(0.0), mEndParam(0.0), mReversed(false) {}

Ellipse::Ellipse(const Vec2d &center, const Vec2d &majorPoint, double ratio, double startParam, double endParam, bool reversed)
    : mCenter(center), mMajorPoint(majorPoint), mRatio(ratio), mStartParam(startParam), mEndParam(endParam), mReversed(reversed) 
{
    correctMajorMinor();
}

ShapeType Ellipse::shapeType() const { return ShapeType::CADA_ELLIPSE; }

Shape *Ellipse::clone() 
{
    Ellipse* pClone = new Ellipse();
    pClone->mCenter = mCenter;
    pClone->mMajorPoint= mMajorPoint;
    pClone->mRatio = mRatio;
    pClone->mStartParam = mStartParam;
    pClone->mEndParam = mEndParam;
    pClone->mReversed = mReversed;
    return pClone;
}

std::vector<Vec2d> Ellipse::getEndPoints() const 
{
    std::vector<Vec2d> ret;
    ret.push_back(getStartPoint());
    ret.push_back(getEndPoint());
    return ret;
}

std::vector<Vec2d> Ellipse::getMiddlePoints() const { return std::vector<Vec2d>(); }

std::vector<Vec2d> Ellipse::getCenterPoints() const 
{
    std::vector<Vec2d> ret;
    ret.push_back(getCenter());
    return ret;
}

Side Ellipse::sideOfPoint(const Vec2d& pt) const 
{
    if(isContains(pt))
    {
        if(!mReversed)
        {
            return RIGHT_HAND;
        }
        else
        {
            return LEFT_HAND;
        }
    }
    else
    {
        if(!mReversed)
        {
            return LEFT_HAND;
        }
        else
        {
            return RIGHT_HAND;
        }
    }
}

bool Ellipse::move(const Vec2d &offset)
{
    if(!offset.isValid() || offset.getMagnitude() < NS::PointTolerance)
        return false;

    mCenter += offset;
    mMajorPoint += offset;
    return true;
}

bool Ellipse::rotate(double rotation, const Vec2d &center)
{
    if(fabs(rotation) < NS::AngleTolerance)
    {
        return false;
    }

    mCenter.rotate(rotation, center);
    mMajorPoint.rotate(rotation, center);
    return true;
}