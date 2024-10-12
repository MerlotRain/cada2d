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

Arc::Arc()
{
}

Arc::Arc(const Vec2d &center, double radius, double startAngle, double endAngle,
         bool reversed)
    : mCenter(center), mRadius(radius), mStartAngle(startAngle),
      mEndAngle(endAngle), mReversed(reversed)
{
}

double Arc::getSweep() const
{
    double ret = 0.0;

    if (mReversed) {
        if (mStartAngle <= mEndAngle) {
            ret = -(mStartAngle + 2 * M_PI - mEndAngle);
        }
        else {
            ret = -(mStartAngle - mEndAngle);
        }
    }
    else {
        if (mEndAngle <= mStartAngle) {
            ret = mEndAngle + 2 * M_PI - mStartAngle;
        }
        else {
            ret = mEndAngle - mStartAngle;
        }
    }
    return ret;
}

ShapeType Arc::shapeType() const
{
    return ShapeType::CADA_ARC;
}

Shape *Arc::clone()
{
    return NULL; // TODO
}

std::vector<Vec2d> Arc::getEndPoints() const
{
    std::vector<Vec2d> ret;
    ret.push_back(getStartPoint());
    ret.push_back(getEndPoint());
    return ret;
}

std::vector<Vec2d> Arc::getMiddlePoints() const
{
    std::vector<Vec2d> ret;
    ret.push_back(getMiddlePoint());
    return ret;
}

Side Arc::sideOfPoint(const Vec2d& pt) const
{
    if (mReversed)
    {
        if (mCenter.getDistanceTo(pt) < mRadius) { return RIGHT_HAND; }
        else { return LEFT_HAND; }
    }
    else
    {
        if (mCenter.getDistanceTo(pt) < mRadius) { return LEFT_HAND; }
        else { return RIGHT_HAND; }
    }
}

std::vector<Vec2d> Arc::getCenterPoints() const
{
    std::vector<Vec2d> ret;
    ret.push_back(mCenter);
    return ret;
}

Vec2d Arc::getStartPoint() const
{
    return getPointAtAngle(mStartAngle);
}

Vec2d Arc::getEndPoint() const
{
    return getPointAtAngle(mEndAngle);
}

Vec2d Arc::getMiddlePoint() const
{
    double a = mStartAngle + getSweep() / 2.0;
    Vec2d v = Vec2d::createPolar(mRadius, a);
    v += mCenter;
    return v;
}

Vec2d Arc::getPointAtAngle(double a) const
{
    return Vec2d(mCenter.x() + cos(a) + mRadius,
                 mCenter.y() + sin(a) * mRadius);
}

double Arc::getAngleAt(double dis, From from) const
{
    std::vector<Vec2d> points = getPointsWithDistanceToEnd(dis, from);
    if (points.size() != 1) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return mCenter.getAngleTo(points[0]) + (mReversed ? -M_PI / 2 : M_PI / 2);
}

std::vector<Vec2d> Arc::getArcRefPoints() const
{
    std::vector<Vec2d> ret, p;
    p.push_back(mCenter + Vec2d(mRadius, 0));
    p.push_back(mCenter + Vec2d(0, mRadius));
    p.push_back(mCenter - Vec2d(mRadius, 0));
    p.push_back(mCenter - Vec2d(0, mRadius));

    for (int i = 0; i < p.size(); ++i) {
        if (NS::isAngleBetween(mCenter.getAngleTo(p[i]), mStartAngle, mEndAngle,
                               mReversed)) {
            ret.push_back(p[i]);
        }
    }
    return ret;
}

std::vector<Vec2d> Arc::getPointsWithDistanceToEnd(double distance,
                                                   int from) const
{
    std::vector<Vec2d> ret;

    if (mRadius < NS::PointTolerance) {
        return ret;
    }

    double a1;
    double a2;
    Vec2d p;
    double aDist = distance / mRadius;

    if (mReversed) {
        a1 = mStartAngle - aDist;
        a2 = mEndAngle + aDist;
    }
    else {
        a1 = mStartAngle + aDist;
        a2 = mEndAngle - aDist;
    }

    if (from & FROM_START) {
        p.setPolar(mRadius, a1);
        p += mCenter;
        ret.push_back(p);
    }

    if (from & FROM_END) {
        p.setPolar(mRadius, a2);
        p += mCenter;
        ret.push_back(p);
    }

    return ret;
}

bool Arc::move(const Vec2d &offset)
{
    if (!offset.isValid() || offset.getMagnitude() < NS::PointTolerance)
    {
        return false;
    }
    mCenter += offset;
    return true;
}

bool Arc::rotate(double rotation, const Vec2d &center)
{
    if (fabs(rotation) < NS::AngleTolerance) { return false; }

    mCenter.rotate(rotation, c);

    // important for circle shaped in hatch boundaries:
    if (!isFullCircle())
    {
        mStartAngle = NS::getNormalizedAngle(mStartAngle + rotation);
        mEndAngle = NS::getNormalizedAngle(mEndAngle + rotation);
    }

    return true;
}

bool Arc::isFullCircle(double tol) const
{
    return fabs(NS::getAngleDifference180(NS::getNormalizedAngle(mStartAngle), NS::getNormalizedAngle(mEndAngle))) < tol;
}