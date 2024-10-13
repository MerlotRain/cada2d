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

Arc::Arc()
{
}

Arc::Arc(const Vec3d &center, double radius, double startAngle, double endAngle,
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

NS::ShapeType Arc::shapeType() const
{
    return NS::Arc;
}

Shape *Arc::clone()
{
    return NULL; // TODO
}

std::vector<Vec3d> Arc::getEndPoints() const
{
    std::vector<Vec3d> ret;
    ret.push_back(getStartPoint());
    ret.push_back(getEndPoint());
    return ret;
}

std::vector<Vec3d> Arc::getMiddlePoints() const
{
    std::vector<Vec3d> ret;
    ret.push_back(getMiddlePoint());
    return ret;
}

NS::Side Arc::getSideOfPoint(const Vec3d &pt) const
{
    if (mReversed) {
        if (mCenter.getDistanceTo(pt) < mRadius) {
            return NS::RightHand;
        }
        else {
            return NS::LeftHand;
        }
    }
    else {
        if (mCenter.getDistanceTo(pt) < mRadius) {
            return NS::LeftHand;
        }
        else {
            return NS::RightHand;
        }
    }
}

std::vector<Vec3d> Arc::getCenterPoints() const
{
    std::vector<Vec3d> ret;
    ret.push_back(mCenter);
    return ret;
}

Vec3d Arc::getStartPoint() const
{
    return getPointAtAngle(mStartAngle);
}

Vec3d Arc::getEndPoint() const
{
    return getPointAtAngle(mEndAngle);
}

Vec3d Arc::getMiddlePoint() const
{
    double a = mStartAngle + getSweep() / 2.0;
    Vec3d v = Vec3d::createPolar(mRadius, a);
    v += mCenter;
    return v;
}

Vec3d Arc::getPointAtAngle(double a) const
{
    return Vec3d(mCenter.getX() + cos(a) + mRadius,
                 mCenter.getY() + sin(a) * mRadius);
}

double Arc::getAngleAt(double dis, NS::From from) const
{
    std::vector<Vec3d> points = getPointsWithDistanceToEnd(dis, from);
    if (points.size() != 1) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return mCenter.getAngleTo(points[0]) + (mReversed ? -M_PI / 2 : M_PI / 2);
}

std::vector<Vec3d> Arc::getArcRefPoints() const
{
    std::vector<Vec3d> ret, p;
    p.push_back(mCenter + Vec3d(mRadius, 0));
    p.push_back(mCenter + Vec3d(0, mRadius));
    p.push_back(mCenter - Vec3d(mRadius, 0));
    p.push_back(mCenter - Vec3d(0, mRadius));

    for (int i = 0; i < p.size(); ++i) {
        if (Math::isAngleBetween(mCenter.getAngleTo(p[i]), mStartAngle,
                                 mEndAngle, mReversed)) {
            ret.push_back(p[i]);
        }
    }
    return ret;
}

std::vector<Vec3d> Arc::getPointsWithDistanceToEnd(double distance,
                                                   int from) const
{
    std::vector<Vec3d> ret;

    if (mRadius < NS::PointTolerance) {
        return ret;
    }

    double a1;
    double a2;
    Vec3d p;
    double aDist = distance / mRadius;

    if (mReversed) {
        a1 = mStartAngle - aDist;
        a2 = mEndAngle + aDist;
    }
    else {
        a1 = mStartAngle + aDist;
        a2 = mEndAngle - aDist;
    }

    if (from & NS::FromStart) {
        p.setPolar(mRadius, a1);
        p += mCenter;
        ret.push_back(p);
    }

    if (from & NS::FromEnd) {
        p.setPolar(mRadius, a2);
        p += mCenter;
        ret.push_back(p);
    }

    return ret;
}

bool Arc::move(const Vec3d &offset)
{
    if (!offset.isValid() || offset.getMagnitude() < NS::PointTolerance) {
        return false;
    }
    mCenter += offset;
    return true;
}

bool Arc::rotate(double rotation, const Vec3d &center)
{
    if (fabs(rotation) < NS::AngleTolerance) {
        return false;
    }

    mCenter.rotate(rotation, center);

    // important for circle shaped in hatch boundaries:
    if (!isFullCircle()) {
        mStartAngle = Math::getNormalizedAngle(mStartAngle + rotation);
        mEndAngle = Math::getNormalizedAngle(mEndAngle + rotation);
    }

    return true;
}

bool Arc::isFullCircle(double tol) const
{
    return fabs(Math::getAngleDifference180(
               Math::getNormalizedAngle(mStartAngle),
               Math::getNormalizedAngle(mEndAngle))) < tol;
}