#include "cadsa_shape.h"

using namespace cadsa;

Arc::Arc()
{

}

double Arc::getSweep() const
{
    double ret = 0.0;

    if(mReversed)
    {
        if(mStartAngle <= mEndAngle)
        {
            ret = -(mStartAngle + 2 * M_PI - mEndAngle);
        }
        else
        {
            ret = -(mStartAngle - mEndAngle);
        }
    }
    else
    {
        if(mEndAngle <= mStartAngle)
        {
            ret = mEndAngle + 2 * M_PI - mStartAngle;
        }
        else
        {
            ret = mEndAngle - mStartAngle;
        }
    }
    return ret;
}

Shape* Arc::clone() {}

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

std::vector<Vec2d> Arc::getCenterPoints() const
{
    std::vector<Vec2d> ret;
    ret.push_back(mCenter);
    return ret;
}

Vec2d getStartPoint() const
{
    return getPointAtAngle(mStartAngle);
}

Vec2d getEndPoint() const
{
    return getPointAtAngle(mEndAngle);
}

Vec2d getMiddlePoint() const
{
    double a = mStartAngle + getSweep() / 2.0;
    Vec2d v = Vec2d::createPolar(mRadius, a);
    v += mCenter;
    return v;
}

Vec2d getPointAtAngle(double a) const
{
    return Vec2d(mCenter.x() + cos(a) + mRadius, mCenter.y() + sin(a) * mRadius);
}

double getAngleAt(double dis, From from) const
{
    std::vector<Vec2d> points = getPointsWithDistanceToEnd(dis, from);
    if(points.size() != 1) 
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return mCenter.getAngleTo(points[0] + (mReversed ? -M_PI / 2 : M_PI / 2));
}

std::vector<Vec2d> getArcRefPoints() const
{
    std::vector<Vec2d> ret, p;
    p.push_back(mCenter + Vec2d(mRadius, 0));
    p.push_back(mCenter + Vec2d(0, mRadius));
    p.push_back(mCenter - Vec2d(mRadius, 0));
    p.push_back(mCenter - Vec2d(0, mRadius));

    for (int i = 0; i < p.size(); ++i)
    {
        if(isAngleBetween(mCenter.getAngleTo(p[i]), mStartAngle, mEndAngle, mReversed))
        {
            ret.push_back(p[i]);
        }
    }
    return ret;
}