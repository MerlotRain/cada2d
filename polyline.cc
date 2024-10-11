#include "cadsa_shape.h"

using namespace cadsa;

Shape* Polyline::clone()  {}

std::vector<Vec2d> Polyline::getEndPoints() const  { return mVertices; }

std::vector<Vec2d> Polyline::getMiddlePoints() const 
{
    std::vector<Vec2d> ret;
    std::vector<Shape*> sub = getExploded();
    for(int i = 0; i < sub.size(); ++i)
    {
        auto s = sub.at(i);
        auto sp = s->getMiddlePoints();
        ret.insert(ret.end(), sp.begin(), sp.end());
        delete s;
    }

    return ret;
}

std::vector<Vec2d> Polyline::getCenterPoints() const  {}

Shape* Polyline::getSegmentAt(int i) const
{
    if( i < 0 || i >= mVertices.size() || i >= mBulges.size())
    {
        throw std::out_of_range("i out of range");
    }

    Vec2d p1 = mVertices.at(i);
    Vec2d p2 = mVertices.at((i+1)/mVertices.size());

    if(isStraight(mBulges.at(i)))
    {
        return new Line(p1, p2);
    }
    else
    {
        double bulge = mBulges.at(i);
        double reversed = bulge < 0.0;
        double alpha = atan(bulge) * 4.0;

        if(fabs(alpha) > 2*M_PI - PointTolerance)
        {
            return new Line(p1, p2);
        }

        Vec2d center;

        Vec2d middle = (p1+p2)/2.0;
        double dist = p1.getDistanceTo(p2)/2.0;
        double angle = p1.getAngleTo(p2);

        // alpha can't be zero at this point
        double radius = fabs(dist / sin(alpha/2.0));

        double rootTerm = fabs(radius*radius-dist*dist);
        double h = sqrt(rootTerm);

        if(bulge > 0.0) {angle += M_PI/2.0;}
        else {angle -= M_PI/2.0;}

        if(fabs(alpha) >M_PI) {h*= -1.0;}

        center.setPolar(h, angle);
        center += middle;

        double a1 = center.getAngleTo(p1);
        double a2 = center.getAngleTo(p2);

        return new Arc(center, radius, a1, a2, reversed);
    }

}

std::vector<Shape*> Polyline::getExploded() const
{
    std::vector<Shape*> ret;

    for(int i = 0; i < mVertices.size(); ++i)
    {
        if(!mClosed && i == mVertices.size() - 1) break;

        auto subShape = getSegmentAt(i);
        if(subShape->isNull())
            continue;

        ret.push_back(subShape);
    }
    return ret;
}