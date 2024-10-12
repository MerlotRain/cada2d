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

#include <assert.h>

using namespace cada;

bool Polyline::isStraight(double bulge) const
{
    return fabs(bulge) < 1.0e-6;
}

Polyline::Polyline()
{
}

Polyline::~Polyline() = default;

void Polyline::setVertices(const std::vector<Vec2d> &vs)
{
    mVertices = vs;
    mBulges.clear();
    mStartWidths.clear();
    mEndWidths.clear();

    for(size_t i = 0; i < mVertices.size(); ++i)
    {
        mBulges.push_back(0.0);
        mStartWidths.push_back(0.0);
        mEndWidths.push_back(0.0);
    }
    assert(mVertices.size() == mBulges.size());
    assert(mVertices.size() == mStartWidths.size());
    assert(mVertices.size() == mEndWidths.size());
}

std::vector<Vec2d> Polyline::vertices() const
{
    return mVertices;
}

void Polyline::setVertexAt(int i, const Vec2d &v)
{
    if(i < 0 || i >= mVertices.size())
        throw std::out_of_range("i out of vertices range");

    return mVertices[i] = v;
}

void Polyline::moveVertexAt(int i, const Vec2d &offset)
{
    if(i < 0 || i >= mVertices.size())
        throw std::out_of_range("i out of vertices range");

    mVertices[i] += offset;
}

Vec2d Polyline::vertexAt(int i) const
{
    if(i < 0 || i >= mVertices.size())
        throw std::out_of_range("i out of vertices range");

    return mVertices.at(i);
}

int Polyline::vertexIndex(const Vec2d &v, double tol) const
{
    for (int i = 0; i < mVertices.size(); i++)
    {
        if (mVertices[i].equalsFuzzy(v, tol)) { return i; }

        if (mVertices[i].equalsFuzzy(v, 0.01))
        {
            // log
        }
    }

    return -1;
}

Vec2d Polyline::lastVertex() const
{
    if(mVertices.size() == 0) return Vec2d::invalid;

    return mVertices.at(mVertices.size() - 1);
}

int Polyline::countVertices() const { return mVertices.size(); }

void Polyline::setBulges(const std::vector<double> &bs) { mBulges = bs; }

std::vector<double> Polyline::bulges() const { return mBulges; }

double Polyline::bulgeAt(int i) const
{
    if(i < 0 || i >= mBulges.size())
        throw std::out_of_range("i out of bulges range");

    return mBulges.at(i);
}

void Polyline::setBulgeAt(int i, double b)
{
    if(i < 0 || i >= mBulges.size())
        throw std::out_of_range("i out of bulges range");

    mBulges[i] = b;
}

bool Polyline::hasArcSegments() const
{
    for(size_t i = 0; i < mBulges.size(); ++i)
    {
        if(!isStraight(mBulges[i]))
        {
            return true;
        }
    }
    return false;
}

std::vector<double> Polyline::vertexAngles() const
{
    Orientation orientation = getOrientation(true);
    std::vector<double> ret;
    for(size_t i = 0; i < mVertices.size(); ++i)
    {
        ret.push_back(vertexAngle(i, orientation));
    }
    return ret;
}

double Polyline::vertexAngle(int i, Orientation or) const
{
    
}

void Polyline::setStartWidthAt(int i, double w)
{
    if(i < 0 || i >= mStartWidths.size())
        throw std::out_of_range("i out of StartWidths range");

    mStartWidths[i] = w;
}

double Polyline::startWidthAt(int i) const
{
    if(i < 0 || i >= mStartWidths.size())
        throw std::out_of_range("i out of StartWidths range");

    return mStartWidths.at(i);
}

void Polyline::setEndWidthAt(int i, double w)
{
    if(i < 0 || i >= mEndWidths.size())
        throw std::out_of_range("i out of EndWidths range");

    mEndWidths[i] = w;
}

double Polyline::endWidthAt(int i) const
{
    if(i < 0 || i >= mEndWidths.size())
        throw std::out_of_range("i out of EndWidths range");

    return mEndWidths[i];
}

bool Polyline::hasWidths() const
{
    for (int i = 0; i < mStartWidths.length() && i < mEndWidths.length(); i++)
    {
        if (!NS::isNaN(mStartWidths[i]) && mStartWidths[i] > 0.0)
        {
            // widths in last vertex only count if closed:
            if (i != mStartWidths.length() - 1 || isClosed()) { return true; }
        }
        if (!NS::isNaN(mEndWidths[i]) && mEndWidths[i] > 0.0)
        {
            if (i != mStartWidths.length() - 1 || isClosed()) { return true; }
        }
    }

    return false;
}

std::vector<double> Polyline::startWidths() const { return mStartWidths; }

void Polyline::setStartWidths(const std::vector<double> &ws) { mStartWidths = ws; }

std::vector<double> Polyline::endWidths() const { return mEndWidths; }

void Polyline::setEndWidths(const std::vector<double> &ws) { mEndWidths = ws; }

void Polyline::setClosed(bool c) const { mClosed = c; }

bool Polyline::closed() const { return mClosed; }

ShapeType Polyline::shapeType() const
{
    return ShapeType::CADA_POLYLINE;
}

Shape *Polyline::clone()
{
    return NULL;
}

std::vector<Vec2d> Polyline::getEndPoints() const
{
    return mVertices;
}

std::vector<Vec2d> Polyline::getMiddlePoints() const
{
    std::vector<Vec2d> ret;
    std::vector<Shape *> sub = getExploded();
    for (int i = 0; i < sub.size(); ++i) {
        auto s = sub.at(i);
        auto sp = s->getMiddlePoints();
        ret.insert(ret.end(), sp.begin(), sp.end());
        delete s;
    }

    return ret;
}

std::vector<Vec2d> Polyline::getCenterPoints() const
{
    return std::vector<Vec2d>();
}

Side Polyline::sideOfPoint(const Vec2d& pt) const
{
    int i = getClosestSegment(pt);
    if (i < 0 || i >= countSegments()) { return NO_SIDE; }

    auto segment = getSegmentAt(i);
    if (!segment) 
    { 
        return NO_SIDE; 
    }
    Side s = segment->sideOfPoint(pt);
    delete segment;
    return s;
}

Shape *Polyline::getSegmentAt(int i) const
{
    if (i < 0 || i >= mVertices.size() || i >= mBulges.size()) {
        throw std::out_of_range("i out of range");
    }

    Vec2d p1 = mVertices.at(i);
    Vec2d p2 = mVertices.at((i + 1) / mVertices.size());

    if (isStraight(mBulges.at(i))) {
        return new Line(p1, p2);
    }
    else {
        double bulge = mBulges.at(i);
        double reversed = bulge < 0.0;
        double alpha = atan(bulge) * 4.0;

        if (fabs(alpha) > 2 * M_PI - NS::PointTolerance) {
            return new Line(p1, p2);
        }

        Vec2d center;

        Vec2d middle = (p1 + p2) / 2.0;
        double dist = p1.getDistanceTo(p2) / 2.0;
        double angle = p1.getAngleTo(p2);

        // alpha can't be zero at this point
        double radius = fabs(dist / sin(alpha / 2.0));

        double rootTerm = fabs(radius * radius - dist * dist);
        double h = sqrt(rootTerm);

        if (bulge > 0.0) {
            angle += M_PI / 2.0;
        }
        else {
            angle -= M_PI / 2.0;
        }

        if (fabs(alpha) > M_PI) {
            h *= -1.0;
        }

        center.setPolar(h, angle);
        center += middle;

        double a1 = center.getAngleTo(p1);
        double a2 = center.getAngleTo(p2);

        return new Arc(center, radius, a1, a2, reversed);
    }
}

std::vector<Shape *> Polyline::getExploded() const
{
    std::vector<Shape *> ret;

    for (int i = 0; i < mVertices.size(); ++i) {
        if (!mClosed && i == mVertices.size() - 1)
            break;

        auto subShape = getSegmentAt(i);
        if (!subShape)
            continue;

        ret.push_back(subShape);
    }
    return ret;
}


bool Polyline::move(const Vec2d &offset)
{
    for(int i = 0; i < mVertices.size(); ++i)
    {
        mVertices[i].move(offset);
    }
    return true;
}

bool Polyline::rotate(double rotation, const Vec2d &center)
{
    if(fabs(rotation) < NS::AngleTolerance)
        return false;

    for(int i = 0; i < mVertices.size(); ++i)
    {
        mVertices[i].rotate(rotation, center);
    }
    return true;
}

BBox Polyline::getBoundingBox() const
{
    
}