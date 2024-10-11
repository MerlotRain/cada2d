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

bool Polyline::isStraight(double bulge) const
{
    return fabs(bulge) < 1.0e-6;
}

Polyline::Polyline()
{
}

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