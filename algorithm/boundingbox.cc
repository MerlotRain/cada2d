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

#include "boundingbox.h"
#include <cada_shape.h>
#include <assert.h>

using namespace cada::shape;

namespace cada {
namespace algorithm {

BoundingBox::BoundingBox(shape::Shape *shape) : mShape(shape)
{
}

shape::BBox BoundingBox::getBoundingBox() const
{
    assert(mShape);
    switch (mShape->getShapeType()) {
    case NS::Point: {
        auto point = dynamic_cast<Point *>(mShape);
        return BBox(point->getPosition(), point->getPosition());
    }
    case NS::Line: {
        auto line = dynamic_cast<Line *>(mShape);
        return BBox(
            Vec2d::getMinimum(line->getStartPoint(), line->getEndPoint()),
            Vec2d::getMaximum(line->getStartPoint(), line->getEndPoint()));
    }
    case NS::Arc: {
        return getArcBoundingBox();
    }
    case NS::Circle: {
        return getCircleBoundingBox();
    }
    case NS::Ellipse: {
        return getEllipseBoundingBox();
    }
    case NS::XLine:
    case NS::Ray: {
        return getXLineBoundingBox();
    }
    case NS::Polyline:
    case NS::BSpline:
        break;

    default:
        break;
    }
    return shape::BBox();
}

shape::BBox BoundingBox::getBoundingBox(shape::Shape *shape)
{
    BoundingBox bb(shape);
    return bb.getBoundingBox();
}

shape::BBox BoundingBox::getArcBoundingBox() const
{
    Arc *arc = dynamic_cast<Arc *>(mShape);
    assert(arc);

    if (!arc->isValid())
        return shape::BBox();
    Vec2d startPoint = arc->getStartPoint();
    Vec2d endPoint = arc->getEndPoint();
    Vec2d center = arc->getCenter();

    Vec2d minV, maxV;
    double minX = std::min(startPoint.x, endPoint.x);
    double minY = std::min(startPoint.y, endPoint.y);
    double maxX = std::max(startPoint.x, endPoint.x);
    double maxY = std::max(startPoint.y, endPoint.y);

    if(startPoint.getDistanceTo(endPoint) < 1.0e-6 && arc->getRadius() > 1.0e5) {
        minV = Vec2d(minX, minY);
        maxV = Vec2d(maxX, maxY);
        return BBox(minV, maxV);
    }

    double a1 = Math::getNormalizedAngle(arc->isReversed() ? arc->getEndAngle() : arc->getStartAngle());
    double a2 = Math::getNormalizedAngle(arc->isReversed() ? arc->getStartAngle() : arc->getEndAngle());

    // check for left limit:
    if ((a1<M_PI && a2>M_PI) ||
            (a1>a2-1.0e-12 && a2>M_PI) ||
            (a1>a2-1.0e-12 && a1<M_PI) ) {

        minX = std::min(center.x - arc->getRadius(), minX);
    }

    // check for right limit:
    if (a1 > a2-1.0e-12) {
        maxX = std::max(center.x + arc->getRadius(), maxX);
    }

    // check for bottom limit:
    if ((a1<(M_PI_2*3) && a2>(M_PI_2*3)) ||
            (a1>a2-1.0e-12    && a2>(M_PI_2*3)) ||
            (a1>a2-1.0e-12    && a1<(M_PI_2*3)) ) {

        minY = std::min(center.y - arc->getRadius(), minY);
    }


    // check for top limit:
    if ((a1<M_PI_2 && a2>M_PI_2) ||
            (a1>a2-1.0e-12   && a2>M_PI_2) ||
            (a1>a2-1.0e-12   && a1<M_PI_2) ) {

        maxY = std::max(center.y + arc->getRadius(), maxY);
    }

    minV = Vec2d(minX, minY);
    maxV = Vec2d(maxX, maxY);
    return BBox(minV, maxV);
}

shape::BBox BoundingBox::getCircleBoundingBox() const
{
    Circle *circle = dynamic_cast<Circle *>(mShape);
    assert(circle);

    return BBox(
        circle->getCenter() - Vec2d(circle->getRadius(), circle->getRadius()),
        circle->getCenter() + Vec2d(circle->getRadius(), circle->getRadius()));
}

shape::BBox BoundingBox::getEllipseBoundingBox() const
{
    Ellipse *ellipse = dynamic_cast<Ellipse *>(mShape);
    assert(ellipse);

    double radius1 = ellipse->getMajorRadius();
    double radius2 = ellipse->getMinorRadius();
    double angle = ellipse->getAngle();
    double a1 = (ellipse->isReversed() ? ellipse->getEndParam() : ellipse->getStartParam());
    double a2 = (ellipse->isReversed() ? ellipse->getStartParam() : ellipse->getEndParam());
    
    Vec2d startPoint = ellipse->getStartPoint();
    Vec2d endPoint = ellipse->getEndPoint();
    Vec2d center = ellipse->getCenter();

    double minX = std::min(startPoint.x, endPoint.x);
    double minY = std::min(startPoint.y, endPoint.y);
    double maxX = std::max(startPoint.x, endPoint.x);
    double maxY = std::max(startPoint.y, endPoint.y);
    Vec2d vp;
    double a = a1;
    do {
        vp.set(center.x + radius1 * cos(a),
               center.y + radius2 * sin(a));
        vp.rotate(angle, center);

        minX = std::min(minX, vp.x);
        minY = std::min(minY, vp.y);
        maxX = std::max(maxX, vp.x);
        maxY = std::max(maxY, vp.y);

        a += 0.03;
    } while (Math::isAngleBetween(a, a1, a2, false) && a < 4 * M_PI);

    return BBox(Vec2d(minX,minY), Vec2d(maxX,maxY));
}

shape::BBox BoundingBox::getXLineBoundingBox() const
{
    XLine *xline = dynamic_cast<XLine *>(mShape);
    assert(xline);

    return BBox(
        Vec2d::getMinimum(xline->getStartPoint(), xline->getEndPoint()),
        Vec2d::getMaximum(xline->getStartPoint(), xline->getEndPoint()));
}

} // namespace algorithm
} // namespace cada