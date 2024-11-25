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

#include <cada_shape.h>
#include <assert.h>

using namespace cada::shape;

namespace cada {
namespace algorithm {

shape::BBox cada_getArcBoundingBox(const shape::Shape *shape)
{
    const Arc *arc = dynamic_cast<const Arc *>(shape);
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

    if (startPoint.getDistanceTo(endPoint) < 1.0e-6 &&
        arc->getRadius() > 1.0e5) {
        minV = Vec2d(minX, minY);
        maxV = Vec2d(maxX, maxY);
        return BBox(minV, maxV);
    }

    double a1 = Math::getNormalizedAngle(
        arc->isReversed() ? arc->getEndAngle() : arc->getStartAngle());
    double a2 = Math::getNormalizedAngle(
        arc->isReversed() ? arc->getStartAngle() : arc->getEndAngle());

    // check for left limit:
    if ((a1 < M_PI && a2 > M_PI) || (a1 > a2 - 1.0e-12 && a2 > M_PI) ||
        (a1 > a2 - 1.0e-12 && a1 < M_PI)) {

        minX = std::min(center.x - arc->getRadius(), minX);
    }

    // check for right limit:
    if (a1 > a2 - 1.0e-12) {
        maxX = std::max(center.x + arc->getRadius(), maxX);
    }

    // check for bottom limit:
    if ((a1 < (M_PI_2 * 3) && a2 > (M_PI_2 * 3)) ||
        (a1 > a2 - 1.0e-12 && a2 > (M_PI_2 * 3)) ||
        (a1 > a2 - 1.0e-12 && a1 < (M_PI_2 * 3))) {

        minY = std::min(center.y - arc->getRadius(), minY);
    }

    // check for top limit:
    if ((a1 < M_PI_2 && a2 > M_PI_2) || (a1 > a2 - 1.0e-12 && a2 > M_PI_2) ||
        (a1 > a2 - 1.0e-12 && a1 < M_PI_2)) {

        maxY = std::max(center.y + arc->getRadius(), maxY);
    }

    minV = Vec2d(minX, minY);
    maxV = Vec2d(maxX, maxY);
    return BBox(minV, maxV);
}

shape::BBox cada_getCircleBoundingBox(const shape::Shape *shape)
{
    const Circle *circle = dynamic_cast<const Circle *>(shape);
    assert(circle);

    return BBox(
        circle->getCenter() - Vec2d(circle->getRadius(), circle->getRadius()),
        circle->getCenter() + Vec2d(circle->getRadius(), circle->getRadius()));
}

shape::BBox cada_getEllipseBoundingBox(const shape::Shape *shape)
{
    const Ellipse *ellipse = dynamic_cast<const Ellipse *>(shape);
    assert(ellipse);

    double radius1 = ellipse->getMajorRadius();
    double radius2 = ellipse->getMinorRadius();
    double angle = ellipse->getAngle();
    double a1 = (ellipse->isReversed() ? ellipse->getEndParam()
                                       : ellipse->getStartParam());
    double a2 = (ellipse->isReversed() ? ellipse->getStartParam()
                                       : ellipse->getEndParam());

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
        vp.set(center.x + radius1 * cos(a), center.y + radius2 * sin(a));
        vp.rotate(angle, center);

        minX = std::min(minX, vp.x);
        minY = std::min(minY, vp.y);
        maxX = std::max(maxX, vp.x);
        maxY = std::max(maxY, vp.y);

        a += 0.03;
    } while (Math::isAngleBetween(a, a1, a2, false) && a < 4 * M_PI);

    return BBox(Vec2d(minX, minY), Vec2d(maxX, maxY));
}

shape::BBox cada_getXLineBoundingBox(const shape::Shape *shape)
{
    const XLine *xline = dynamic_cast<const XLine *>(shape);
    assert(xline);

    return BBox(
        Vec2d::getMinimum(xline->getStartPoint(), xline->getEndPoint()),
        Vec2d::getMaximum(xline->getStartPoint(), xline->getEndPoint()));
}

shape::BBox cada_getPolylineBoundingBox(const shape::Polyline* pline)
{
    assert(pline);
    BBox ret;

    if (pline->hasWidths()) {
        auto&& outline = pline->getOutline();
        for (int i = 0; i < outline.size(); i++) {
            BBox bb = outline[i]->getBoundingBox();
            ret.growToInclude(bb);
        }
        return ret;
    }

    if (pline->countVertices() == 1) {
        ret = BBox(pline->getVertexAt(0), pline->getVertexAt(0));
    }

    auto&& sub = pline->getExploded();
    for (auto&& ss : sub)
    {
        BBox bb = ss->getBoundingBox();
        ret.growToInclude(bb);
    }

    return ret;
}
     

shape::BBox cada_getBoundingBox(const shape::Shape *shape)
{
    assert(shape);
    switch (shape->getShapeType()) {
    case NS::Point: {
        auto point = dynamic_cast<const Point *>(shape);
        return BBox(point->getPosition(), point->getPosition());
    }
    case NS::Line: {
        auto line = dynamic_cast<const Line *>(shape);
        return BBox(
            Vec2d::getMinimum(line->getStartPoint(), line->getEndPoint()),
            Vec2d::getMaximum(line->getStartPoint(), line->getEndPoint()));
    }
    case NS::Arc: {
        return cada_getArcBoundingBox(shape);
    }
    case NS::Circle: {
        return cada_getCircleBoundingBox(shape);
    }
    case NS::Ellipse: {
        return cada_getEllipseBoundingBox(shape);
    }
    case NS::XLine:
    case NS::Ray: {
        return cada_getXLineBoundingBox(shape);
    }
    case NS::Polyline: {
        return cada_getPolylineBoundingBox(dynamic_cast<const Polyline*>(shape));
    }
    case NS::BSpline:
        break;
    default:
        break;
    }
    return shape::BBox();
}

} // namespace algorithm
} // namespace cada
