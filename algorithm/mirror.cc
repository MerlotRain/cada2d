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

/* -------------------------------- functions ------------------------------- */
bool cada_mirror(shape::Shape *shape, const shape::Vec2d &v1,
                 const shape::Vec2d &v2);

bool cada_arc_mirror(shape::Arc *Arc, const shape::Vec2d &v1,
                     const shape::Vec2d &v2);
bool cada_ellipse_mirror(shape::Ellipse *e, const shape::Vec2d &v1,
                         const shape::Vec2d &v2);
/* ---------------------------------- impl ---------------------------------- */

bool cada_mirror(shape::Shape *shape, const shape::Vec2d &v1,
                 const shape::Vec2d &v2)
{
    assert(shape);

    switch (shape->getShapeType()) {
    case NS::Point: {
        auto p = dynamic_cast<Point *>(shape);
        p->setPosition(p->getPosition().mirror(v1, v2));
        return true;
    }
    case NS::Line: {
        auto l = dynamic_cast<Line *>(shape);
        l->setStartPoint(l->getStartPoint().mirror(v1, v2));
        l->setEndPoint(l->getEndPoint().mirror(v1, v2));
        return true;
    }
    case NS::Arc:
        return cada_arc_mirror(dynamic_cast<Arc *>(shape), v1, v2);
    case NS::Circle: {
        auto c = dynamic_cast<Circle *>(shape);
        c->setCenter(c->getCenter().mirror(v1, v2));
        return true;
    }
    case NS::Ellipse: {
        return cada_ellipse_mirror(dynamic_cast<Ellipse *>(shape), v1, v2);
    }
    case NS::XLine:
    case NS::Ray: {
        auto xl = dynamic_cast<XLine *>(shape);
        Vec2d sp = xl->getEndPoint();
        Vec2d basePoint = xl->getBasePoint();
        basePoint.mirror(v1, v2);
        sp.mirror(v1, v2);

        xl->setBasePoint(basePoint);
        xl->setDirectionVector(sp - basePoint);
        return true;
    }
    case NS::Polyline: {
        auto pl = dynamic_cast<Polyline *>(shape);
        auto &vertices = pl->getVertices();
        auto &bulges = pl->getBulges();

        for (auto &v : vertices) {
            v.mirror(v1, v2);
        }
        for (auto &b : bulges) {
            b *= -1;
        }
        return true;
    }
    case NS::Spline:
        break;
    default:
        break;
    }
    return false;
}

bool cada_arc_mirror(shape::Arc *arc, const shape::Vec2d &v1,
                     const shape::Vec2d &v2)
{
    Vec2d center = arc->getCenter();
    double startAngle = arc->getStartAngle();
    double endAngle = arc->getEndAngle();
    center.mirror(v1, v2);

    if (arc->isFullCircle()) {
        arc->setCenter(center);
        return true;
    }

    arc->setReversed(!arc->isReversed());
    Vec2d v;
    v.setPolar(1.0, startAngle);
    v.mirror(Vec2d(0.0, 0.0), v2 - v1);
    startAngle = v.getAngle();

    v.setPolar(1.0, endAngle);
    v.mirror(Vec2d(0.0, 0.0), v2 - v1);
    endAngle = v.getAngle();

    arc->setCenter(center);
    arc->setStartAngle(startAngle);
    arc->setEndAngle(endAngle);

    return true;
}

bool cada_ellipse_mirror(shape::Ellipse *e, const shape::Vec2d &v1,
                         const shape::Vec2d &v2)
{
    Vec2d center = e->getCenter();
    Vec2d majorPoint = e->getMajorPoint();
    bool reversed = e->isReversed();

    Vec2d mp = center + majorPoint;
    Vec2d sp = e->getStartPoint();
    Vec2d ep = e->getEndPoint();

    center.mirror(v1, v2);
    mp.mirror(v1, v2);

    majorPoint = mp - center;

    if (!e->isFullEllipse()) {
        reversed = (!reversed);

        sp.mirror(v1, v2);
        e->setStartParam(e->getParamTo(sp));

        ep.mirror(v1, v2);
        e->setEndParam(e->getParamTo(ep));
    }

    e->setCenter(center);
    e->setMajorPoint(majorPoint);
    e->setReversed(reversed);

    return true;
}

} // namespace algorithm
} // namespace cada
