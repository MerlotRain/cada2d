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

extern bool cada_move(shape::Shape *shape, const shape::Vec2d &offset);
extern bool cada_arc_move(shape::Arc* a, const shape::Vec2d &offset);

bool cada_line_stretch(shape::Line* l, const shape::Polyline* area, const shape::Vec2d& offset)
{
    assert(l);
    assert(area);

    bool ret = false;

    if (area->contains(l->getStartPoint(), true)) {
        l->setStartPoint(l->getStartPoint() + offset);
        ret = true;
    }
    if (area->contains(l->getEndPoint(), true)) {
        l->setEndPoint(l->getEndPoint() + offset);
        ret = true;
    }

    return ret;
}

bool cada_arc_stretch(shape::Arc* a, const shape::Polyline* area, const shape::Vec2d &offset)
{
    assert(a);
    assert(area);
    bool ret = false;

    if (area->contains(a->getStartPoint(), true) && area->contains(a->getEndPoint(), true)) {
        return cada_arc_move(a, offset);
    }

    if (area->contains(a->getStartPoint(), true)) {
        a->moveStartPoint(a->getStartPoint() + offset);
        ret = true;
    }
    else if (area->contains(a->getEndPoint(), true)) {
        a->moveEndPoint(a->getEndPoint() + offset);
        ret = true;
    }

    return ret;
}



bool cada_stretch(shape::Shape *shape, std::vector<shape::Vec2d> &&vertex,
                  const shape::Vec2d &offset)
{
    auto area =
        ShapeFactory::instance()->createPolyline(std::move(vertex), true);

    assert(shape);
    switch (shape->getShapeType()) {
    case NS::Line:
    {
        auto l = dynamic_cast<shape::Line*>(shape);
        return cada_line_stretch(l, area.release(), offset);
    }
    case NS::Arc:
    {
        auto a = dynamic_cast<shape::Arc*>(shape);
        return cada_arc_stretch(a, area.release(), offset);
    }
    case NS::Ray:
    {
        auto r = dynamic_cast<shape::Ray*>(shape);
        bool ret = false;

        if(area->contains(r->getBasePoint(), true)) {
            r->setBasePoint(r->getBasePoint() + offset);
            ret = true;
        }
        return ret;
    }
    case NS::Polyline:
    {
        auto poly = dynamic_cast<shape::Polyline*>(shape);
        auto& vertices = poly->getVertices();
        for(auto& v : vertices)
        {
            if(area->contains(v, true))
            {
                v.move(offset);
            }
        }
        return true;
    }
    case NS::BSpline:
        break;
    default: {
        if (area->containsShape(shape)) {
            return cada_move(shape, offset);
        }
        break;
    }
    };
    return false;
}

} // namespace algorithm
} // namespace cada
