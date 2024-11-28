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

/* ---------------------------- extern functions ---------------------------- */
bool cada_trimStartPoint(shape::Shape *shape, const shape::Vec2d &trimPoint,
                         const shape::Vec2d &clickPoint, bool extend);
bool cada_trimEndPoint(shape::Shape *shape, const shape::Vec2d &trimPoint,
                       const shape::Vec2d &clickPoint, bool extend);
NS::Ending cada_getTrimEnd(shape::Shape *shape, const shape::Vec2d &trimPoint,
                           const shape::Vec2d &clickPoint);

NS::Ending cada_line_trimEnd(shape::Line *l, const shape::Vec2d &trimPoint,
                             const shape::Vec2d &clickPoint);

/* -------------------------- extern function impls ------------------------- */

bool cada_trimStartPoint(shape::Shape *shape, const shape::Vec2d &trimPoint,
                         const shape::Vec2d &clickPoint, bool extend)
{
    assert(shape);

    switch (shape->getShapeType()) {
    case NS::Line: {
        auto l = dynamic_cast<shape::Line *>(shape);
        Vec2d tp = l->getClosestPointOnShape(trimPoint, false);
        if (!tp.isValid()) {
            return false;
        }
        l->setStartPoint(tp);
        return true;
    }
    case NS::Arc: {
        auto a = dynamic_cast<shape::Arc *>(shape);
        a->setStartAngle(a->getCenter().getAngleTo(trimPoint));
        return true;
    }
    case NS::Ellipse: {
        auto e = dynamic_cast<shape::Ellipse *>(shape);
        e->setStartParam(e->getParamTo(trimPoint));
        return true;
    }
    case NS::XLine:
    case NS::Ray: {
        auto xl = dynamic_cast<shape::XLine *>(shape);
        Vec2d tp = xl->getClosestPointOnShape(trimPoint, false);
        if (!tp.isValid()) {
            return false;
        }
        xl->setBasePoint(tp);
        return true;
    }
    case NS::Polyline:
        // TODO
        break;
    case NS::Spline:
        // TODO
        break;
    default:
        break;
    };
    return false;
}

bool cada_trimEndPoint(shape::Shape *shape, const shape::Vec2d &trimPoint,
                       const shape::Vec2d &clickPoint, bool extend)
{
    assert(shape);

    switch (shape->getShapeType()) {
    case NS::Line: {
        auto l = dynamic_cast<shape::Line *>(shape);
        Vec2d tp = l->getClosestPointOnShape(trimPoint, false);
        if (!tp.isValid()) {
            return false;
        }
        l->setEndPoint(tp);
        return true;
    }
    case NS::Arc: {
        auto a = dynamic_cast<shape::Arc *>(shape);
        a->setEndAngle(a->getCenter().getAngleTo(trimPoint));
        return true;
    }
    case NS::Ellipse: {
        auto e = dynamic_cast<shape::Ellipse *>(shape);
        e->setEndParam(e->getParamTo(trimPoint));
        return true;
    }
    case NS::XLine:
    case NS::Ray: {
        auto xl = dynamic_cast<shape::XLine *>(shape);
        Vec2d tp = xl->getClosestPointOnShape(trimPoint, false);
        if (!tp.isValid()) {
            return false;
        }
        xl->setBasePoint(tp);
        xl->setDirectionVector(-(xl->getDirectionVector()));
        return true;
    }
    case NS::Polyline:
        // TODO
        break;
    case NS::Spline:
        // TODO
        break;
    default:
        break;
    };
    return false;
}

NS::Ending cada_getTrimEnd(shape::Shape *shape, const shape::Vec2d &trimPoint,
                           const shape::Vec2d &clickPoint)
{
    assert(shape);

    switch (shape->getShapeType()) {
    case NS::Line: {
        auto l = dynamic_cast<shape::Line *>(shape);
        return cada_line_trimEnd(l, trimPoint, clickPoint);
    }
    case NS::Arc: {
        auto a = dynamic_cast<shape::Arc *>(shape);
        double angleToTrimPoint = a->getCenter().getAngleTo(trimPoint);
        double angleToClickPoint = a->getCenter().getAngleTo(clickPoint);

        if (Math::getAngleDifference(angleToClickPoint, angleToTrimPoint) >
            M_PI) {
            if (a->isReversed()) {
                return NS::EndingEnd;
            }
            else {
                return NS::EndingStart;
            }
        }
        else {
            if (a->isReversed()) {
                return NS::EndingStart;
            }
            else {
                return NS::EndingEnd;
            }
        }
        break;
    }
    case NS::Ellipse: {
        auto e = dynamic_cast<shape::Ellipse *>(shape);
        double paramToClickPoint = e->getParamTo(clickPoint);
        double paramToTrimPoint = e->getParamTo(trimPoint);

        if (Math::getAngleDifference(paramToTrimPoint, paramToClickPoint) >
            M_PI) {
            return NS::EndingStart;
        }
        else {
            return NS::EndingEnd;
        }
        break;
    }
    case NS::XLine:
    case NS::Ray: {
        auto xl = dynamic_cast<shape::XLine *>(shape);
        auto l = xl->getLineShape();
        return cada_line_trimEnd(l.get(), trimPoint, clickPoint);
    }
    case NS::Polyline:
        // TODO
        break;
    case NS::Spline:
        // TODO
        break;
    default:
        break;
    };
    return NS::Ending::EndingNone;
}

NS::Ending cada_line_trimEnd(shape::Line *l, const shape::Vec2d &trimPoint,
                             const shape::Vec2d &clickPoint)
{
    assert(l);
    double lineAngle = l->getAngle();
    double angleToClickPoint = trimPoint.getAngleTo(clickPoint);
    double angleDifference = lineAngle - angleToClickPoint;

    if (angleDifference < 0.0) {
        angleDifference *= -1.0;
    }
    if (angleDifference > M_PI) {
        angleDifference = 2 * M_PI - angleDifference;
    }

    if (angleDifference < M_PI / 2.0) {
        return NS::EndingStart;
    }
    else {
        return NS::EndingEnd;
    }
}

} // namespace algorithm
} // namespace cada