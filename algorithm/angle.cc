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

double cada_getAngleAt(const shape::Shape *shape, double distance,
                       NS::From from);
double cada_polyline_getAngleAt(const shape::Polyline *polyline,
                                double distance, NS::From from);

/* ---------------------------------- impl ---------------------------------- */

double cada_getAngleAt(const shape::Shape *shape, double distance,
                       NS::From from)
{
    assert(shape != nullptr);
    switch (shape->getShapeType()) {
    case NS::Point:
        return std::numeric_limits<double>::quiet_NaN();
    case NS::Line: {
        auto l = dynamic_cast<const Line *>(shape);
        return l->getAngle();
    }
    case NS::Arc: {
        auto a = dynamic_cast<const Arc *>(shape);
        std::vector<Vec2d> points =
            shape->getPointsWithDistanceToEnd(distance, from);
        if (points.size() != 1) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        return a->getCenter().getAngleTo(points[0]) +
               (a->isReversed() ? -M_PI / 2 : M_PI / 2);
    }
    case NS::Circle:
        return std::numeric_limits<double>::quiet_NaN();
    case NS::Ellipse:
        // TODO
        break;
    case NS::XLine:
    case NS::Ray: {
        auto xl = dynamic_cast<const XLine *>(shape);
        return xl->getAngle();
    }
    case NS::Polyline:
        return cada_polyline_getAngleAt(dynamic_cast<const Polyline *>(shape),
                                        distance, from);
    case NS::BSpline:
        // TODO
    default:
        break;
    }
    return 0.0;
}

double cada_polyline_getAngleAt(const shape::Polyline *polyline,
                                double distance, NS::From from)
{
    auto &&sub = polyline->getExploded();

    if (from & NS::AlongPolyline) {
        double remainingDist;
        double len;

        if (from & NS::FromStart) {
            remainingDist = distance;
            for (int i = 0; i < sub.size(); i++) {
                len = sub[i]->getLength();
                if (remainingDist > len) {
                    remainingDist -= len;
                }
                else {
                    return sub[i]->getAngleAt(remainingDist, NS::FromStart);
                }
            }
        }

        if (from & NS::FromEnd) {
            remainingDist = distance;
            for (int i = sub.size() - 1; i >= 0; i--) {
                len = sub[i]->getLength();
                if (remainingDist > len) {
                    remainingDist -= len;
                }
                else {
                    return sub[i]->getAngleAt(remainingDist, NS::FromEnd);
                }
            }
        }
    }

    return std::numeric_limits<double>::quiet_NaN();
}


} // namespace algorithm
} // namespace cada
