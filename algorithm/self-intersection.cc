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
#include <cada_excption.h>
#include <assert.h>

using namespace cada::shape;

namespace cada {
namespace algorithm {

std::vector<shape::Vec2d>
cada_get_polyline_self_intersection_points(const shape::Polyline *poly,
                                           double tolerance)
{
    assert(poly);
    std::vector<shape::Vec2d> ret;

    bool cl = poly->isGeometricallyClosed();

    auto &&segments = poly->getExploded();
    for (size_t i = 0; i < segments.size(); ++i) {
        auto &&segment = poly->getSegmentAt(i);
        for (size_t j = i + 1; j < segments.size(); ++j) {
            auto &&other_segment = poly->getSegmentAt(j);
            std::vector<Vec2d> ips =
                segment->getIntersectionPoints(other_segment.release());
            for (size_t n = 0; n < ips.size(); ++n) {
                Vec2d ip = ips[n];
                if (j == i + 1 &&
                    ip.equalsFuzzy(segment->getEndPoint(), tolerance)) {
                    continue;
                }

                if (cl) {
                    if (i == 0 && j == segments.size() - 1 &&
                        ip.equalsFuzzy(segment->getStartPoint(), tolerance)) {
                        continue;
                    }
                }
                ret.push_back(ip);
            }
        }
    }
    return ret;
}

std::vector<shape::Vec2d>
cada_get_polyline_self_intersection_points(const shape::BSpline *spline,
                                           double tolerance)
{
    return std::vector<shape::Vec2d>();
}

std::vector<shape::Vec2d>
cada_getSelfIntersectionPoints(const shape::Shape *shape, double tolerance)
{
    assert(shape);
    if (shape->getShapeType() == NS::Polyline) {
        return cada_get_polyline_self_intersection_points(
            dynamic_cast<const shape::Polyline *>(shape), tolerance);
    }
    else if (shape->getShapeType() == NS::BSpline) {
        return cada_get_polyline_self_intersection_points(
            dynamic_cast<const shape::BSpline *>(shape), tolerance);
    }
    else {
        throw UnsupportedOperationException();
    }
}

} // namespace algorithm
} // namespace cada