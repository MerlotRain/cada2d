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

#include "cada_ns.h"
#include <vector>
#include <memory>

namespace cada {
namespace shape {
class Shape;
struct Vec2d;
struct BBox;
} // namespace shape
} // namespace cada

// clang-format off

namespace cada {
namespace algorithm {

extern shape::BBox cada_getBoundingBox(const shape::Shape *shape);
extern double cada_getLength(const shape::Shape *shape);
extern shape::Vec2d cada_getVectorTo(const shape::Shape *shape, const shape::Vec2d &point, bool limited, double strictRange);
extern bool cada_equals(const shape::Shape *shape, const shape::Shape *other, double tolerance);
extern double cada_getDistanceTo(const shape::Shape *shape, const shape::Vec2d &point, bool limited, double strictRange);
extern std::vector<shape::Vec2d> cada_getPointsWithDistanceToEnd(const shape::Shape *shape, double distance, int from);
extern shape::Vec2d cada_getPointOnShape(const shape::Shape *shape);
extern double cada_getAngleAt(const shape::Shape *shape, double distance, NS::From from);
extern std::vector<shape::Vec2d> cada_getIntersectionPoints(const shape::Shape *shape, const shape::Shape *other, bool limited, bool same, bool force);
extern std::vector<shape::Vec2d> cada_getSelfIntersectionPoints(const shape::Shape *shape, double tolerance);
extern double cada_getDirection1(const shape::Shape *shape);
extern double cada_getDirection2(const shape::Shape *shape);
extern cada::NS::Side cada_getSideOfPoint(const shape::Shape *shape, const shape::Vec2d &point);
extern bool cada_reverse(shape::Shape *shape);
extern bool cada_trimStartPoint(shape::Shape *shape, const shape::Vec2d &trimPoint, const shape::Vec2d &clickPoint, bool extend);
extern bool cada_trimEndPoint(shape::Shape *shape, const shape::Vec2d &trimPoint, const shape::Vec2d &clickPoint, bool extend);
extern NS::Ending cada_getTrimEnd(shape::Shape *shape, const shape::Vec2d &trimPoint, const shape::Vec2d &clickPoint);
extern double cada_getDistanceFromStart(const shape::Shape *shape, const shape::Vec2d &p);
extern std::vector<double> cada_getDistancesFromStart(const shape::Shape *shape, const shape::Vec2d &p);
extern bool cada_move(shape::Shape *shape, const shape::Vec2d &offset);
extern bool cada_rotate(shape::Shape *shape, double rotation, const shape::Vec2d &center);
extern bool cada_scale(shape::Shape *shape, const shape::Vec2d &scaleFactors, const shape::Vec2d &center);
extern bool cada_mirror(shape::Shape *shape, const shape::Vec2d &v1, const shape::Vec2d &v2);
extern bool cada_stretch(shape::Shape *shape, std::vector<shape::Vec2d> &&vertex, const shape::Vec2d &offset);
extern std::vector<std::unique_ptr<shape::Shape>> cada_getOffsetShapes(const shape::Shape *shape, double distance, int number, NS::Side side, NS::JoinType join, const shape::Vec2d &position);
extern std::vector<std::unique_ptr<shape::Shape>> cada_splitAt(const shape::Shape *shape, const std::vector<shape::Vec2d> &points);

extern bool cada_polylineContains(const shape::Polyline *pline, const shape::Vec2d &pt);
} // namespace algorithm
} // namespace cada

// clang-format on