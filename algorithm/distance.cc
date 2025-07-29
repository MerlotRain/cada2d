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
#include <numeric>
#include <limits>
#include <cassert>

using namespace cada::shape;

namespace cada {
namespace algorithm {

/* -------------------------------- functions ------------------------------- */

double cada_getDistanceTo(const shape::Shape *shape, const shape::Vec2d &point,
                          bool limited, double strictRange);
std::vector<shape::Vec2d>
cada_getPointsWithDistanceToEnd(const shape::Shape *shape, double distance,
                                int from);
double cada_getDistanceFromStart(const shape::Shape *shape,
                                 const shape::Vec2d &p);
std::vector<double> cada_getDistancesFromStart(const shape::Shape *shape,
                                               const shape::Vec2d &p);

/* ------------------------- PointsWithDistanceToEnd ------------------------ */
std::vector<shape::Vec2d>
cada_arc_getPointsWithDistanceToEnd(const shape::Arc *a, double distance,
                                    int from);
std::vector<shape::Vec2d>
cada_polyline_getPointsWithDistanceToEnd(const shape::Polyline *polyline,
                                         double distance, int from);

/* ---------------------------------- impls --------------------------------- */

double cada_getDistanceTo(const shape::Shape *shape, const shape::Vec2d &point,
                          bool limited, double strictRange)
{
    assert(shape);
    Vec2d v = shape->getVectorTo(point, limited, strictRange);
    if (v.isValid()) {
        return v.getMagnitude();
    }
    return std::numeric_limits<double>::quiet_NaN();
}

std::vector<shape::Vec2d>
cada_getPointsWithDistanceToEnd(const shape::Shape *shape, double distance,
                                int from)
{
    assert(shape);

    switch (shape->getShapeType()) {
    case NS::Point:
        return std::vector<shape::Vec2d>();
    case NS::Line: {
        auto l = dynamic_cast<const Line *>(shape);
        std::vector<shape::Vec2d> ret;

        if (from & NS::FromStart) {
            Vec2d normalStart =
                (l->getEndPoint() - l->getStartPoint()).getNormalized();
            ret.emplace_back(l->getStartPoint() + normalStart * distance);
        }
        if (from & NS::FromEnd) {
            Vec2d normalEnd =
                (l->getStartPoint() - l->getEndPoint()).getNormalized();
            ret.emplace_back(l->getEndPoint() + normalEnd * distance);
        }
        return ret;
    }
    case NS::Arc:
        return cada_arc_getPointsWithDistanceToEnd(
            dynamic_cast<const Arc *>(shape), distance, from);
    case NS::Circle:
        return std::vector<shape::Vec2d>();
    case NS::Ellipse:
        return std::vector<shape::Vec2d>();
    case NS::XLine:
        return std::vector<shape::Vec2d>();
    case NS::Ray: {
        auto r = dynamic_cast<const Ray *>(shape);
        std::vector<Vec2d> ret;
        double a1 = r->getAngle();

        Vec2d dv;
        dv.setPolar(distance, a1);

        if (from & NS::FromStart) {
            ret.emplace_back(r->getBasePoint() + dv);
        }

        return ret;
    }
    case NS::Polyline:
        return cada_polyline_getPointsWithDistanceToEnd(
            dynamic_cast<const Polyline *>(shape), distance, from);
    case NS::Spline:
    default:
        break;
    }

    return std::vector<shape::Vec2d>();
}

double cada_getDistanceFromStart(const shape::Shape *shape,
                                 const shape::Vec2d &p)
{
    assert(shape);
    switch (shape->getShapeType()) {
    case NS::Line: {
        auto &&l = dynamic_cast<const shape::Line *>(shape);

        double ret = l->getStartPoint().getDistanceTo(p);

        Vec2d p2 = l->getClosestPointOnShape(p, false);
        double angle = l->getStartPoint().getAngleTo(p2);
        if (Math::isSameDirection(l->getAngle(), angle, M_PI / 2)) {
            return ret;
        }
        else {
            return -ret;
        }
    }
    case NS::Arc: {
        auto a = dynamic_cast<const shape::Arc *>(shape);
        double a1 = a->getStartAngle();
        double ap = a->getCenter().getAngleTo(p);
        if (a->isReversed()) {
            return Math::getAngleDifference(ap, a1) * a->getRadius();
        }
        else {
            return Math::getAngleDifference(a1, ap) * a->getRadius();
        }
    }
    case NS::Ellipse: {
        std::vector<double> res = shape->getDistancesFromStart(p);
        if (res.empty()) {
            return std::numeric_limits<double>::max();
        }
        return res.front();
    }
    case NS::XLine:
    case NS::Ray: {
        auto &&xline = dynamic_cast<const XLine *>(shape);
        double ret = xline->getBasePoint().getDistanceTo(p);

        Vec2d p2 = xline->getClosestPointOnShape(p, false);
        double angle = xline->getBasePoint().getAngleTo(p2);
        if (Math::isSameDirection(xline->getAngle(), angle, M_PI / 2)) {
            return ret;
        }
        else {
            return -ret;
        }
    }
    default:
        break;
    }

    return std::numeric_limits<double>::max();
}

std::vector<double> cada_getDistancesFromStart(const shape::Shape *shape,
                                               const shape::Vec2d &p)
{
    assert(shape);
    std::vector<double> ret;
    if (shape->getShapeType() == NS::Polyline) {
        auto &&pline = dynamic_cast<const shape::Polyline *>(shape);
        double len = 0.0;
        for (int i = 0; i < pline->countSegments(); i++) {
            auto &&segment = pline->getSegmentAt(i);
            if (segment->getDistanceTo(p) < 0.0001) {
                ret.push_back(len + segment->getDistanceFromStart(p));
            }
            len += segment->getLength();
        }

        // point is not on polyline, return distance to point closest to
        // position:
        if (ret.empty()) {
            ret.push_back(pline->getLengthTo(p, true));
        }

        return ret;
    }
    else {
        ret.push_back(shape->getDistanceFromStart(p));
    }
    return ret;
}

std::vector<shape::Vec2d>
cada_arc_getPointsWithDistanceToEnd(const shape::Arc *a, double distance,
                                    int from)
{
    std::vector<Vec2d> ret;

    if (a->getRadius() < NS::PointTolerance) {
        return ret;
    }

    double a1;
    double a2;
    Vec2d p;
    double aDist = distance / a->getRadius();

    if (a->isReversed()) {
        a1 = a->getStartAngle() - aDist;
        a2 = a->getEndAngle() + aDist;
    }
    else {
        a1 = a->getStartAngle() + aDist;
        a2 = a->getEndAngle() - aDist;
    }

    if (from & NS::FromStart) {
        p.setPolar(a->getRadius(), a1);
        p += a->getCenter();
        ret.emplace_back(p);
    }

    if (from & NS::FromEnd) {
        p.setPolar(a->getRadius(), a2);
        p += a->getCenter();
        ret.emplace_back(p);
    }

    return ret;
}

std::vector<shape::Vec2d>
cada_polyline_getPointsWithDistanceToEnd(const shape::Polyline *polyline,
                                         double distance, int from)
{
    std::vector<shape::Vec2d> ret;

    auto &&sub = polyline->getExploded();

    if (sub.empty()) {
        return ret;
    }

    if (from & NS::AlongPolyline) {
        double remainingDist;
        double len;

        if (from & NS::FromStart) {
            if (distance < 0.0) {
                auto sub_vec = sub.front()->getPointsWithDistanceToEnd(
                    distance, NS::FromStart);
                // extend at start:
                ret.insert(ret.end(), sub_vec.begin(), sub_vec.end());
            }
            else {
                remainingDist = distance;
                for (int i = 0; i < sub.size(); i++) {
                    len = sub[i]->getLength();
                    if (remainingDist > len) {
                        remainingDist -= len;
                    }
                    else {
                        auto sub_vec = sub[i]->getPointsWithDistanceToEnd(
                            remainingDist, NS::FromStart);
                        ret.insert(ret.end(), sub_vec.begin(), sub_vec.end());
                        break;
                    }
                }
            }
        }

        if (from & NS::FromEnd) {
            if (distance < 0.0) {
                // extend at end:
                auto sub_vec = sub.back()->getPointsWithDistanceToEnd(
                    distance, NS::FromEnd);
                ret.insert(ret.end(), sub_vec.begin(), sub_vec.end());
            }
            else {
                remainingDist = distance;
                for (int i = sub.size() - 1; i >= 0; i--) {
                    len = sub[i]->getLength();
                    if (remainingDist > len) {
                        remainingDist -= len;
                    }
                    else {
                        auto sub_vec = sub[i]->getPointsWithDistanceToEnd(
                            remainingDist, NS::FromEnd);
                        ret.insert(ret.end(), sub_vec.begin(), sub_vec.end());
                        break;
                    }
                }
            }
        }
    }
    else {

        for (auto &it : sub) {
            auto &&it_ret = it->getPointsWithDistanceToEnd(distance, from);
            ret.insert(ret.end(), it_ret.begin(), it_ret.end());
        }
    }

    return ret;
}

} // namespace algorithm
} // namespace cada
