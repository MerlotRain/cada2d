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

std::vector<std::unique_ptr<shape::Shape>>
cada_split_arc(const shape::Arc *a, const std::vector<shape::Vec2d> &points)
{
    assert(a);
    std::vector<std::unique_ptr<shape::Shape>> res;

    if (a->isReversed()) {
        auto arc = a->clone();
        arc->reverse();
        res = cada_split_arc(arc.release(), points);
        for (auto &r : res) {
            r->reverse();
        }
        return res;
    }

    Vec2d start_point = a->getStartPoint();
    Vec2d end_point = a->getEndPoint();

    std::vector<Vec2d> sorted_points =
        Vec2d::getSortedByAngle(points, a->getCenter(), a->getStartAngle());

    if (!start_point.equalsFuzzy(sorted_points[0])) {
        sorted_points.insert(sorted_points.begin(), start_point);
    }
    if (!end_point.equalsFuzzy(sorted_points.back())) {
        sorted_points.push_back(end_point);
    }

    for (size_t i = 0; i < sorted_points.size() - 1; i++) {
        if (sorted_points[i].equalsFuzzy(sorted_points[i + 1])) {
            continue;
        }

        auto seg = a->clone();
        double a1 = a->getCenter().getAngleTo(sorted_points[i]);
        double a2 = a->getCenter().getAngleTo(sorted_points[i + 1]);
        if (fabs(Math::getAngleDifference180(a1, a2) * a->getRadius()) <
            0.001) {
            continue;
        }
        seg->setStartAngle(a1);
        seg->setEndAngle(a2);
        res.emplace_back(std::move(seg));
    }

    return res;
}

std::vector<std::unique_ptr<shape::Shape>>
cada_split_circle(const shape::Circle *c,
                  const std::vector<shape::Vec2d> &points)
{
    assert(c);

    std::vector<std::unique_ptr<shape::Shape>> ret;

    double refAngle = c->getCenter().getAngleTo(points[0]);
    Vec2d start_point;
    Vec2d end_point;

    start_point = end_point =
        c->getCenter() + Vec2d::createPolar(c->getRadius(), refAngle);
    std::vector<Vec2d> sorted_points =
        Vec2d::getSortedByAngle(points, c->getCenter(), refAngle);

    if (!start_point.equalsFuzzy(sorted_points[0])) {
        sorted_points.insert(sorted_points.begin(), start_point);
    }
    if (!end_point.equalsFuzzy(sorted_points.back())) {
        sorted_points.push_back(end_point);
    }
    for (size_t i = 0; i < sorted_points.size() - 1; i++) {
        if (sorted_points[i].equalsFuzzy(sorted_points[i + 1])) {
            continue;
        }

        ret.emplace_back(std::move(ShapeFactory::instance()->createArc(
            c->getCenter(), c->getRadius(),
            c->getCenter().getAngleTo(sorted_points[i]),
            c->getCenter().getAngleTo(sorted_points[i + 1]), false)));
    }

    return ret;
}

std::vector<std::unique_ptr<shape::Shape>>
cada_split_ellipse(const shape::Ellipse *e,
                   const std::vector<shape::Vec2d> &points)
{
    assert(e);
    std::vector<std::unique_ptr<shape::Shape>> ret;

    if (e->isReversed()) {
        auto ellipse = e->clone();
        ellipse->reverse();
        ret = cada_split_ellipse(ellipse.release(), points);
        for (auto &item : ret) {
            item->reverse();
        }
        return ret;
    }
    Vec2d startPoint = e->getStartPoint();
    Vec2d endPoint = e->getEndPoint();

    std::vector<Vec2d> sortedPoints = Vec2d::getSortedByAngle(
        points, e->getCenter(), e->getCenter().getAngleTo(startPoint));

    if (!startPoint.equalsFuzzy(sortedPoints[0])) {
        sortedPoints.insert(sortedPoints.begin(), startPoint);
    }
    if (!endPoint.equalsFuzzy(sortedPoints.back())) {
        sortedPoints.push_back(endPoint);
    }
    for (size_t i = 0; i < sortedPoints.size() - 1; i++) {
        if (sortedPoints[i].equalsFuzzy(sortedPoints[i + 1])) {
            continue;
        }

        auto seg = e->clone();
        seg->setStartParam(seg->getParamTo(sortedPoints[i]));
        seg->setEndParam(seg->getParamTo(sortedPoints[i + 1]));
        ret.emplace_back(std::move(seg));
    }

    return ret;
}

std::vector<std::unique_ptr<shape::Shape>>
cada_split_xline(const shape::XLine *xl,
                 const std::vector<shape::Vec2d> &points)
{
    assert(xl);
    std::vector<std::unique_ptr<shape::Shape>> ret;

    Vec2d directionVector = xl->getDirectionVector();
    std::vector<Vec2d> sortedPoints = Vec2d::getSortedByDistance(
        points, xl->getBasePoint() - directionVector * 1e9);

    ret.emplace_back(
        ShapeFactory::instance()->createRay(sortedPoints[0], -directionVector));

    for (size_t i = 0; i < sortedPoints.size() - 1; i++) {
        if (sortedPoints[i].equalsFuzzy(sortedPoints[i + 1])) {
            continue;
        }

        ret.emplace_back(ShapeFactory::instance()->createLine(
            sortedPoints[i], sortedPoints[i + 1]));
    }

    ret.emplace_back(ShapeFactory::instance()->createRay(sortedPoints.back(),
                                                         directionVector));

    return ret;
}

std::vector<std::unique_ptr<shape::Shape>>
cada_split_ray(const shape::Ray *r, const std::vector<shape::Vec2d> &points)
{
    assert(r);
    std::vector<std::unique_ptr<shape::Shape>> ret;

    Vec2d basePoint = r->getBasePoint();
    Vec2d directionVector = r->getDirectionVector();
    std::vector<Vec2d> sortedPoints =
        Vec2d::getSortedByDistance(points, basePoint);

    if (!basePoint.equalsFuzzy(sortedPoints[0])) {
        sortedPoints.insert(sortedPoints.begin(), basePoint);
    }

    for (size_t i = 0; i < sortedPoints.size() - 1; i++) {
        if (sortedPoints[i].equalsFuzzy(sortedPoints[i + 1])) {
            continue;
        }

        ret.emplace_back(ShapeFactory::instance()->createLine(
            sortedPoints[i], sortedPoints[i + 1]));
    }

    ret.emplace_back(ShapeFactory::instance()->createRay(sortedPoints.back(),
                                                         directionVector));

    return ret;
}

std::vector<std::unique_ptr<shape::Shape>>
cada_split_line(const shape::Line *l, const std::vector<shape::Vec2d> &points)
{
    assert(l);
    std::vector<std::unique_ptr<shape::Shape>> ret;

    Vec2d startPoint = l->getStartPoint();
    Vec2d endPoint = l->getEndPoint();
    std::vector<Vec2d> sortedPoints =
        Vec2d::getSortedByDistance(points, startPoint);

    if (!startPoint.equalsFuzzy(sortedPoints[0])) {
        sortedPoints.insert(sortedPoints.begin(), startPoint);
    }
    if (!endPoint.equalsFuzzy(sortedPoints.back())) {
        sortedPoints.push_back(endPoint);
    }

    for (size_t i = 0; i < sortedPoints.size() - 1; i++) {
        if (sortedPoints[i].equalsFuzzy(sortedPoints[i + 1])) {
            continue;
        }
        ret.emplace_back(ShapeFactory::instance()->createLine(
            sortedPoints[i], sortedPoints[i + 1]));
    }

    return ret;
}

std::vector<std::unique_ptr<shape::Shape>>
cada_splitAt(const shape::Shape *shape, const std::vector<shape::Vec2d> &points)
{
    assert(shape);
    if (points.size() == 0) {
        std::vector<std::unique_ptr<shape::Shape>> res;
        res.emplace_back(shape->clone());
    }

    switch (shape->getShapeType()) {
    case NS::Line: {
        auto l = dynamic_cast<const shape::Line *>(shape);
        return cada_split_line(l, points);
    }
    case NS::Arc: {
        auto a = dynamic_cast<const shape::Arc *>(shape);
        return cada_split_arc(a, points);
    }
    case NS::Circle: {
        auto c = dynamic_cast<const shape::Circle *>(shape);
        return cada_split_circle(c, points);
    }
    case NS::Ellipse: {
        auto e = dynamic_cast<const shape::Ellipse *>(shape);
        return cada_split_ellipse(e, points);
    }
    case NS::XLine: {
        auto xl = dynamic_cast<const shape::XLine *>(shape);
        return cada_split_xline(xl, points);
    }
    case NS::Ray: {
        auto r = dynamic_cast<const shape::Ray *>(shape);
        return cada_split_ray(r, points);
    }
    case NS::Polyline:
        // TODO
        break;
    case NS::BSpline:
        // TODO
        break;
    default:
        break;
    }
    return std::vector<std::unique_ptr<shape::Shape>>();
}

} // namespace algorithm
} // namespace cada
