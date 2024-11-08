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

#include "cada_algorithm.h"
#include "algorithm_help.h"
#include <assert.h>
#include <cmath>
#include <cada_math.h>

using namespace cada::shape;

namespace cada {
namespace algorithm {

/* ---------------------------- extern functions ---------------------------- */

/**
 * @brief Calculate the coordinates of points evenly distributed on a line
 * segment.
 *
 * @param v1
 * @param v2
 * @param n
 * @return std::vector<shape::Vec2d>
 */
std::vector<shape::Vec2d>
calculate_equidistant_points_on_line(const shape::Vec2d &v1,
                                     const shape::Vec2d &v2, size_t n);

/**
 * @brief Calculate the coordinates of a point matrix uniformly distributed
 * within a surface range.
 *
 * The first and third line segments are divided into \a col parts, and the
 * second and fourth line segments are divided into \a col parts, with all
 * points falling at the intersection of the lines formed by these symmetrical
 * points.
 *
 * @param v1
 * @param v2
 * @param v3
 * @param v4
 * @param col
 * @param row
 * @return std::vector<shape::Vec2d>
 */
std::vector<shape::Vec2d> calculate_equidistant_distribution_points_on_surface(
    const shape::Vec2d &v1, const shape::Vec2d &v2, const shape::Vec2d &v3,
    const shape::Vec2d &v4, size_t col, size_t row);

/**
 * @brief Calculate the angle bisector of the line between two line segments
 *
 * Two intersecting lines divide the plane into four regions, and the lines are
 * divided into four rays. Based on the two output position coordinates, two
 * rays are obtained, and the angle bisector is distributed within the rays.
 *
 * @param l1
 * @param l2
 * @param pos1
 * @param pos2
 * @param line_length
 * @param line_number
 * @return std::vector<std::unique_ptr<shape::Line>>
 */
std::vector<std::unique_ptr<shape::Line>>
calculate_angle_bisector_of_two_line_segments(
    const shape::Line *l1, const shape::Line *l2, const shape::Vec2d &pos1,
    const shape::Vec2d &pos2, double line_length, int line_number);

/**
 * @brief Calculate the common tangent of any two circles.
 *
 * @param c1
 * @param c2
 * @return std::vector<std::unique_ptr<shape::Line>>
 */
std::vector<std::unique_ptr<shape::Line>>
calculate_common_tangent_between_two_circles(const shape::Circle *c1,
                                             const shape::Circle *c2);

std::vector<std::unique_ptr<shape::Line>>
calculate_orthogonal_tangent_between_shape_and_line(const shape::Line *line,
                                                    const shape::Shape *shape);

std::vector<std::unique_ptr<shape::Shape>>
auto_split(const shape::Vec2d &pos, const shape::Shape *shape,
           const std::vector<shape::Shape *> &intersecting_shapes, bool extend);

bool break_out_gap(const shape::Vec2d &pos, const shape::Shape *shape,
                   std::vector<shape::Shape *> &additional);

std::vector<std::unique_ptr<shape::Shape>>
bevel_shapes(const shape::Shape *shap1, const shape::Vec2d &pos1,
             const shape::Shape *shape2, const shape::Vec2d &pos2, bool trim,
             double distance1, double distance2);

std::vector<std::unique_ptr<shape::Shape>>
round_shapes(const shape::Shape *shap1, const shape::Vec2d &pos1,
             const shape::Shape *shape2, const shape::Vec2d &pos2, bool trim,
             bool same_polyline, double radius, const shape::Vec2d &pos);

std::unique_ptr<shape::Shape> lengthen(const shape::Shape *shape,
                                       const shape::Vec2d &position,
                                       bool trim_start, double amount);

/* ------------------------- inner static functions ------------------------- */

static std::vector<Vec2d>
cada_get_intersection_points(const shape::Shape *shape,
                             const std::vector<shape::Shape *> &other_shapes,
                             bool on_shape, bool on_other_shape);

static std::unique_ptr<shape::Shape>
cada_get_closest_intersection_point_distances(
    shape::Shape *shape, const std::vector<Vec2d> &intersections,
    const Vec2d &pos, std::array<std::pair<Vec2d, double>, 2> &res);

/* ----------------------------- function impls ----------------------------- */

std::vector<shape::Vec2d>
calculate_equidistant_points_on_line(const shape::Vec2d &v1,
                                     const shape::Vec2d &v2, size_t n)
{
    std::vector<shape::Vec2d> rets;
    assert(n > 2);

    double dx = v2.x - v1.x;
    double dy = v2.y - v1.y;

    double step_x = dx / (n - 1);
    double step_y = dy / (n - 1);

    for (size_t i = 0; i < n; ++i) {
        rets.emplace_back(Vec2d(v1.x + step_x * i, v1.y + step_y * i));
    }
    return rets;
}

std::vector<shape::Vec2d> calculate_equidistant_distribution_points_on_surface(
    const shape::Vec2d &v1, const shape::Vec2d &v2, const shape::Vec2d &v3,
    const shape::Vec2d &v4, size_t col, size_t row)
{
    assert(col > 2);
    assert(row > 2);

    std::vector<Vec2d> line1_pts =
        calculate_equidistant_points_on_line(v1, v4, row);
    std::vector<Vec2d> line2_pts =
        calculate_equidistant_points_on_line(v2, v3, row);

    assert(line1_pts.size() != 0);
    assert(line2_pts.size() != 0);
    assert(line1_pts.size() == line2_pts.size());

    std::vector<Vec2d> rets;
    for (size_t i = 0; i < line1_pts.size(); ++i) {
        auto &p1 = line1_pts.at(i);
        auto &p2 = line2_pts.at(i);
        auto &&pps = calculate_equidistant_points_on_line(p1, p2, col);
        rets.insert(rets.end(), pps.begin(), pps.end());
    }
    return rets;
}

std::vector<std::unique_ptr<shape::Line>>
calculate_angle_bisector_of_two_line_segments(
    const shape::Line *l1, const shape::Line *l2, const shape::Vec2d &pos1,
    const shape::Vec2d &pos2, double line_length, int line_number)
{
    assert(l1);
    assert(l2);
    assert(line_number > 0);
    assert(line_length > 0);

    auto ips = l1->getIntersectionPoints(l2, false);
    if (ips.empty()) {
        return std::vector<std::unique_ptr<shape::Line>>();
    }

    std::vector<std::unique_ptr<shape::Line>> rets;

    Vec2d ip = ips[0];

    double angle1 = ip.getAngleTo(l1->getClosestPointOnShape(pos1));
    double angle2 = ip.getAngleTo(l2->getClosestPointOnShape(pos2));
    double angleDiff = Math::getAngleDifference(angle1, angle2);
    if (angleDiff > M_PI) {
        angleDiff = angleDiff - 2 * M_PI;
    }

    for (int i = 0; i < line_number; ++i) {
        double angle = angle1 + (angleDiff / (line_number + 1) * i);
        Vec2d vec;
        vec.setPolar(line_length, angle);
        rets.emplace_back(ShapeFactory::instance()->createLine(ip, ip + vec));
    }
    return rets;
}

std::vector<std::unique_ptr<shape::Line>>
calculate_common_tangent_between_two_circles(const shape::Circle *c1,
                                             const shape::Circle *c2)
{
    assert(c1);
    assert(c2);

    Vec2d offs1, offs2;

    Vec2d cc1 = c1->getCenter();
    Vec2d cc2 = c2->getCenter();
    double cr1 = c1->getRadius();
    double cr2 = c2->getRadius();

    double angle1 = cc1.getAngleTo(cc2);
    double dist1 = cc1.getDistanceTo(cc2);
    if (dist1 < 1.0e-6) {
        return std::vector<std::unique_ptr<shape::Line>>();
    }

    std::vector<std::unique_ptr<shape::Line>> tangents;

    // outer tangents:
    double dist2 = cr2 - cr1;
    if (dist1 > dist2) {
        double angle2 = asin(dist2 / dist1);
        double angt1 = angle1 + angle2 + M_PI / 2.0;
        double angt2 = angle1 - angle2 - M_PI / 2.0;
        offs1 = Vec2d();
        offs2 = Vec2d();

        offs1.setPolar(cr1, angt1);
        offs2.setPolar(cr2, angt1);

        tangents.emplace_back(
            ShapeFactory::instance()->createLine(cc1 + offs1, cc2 + offs2));

        offs1.setPolar(cr1, angt2);
        offs2.setPolar(cr2, angt2);

        tangents.emplace_back(
            ShapeFactory::instance()->createLine(cc1 - offs1, cc2 + offs2));
    }

    // inner tangents:
    double dist3 = cr2 + cr1;
    if (dist1 > dist3) {
        double angle3 = asin(dist3 / dist1);
        double angt3 = angle1 + angle3 + M_PI / 2.0;
        double angt4 = angle1 - angle3 - M_PI / 2.0;
        offs1 = Vec2d();
        offs2 = Vec2d();

        offs1.setPolar(cr1, angt3);
        offs2.setPolar(cr2, angt3);

        tangents.emplace_back(
            ShapeFactory::instance()->createLine(cc1 - offs1, cc2 + offs2));

        offs1.setPolar(cr1, angt4);
        offs2.setPolar(cr2, angt4);

        tangents.emplace_back(
            ShapeFactory::instance()->createLine(cc1 - offs1, cc2 + offs2));
    }

    return tangents;
}

std::vector<std::unique_ptr<shape::Line>>
calculate_orthogonal_tangent_between_shape_and_line(const shape::Line *line,
                                                    const shape::Shape *shape)
{
    assert(line);
    assert(shape);
    std::vector<std::unique_ptr<shape::Line>> ret;

    std::unique_ptr<shape::Line> auxLine1, auxLine2;
    std::vector<Vec2d> ips, ips1, ips2;

    double lineAngle = line->getAngle();

    if (isCircleShape(shape) || isArcShape(shape)) {

        // line parallel to line through center of circle:
        auxLine1 = ShapeFactory::instance()->createLine(centerOfACE(shape),
                                                        lineAngle, 100.0);

        // intersections of parallel with circle:
        ips1 = shape->getIntersectionPoints(auxLine1.release(), false);
        for (size_t i = 0; i < ips1.size(); i++) {
            // candidate:
            auxLine2 = ShapeFactory::instance()->createLine(
                ips1[i], lineAngle + M_PI / 2, 100.0);
            ips2 = line->getIntersectionPoints(auxLine2.release(), false);
            if (ips2.size() == 1) {
                ret.emplace_back(std::move(
                    ShapeFactory::instance()->createLine(ips1[i], ips2[0])));
            }
        }
    }
    else if (isEllipseShape(shape)) {
        auto e = dynamic_cast<const Ellipse *>(shape);
        Vec2d center = e->getCenter();
        // circle around ellipse:
        auto auxCircle =
            ShapeFactory::instance()->createCircle(center, e->getMajorRadius());

        std::vector<Vec2d> foci = e->getFoci();
        auxLine1 =
            ShapeFactory::instance()->createLine(foci[0], lineAngle, 100.0);
        auxLine2 =
            ShapeFactory::instance()->createLine(foci[1], lineAngle, 100.0);

        ips1 = auxLine1->getIntersectionPoints(auxCircle.release(), false);
        ips2 = auxLine2->getIntersectionPoints(auxCircle.release(), false);
        Vec2d pointOfContact1 = Vec2d::invalid;
        Vec2d pointOfContact2 = Vec2d::invalid;

        if (ips1.size() >= 1 && ips2.size() >= 1) {
            if (ips1[0].equalsFuzzy(ips2[0])) {
                pointOfContact1 = ips1[0];
            }
            else {
                auxLine1 =
                    ShapeFactory::instance()->createLine(ips1[0], ips2[0]);
                ips = shape->getIntersectionPoints(auxLine1.release(), false);
                if (ips.size() >= 1) {
                    pointOfContact1 = ips[0];
                }
            }
        }

        if (ips1.size() >= 2 && ips2.size() >= 2) {
            if (ips1[1].equalsFuzzy(ips2[1])) {
                pointOfContact2 = ips1[1];
            }
            else {
                auxLine2 =
                    ShapeFactory::instance()->createLine(ips1[1], ips2[1]);
                ips = shape->getIntersectionPoints(auxLine2.release(), false);
                if (ips.size() >= 1) {
                    pointOfContact2 = ips[0];
                }
            }
        }

        if (pointOfContact1.isValid()) {
            Vec2d pointOnLine1 =
                line->getClosestPointOnShape(pointOfContact1, false);
            ret.emplace_back(std::move(ShapeFactory::instance()->createLine(
                pointOfContact1, pointOnLine1)));
        }
        if (pointOfContact1.isValid()) {
            Vec2d pointOnLine2 =
                line->getClosestPointOnShape(pointOfContact2, false);
            ret.emplace_back(std::move(ShapeFactory::instance()->createLine(
                pointOfContact2, pointOnLine2)));
        }
    }

    return ret;
}

/**
 * Breaks the closest segment in shape to position between two intersections
 * with otherShapes or
 * extends a shape to the next two (imaginary) intersections with otherShapes.
 *
 * \param extend True: extending instead of breaking out.
 *
 * \return Array of three new shapes which each might be undefined if its
 * length would otherwise be 0.
 * The first shape is the rest at the start of the shape.
 * The second shape is the rest at the end of the shape.
 * The third shape is the segment self in its new shape.
 */
std::vector<std::unique_ptr<shape::Shape>>
auto_split(const shape::Vec2d &pos, const shape::Shape *shape,
           const std::vector<shape::Shape *> &intersecting_shapes, bool extend)
{
    assert(shape);
    assert(intersecting_shapes.size() > 0);

    std::unique_ptr<shape::Shape> mutable_shape = shape->clone();

    std::vector<std::unique_ptr<shape::Shape>> res;
    // get intersection points:
    std::vector<Vec2d> ips = cada_get_intersection_points(
        mutable_shape.get(), intersecting_shapes, !extend, extend);
    if (ips.size() == 0) {
        // no intersections with other shapes or self,
        // return whole shape as segment:
        res.emplace_back(nullptr);
        res.emplace_back(nullptr);
        res.emplace_back(std::move(mutable_shape));
        return res;
    }

    // convert circle to arc:
    if (mutable_shape->getShapeType() == NS::Circle) {
        auto c = dynamic_cast<const shape::Circle *>(mutable_shape.get());
        double ap = c->getCenter().getAngleTo(pos);
        auto arc = ShapeFactory::instance()->createArc(
            c->getCenter(), c->getRadius(), ap, ap, false);

        double maxD = 0.0;
        Vec2d p = Vec2d::invalid;
        for (size_t i = 0; i < ips.size(); i++) {
            Vec2d ip = ips[i];
            double d = arc->getDistanceFromStart(ip);
            if (d > maxD) {
                maxD = d;
                p = ip;
            }
        }

        // no intersections:
        if (!p.isValid()) {
            res.emplace_back(nullptr);
            res.emplace_back(nullptr);
            res.emplace_back(std::move(mutable_shape));
            return res;
        }

        // angle at intersection point closest to end of arc is where we split
        // the circle:
        ap = c->getCenter().getAngleTo(p);
        mutable_shape = std::move(ShapeFactory::instance()->createArc(
            c->getCenter(), c->getRadius(), ap, ap, false));
    }

    // find intersection points closest to position:
    // array of two distances and two point vectors:
    std::array<std::pair<Vec2d, double>, 2> cutDistances;
    auto tmp = cada_get_closest_intersection_point_distances(
        mutable_shape.release(), ips, pos, cutDistances);
    mutable_shape.reset(tmp.release());
}

bool break_out_gap(const shape::Vec2d &pos, const shape::Shape *shape,
                   std::vector<shape::Shape *> &additional)
{
    return false;
}

std::vector<std::unique_ptr<shape::Shape>>
bevel_shapes(const shape::Shape *shap1, const shape::Vec2d &pos1,
             const shape::Shape *shape2, const shape::Vec2d &pos2, bool trim,
             double distance1, double distance2)
{
    return std::vector<std::unique_ptr<shape::Shape>>();
}

std::vector<std::unique_ptr<shape::Shape>>
round_shapes(const shape::Shape *shap1, const shape::Vec2d &pos1,
             const shape::Shape *shape2, const shape::Vec2d &pos2, bool trim,
             bool same_polyline, double radius, const shape::Vec2d &pos)
{
    return std::vector<std::unique_ptr<shape::Shape>>();
}

std::unique_ptr<shape::Shape> lengthen(const shape::Shape *shape,
                                       const shape::Vec2d &position,
                                       bool trim_start, double amount)
{
    return nullptr;
}

/* ------------------------- inner static functions ------------------------- */

std::vector<Vec2d>
cada_get_intersection_points(const shape::Shape *shape,
                             const std::vector<shape::Shape *> &other_shapes,
                             bool on_shape, bool on_other_shape)
{
    assert(shape);
    assert(other_shapes.size() > 0);

    std::vector<Vec2d> intersections;
    // treat start and end points as intersection points for open shapes:
    bool is_circle = isCircleShape(shape);
    bool is_full_ellipse = isFullEllipseShape(shape);
    bool is_xline_shape = (shape->getShapeType() == NS::XLine);
    bool is_polyline_shape = shape->getShapeType() == NS::Polyline;
    bool is_polyline_geo_closed =
        is_polyline_shape &&
        (dynamic_cast<const shape::Polyline *>(shape)->isGeometricallyClosed());
    bool is_spline_shape = shape->getShapeType() == NS::BSpline;
    bool is_spline_closed =
        is_spline_shape &&
        (dynamic_cast<const shape::BSpline *>(shape)->isClosed());
    if (on_shape && !is_circle && !is_full_ellipse && !is_xline_shape &&
        (!is_polyline_shape || !is_polyline_geo_closed) &&
        (!is_spline_shape || !is_spline_closed)) {

        Vec2d sp = shape->getStartPoint();
        intersections.push_back(sp);

        if (!isRayShape(shape)) {
            Vec2d ep = shape->getEndPoint();
            intersections.push_back(ep);
        }
    }

    // find all intersection points:
    for (auto &&other : other_shapes) {
        std::vector<Vec2d> sol =
            shape->getIntersectionPoints(other, on_shape, false, true);
        for (size_t k = 0; k < sol.size(); k++) {
            if (!on_other_shape || other->isOnShape(sol[k])) {
                intersections.push_back(sol[k]);
            }
        }
    }

    std::vector<Vec2d> selfIntersectionPoints =
        shape->getSelfIntersectionPoints();

    // add self intersection points to list:
    if (selfIntersectionPoints.size() != 0) {
        intersections.insert(intersections.end(),
                             selfIntersectionPoints.begin(),
                             selfIntersectionPoints.end());
    }

    return intersections;
}

std::unique_ptr<shape::Shape> cada_get_closest_intersection_point_distances(
    shape::Shape *shape, const std::vector<Vec2d> &intersections,
    const Vec2d &pos, std::array<std::pair<Vec2d, double>, 2> &res)
{
    assert(shape);
    std::unique_ptr<shape::Shape> mutable_shape;
    mutable_shape.reset(shape);

    // closed circular shapes:
    bool circular = false;
    if (mutable_shape->getShapeType() == NS::Circle) {
        auto cir = dynamic_cast<const shape::Circle *>(mutable_shape.get());
        double a = cir->getCenter().getAngleTo(pos);
        mutable_shape.reset(
            ShapeFactory::instance()
                ->createArc(cir->getCenter(), cir->getRadius(), a, a, false)
                .release());
        circular = true;
    }

    if (mutable_shape->getShapeType() == NS::Polyline) {
        auto poly = dynamic_cast<const shape::Polyline *>(mutable_shape.get());
        if (poly->isGeometricallyClosed())
            circular = true;
    }
    if (mutable_shape->getShapeType() == NS::Arc) {
        circular = true;
    }

    double pDist = mutable_shape->getDistanceFromStart(pos);

    std::unique_ptr<Line> orthoLine;
    bool reversedShape = false;
    if (mutable_shape->getShapeType() == NS::Ellipse) {
        auto ellipse =
            dynamic_cast<const shape::Ellipse *>(mutable_shape.get());
        orthoLine =
            ShapeFactory::instance()->createLine(ellipse->getCenter(), pos);
        if (ellipse->isReversed()) {
            mutable_shape->reverse();
            reversedShape = true;
        }
    }

    // find intersection points directly before and after clicked position:
    double cutDist1 = std::numeric_limits<double>::quiet_NaN();
    double cutDist2 = std::numeric_limits<double>::quiet_NaN();
    Vec2d cutPos1 = Vec2d::invalid;
    Vec2d cutPos2 = Vec2d::invalid;

    // for circular shapes, also find intersections closest to and furthest from
    // start point:
    double cutDistMax = std::numeric_limits<double>::quiet_NaN();
    double cutDistMin = std::numeric_limits<double>::quiet_NaN();
    Vec2d cutPosMax = Vec2d::invalid;
    Vec2d cutPosMin = Vec2d::invalid;

    double dist = 0.0;
    for (size_t i = 0; i < intersections.size(); i++) {
        Vec2d ip = intersections[i];

        if (mutable_shape->getShapeType() == NS::Ellipse) {
            auto ellipse =
                dynamic_cast<const shape::Ellipse *>(mutable_shape.get());
            dist = Math::getAngleDifference(
                orthoLine->getAngle(), ellipse->getCenter().getAngleTo(ip));
            if (std::isnan(cutDist1) || dist < cutDist1) {
                cutPos1 = ip;
                cutDist1 = dist;
            }
            if (std::isnan(cutDist2) || dist > cutDist2) {
                cutPos2 = ip;
                cutDist2 = dist;
            }
        }
        else {
            std::vector<double> dists =
                mutable_shape->getDistancesFromStart(ip);
            for (size_t k = 0; k < dists.size(); k++) {
                dist = dists[k];

                // largest distance to start
                // but smaller than click point:
                if (dist < pDist) {
                    if (std::isnan(cutDist1) || dist > cutDist1) {
                        cutDist1 = dist;
                        cutPos1 = ip;
                    }
                }

                if (circular) {
                    if (std::isnan(cutDistMax) || dist > cutDistMax) {
                        cutDistMax = dist;
                        cutPosMax = ip;
                    }
                }

                // smallest distance to start
                // but larger than click point
                if (dist > pDist) {
                    if (std::isnan(cutDist2) || dist < cutDist2) {
                        cutDist2 = dist;
                        cutPos2 = ip;
                    }
                }

                if (circular) {
                    if (std::isnan(cutDistMin) ||
                        (dist < cutDistMin &&
                         (dist > pDist ||
                          mutable_shape->getShapeType() == NS::Polyline))) {
                        cutDistMin = dist;
                        cutPosMin = ip;
                    }
                }
            }
        }
    }

    if (circular) {
        if (std::isnan(cutDist1)) {
            cutDist1 = cutDistMax;
            cutPos1 = cutPosMax;
        }
        if (std::isnan(cutDist2)) {
            cutDist2 = cutDistMin;
            cutPos2 = cutPosMin;
        }
    }

    if (mutable_shape->getShapeType() == NS::Ellipse) {
        auto ellipse =
            dynamic_cast<const shape::Ellipse *>(mutable_shape.get());
        if (ellipse->isReversed()) {
            std::swap(cutPos1, cutPos2);
            std::swap(cutDist1, cutDist2);
        }
    }

    // open shape: cut to start or end point:
    bool is_circle = mutable_shape->getShapeType() == NS::Circle;
    bool is_full_ellipse = isFullEllipseShape(mutable_shape.get());
    bool is_xline_shape = (mutable_shape->getShapeType() == NS::XLine);
    bool is_ray_shape = (mutable_shape->getShapeType() == NS::Ray);
    bool is_polyline_shape = mutable_shape->getShapeType() == NS::Polyline;
    bool is_polyline_geo_closed =
        is_polyline_shape &&
        (dynamic_cast<const shape::Polyline *>(mutable_shape.get())
             ->isGeometricallyClosed());
    bool is_spline_shape = mutable_shape->getShapeType() == NS::BSpline;
    bool is_spline_closed =
        is_spline_shape &&
        (dynamic_cast<const shape::BSpline *>(mutable_shape.get())->isClosed());
    if (!is_circle && !is_full_ellipse && !is_xline_shape && !is_ray_shape &&
        (!is_polyline_shape || !is_polyline_geo_closed) &&
        (!is_spline_shape || !is_spline_closed)) {

        if (!cutPos1.isValid()) {
            cutDist1 = 0.0;
            cutPos1 = mutable_shape->getStartPoint();
        }
        if (!cutPos2.isValid()) {
            cutDist2 = mutable_shape->getLength();
            cutPos2 = mutable_shape->getEndPoint();
        }
    }

    if (reversedShape) {
        mutable_shape->reverse();
    }

    res[0] = {cutPos1, cutDist1};
    res[1] = {cutPos2, cutDist2};
    return mutable_shape;
};

} // namespace algorithm
} // namespace cada