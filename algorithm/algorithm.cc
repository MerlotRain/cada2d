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

#include <cavaliercontours.h>

extern "C" {
#include <apollonius.h>
}

using namespace cada::shape;

namespace cada {
namespace algorithm {

/* -------------------------- foundation functions -------------------------- */

std::unique_ptr<Shape> TrimStartPoint(shape::Shape *shape,
                                      const Vec2d &trimPoint,
                                      const Vec2d &clickPoint);
std::unique_ptr<Shape> TrimEndPoint(shape::Shape *shape, const Vec2d &trimPoint,
                                    const Vec2d &clickPoint);
std::unique_ptr<Shape> xLineToRay(const shape::Shape *shape);
std::unique_ptr<Shape> rayToXLine(const shape::Shape *shape);
std::unique_ptr<Shape> rayToLine(const shape::Shape *shape);

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
bevel_shapes(const shape::Shape *shp1, const shape::Vec2d &clickPos1,
             const shape::Shape *shp2, const shape::Vec2d &clickPos2, bool trim,
             bool samePolyline, double distance1, double distance2);

std::vector<std::unique_ptr<shape::Shape>>
round_shapes(const shape::Shape *shp1, const shape::Vec2d &pos1,
             const shape::Shape *shp2, const shape::Vec2d &pos2, bool trim,
             bool samepolyline, double radius, const shape::Vec2d &solutionPos);

bool lengthen(shape::Shape *shape, const shape::Vec2d &position,
              bool trim_start, double amount);

std::vector<std::unique_ptr<Circle>> apollonius_solutions(const Shape *shape1,
                                                          const Shape *shape2,
                                                          const Shape *shape3);
/* ------------------------- inner static functions ------------------------- */

static std::vector<Vec2d>
cada__get_intersection_points(const Shape *shape,
                              const std::vector<Shape *> &otherShapes,
                              bool onShape, bool onOtherShapes);

static std::unique_ptr<shape::Shape> cada__closest_intersection_point_distances(
    const shape::Shape *shp, const std::vector<Vec2d> &intersections,
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
auto_split(const shape::Vec2d &pos, const shape::Shape *shp,
           const std::vector<shape::Shape *> &otherShapes, bool extend)
{
    std::vector<std::unique_ptr<shape::Shape>> res;
    std::unique_ptr<Shape> shape = shp->clone();
    assert(shape);
    // get intersection points:
    auto &&ips = cada__get_intersection_points(shape.get(), otherShapes,
                                               !extend, extend);
    if (ips.size() == 0) {
        // no intersections with other shapes or self,
        // return whole shape as segment:
        res.emplace_back(nullptr);
        res.emplace_back(nullptr);
        res.emplace_back(shape->clone());
        return res;
    }

    // convert circle to arc:
    if (isCircleShape(shape.get())) {
        auto &&circle = dynamic_cast<shape::Circle *>(shape.get());
        auto ap = circle->getCenter().getAngleTo(pos);
        auto &&arc = ShapeFactory::instance()->createArc(
            circle->getCenter(), circle->getRadius(), ap, ap, false);

        double maxD = std::numeric_limits<double>::quiet_NaN();
        Vec2d p = Vec2d::invalid;
        for (size_t i = 0; i < ips.size(); i++) {
            Vec2d ip = ips[i];
            double d = arc->getDistanceFromStart(ip);
            if (std::isnan(maxD) || d > maxD) {
                maxD = d;
                p = ip;
            }
        }

        // no intersections:
        if (!p.isValid()) {
            res.emplace_back(nullptr);
            res.emplace_back(nullptr);
            res.emplace_back(shape->clone());
            return res;
        }

        // angle at intersection point closest to end of arc is where we split
        // the circle:
        ap = circle->getCenter().getAngleTo(p);
        shape.reset(ShapeFactory::instance()
                        ->createArc(circle->getCenter(), circle->getRadius(),
                                    ap, ap, false)
                        .release());
    }

    // find intersection points closest to position:
    // array of two distances and two point vectors:
    std::array<std::pair<Vec2d, double>, 2> cutDistances;
    auto tmp = cada__closest_intersection_point_distances(shape.release(), ips,
                                                          pos, cutDistances);
    shape.reset(tmp.release());
    // distance along shape to clicked position:
    // var dPosition = ;

    // make sure direction of shape does not change in the process:
    // intersectionPointDistances-》sort();

    double cutDist1 = cutDistances[0].second;
    double cutDist2 = cutDistances[1].second;
    Vec2d cutPos1 = cutDistances[0].first;
    Vec2d cutPos2 = cutDistances[1].first;

    // if we only have one cutting point (XLine, Ray), make it the first
    // parameter:
    if (cutDist1 == std::numeric_limits<double>::quiet_NaN()) {
        cutDist1 = cutDist2;
        cutPos1 = cutPos2;
        cutDist2 = std::numeric_limits<double>::quiet_NaN();
        cutPos2 = Vec2d::invalid;
    }

    return auto_split_manual(shape.release(), cutDist1, cutDist2, cutPos1,
                             cutPos2, pos, extend);
}

std::vector<std::unique_ptr<shape::Shape>>
auto_split_manual(const shape::Shape *shp, double cutDist1, double cutDist2,
                  Vec2d cutPos1, Vec2d cutPos2, const Vec2d &position,
                  bool extend)
{
    assert(shp);

    if (std::isnan(cutDist1) && !cutPos1.isValid()) {
        cutDist1 = shp->getDistanceFromStart(cutPos1);
    }
    if (std::isnan(cutDist2) && !cutPos2.isValid()) {
        cutDist2 = shp->getDistanceFromStart(cutPos2);
    }

    std::vector<std::unique_ptr<shape::Shape>> res;

    if (std::isnan(cutDist2)) {
        if (Math::fuzzyCompare(cutDist1, 0.0) &&
            shp->getStartPoint().equalsFuzzy(cutPos1)) {
            res.emplace_back(nullptr);
            res.emplace_back(nullptr);
            res.emplace_back(shp->clone());
            return res;
        }
    }
    else {
        if (Math::fuzzyAngleCompare(cutDist1, 0.0) &&
            shp->getStartPoint().equalsFuzzy(cutPos1) &&
            Math::fuzzyCompare(cutDist2, shp->getLength()) &&
            shp->getEndPoint().equalsFuzzy(cutPos2)) {
            res.emplace_back(nullptr);
            res.emplace_back(nullptr);
            res.emplace_back(shp->clone());
            return res;
        }
    }

    // lines:
    if (isXLineShape(shp)) {
        std::unique_ptr<shape::Line> rest1;
        std::unique_ptr<shape::Line> rest2;
        std::unique_ptr<shape::Line> segment;
        auto &&shape = dynamic_cast<const shape::Line *>(shp);
        rest1 = shape->clone();
        rest2 = shape->clone();

        if (cutDist1 < cutDist2) {
            rest1->trimEndPoint(cutPos1);
            rest2->trimStartPoint(cutPos2);
        }
        else {
            rest1->trimEndPoint(cutPos1);
            rest2->trimStartPoint(cutPos2);
        }

        segment = shape->clone();
        segment->setStartPoint(cutPos1);
        segment->setEndPoint(cutPos2);

        if (rest1->getLength() < NS::PointTolerance) {
            rest1 = nullptr;
        }
        if (rest2->getLength() < NS::PointTolerance) {
            rest2 = nullptr;
        }

        res.emplace_back(rest1.release());
        res.emplace_back(rest2.release());
        res.emplace_back(segment.release());
    }
    // xline:
    else if (isXLineShape(shp)) {
        std::unique_ptr<shape::Shape> rest1;
        std::unique_ptr<shape::Shape> rest2;
        std::unique_ptr<shape::Shape> segment;

        auto &&shape = dynamic_cast<const shape::XLine *>(shp);
        auto &&line = shape->getLineShape();
        cutPos1 = line->getPointWithDistanceToStart(cutDist1);
        if (std::isnan(cutDist2)) {
            cutPos2 = Vec2d::invalid;
        }
        else {
            cutPos2 = line->getPointWithDistanceToStart(cutDist2);
        }

        if (!std::isnan(cutDist1) && !std::isnan(cutDist2) &&
            cutDist1 > cutDist2) {
            std::swap(cutDist1, cutDist2);
        }
        // <--------x---------------x--------->
        // rest2   cp2   segment   cp1   rest1
        if (!std::isnan(cutDist1) && !std::isnan(cutDist2)) {
            rest1 = ShapeFactory::instance()->createRay(
                cutPos1, Vec2d::createPolar(1.0, shape->getDirection2()));
            rest2 = ShapeFactory::instance()->createRay(
                cutPos2, Vec2d::createPolar(1.0, shape->getDirection1()));
            segment = ShapeFactory::instance()->createLine(cutPos1, cutPos2);
        }
        // <-o--------------x----------------->
        //  pos  segment   cp1   rest1
        // <----------------x-------------o--->
        //        rest1    cp1  segment  pos
        else if (!std::isnan(cutDist1)) {
            rest1 = ShapeFactory::instance()->createRay(
                cutPos1, Vec2d::createPolar(1.0, shape->getDirection2()));
            segment = ShapeFactory::instance()->createRay(
                cutPos1, Vec2d::createPolar(1.0, shape->getDirection1()));

            double distSegment = segment->getDistanceTo(position);
            if (std::isnan(distSegment)) {
                std::swap(rest1, segment);
            }
            rest2 = nullptr;
        }
        res.emplace_back(rest1.release());
        res.emplace_back(rest2.release());
        res.emplace_back(segment.release());
    }
    // ray:
    else if (isRayShape(shp)) {
        auto &&shape = dynamic_cast<const Ray *>(shp);
        std::unique_ptr<shape::Shape> rest1;
        std::unique_ptr<shape::Shape> rest2;
        std::unique_ptr<shape::Shape> segment;

        if (!std::isnan(cutDist1) && !std::isnan(cutDist2) &&
            Math::sign(cutDist1) != Math::sign(cutDist2)) {
            std::swap(cutDist1, cutDist2);
        }

        // <--------x-------o-------x---------
        // rest2   cp2   segment   cp1   rest1
        if (cutPos1.isValid() && cutPos2.isValid()) {
            rest1 = ShapeFactory::instance()->createLine(shape->getBasePoint(),
                                                         cutPos1);
            segment = ShapeFactory::instance()->createLine(cutPos1, cutPos2);
            rest2 = ShapeFactory::instance()->createRay(
                cutPos2, Vec2d::createPolar(1.0, shape->getDirection1()));
        }

        // <-------o--------x-----------------
        //      segment    cp1     rest1
        // <----------------x--------o--------
        //       rest1     cp1    segment
        else if (!cutPos1.isValid()) {
            rest1 = ShapeFactory::instance()->createLine(shape->getBasePoint(),
                                                         cutPos1);
            segment = ShapeFactory::instance()->createRay(
                cutPos1, Vec2d::createPolar(1.0, shape->getDirection1()));
            rest2 = nullptr;

            double distSegment = segment->getDistanceTo(position);
            if (std::isnan(distSegment)) {
                std::swap(rest1, segment);
            }
        }
        res.emplace_back(rest1.release());
        res.emplace_back(rest2.release());
        res.emplace_back(segment.release());
    }
    // arc:
    else if (shp->getShapeType() == NS::Arc) {
        auto &&shape = dynamic_cast<const Arc *>(shp);
        std::unique_ptr<shape::Arc> rest1;
        std::unique_ptr<shape::Arc> rest2;
        std::unique_ptr<shape::Arc> segment;
        if (cutDist1 > cutDist2) {
            std::swap(cutDist1, cutDist2);
        }

        rest1 = shape->clone();
        rest2 = shape->clone();
        rest1->trimEndPoint(cutPos1);
        rest2->trimStartPoint(cutPos2);

        segment = shape->clone();
        segment->setStartAngle(segment->getCenter().getAngleTo(cutPos1));
        segment->setEndAngle(segment->getCenter().getAngleTo(cutPos2));

        if (!extend) {
            double angleLength1 = rest1->getAngleLength(true);
            double angleLength2 = rest2->getAngleLength(true);

            // rest1 is the same as the segment:
            bool same1 = Math::fuzzyAngleCompare(rest1->getStartAngle(),
                                                 segment->getStartAngle()) &&
                         Math::fuzzyAngleCompare(rest1->getEndAngle(),
                                                 segment->getEndAngle()) &&
                         rest1->isReversed() == segment->isReversed();

            // catch common errors:
            if (angleLength1 + angleLength2 > shape->getAngleLength() ||
                same1) {
                rest1->trimEndPoint(cutDist2);
                rest2->trimStartPoint(cutDist1);

                segment->trimStartPoint(cutDist2);
                segment->trimEndPoint(cutDist1);

                angleLength1 = rest1->getAngleLength(true);
                angleLength2 = rest2->getAngleLength(true);
            }
            if (angleLength1 < 1.0e-5) {
                rest1 = nullptr;
            }

            if (angleLength2 < 1.0e-5) {
                rest2 = nullptr;
            }
        }

        res.emplace_back(rest1.release());
        res.emplace_back(rest2.release());
        res.emplace_back(segment.release());
    }
    // circles:
    else if (isCircleShape(shp)) {
        auto &&shape = dynamic_cast<const Circle *>(shp);
        std::unique_ptr<shape::Arc> rest1;
        std::unique_ptr<shape::Arc> rest2;
        std::unique_ptr<shape::Arc> segment;
        if (std::isnan(cutDist1) || std::isnan(cutDist2)) {
            // nullptr
        }
        else {
            double angle1 = shape->getCenter().getAngleTo(cutPos1);
            double angle2 = shape->getCenter().getAngleTo(cutPos2);

            rest1 = ShapeFactory::instance()->createArc(
                shape->getCenter(), shape->getRadius(), angle1, angle2, false);
            rest2 = nullptr;
            segment = ShapeFactory::instance()->createArc(
                shape->getCenter(), shape->getRadius(), angle2, angle1, false);

            if (!position.isValid()) {
                double cursorAngle = shape->getCenter().getAngleTo(position);

                if (Math::isAngleBetween(cursorAngle, angle1, angle2, false)) {
                    rest1->setStartAngle(angle2);
                    rest1->setEndAngle(angle1);
                    segment->setStartAngle(angle1);
                    segment->setEndAngle(angle2);
                }
            }
            double angleLength = rest1->getAngleLength(true);
            if (angleLength < NS::PointTolerance) {
                rest1 = nullptr;
            }
        }
        res.emplace_back(rest1.release());
        res.emplace_back(rest2.release());
        res.emplace_back(segment.release());
    }
    // ellipse arcs:
    else if (isEllipseShape(shp) && !isFullEllipseShape(shp)) {
        auto &&shape = dynamic_cast<const Ellipse *>(shp);
        std::unique_ptr<Ellipse> rest1;
        std::unique_ptr<Ellipse> rest2;
        std::unique_ptr<Ellipse> segment;

        rest1 = shape->clone();
        rest2 = shape->clone();

        rest1->trimEndPoint(cutPos1, cutPos2);
        rest2->trimStartPoint(cutPos1, cutPos2);

        segment = shape->clone();
        segment->trimStartPoint(cutPos1, cutPos1);
        segment->trimEndPoint(cutPos2, cutPos2);

        double angleLength1 = rest1->getAngleLength(true);
        double angleLength2 = rest2->getAngleLength(true);
        if (angleLength1 + angleLength2 > shape->getAngleLength()) {
            rest1->trimEndPoint(cutPos2, cutPos2);
            rest2->trimStartPoint(cutPos1, cutPos1);
            segment->trimStartPoint(cutPos2, cutPos2);
            segment->trimEndPoint(cutPos1, cutPos1);

            angleLength1 = rest1->getAngleLength(true);
            angleLength2 = rest2->getAngleLength(true);
        }
        if (angleLength1 < 1.0e-5) {
            rest1 = nullptr;
        }

        if (angleLength2 < 1.0e-5) {
            rest2 = nullptr;
        }
        res.emplace_back(rest1.release());
        res.emplace_back(rest2.release());
        res.emplace_back(segment.release());
    }
    // full ellipses:
    else if (isEllipseShape(shp) && isFullEllipseShape(shp)) {

        auto &&shape = dynamic_cast<const Ellipse *>(shp);
        std::unique_ptr<Ellipse> rest1;
        std::unique_ptr<Ellipse> rest2;
        std::unique_ptr<Ellipse> segment;

        if (!cutPos1.isValid() || !cutPos2.isValid()) {
            //
        }
        else {
            double angle1 = shape->getParamTo(cutPos1);
            double angle2 = shape->getParamTo(cutPos2);

            rest1 = ShapeFactory::instance()->createEllipse(
                shape->getCenter(), shape->getMajorPoint(), shape->getRatio(),
                angle1, angle2, false);
            rest2 = nullptr;

            segment = ShapeFactory::instance()->createEllipse(
                shape->getCenter(), shape->getMajorPoint(), shape->getRatio(),
                angle2, angle1, false);

            if (!position.isValid()) {
                double cursorAngle = shape->getParamTo(position);

                if (Math::isAngleBetween(cursorAngle, angle1, angle2, false)) {
                    rest1->setStartParam(angle2);
                    rest1->setEndParam(angle1);
                    segment->setStartParam(angle1);
                    segment->setEndAngle(angle2);
                }
            }

            double angleLength1 = rest1->getAngleLength();
            if (angleLength1 < NS::AngleTolerance) {
                rest1 = nullptr;
            }
        }
        res.emplace_back(rest1.release());
        res.emplace_back(rest2.release());
        res.emplace_back(segment.release());
    }
    // polyline:
    else if (isPolylineShape(shp)) {
        auto shp_clone = shp->clone();
        auto &&shape = dynamic_cast<Polyline *>(shp->clone().get());
        std::unique_ptr<Shape> rest1;
        std::unique_ptr<Shape> rest2;
        std::unique_ptr<Shape> segment;

        bool closed = shape->isGeometricallyClosed();
        if (closed) {
            shape->relocateStartPoint(cutDist1);
            shape->convertToOpen();
            cutDist2 -= cutDist1;
            if (cutDist2 < 0.0) {
                cutDist2 = shape->getLength() + cutDist2;
            }

            cutDist1 = 0.0;
        }

        rest1 = shape->clone();
        rest2 = shape->clone();
        segment = shape->clone();

        if (closed) {
            rest1->trimEndPoint(cutPos2);
            segment = nullptr;
            rest2->trimStartPoint(cutDist2);
        }
        else {
            // make sure point 1 is closer to the start of the polyline:
            if (cutDist1 > cutDist2) {
                std::swap(cutDist1, cutDist2);
            }

            rest1->trimEndPoint(cutDist1);

            double l1 = segment->getLength();
            segment->trimStartPoint(cutDist1);
            double l2 = segment->getLength();
            segment->trimEndPoint(cutDist2 - (l1 - l2));

            rest2->trimStartPoint(cutDist2);
        }

        if (segment) {
            if (segment->getLength() < NS::PointTolerance ||
                (closed && Math::fuzzyCompare(segment->getLength(),
                                              shape->getLength()))) {
                segment = nullptr;
            }
        }

        if (rest1) {
            if (rest1->getLength() < NS::PointTolerance ||
                (closed &&
                 Math::fuzzyCompare(rest1->getLength(), shape->getLength()))) {
                rest1 = nullptr;
            }
        }

        if (rest2) {
            if (rest2->getLength() < NS::PointTolerance ||
                (closed &&
                 Math::fuzzyCompare(rest2->getLength(), shape->getLength()))) {
                rest2 = nullptr;
            }
        }

        if (segment && rest1 && rest2) {
            double disRest1 = rest1->getDistanceTo(position);
            double disRest2 = rest2->getDistanceTo(position);
            if (disRest1 < disRest2 || std::isnan(cutDist2)) {
                segment.reset(rest1.release());
                rest1 = nullptr;
            }
            else {
                segment.reset(rest2.release());
                rest2 = nullptr;
            }
        }

        res.emplace_back(rest1.release());
        res.emplace_back(rest2.release());
        res.emplace_back(segment.release());
    }
    // spline:
    else if (isSplineShape(shp)) {
        auto &&shape = dynamic_cast<const BSpline *>(shp);
        std::unique_ptr<BSpline> rest1;
        std::unique_ptr<BSpline> rest2;
        std::unique_ptr<BSpline> segment;
        rest1 = shape->clone();
        rest2 = shape->clone();
        segment = shape->clone();

        double tAtCutPos1 = shape->getTAtDistance(cutDist1);
        double tAtCutPos2 = shape->getTAtDistance(cutDist2);
        double tMax = shape->getTMax();

        if (shape->getStartPoint().equalsFuzzy(shape->getEndPoint())) {
            if (Math::fuzzyCompare(tAtCutPos1, shape->getTMax())) {
                tAtCutPos1 = shape->getTMin();
            }
        }

        if (tAtCutPos1 < tAtCutPos2) {
            if (Math::fuzzyCompare(tAtCutPos1, 0.0)) {
                rest1 = nullptr;
            }
            else {
                rest1->trimEndPoint(cutDist1);
                // positions are more precise but
                // distances take into account possible self intersections:
                rest1->setEndPoint(cutPos1);
            }

            double l1 = segment->getLength();
            segment->trimStartPoint(cutDist1);
            segment->setStartPoint(cutPos1);
            double l2 = segment->getLength();
            segment->trimEndPoint(cutDist2 - (l1 - l2));
            segment->setEndPoint(cutPos2);

            if (Math::fuzzyCompare(tAtCutPos2, tMax)) {
                rest2 = nullptr;
            }
            else {
                rest2->trimStartPoint(cutDist2);
                rest2->setStartPoint(cutPos2);
            }
        }
        else {
            if (Math::fuzzyCompare(tAtCutPos1, 0.0)) {
                rest1 = nullptr;
            }
            else {
                rest1->trimEndPoint(cutDist2);
                rest1->setEndPoint(cutPos2);
            }
            double l1 = segment->getLength();
            segment->trimStartPoint(cutDist2);
            segment->setStartPoint(cutPos2);
            double l2 = segment->getLength();
            segment->trimEndPoint(cutDist1 - (l1 - l2));
            segment->setEndPoint(cutPos1);

            if (Math::fuzzyCompare(tAtCutPos2, tMax)) {
                rest2 = nullptr;
            }
            else {
                rest2->trimStartPoint(cutDist1);
                rest2->setStartPoint(cutPos1);
            }
        }

        if (segment) {
            if (!segment->isValid() ||
                segment->getLength() < NS::PointTolerance) {
                segment = nullptr;
            }
        }

        if (rest1) {
            if (!rest1->isValid() || rest1->getLength() < NS::PointTolerance) {
                rest1 = nullptr;
            }
        }

        if (rest2) {
            if (!rest2->isValid() || rest2->getLength() < NS::PointTolerance) {
                rest2 = nullptr;
            }
        }

        res.emplace_back(rest1.release());
        res.emplace_back(rest2.release());
        res.emplace_back(segment.release());
    }
    return res;
}

bool break_out_gap(const shape::Vec2d &pos, const shape::Shape *shape,
                   std::vector<shape::Shape *> &additional)
{
    return false;
}

std::vector<std::unique_ptr<Shape>>
bevel_shapes(const shape::Shape *shp1, const shape::Vec2d &clickPos1,
             const shape::Shape *shp2, const shape::Vec2d &clickPos2, bool trim,
             bool samePolyline, double distance1, double distance2)
{
    std::vector<std::unique_ptr<Shape>> results;

    std::unique_ptr<Shape> shape1(shp1->clone());
    std::unique_ptr<Shape> shape2(shp2->clone());

    // convert circles to arcs:
    if (isCircleShape(shape1.get())) {
        auto &&circle = dynamic_cast<Circle *>(shape1.release());
        shape1 = std::move(circle->toArc());
    }
    if (isCircleShape(shape2.get())) {
        auto &&circle = dynamic_cast<Circle *>(shape2.release());
        shape2 = std::move(circle->toArc());
    }

    std::unique_ptr<Shape> simpleShape1, simpleShape2;
    int i1, i2;

    if (isPolylineShape(shape1.get())) {
        auto &&pline = dynamic_cast<Polyline *>(shape1.release());
        i1 = pline->getClosestSegment(clickPos1);
        if (i1 == -1) {
            return results;
        }
        simpleShape1 = pline->getSegmentAt(i1);
    }
    else {
        simpleShape1 = shape1->clone();
    }

    if (isPolylineShape(shape2.get())) {
        auto &&pline = dynamic_cast<Polyline *>(shape2.release());
        i2 = pline->getClosestSegment(clickPos2);
        if (i2 == -1) {
            return results;
        }
        simpleShape2 = pline->getSegmentAt(i2);
    }
    else {
        simpleShape2 = shape2->clone();
    }

    // get intersection point(s) between two shapes:
    auto &&sol = simpleShape1->getIntersectionPoints(simpleShape2.get(), false);

    if (sol.size() == 0) {
        return results;
    }

    // var trimmed1 = shape1.clone();
    // var trimmed2 = shape2.clone();
    std::unique_ptr<Shape> trimmed1, trimmed2;
    if (samePolyline) {
        trimmed1 = simpleShape1->clone();
        trimmed2 = simpleShape2->clone();
    }
    else {
        // maybe
        trimmed1 = shape1->clone();
        trimmed2 = shape2->clone();
    }

    // trim shapes to intersection:
    auto start1 = NS::FromAny;
    auto is = clickPos1.getClosest(sol);
    auto ending1 = NS::EndingNone;

    ending1 = trimmed1->getTrimEnd(is, clickPos1);
    switch (ending1) {
    case NS::EndingStart:
        trimmed1 = TrimStartPoint(trimmed1.release(), is, clickPos1);
        start1 = NS::FromStart;
        break;
    case NS::EndingEnd:
        trimmed1 = TrimEndPoint(trimmed1.release(), is, clickPos1);
        if (isRayShape(trimmed1.get())) {
            start1 = NS::FromStart;
            ending1 = NS::EndingStart;
        }
        else {
            start1 = NS::FromEnd;
        }
        break;
    default:
        break;
    }

    auto start2 = NS::FromAny;
    is = clickPos2.getClosest(sol);
    auto ending2 = NS::EndingNone;

    ending2 = trimmed2->getTrimEnd(is, clickPos2);
    switch (ending2) {
    case NS::EndingStart:
        trimmed2 = TrimStartPoint(trimmed2.release(), is, clickPos2);
        start2 = NS::FromStart;
        break;
    case NS::EndingEnd:
        trimmed2 = TrimEndPoint(trimmed2.release(), is, clickPos2);
        if (isRayShape(trimmed2.get())) {
            start2 = NS::FromStart;
            ending2 = NS::EndingStart;
        }
        else {
            start2 = NS::FromEnd;
        }

        break;
    default:
        break;
    }

    // find definitive bevel points:
    std::unique_ptr<Shape> t1, t2;
    if (isCircleShape(trimmed1.get())) {
        auto &&circle = dynamic_cast<Circle *>(trimmed1.get());
        t1 = circle->toArc(circle->getCenter().getAngleTo(is));
        start1 = NS::FromAny;
    }
    else {
        t1 = trimmed1->clone();
    }
    if (isCircleShape(trimmed2.get())) {
        auto &&circle = dynamic_cast<Circle *>(trimmed2.get());
        t2 = circle->toArc(circle->getCenter().getAngleTo(is));
        start2 = NS::FromAny;
    }
    else {
        t2 = trimmed2->clone();
    }

    std::vector<Vec2d> bp1s = t1->getPointsWithDistanceToEnd(distance1, start1);
    if (bp1s.size() == 0) {
        return results;
    }
    Vec2d bp1 = clickPos2.getClosest(bp1s);

    std::vector<Vec2d> bp2s = t2->getPointsWithDistanceToEnd(distance2, start2);
    if (bp2s.size() == 0) {
        return results;
    }
    Vec2d bp2 = clickPos2.getClosest(bp2s);

    // final trim:
    if (trim || samePolyline) {
        switch (ending1) {
        case NS::EndingStart:
            trimmed1->trimStartPoint(bp1);
            break;
        case NS::EndingEnd:
            trimmed1->trimEndPoint(bp1);
            break;
        default:
            break;
        }

        switch (ending2) {
        case NS::EndingStart:
            trimmed2->trimStartPoint(bp2);
            break;
        case NS::EndingEnd:
            trimmed2->trimEndPoint(bp2);
            break;
        default:
            break;
        }
    }

    auto &&bevel = ShapeFactory::instance()->createLine(bp1, bp2);
    if (samePolyline) {
        auto &&pline = dynamic_cast<const Polyline *>(shape1.get());
        auto &&pl = pline->modifyPolylineCorner(trimmed1.release(), ending1, i1,
                                                trimmed2.release(), ending2, i2,
                                                bevel.release());

        results.emplace_back(std::move(pl));
    }
    else {
        results.emplace_back(std::move(trimmed1));
        results.emplace_back(std::move(bevel));
        results.emplace_back(std::move(trimmed2));
    }
    return results;
}

std::vector<std::unique_ptr<shape::Shape>>
round_shapes(const shape::Shape *shp1, const shape::Vec2d &clickPos1,
             const shape::Shape *shp2, const shape::Vec2d &clickPos2, bool trim,
             bool samepolyline, double radius, const shape::Vec2d &pos)
{
    std::vector<std::unique_ptr<shape::Shape>> results;

    if (!shp1 || !clickPos1.isValid() || !shp2 || !clickPos2.isValid()) {
        return results;
    }

    Vec2d p = pos;
    if (!p.isValid()) {
        p = clickPos2;
    }

    auto &&shape1 = shp1->clone();
    auto &&shape2 = shp2->clone();

    std::unique_ptr<Shape> s1, s2;

    // convert circles to arcs:
    if (isCircleShape(shape1.get())) {
        auto &&circle = dynamic_cast<Circle *>(shape1.get());
        s1 = circle->toArc();
    }
    else {
        s1 = shape1->clone();
    }

    if (isCircleShape(shape2.get())) {
        auto &&circle = dynamic_cast<Circle *>(shape2.get());
        s2 = circle->toArc();
    }
    else {
        s2 = shape2->clone();
    }

    std::unique_ptr<Shape> simpleShape1, simpleShape2;
    int i1, i2;

    // rounding polylines?
    // round segments instead:
    if (isPolylineShape(shape1.get())) {
        auto &&pl1 = dynamic_cast<Polyline *>(shape1.get());
        i1 = pl1->getClosestSegment(clickPos1);
        if (i1 == -1) {
            return results;
        }
        simpleShape1 = pl1->getSegmentAt(i1);
    }
    else {
        simpleShape1 = s1->clone();
    }

    if (isPolylineShape(shape2.get())) {
        auto &&pl2 = dynamic_cast<Polyline *>(shape2.get());
        i2 = pl2->getClosestSegment(clickPos2);
        if (i2 == -1) {
            return results;
        }
        simpleShape2 = pl2->getSegmentAt(i2);
    }
    else {
        simpleShape2 = s2->clone();
    }

    // create two temporary parallels:
    auto &&parallels1 =
        simpleShape1->getOffsetShapes(radius, 1, cada::NS::NoSide, p);
    auto &&parallels2 =
        simpleShape2->getOffsetShapes(radius, 1, cada::NS::NoSide, p);

    if (parallels1.size() != 1 || parallels2.size() != 1) {
        return results;
    }

    auto &&parallel1 = parallels1[0];
    auto &&parallel2 = parallels2[0];

    auto &&ipParallel =
        parallel1->getIntersectionPoints(parallel2.get(), false);
    if (ipParallel.empty()) {
        return results;
    }

    auto &&sol2 =
        simpleShape1->getIntersectionPoints(simpleShape2.get(), false);

    // there might be two intersections: choose the closest:
    Vec2d ip = p.getClosest(ipParallel);
    Vec2d p1 = simpleShape1->getClosestPointOnShape(ip, false);
    Vec2d p2 = simpleShape2->getClosestPointOnShape(ip, false);
    double ang1 = ip.getAngleTo(p1);
    double ang2 = ip.getAngleTo(p2);
    bool reversed = (Math::getAngleDifference(ang1, ang2) > M_PI);

    auto &&arc =
        ShapeFactory::instance()->createArc(ip, radius, ang1, ang2, reversed);
    // Vec2d bp1 = arc.getStartPoint();
    // Vec2d bp2 = arc.getEndPoint();

    std::unique_ptr<Shape> trimmed1, trimmed2;
    if (samepolyline) {
        trimmed1 = simpleShape1->clone();
        trimmed2 = simpleShape2->clone();
    }
    else {
        trimmed1 = shape1->clone();
        trimmed2 = shape2->clone();
    }

    NS::Ending ending1 = NS::EndingNone;
    NS::Ending ending2 = NS::EndingNone;

    if (trim || samepolyline) {
        // trim entities to intersection
        Vec2d is2 = clickPos2.getClosest(sol2);
        ending1 = trimmed1->getTrimEnd(is2, clickPos1);
        switch (ending1) {
        case NS::EndingStart:
            trimmed1->trimStartPoint(p1, clickPos1);
            if (isXLineShape(trimmed1.get())) {
                trimmed1 = xLineToRay(trimmed1.release());
            }
            break;
        case NS::EndingEnd:
            trimmed1->trimEndPoint(p1, clickPos1);
            if (isXLineShape(trimmed1.get())) {
                trimmed1 = xLineToRay(trimmed1.release());
            }
            else if (isRayShape(trimmed1.get())) {
                trimmed1 = rayToLine(trimmed1.release());
            }
            break;
        default:
            break;
        }

        is2 = clickPos1.getClosest(sol2);
        ending2 = trimmed2->getTrimEnd(is2, clickPos2);
        switch (ending2) {
        case NS::EndingStart:
            trimmed2->trimStartPoint(p2, clickPos2);
            if (isXLineShape(trimmed2.get())) {
                trimmed2 = xLineToRay(trimmed2.release());
            }
            break;
        case NS::EndingEnd:
            trimmed2->trimEndPoint(p2, clickPos2);
            if (isXLineShape(trimmed2.get())) {
                trimmed2 = xLineToRay(trimmed2.release());
            }
            else if (isRayShape(trimmed2.get())) {
                trimmed2 = rayToLine(trimmed2.release());
            }
            break;
        default:
            break;
        }
    }

    if (samepolyline) {
        auto &&pl1 = dynamic_cast<Polyline *>(shape1.get());
        auto &&pl =
            pl1->modifyPolylineCorner(trimmed1.get(), ending1, i1,
                                      trimmed2.get(), ending2, i2, arc.get());
        results.emplace_back(std::move(pl));
        return results;
    }

    if (Math::fuzzyAngleCompare(arc->getStartAngle(), arc->getEndAngle())) {
        // rounding is full circle (two shapes are tangential):
        return results;
    }

    if (isArcShape(trimmed1.get())) {
        auto &&arc1 = dynamic_cast<Arc *>(trimmed1.get());
        if (Math::fuzzyAngleCompare(arc1->getStartAngle(),
                                    arc1->getEndAngle())) {
            // trimmed shape is full circle (two shapes are tangential):
            return results;
        }
    }
    if (isArcShape(trimmed2.get())) {
        auto &&arc2 = dynamic_cast<Arc *>(trimmed2.get());
        if (Math::fuzzyAngleCompare(arc2->getStartAngle(),
                                    arc2->getEndAngle())) {
            // trimmed shape is full circle (two shapes are tangential):
            return results;
        }
    }

    results.emplace_back(std::move(trimmed1));
    results.emplace_back(std::move(arc));
    results.emplace_back(std::move(trimmed2));
    return results;
}

bool lengthen(shape::Shape *shape, const shape::Vec2d &position,
              bool trimStartPoint, double amount)
{
    assert(shape);

    bool trimStartPoint = false;

    if (isPolylineShape(shape)) {
        auto &&pline = dynamic_cast<const shape::Polyline *>(shape);
        trimStartPoint = pline->getLengthTo(position) < pline->getLength() / 2;
    }
    else {
        trimStartPoint = position.getDistanceTo(shape->getStartPoint()) <
                         position.getDistanceTo(shape->getEndPoint());
    }

    NS::From from;
    if (trimStartPoint) {
        from = NS::FromStart;
    }
    else {
        from = NS::FromEnd;
    }

    std::vector<Vec2d> iss;
    if (isPolylineShape(shape)) {
        auto &&pline = dynamic_cast<const shape::Polyline *>(shape);
        iss = pline->getPointsWithDistanceToEnd(-amount,
                                                from | NS::AlongPolyline);
    }
    else {
        iss = shape->getPointsWithDistanceToEnd(-amount,
                                                from | NS::AlongPolyline);
    }

    auto &&is = position.getClosest(iss);

    if (!is.isValid()) {
        return false;
    }

    if (trimStartPoint) {
        shape->trimStartPoint(is, is, amount > 0);
    }
    else {
        shape->trimEndPoint(is, is, amount > 0);
    }

    return true;
}

std::vector<std::unique_ptr<Circle>> apollonius_solutions(const Shape *shape1,
                                                          const Shape *shape2,
                                                          const Shape *shape3)
{
    assert(shape1);
    assert(shape2);
    assert(shape3);

    apollonius_t *apo = apollonius_init();
    if (!apo) {
        return {};
    }

#define apo_append_shape(shp)                                                 \
    {                                                                         \
        if (shp->getShapeType() == NS::Point) {                               \
            auto p = dynamic_cast<const shape::Point *>(shp);                 \
            if (0 != apollonius_add_point(apo, p->getPosition().x,            \
                                          p->getPosition().y))                \
                goto apo_error;                                               \
        }                                                                     \
        else if (shp->getShapeType() == NS::Line) {                           \
            auto l = dynamic_cast<const shape::Line *>(shp);                  \
            if (0 != apollonius_add_line(                                     \
                         apo, l->getStartPoint().x, l->getStartPoint().y,     \
                         l->getEndPoint().x, l->getEndPoint().y))             \
                goto apo_error;                                               \
        }                                                                     \
        else if (shp->getShapeType() == NS::Circle) {                         \
            auto c = dynamic_cast<const shape::Circle *>(shp);                \
            if (0 != apollonius_add_circle(apo, c->getCenter().x,             \
                                           c->getCenter().y, c->getRadius())) \
                goto apo_error;                                               \
        }                                                                     \
        else if (shp->getShapeType() == NS::Arc) {                            \
            auto a = dynamic_cast<const shape::Arc *>(shp);                   \
            if (0 != apollonius_add_circle(apo, a->getCenter().x,             \
                                           a->getCenter().y, a->getRadius())) \
                goto apo_error;                                               \
        }                                                                     \
        else {                                                                \
            goto apo_error;                                                   \
        }                                                                     \
    }

    apo_append_shape(shape1);
    apo_append_shape(shape2);
    apo_append_shape(shape3);
    apollonius_solution *sulu;
    if (apollonius_solve(apo, &sulu) == 0) {
        std::vector<std::unique_ptr<Circle>> circles;
        for (size_t i = 0; i < sulu->count; ++i) {
            circles.emplace_back(ShapeFactory::instance()->createCircle(
                Vec2d(sulu->circles[i].cx, sulu->circles[i].cy),
                sulu->circles[i].r));
        }
        apollonius_free(apo);
        apollonius_solution_free(sulu);
        return circles;
    }

apo_error:
    apollonius_free(apo);
    return {};
}

/* ------------------------- inner static functions ------------------------- */

std::vector<Vec2d>
cada__get_intersection_points(const Shape *shape,
                              const std::vector<Shape *> &otherShapes,
                              bool onShape, bool onOtherShapes)
{
    std::vector<Vec2d> intersections;
    assert(shape);

    // treat start and end points as intersection points for open shapes:
    // treat start and end points as intersection points for open shapes:
    if (onShape && !isCircleShape(shape) && !isFullEllipseShape(shape) &&
        !isXLineShape(shape) &&
        (!isPolylineShape(shape) || !shapeIsGeometricallyClosed(shape)) &&
        (!isSplineShape(shape) || !shapeIsClosed(shape))) {

        Vec2d sp = shape->getStartPoint();

        intersections.push_back(sp);

        if (!isRayShape(shape)) {
            intersections.push_back(shape->getEndPoint());
        }
    }

    // find all intersection points:
    for (size_t i = 0; i < otherShapes.size(); ++i) {
        auto &&otherShape = otherShapes[i];

        auto &&sol =
            shape->getIntersectionPoints(otherShape, onShape, false, true);
        for (size_t k = 0; k < sol.size(); ++k) {
            if (!onOtherShapes || otherShape->isOnShape(sol[k])) {
                intersections.push_back(sol[k]);
            }
        }
    }

    auto &&selfIntersectionPoints = shape->getSelfIntersectionPoints();

    // add self intersection points to list:
    if (selfIntersectionPoints.size() != 0) {
        intersections.insert(intersections.end(),
                             selfIntersectionPoints.begin(),
                             selfIntersectionPoints.end());
    }

    return intersections;
}

std::unique_ptr<shape::Shape> cada__closest_intersection_point_distances(
    const shape::Shape *shp, const std::vector<Vec2d> &intersections,
    const Vec2d &pos, std::array<std::pair<Vec2d, double>, 2> &res)
{
    assert(shp);
    // inner clone, and return clone or reset result
    std::unique_ptr<shape::Shape> shape = shp->clone();

    // closed circular shapes:
    bool circular = false;
    if (shape->getShapeType() == NS::Circle) {
        auto cir = dynamic_cast<const shape::Circle *>(shape.get());
        double a = cir->getCenter().getAngleTo(pos);
        shape.reset(
            ShapeFactory::instance()
                ->createArc(cir->getCenter(), cir->getRadius(), a, a, false)
                .release());
        circular = true;
    }

    if (shape->getShapeType() == NS::Polyline) {
        auto poly = dynamic_cast<const shape::Polyline *>(shape.get());
        if (poly->isGeometricallyClosed())
            circular = true;
    }
    if (shape->getShapeType() == NS::Arc) {
        circular = true;
    }

    double pDist = shape->getDistanceFromStart(pos);

    std::unique_ptr<Line> orthoLine;
    bool reversedShape = false;
    if (shape->getShapeType() == NS::Ellipse) {
        auto ellipse = dynamic_cast<const shape::Ellipse *>(shape.get());
        orthoLine =
            ShapeFactory::instance()->createLine(ellipse->getCenter(), pos);
        if (ellipse->isReversed()) {
            shape->reverse();
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

        if (shape->getShapeType() == NS::Ellipse) {
            auto ellipse = dynamic_cast<const shape::Ellipse *>(shape.get());
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
            std::vector<double> dists = shape->getDistancesFromStart(ip);
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
                          shape->getShapeType() == NS::Polyline))) {
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

    if (shape->getShapeType() == NS::Ellipse) {
        auto ellipse = dynamic_cast<const shape::Ellipse *>(shape.get());
        if (ellipse->isReversed()) {
            std::swap(cutPos1, cutPos2);
            std::swap(cutDist1, cutDist2);
        }
    }

    // open shape: cut to start or end point:
    if (!isCircleShape(shape.get()) && !isFullEllipseShape(shape.get()) &&
        !isXLineShape(shape.get()) && !isRayShape(shape.get()) &&
        (!isPolylineShape(shape.get()) ||
         !shapeIsGeometricallyClosed(shape.get())) &&
        (!isSplineShape(shape.get()) || !shapeIsClosed(shape.get()))) {

        if (!cutPos1.isValid()) {
            cutDist1 = 0.0;
            cutPos1 = shape->getStartPoint();
        }
        if (!cutPos2.isValid()) {
            cutDist2 = shape->getLength();
            cutPos2 = shape->getEndPoint();
        }
    }

    if (reversedShape) {
        shape->reverse();
    }

    res[0] = {cutPos1, cutDist1};
    res[1] = {cutPos2, cutDist2};
    return shape;
};

std::unique_ptr<Shape> TrimStartPoint(shape::Shape *shape,
                                      const Vec2d &trimPoint,
                                      const Vec2d &clickPoint)
{
    shape->trimStartPoint(trimPoint, clickPoint);
    if (isXLineShape(shape)) {
        auto &&xline = dynamic_cast<XLine *>(shape);
        return ShapeFactory::instance()->createRay(xline->getBasePoint(),
                                                   xline->getDirectionVector());
    }
    else {
        return shape->clone();
    }
}

std::unique_ptr<Shape> TrimEndPoint(shape::Shape *shape, const Vec2d &trimPoint,
                                    const Vec2d &clickPoint)
{
    shape->trimEndPoint(trimPoint, clickPoint);
    if (isXLineShape(shape)) {
        auto &&xline = dynamic_cast<XLine *>(shape);
        return ShapeFactory::instance()->createRay(xline->getBasePoint(),
                                                   xline->getDirectionVector());
    }
    else {
        return shape->clone();
    }
}

std::unique_ptr<Shape> xLineToRay(const shape::Shape *shape)
{
    assert(isXLineShape(shape));
    auto &&xline = dynamic_cast<const XLine *>(shape);
    return ShapeFactory::instance()->createRay(xline->getBasePoint(),
                                               xline->getDirectionVector());
}

std::unique_ptr<Shape> rayToXLine(const shape::Shape *shape)
{
    assert(isRayShape(shape));
    auto &&ray = dynamic_cast<const Ray *>(shape);
    return ShapeFactory::instance()->createXLine(ray->getBasePoint(),
                                                 ray->getDirectionVector());
}

std::unique_ptr<Shape> rayToLine(const shape::Shape *shape)
{
    assert(isRayShape(shape));
    auto &&ray = dynamic_cast<const Ray *>(shape);
    return ShapeFactory::instance()->createLine(ray->getBasePoint(),
                                                ray->getDirectionVector());
}

} // namespace algorithm
} // namespace cada