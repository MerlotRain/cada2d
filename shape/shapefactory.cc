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

#include "cada_shape.h"
#include <cmath>
#include <Eigen/Dense>

namespace cada {
namespace shape {

bool threePointCollinear(const Vec2d &A, const Vec2d &B, const Vec2d &C)
{
    double area = A.x * (B.y - C.y) + B.x * (C.y - A.y) + C.x * (A.y - B.y);
    return area == 0;
}

ShapeFactory *g_shapefactory = nullptr;

auto ShapeFactory::instance() -> const ShapeFactory *
{
    static std::once_flag flag;
    std::call_once(flag, [&]() { g_shapefactory = new ShapeFactory(); });
    return g_shapefactory;
}

auto ShapeFactory::createPoint() const -> std::unique_ptr<Point>
{
    return std::unique_ptr<Point>();
}

auto ShapeFactory::createPoint(double x, double y) const
    -> std::unique_ptr<Point>
{
    return std::unique_ptr<Point>(new Point(x, y));
}

auto ShapeFactory::createPoint(const Vec2d &point) const
    -> std::unique_ptr<Point>
{
    return std::unique_ptr<Point>(new Point(point));
}

auto ShapeFactory::createLine() const -> std::unique_ptr<Line>
{
    return std::unique_ptr<Line>(new Line());
}

auto ShapeFactory::createLine(double x1, double y1, double x2, double y2) const
    -> std::unique_ptr<Line>
{
    return std::unique_ptr<Line>(new Line(x1, y1, x2, y2));
}

auto ShapeFactory::createLine(const Vec2d &startPoint,
                              const Vec2d &endPoint) const
    -> std::unique_ptr<Line>
{
    return std::unique_ptr<Line>(new Line(startPoint, endPoint));
}

auto ShapeFactory::createLine(const Vec2d &startPoint, double angle,
                              double ditance) const -> std::unique_ptr<Line>
{
    return std::unique_ptr<Line>(new Line(startPoint, angle, ditance));
}

auto ShapeFactory::createPolyline() const -> std::unique_ptr<Polyline>
{
    return std::unique_ptr<Polyline>(new Polyline());
}

auto ShapeFactory::createPolyline(std::vector<Vec2d> &&vertrices, bool closed,
                                  std::vector<double> &&bulges,
                                  std::vector<double> &&endWidths,
                                  std::vector<double> &&startWidths) const
    -> std::unique_ptr<Polyline>
{
    auto polyline =
        std::unique_ptr<Polyline>(new Polyline(std::move(vertrices), closed));
    polyline->setBulges(std::move(bulges));
    polyline->setEndWidths(std::move(endWidths));
    polyline->setStartWidths(std::move(startWidths));
    return polyline;
}

auto ShapeFactory::createArc() const -> std::unique_ptr<Arc>
{
    return std::unique_ptr<Arc>(new Arc());
}

auto ShapeFactory::createArc(const Vec2d &center, double radius,
                             double startAngle, double endAngle,
                             bool reversed) const -> std::unique_ptr<Arc>
{
    return std::unique_ptr<Arc>(
        new Arc(center, radius, startAngle, endAngle, reversed));
}

auto ShapeFactory::createArc(double cx, double xy, double radius,
                             double startAngle, double endAngle,
                             bool reversed) const -> std::unique_ptr<Arc>
{
    return std::unique_ptr<Arc>(
        new Arc(cx, xy, radius, startAngle, endAngle, reversed));
}

auto ShapeFactory::createArcFrom3Point(const Vec2d &startPoint,
                                       const Vec2d &point,
                                       const Vec2d &endPoint) const
    -> std::unique_ptr<Arc>
{
    // intersection of two middle lines

    // middle points between first two points:
    Vec2d mp1 = Vec2d::getAverage(startPoint, point);
    double a1 = startPoint.getAngleTo(point) + M_PI / 2.0;
    // direction from middle point to center:
    Vec2d dir1 = Vec2d::createPolar(1.0, a1);

    // middle points between last two points:
    Vec2d mp2 = Vec2d::getAverage(point, endPoint);
    double a2 = point.getAngleTo(endPoint) + M_PI / 2.0;
    // direction from middle point to center:
    Vec2d dir2 = Vec2d::createPolar(1.0, a2);

    auto midLine1 = createLine(mp1, mp1 + dir1);
    auto midLine2 = createLine(mp2, mp2 + dir2);

    std::vector<Vec2d> ips =
        midLine1->getIntersectionPoints(midLine2.release(), false);
    if (ips.size() != 1) {
        return nullptr;
    }

    Vec2d center = ips[0];
    double radius = center.getDistanceTo(endPoint);
    double angle1 = center.getAngleTo(startPoint);
    double angle2 = center.getAngleTo(endPoint);
    bool reversed =
        Math::isAngleBetween(center.getAngleTo(point), angle1, angle2, true);

    return createArc(center, radius, angle1, angle2, reversed);
}

auto ShapeFactory::createArcFrom2PBulgs(const Vec2d &startPoint,
                                        const Vec2d &endPoint,
                                        double bulge) const
    -> std::unique_ptr<Arc>
{
    auto arc = createArc();

    arc->mReversed = (bulge < 0.0);
    double alpha = atan(bulge) * 4.0;

    Vec2d middle = (startPoint + endPoint) / 2.0;
    double dist = startPoint.getDistanceTo(endPoint) / 2.0;

    // alpha can't be 0.0 at this point
    arc->mRadius = fabs(dist / sin(alpha / 2.0));

    double wu = fabs(std::pow(arc->mRadius, 2.0) - std::pow(dist, 2.0));
    double h = sqrt(wu);
    double angle = startPoint.getAngleTo(endPoint);

    if (bulge > 0.0) {
        angle += M_PI / 2.0;
    }
    else {
        angle -= M_PI / 2.0;
    }

    if (fabs(alpha) > M_PI) {
        h *= -1.0;
    }

    arc->mCenter.setPolar(h, angle);
    arc->mCenter += middle;
    arc->mStartAngle = arc->mCenter.getAngleTo(startPoint);
    arc->mEndAngle = arc->mCenter.getAngleTo(endPoint);

    return arc;
}

auto ShapeFactory::createArcFromTangential(const Vec2d &startPoint,
                                           const Vec2d &pos, double direction,
                                           double radius) const
    -> std::unique_ptr<Arc>
{
    auto arc = createArc();

    arc->mRadius = radius;

    // orthogonal to base entity:
    Vec2d ortho;
    ortho.setPolar(radius, direction + M_PI / 2.0);

    // two possible center points for arc:
    Vec2d center1 = startPoint + ortho;
    Vec2d center2 = startPoint - ortho;
    if (center1.getDistanceTo(pos) < center2.getDistanceTo(pos)) {
        arc->mCenter = center1;
    }
    else {
        arc->mCenter = center2;
    }

    // angles:
    arc->mStartAngle = arc->mCenter.getAngleTo(startPoint);
    arc->mEndAngle = arc->mCenter.getAngleTo(pos);

    // handle arc direction:
    arc->mReversed = false;
    double diff = Math::getNormalizedAngle(arc->getDirection1() - direction);
    if (fabs(diff - M_PI) < 1.0e-1) {
        arc->mReversed = true;
    }

    return arc;
}

auto ShapeFactory::createArcFromBiarc(const Vec2d &startPoint,
                                      double startDirection,
                                      const Vec2d &endPoint,
                                      double endDirection, bool secondTry) const
    -> std::vector<std::unique_ptr<Arc>>
{
    std::vector<std::unique_ptr<Arc>> rets;

    double length = startPoint.getDistanceTo(endPoint);
    double angle = startPoint.getAngleTo(endPoint);

    double alpha = Math::getAngleDifference180(startDirection, angle);
    double beta = Math::getAngleDifference180(angle, endDirection);

    double theta;
    if ((alpha > 0 && beta > 0) || (alpha < 0 && beta < 0)) {
        // same sign: C-shaped curve:
        theta = alpha;
    }
    else {
        // different sign: S-shaped curve:
        theta = (3.0 * alpha - beta) / 2.0;
    }

    Vec2d startNormal(-sin(startDirection), cos(startDirection));
    Vec2d jointPointNormal(-sin(theta + startDirection),
                           cos(theta + startDirection));

    double term1 = (length / (2.0 * sin((alpha + beta) / 2.0)));

    double radius1 =
        term1 * (sin((beta - alpha + theta) / 2.0) / sin(theta / 2.0));
    double radius2 = term1 * (sin((2.0 * alpha - theta) / 2.0) /
                              sin((alpha + beta - theta) / 2.0));

    // failed, might succeed in reverse direction:
    if (std::fabs(radius1) < NS::PointTolerance ||
        std::fabs(radius2) < NS::PointTolerance || !Math::isNormal(radius1) ||
        !Math::isNormal(radius2)) {

        if (secondTry) {
            return rets;
        }

        std::vector<std::unique_ptr<Arc>> list =
            createArcFromBiarc(endPoint, endDirection + M_PI, startPoint,
                               startDirection + M_PI, true);
        if (list.empty()) {
            return rets;
        }

        for (size_t i = 0; i < list.size(); i++) {
            list[i]->reverse();
        }
        rets.emplace_back(list[1].release());
        rets.emplace_back(list[0].release());
        return rets;
    }

    Vec2d jointPoint = startPoint + (startNormal - jointPointNormal) * radius1;

    Vec2d center1 = startPoint + startNormal * radius1;
    Vec2d center2 = jointPoint + jointPointNormal * radius2;

    auto arc1 =
        createArc(center1, std::fabs(radius1), center1.getAngleTo(startPoint),
                  center1.getAngleTo(jointPoint));
    if (std::fabs(Math::getAngleDifference180(arc1->getDirection1(),
                                              startDirection)) > 0.1) {
        arc1->setReversed(true);
    }

    auto arc2 =
        createArc(center2, std::fabs(radius2), center2.getAngleTo(jointPoint),
                  center2.getAngleTo(endPoint));
    if (std::fabs(Math::getAngleDifference180(arc2->getDirection2() + M_PI,
                                              endDirection)) > 0.1) {
        arc2->setReversed(true);
    }

    rets.emplace_back(std::move(arc1));
    rets.emplace_back(std::move(arc2));
    return rets;
}

auto ShapeFactory::createCircle() const -> std::unique_ptr<Circle>
{
    return std::unique_ptr<Circle>();
}

auto ShapeFactory::createCircle(const Vec2d &center, double radius) const
    -> std::unique_ptr<Circle>
{
    return std::unique_ptr<Circle>(new Circle(center, radius));
}

auto ShapeFactory::createCircle(double cx, double cy, double radius) const
    -> std::unique_ptr<Circle>
{
    return std::unique_ptr<Circle>(new Circle(cx, cy, radius));
}

auto ShapeFactory::createCircleFrom2Points(const Vec2d &p1,
                                           const Vec2d &p2) const
    -> std::unique_ptr<Circle>
{
    Vec2d center = (p1 + p2) / 2.0;
    double radius = p1.getDistanceTo(p2) / 2.0;
    return createCircle(center, radius);
}

auto ShapeFactory::createCircleFrom3Points(const Vec2d &p1, const Vec2d &p2,
                                           const Vec2d &p3) const
    -> std::unique_ptr<Circle>
{
    // intersection of two middle lines

    // middle points between first two points:
    Vec2d mp1 = Vec2d::getAverage(p1, p2);
    double a1 = p1.getAngleTo(p2) + M_PI / 2.0;
    // direction from middle point to center:
    Vec2d dir1 = Vec2d::createPolar(1.0, a1);

    // middle points between last two points:
    Vec2d mp2 = Vec2d::getAverage(p2, p3);
    double a2 = p2.getAngleTo(p3) + M_PI / 2.0;
    // direction from middle point to center:
    Vec2d dir2 = Vec2d::createPolar(1.0, a2);

    auto midLine1 = createLine(mp1, mp1 + dir1);
    auto midLine2 = createLine(mp2, mp2 + dir2);

    std::vector<Vec2d> ips =
        midLine1->getIntersectionPoints(midLine2.release(), false);
    if (ips.size() != 1) {
        return nullptr;
    }

    Vec2d center = ips[0];
    double radius = center.getDistanceTo(p3);

    return createCircle(center, radius);
}

auto ShapeFactory::createEllipse() const -> std::unique_ptr<Ellipse>
{
    return std::unique_ptr<Ellipse>();
}

auto ShapeFactory::createEllipse(const Vec2d &center, const Vec2d &majorPoint,
                                 double ratio, double startParam,
                                 double endParam, bool reversed) const
    -> std::unique_ptr<Ellipse>
{
    return std::unique_ptr<Ellipse>(
        new Ellipse(center, majorPoint, ratio, startParam, endParam, reversed));
}

auto ShapeFactory::createEllipseFromInscribed(const Vec2d &p1, const Vec2d &p2,
                                              const Vec2d &p3, const Vec2d &p4,
                                              const Vec2d &centerHint) const
    -> std::unique_ptr<Ellipse>
{
    return nullptr;
}

auto ShapeFactory::createEllipseFrom4Points(const Vec2d &p1, const Vec2d &p2,
                                            const Vec2d &p3,
                                            const Vec2d &p4) const
    -> std::unique_ptr<Ellipse>
{
    if (threePointCollinear(p1, p2, p3) && threePointCollinear(p1, p2, p4)) {
        return nullptr;
    }

    auto solveEllipse =
        [](const std::vector<Vec2d> &points) -> Eigen::VectorXd {
        Eigen::MatrixXd A(4, 5);
        Eigen::VectorXd b(4);

        for (int i = 0; i < 4; ++i) {
            double x = points[i].x;
            double y = points[i].y;
            A(i, 0) = x * x; // A
            A(i, 1) = x * y; // B
            A(i, 2) = y * y; // C
            A(i, 3) = x;     // D
            A(i, 4) = y;     // E
            b(i) = -1.0;     // F
        }

        Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);
        return x;
    };

    std::vector<Vec2d> points = {p1, p2, p3, p4};
    Eigen::VectorXd coeffs = solveEllipse(points);

    double A = coeffs(0);
    double B = coeffs(1);
    double C = coeffs(2);
    double D = coeffs(3);
    double E = coeffs(4);
    double F = -1;

    Vec2d center(-D / (2 * A), -E / (2 * C));

    double term1 = 4 * A * C - B * B;
    double majorLength = sqrt(
        (2 * (A * center.x * center.x + C * center.y * center.y - F)) / term1);

    double minorLength =
        sqrt((2 * (A * center.x * center.x + C * center.y * center.y - F)) /
             (4 * A * C));

    double majorAxis = std::max(majorLength, minorLength);
    double minorAxis = std::min(majorLength, minorLength);
    double ratio = majorAxis / minorAxis;

    double theta = 0.5 * atan2(B, A - C);

    Vec2d majorPoint(center.x + majorAxis * cos(theta),
                     center.y + majorAxis * sin(theta));

    return createEllipse(center, majorPoint, ratio, 0, 360, false);
}

auto ShapeFactory::createXLine() const -> std::unique_ptr<XLine>
{
    return std::unique_ptr<XLine>(new XLine());
}

auto ShapeFactory::createXLine(const Vec2d &basePoint,
                               const Vec2d &directionVector) const
    -> std::unique_ptr<XLine>
{
    return std::unique_ptr<XLine>(new XLine(basePoint, directionVector));
}

auto ShapeFactory::createXLine(const Vec2d &basePoint, double angle,
                               double distance) const -> std::unique_ptr<XLine>
{
    return std::unique_ptr<XLine>(new XLine(basePoint, angle, distance));
}

auto ShapeFactory::createRay() const -> std::unique_ptr<Ray>
{
    return std::unique_ptr<Ray>(new Ray());
}

auto ShapeFactory::createRay(const Vec2d &basePoint,
                             const Vec2d &directionVector) const
    -> std::unique_ptr<Ray>
{
    return std::unique_ptr<Ray>(new Ray(basePoint, directionVector));
}

auto ShapeFactory::createRay(const Vec2d &basePoint, double angle,
                             double distance) const -> std::unique_ptr<Ray>
{
    return std::unique_ptr<Ray>(new Ray(basePoint, angle, distance));
}

auto ShapeFactory::createBSpline() const -> std::unique_ptr<BSpline>
{
    return std::unique_ptr<BSpline>(new BSpline());
}

auto ShapeFactory::createBSpline(std::vector<Vec2d> &&controlPoints,
                                 int degree) const -> std::unique_ptr<BSpline>
{
    return std::unique_ptr<BSpline>(
        new BSpline(std::move(controlPoints), degree));
}

} // namespace shape
} // namespace cada
