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
#include "cada_algorithm.h"
#include <cmath>
#include <mutex>
#include <algorithm>
#include <assert.h>

namespace cada {
namespace shape {

bool threePointCollinear(const Vec2d &A, const Vec2d &B, const Vec2d &C)
{
    double area = A.x * (B.y - C.y) + B.x * (C.y - A.y) + C.x * (A.y - B.y);
    return area == 0;
}

ShapeFactory *g_shapefactory = nullptr;

ShapeFactory::ShapeFactory() = default;

const ShapeFactory *ShapeFactory::instance()
{
    static std::once_flag flag;
    std::call_once(flag, [&]() { g_shapefactory = new ShapeFactory(); });
    return g_shapefactory;
}

std::unique_ptr<Point> ShapeFactory::createPoint() const
{
    return std::unique_ptr<Point>(new Point());
}

std::unique_ptr<Point> ShapeFactory::createPoint(double x, double y) const

{
    return std::unique_ptr<Point>(new Point(x, y));
}

std::unique_ptr<Point> ShapeFactory::createPoint(const Vec2d &point) const

{
    return std::unique_ptr<Point>(new Point(point));
}

std::unique_ptr<Line> ShapeFactory::createLine() const
{
    return std::unique_ptr<Line>(new Line());
}

std::unique_ptr<Line> ShapeFactory::createLine(double x1, double y1, double x2,
                                               double y2) const

{
    return std::unique_ptr<Line>(new Line(x1, y1, x2, y2));
}

std::unique_ptr<Line> ShapeFactory::createLine(const Vec2d &startPoint,
                                               const Vec2d &endPoint) const

{
    return std::unique_ptr<Line>(new Line(startPoint, endPoint));
}

std::unique_ptr<Line> ShapeFactory::createLine(const Vec2d &startPoint,
                                               double angle,
                                               double ditance) const
{
    return std::unique_ptr<Line>(new Line(startPoint, angle, ditance));
}

std::unique_ptr<Polyline> ShapeFactory::createPolyline() const
{
    return std::unique_ptr<Polyline>(new Polyline());
}

std::unique_ptr<Polyline>
ShapeFactory::createPolyline(std::vector<Vec2d> &&vertrices, bool closed,
                             std::vector<double> &&bulges) const

{
    auto polyline =
        std::unique_ptr<Polyline>(new Polyline(std::move(vertrices), closed));
    polyline->setBulges(std::move(bulges));
    return polyline;
}

std::unique_ptr<Arc> ShapeFactory::createArc() const
{
    return std::unique_ptr<Arc>(new Arc());
}

std::unique_ptr<Arc> ShapeFactory::createArc(const Vec2d &center, double radius,
                                             double startAngle, double endAngle,
                                             bool reversed) const
{
    return std::unique_ptr<Arc>(
        new Arc(center, radius, startAngle, endAngle, reversed));
}

std::unique_ptr<Arc> ShapeFactory::createArc(double cx, double xy,
                                             double radius, double startAngle,
                                             double endAngle,
                                             bool reversed) const
{
    return std::unique_ptr<Arc>(
        new Arc(cx, xy, radius, startAngle, endAngle, reversed));
}

std::unique_ptr<Arc>
ShapeFactory::createArcFrom3Point(const Vec2d &startPoint, const Vec2d &point,
                                  const Vec2d &endPoint) const

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
        midLine1->getIntersectionPoints(midLine2.get(), false);
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

std::unique_ptr<Arc> ShapeFactory::createArcFrom2PBulge(const Vec2d &startPoint,
                                                        const Vec2d &endPoint,
                                                        double bulge) const

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

std::unique_ptr<Arc>
ShapeFactory::createArcFromTangential(const Vec2d &startPoint, const Vec2d &pos,
                                      double direction, double radius) const

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

std::vector<std::unique_ptr<Arc>>
ShapeFactory::createArcFromBiarc(const Vec2d &startPoint, double startDirection,
                                 const Vec2d &endPoint, double endDirection,
                                 bool secondTry) const

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

std::unique_ptr<Circle> ShapeFactory::createCircle() const
{
    return std::unique_ptr<Circle>();
}

std::unique_ptr<Circle> ShapeFactory::createCircle(const Vec2d &center,
                                                   double radius) const

{
    return std::unique_ptr<Circle>(new Circle(center, radius));
}

std::unique_ptr<Circle> ShapeFactory::createCircle(double cx, double cy,
                                                   double radius) const

{
    return std::unique_ptr<Circle>(new Circle(cx, cy, radius));
}

std::unique_ptr<Circle>
ShapeFactory::createCircleFrom2Points(const Vec2d &p1, const Vec2d &p2) const

{
    Vec2d center = (p1 + p2) / 2.0;
    double radius = p1.getDistanceTo(p2) / 2.0;
    return createCircle(center, radius);
}

std::unique_ptr<Circle>
ShapeFactory::createCircleFrom3Points(const Vec2d &p1, const Vec2d &p2,
                                      const Vec2d &p3) const

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
        midLine1->getIntersectionPoints(midLine2.get(), false);
    if (ips.size() != 1) {
        return nullptr;
    }

    Vec2d center = ips[0];
    double radius = center.getDistanceTo(p3);

    return createCircle(center, radius);
}

std::unique_ptr<Ellipse> ShapeFactory::createEllipse() const
{
    return std::unique_ptr<Ellipse>(new Ellipse());
}

std::unique_ptr<Ellipse>
ShapeFactory::createEllipse(const Vec2d &center, const Vec2d &majorPoint,
                            double ratio, double startParam, double endParam,
                            bool reversed) const

{
    return std::unique_ptr<Ellipse>(
        new Ellipse(center, majorPoint, ratio, startParam, endParam, reversed));
}

std::unique_ptr<Ellipse>
ShapeFactory::createEllipseFromQuadratic(double a, double b, double c) const
{
    const double d = a - b;
    const double s = hypot(d, c);

    if (s > a + b)
        return nullptr;
    std::unique_ptr<Ellipse> ellipse = createEllipse();
    if (a >= b) {
        ellipse->setMajorPoint(Vec2d::createPolar(1.0, atan2(d + s, -c)) /
                               sqrt(0.5 * (a + b - s)));
    }
    else {
        ellipse->setMajorPoint(Vec2d::createPolar(1.0, atan2(-c, s - d)) /
                               sqrt(0.5 * (a + b - s)));
    }
    ellipse->setRatio(sqrt((a + b - s) / (a + b + s)));
    return ellipse;
}

std::unique_ptr<Ellipse>
ShapeFactory::createEllipseFromInscribed(const Vec2d &p1, const Vec2d &p2,
                                         const Vec2d &p3, const Vec2d &p4,
                                         const Vec2d &centerHint) const

{
    std::vector<std::unique_ptr<Line>> quad;
    quad.emplace_back(createLine(p1, p2));
    quad.emplace_back(createLine(p2, p3));
    quad.emplace_back(createLine(p3, p4));
    quad.emplace_back(createLine(p4, p1));

    // center of original square projected, intersection of diagonal
    Vec2d centerProjection;
    {
        auto &&deigonal1 =
            createLine(quad[0]->getStartPoint(), quad[1]->getEndPoint());
        auto &&deigonal2 =
            createLine(quad[1]->getStartPoint(), quad[2]->getEndPoint());
        auto sol = deigonal1->getIntersectionPoints(deigonal2.get());
        if (sol.size() == 0) // this should not happen
            return nullptr;

        centerProjection = sol.at(0);
    }

    // holds the tangential points on edges, in the order of edges: 1 3 2 0
    std::vector<Vec2d> tangent;
    int parallel = 0;
    int parallel_index = 0;
    for (int i = 0; i <= 1; ++i) {
        auto sol1 = quad[i]->getIntersectionPoints(quad[(i + 1) % 4].get());
        Vec2d direction;
        if (sol1.size() == 0) {
            direction = quad[i]->getEndPoint() - quad[i]->getStartPoint();
            ++parallel;
            parallel_index = i;
        }
        else {
            direction = sol1.at(0) - centerProjection;
        }

        auto l = createLine(centerProjection, centerProjection + direction);
        for (int k = 1; k <= 3; k += 2) {
            auto sol2 = l->getIntersectionPoints(quad[(i + k) % 4].get());
            if (sol2.size() > 0)
                tangent.push_back(sol2.at(0));
        }
    }

    if (tangent.size() < 3)
        return nullptr;

    // find ellipse center by projection
    Vec2d ellipseCenter;
    {
        auto cl0 =
            createLine(quad[1]->getEndPoint(), (tangent[0] + tangent[2]) * 0.5);
        auto cl1 =
            createLine(quad[2]->getEndPoint(), (tangent[1] + tangent[2]) * 0.5);
        auto sol = cl0->getIntersectionPoints(cl1.get(), false);
        if (sol.size() == 0) {
            return nullptr;
        }
        ellipseCenter = sol.at(0);
    }

    if (parallel == 1) {
        // trapezoid
        auto &&l0 = quad[parallel_index].get();
        auto &&l1 = quad[(parallel_index + 2) % 4].get();
        Vec2d centerPoint = (l0->getMiddlePoint() + l1->getMiddlePoint()) * 0.5;
        // not symmetric, no inscribed ellipse
        if (fabs(centerPoint.getDistanceTo(l0->getStartPoint()) -
                 centerPoint.getDistanceTo(l0->getEndPoint())) >
            NS::PointTolerance)
            return nullptr;
        // symmetric
        double d = l0->getDistanceTo(centerPoint);
        double l = ((l0->getLength() + l1->getLength())) * 0.25;
        double k = 4. * d / fabs(l0->getLength() - l1->getLength());
        double theta = d / (l * k);
        if (theta >= 1. || d < NS::PointTolerance) {

            return nullptr;
        }
        theta = asin(theta);

        // major axis
        double a = d / (k * tan(theta));
        auto tmpRet = createEllipse(Vec2d(0.0, 0.0), Vec2d(a, 0.0), d / a, 0,
                                    2 * M_PI, false);
        tmpRet->rotate(l0->getAngle());
        tmpRet->setCenter(centerPoint);
        return tmpRet;
    }

    std::vector<double> dn(3);
    Vec2d angleVector;

    for (size_t i = 0; i < tangent.size(); i++) {
        // relative to ellipse center
        tangent[i] -= ellipseCenter;
    }
    std::vector<std::vector<double>> mt;
    mt.clear();
    const double symTolerance = 20. * NS::PointTolerance;
    for (const Vec2d &vp : tangent) {
        // form the linear equation need to remove duplicated {x^2, xy, y^2}
        // terms due to symmetry (x => -x, y=> -y) i.e. rotation of 180 degrees
        // around ellipse center
        std::vector<double> mtRow;
        mtRow.push_back(vp.x * vp.x);
        mtRow.push_back(vp.x * vp.y);
        mtRow.push_back(vp.y * vp.y);
        const double l = hypot(hypot(mtRow[0], mtRow[1]), mtRow[2]);
        bool addRow(true);
        for (const auto &v : mt) {
            const Vec2d dv(v[0] - mtRow[0], v[1] - mtRow[1]);
            if (dv.getMagnitude() < symTolerance * l) {
                // symmetric
                addRow = false;
                break;
            }
        }
        if (addRow) {
            mtRow.push_back(1.);
            mt.push_back(mtRow);
        }
    }
    switch (mt.size()) {
    case 2: {
        // the quadrilateral is a parallelogram fixme, need to handle degenerate
        // case better double angle(center.angleTo(tangent[0]));
        Vec2d majorP(tangent[0]);
        double dx(majorP.getMagnitude());
        if (dx < NS::PointTolerance)
            return nullptr;
        // refuse to return zero size ellipse
        angleVector.set(majorP.x / dx, -majorP.y / dx);
        for (size_t i = 0; i < tangent.size(); i++)
            tangent[i].rotate(angleVector);

        Vec2d minorP(tangent[2]);
        double dy2(minorP.getSquaredMagnitude());
        // refuse to return zero size ellipse
        if (fabs(minorP.y) < NS::PointTolerance || dy2 < NS::PointTolerance)
            return nullptr;
        double ia2 = 1. / (dx * dx);
        double ib2 = 1. / (minorP.y * minorP.y);
        dn[0] = ia2;
        dn[1] = -2. * ia2 * minorP.x / minorP.y;
        dn[2] = ib2 * ia2 * minorP.x * minorP.x + ib2;
    } break;
    case 4:
        mt.pop_back();
        // only 3 points needed to form the qudratic form
        if (!Math::linearSolver(mt, dn))
            return nullptr;
        break;
    default:
        return nullptr; // invalid quadrilateral
    }

    auto ellipseRet = createEllipseFromQuadratic(dn[0], dn[1], dn[2]);
    if (!ellipseRet)
        return nullptr;
    ellipseRet->setCenter(ellipseCenter);

    // need to rotate back, for the parallelogram case
    if (angleVector.valid) {
        angleVector.y *= -1.;
        ellipseRet->setCenter(
            ellipseRet->getCenter().rotate(ellipseCenter, angleVector));
        ellipseRet->setMajorPoint(ellipseRet->getCenter().rotate(angleVector));
    }
    return ellipseRet;
}

std::unique_ptr<Ellipse>
ShapeFactory::createEllipseFrom4Points(const Vec2d &p1, const Vec2d &p2,
                                       const Vec2d &p3, const Vec2d &p4) const

{
    std::vector<Vec2d> sol = {p1, p2, p3, p4};
    std::vector<std::vector<double>> mt;
    std::vector<double> dn;
    int mSize(4);
    mt.resize(mSize);
    for (int i = 0; i < mSize;
         i++) { // form the linear equation, c0 x^2 + c1 x + c2 y^2 + c3 y = 1
        mt[i].resize(mSize + 1);
        mt[i][0] = sol[i].x * sol[i].x;
        mt[i][1] = sol[i].x;
        mt[i][2] = sol[i].y * sol[i].y;
        mt[i][3] = sol[i].y;
        mt[i][4] = 1.;
    }
    if (!Math::linearSolver(mt, dn))
        return nullptr;
    double d(1. + 0.25 * (dn[1] * dn[1] / dn[0] + dn[3] * dn[3] / dn[2]));
    if (fabs(dn[0]) < NS::PointTolerance || fabs(dn[2]) < NS::PointTolerance ||
        d / dn[0] < NS::PointTolerance || d / dn[2] < NS::PointTolerance) {
        // ellipse not defined
        return nullptr;
    }

    Vec2d center(-0.5 * dn[1] / dn[0], -0.5 * dn[3] / dn[2]); // center
    d = sqrt(d / dn[0]);
    Vec2d majorP(d, 0.); // major point
    double ratio = sqrt(dn[0] / dn[2]);

    return createEllipse(center, majorP, ratio, 0, 2 * M_PI, false);
}

std::unique_ptr<XLine> ShapeFactory::createXLine() const
{
    return std::unique_ptr<XLine>(new XLine());
}

std::unique_ptr<XLine>
ShapeFactory::createXLine(const Vec2d &basePoint,
                          const Vec2d &directionVector) const

{
    return std::unique_ptr<XLine>(new XLine(basePoint, directionVector));
}

std::unique_ptr<XLine> ShapeFactory::createXLine(const Vec2d &basePoint,
                                                 double angle,
                                                 double distance) const
{
    return std::unique_ptr<XLine>(new XLine(basePoint, angle, distance));
}

std::unique_ptr<Ray> ShapeFactory::createRay() const
{
    return std::unique_ptr<Ray>(new Ray());
}

std::unique_ptr<Ray> ShapeFactory::createRay(const Vec2d &basePoint,
                                             const Vec2d &directionVector) const

{
    return std::unique_ptr<Ray>(new Ray(basePoint, directionVector));
}

std::unique_ptr<Ray> ShapeFactory::createRay(const Vec2d &basePoint,
                                             double angle,
                                             double distance) const
{
    return std::unique_ptr<Ray>(new Ray(basePoint, angle, distance));
}

std::unique_ptr<Spline> ShapeFactory::createBSpline() const
{
    return std::unique_ptr<Spline>(new Spline());
}

std::unique_ptr<Spline>
ShapeFactory::createBSpline(std::vector<Vec2d> &&controlPoints,
                            int degree) const
{
    return std::unique_ptr<Spline>(
        new Spline(std::move(controlPoints), degree));
}

std::vector<std::unique_ptr<Shape>>
ShapeFactory::createPolygon(const Vec2d &position1, const Vec2d &position2,
                            NS::PolygonOption option, bool create_polyline,
                            bool useRadius, double radius,
                            size_t numberOfCorners) const
{
    assert(numberOfCorners > 2);

    std::vector<Vec2d> corners;
    switch (option) {
    case NS::WithCenterCorner: {
        Vec2d center = position1;
        Vec2d corner = position2;
        for (size_t n = 1; n <= numberOfCorners; ++n) {
            Vec2d c = corner;
            c.rotate((M_PI * 2.0) / numberOfCorners * n, center);
            corners.push_back(c);
        }
        goto inner_create_shapes;
    }
    case NS::With2PointsOfSide: {
        Vec2d corner1 = position1;
        Vec2d corner2 = position2;

        Vec2d c = corner1;
        double len = corner1.getDistanceTo(corner2);
        double ang1 = corner1.getAngleTo(corner2);
        double ang = ang1;

        for (size_t n = 1; n <= numberOfCorners; ++n) {
            Vec2d edge;
            edge.setPolar(len, ang);

            corners.push_back(Vec2d(c.x, c.y, c.valid));
            c = c + edge;

            ang = ang1 + (2 * M_PI) / numberOfCorners * n;
        }
        goto inner_create_shapes;
    }
    case NS::WithCenterSide: {
        Vec2d center = position1;
        Vec2d middleOfSide = position2;

        double angle = M_PI / numberOfCorners;
        double dist = center.getDistanceTo(middleOfSide);
        double opp = std::tan(angle) * dist;
        double hyp = std::sqrt((dist * dist) + (opp * opp));
        Vec2d v =
            Vec2d::createPolar(hyp, center.getAngleTo(middleOfSide) + angle);
        Vec2d corner = center + v;

        for (size_t n = 1; n < numberOfCorners; ++n) {
            Vec2d c = corner;
            c.rotate((M_PI * 2.0) / numberOfCorners * n, center);
            corners.push_back(c);
        }
        goto inner_create_shapes;
    }
    case NS::WithSideSide: {
        Vec2d corner1 = position1;
        Vec2d corner2 = position2;

        double angle = M_PI / numberOfCorners;
        double dist = corner1.getDistanceTo(corner2) / 2.0;
        double opp = std::tan(angle) * dist;
        double hyp = std::sqrt((dist * dist) + (opp * opp));
        Vec2d cen = Vec2d::createPolar(dist, corner1.getAngleTo(corner2));
        Vec2d center = corner1 + cen;

        Vec2d v = Vec2d::createPolar(hyp, center.getAngleTo(corner1) + angle);
        Vec2d corner = center + v;

        if (numberOfCorners % 2 == 1) {
            double newdist = (dist / (dist + hyp)) * (dist * 2);
            double newopp = std::tan(angle) * newdist;
            double newhyp = std::sqrt((newdist * newdist) + (newopp * newopp));
            Vec2d newcen =
                Vec2d::createPolar(newdist, corner1.getAngleTo(corner2));
            Vec2d newcenter = corner1 + newcen;

            v = Vec2d::createPolar(newhyp,
                                   newcenter.getAngleTo(corner1) + angle);
            corner = newcenter + v;
            center = newcenter;
        }
        for (size_t n = 1; n <= numberOfCorners; ++n) {
            Vec2d c = corner;
            c.rotate((M_PI * 2.0) / numberOfCorners * n, center);
            corners.push_back(c);
        }
        goto inner_create_shapes;
    }
    default:
        return std::vector<std::unique_ptr<Shape>>();
        ;
    }

inner_create_shapes:
    std::vector<std::unique_ptr<Shape>> shapes;
    for (size_t i = 0; i < corners.size(); ++i) {
        shapes.emplace_back(
            createLine(corners.at(i), corners.at((i + 1) % corners.size())));
    }
    if (useRadius && radius > 0) {
        std::vector<std::unique_ptr<Shape>> newShapes;
        Vec2d cursor = Vec2d::invalid;
        for (size_t i = 0; i < shapes.size(); ++i) {
            auto &&s1 = shapes.at(i);
            Vec2d clickPos1 =
                s1->getPointWithDistanceToEnd(s1->getLength() / 3);
            auto &&s2 = shapes.at((i + 1) % shapes.size());
            Vec2d clickPos2 =
                s2->getPointWithDistanceToEnd(s2->getLength() / 3);
            Vec2d pos = Vec2d::getAverage(clickPos1, clickPos2);
            auto &&res =
                algorithm::round_shapes(s1.get(), clickPos1, s2.get(),
                                        clickPos2, true, false, radius, pos);
            if (res.size() > 2) {
                if (!cursor.isValid()) {
                    newShapes.emplace_back(
                        createLine(cursor, res[1]->getStartPoint()));
                }
                newShapes.emplace_back(res[1]->clone());
                cursor = res[1]->getEndPoint();
            }
        }
        if (newShapes.size() > 0) {
            auto unshift =
                createLine(newShapes[newShapes.size() - 1]->getEndPoint(),
                           newShapes[0]->getStartPoint());
            newShapes.insert(newShapes.begin(), std::move(unshift));
        }
        shapes = std::move(newShapes);
    }
    if (create_polyline) {
        auto pl = createPolyline();
        for (auto &&s : shapes) {
            pl->appendShape(s.get());
        }
        pl->autoClose();
        std::vector<std::unique_ptr<Shape>> res;
        res.emplace_back(std::move(pl));
        return res;
    }
    else {
        return shapes;
    }
}

} // namespace shape
} // namespace cada
