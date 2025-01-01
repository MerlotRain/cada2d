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

#include <cada2d/RArc.h>
#include <cada2d/RCircle.h>
#include <cada2d/REllipse.h>
#include <cada2d/RLine.h>
#include <cada2d/RPolyline.h>
#include <cada2d/RRay.h>
#include <cada2d/RS.h>
#include <cada2d/RSpline.h>
#include <cada2d/RXLine.h>
#include <cada2d/private/RShapePrivate.h>

static double ellipse2tr(double x, double y, double AA, double BB, double CC,
                         double DD, double EE, double FF)
{
    return (AA * x * x + BB * x * y + CC * y * y + DD * x + EE * y + FF);
}

static void moveList(std::vector<RVector> &list, const RVector &offset)
{
    for (size_t i = 0; i < list.size(); i++) { list[i].move(offset); }
}

static void scaleList(std::vector<RVector> &list, const RVector &factors,
                      const RVector &center = RVector::nullVector)
{
    for (size_t i = 0; i < list.size(); i++) { list[i].scale(factors, center); }
}

static void rotateList(std::vector<RVector> &list, double rotation)
{
    for (size_t i = 0; i < list.size(); i++) { list[i].rotate(rotation); }
}

bool RShapePrivate::isPointShape(const RShape &s)
{
    return s.getShapeType() == RS::Point;
}

bool RShapePrivate::isLineShape(const RShape &s)
{
    return s.getShapeType() == RS::Line;
}

bool RShapePrivate::isArcShape(const RShape &s)
{
    return s.getShapeType() == RS::Arc;
}

bool RShapePrivate::isCircleShape(const RShape &s)
{
    return s.getShapeType() == RS::Circle;
}

bool RShapePrivate::isEllipseShape(const RShape &s)
{
    return s.getShapeType() == RS::Ellipse;
}

bool RShapePrivate::isFullEllipseShape(const RShape &s)
{
    return s.getShapeType() == RS::Ellipse &&
           dynamic_cast<const REllipse &>(s).isFullEllipse();
}

bool RShapePrivate::isPolylineShape(const RShape &s)
{
    return s.getShapeType() == RS::Polyline;
}

bool RShapePrivate::isSplineShape(const RShape &s)
{
    return s.getShapeType() == RS::Spline;
}

bool RShapePrivate::isXLineShape(const RShape &s)
{
    return s.getShapeType() == RS::XLine;
}

bool RShapePrivate::isRayShape(const RShape &s)
{
    return s.getShapeType() == RS::Ray;
}

std::vector<std::shared_ptr<RShape>> RShapePrivate::getReversedShapeList(
        const std::vector<std::shared_ptr<RShape>> &shapes)
{
    return std::vector<std::shared_ptr<RShape>>();
}
std::shared_ptr<RShape> RShapePrivate::scaleArc(const RShape &shape,
                                                const RVector &scaleFactors,
                                                const RVector &center)
{
    return std::shared_ptr<RShape>();
}

std::vector<RVector> RShapePrivate::getIntersectionPoints(const RShape &shape1,
                                                          const RShape &shape2,
                                                          bool limited,
                                                          bool same, bool force)
{
    std::vector<RVector> empty;

    bool gotInfiniteShape = false;
    if (shape1.getShapeType() == RS::XLine ||
        shape2.getShapeType() == RS::XLine ||
        shape1.getShapeType() == RS::Ray || shape2.getShapeType() == RS::Ray)
    {

        gotInfiniteShape = true;
    }

    if (limited && !gotInfiniteShape)
    {
        // 20120425: allow for a bit of error, e.g. for vertical line that
        // is tangent to ellipse / circle:
        RBox bb1 = shape1.getBoundingBox().growXY(1e-2);
        RBox bb2 = shape2.getBoundingBox().growXY(1e-2);
        if (!bb1.intersects(bb2)) { return empty; }
    }

    {
        const RLine *line1 = dynamic_cast<const RLine *>(&shape1);
        if (line1 != NULL)
        {
            if (same) { return empty; }
            const RLine *line2 = dynamic_cast<const RLine *>(&shape2);
            if (line2 != NULL)
            {
                return getIntersectionPointsLL(*line1, *line2, limited);
            }
            const RArc *arc2 = dynamic_cast<const RArc *>(&shape2);
            if (arc2 != NULL)
            {
                return getIntersectionPointsLA(*line1, *arc2, limited);
            }
            const RCircle *circle2 = dynamic_cast<const RCircle *>(&shape2);
            if (circle2 != NULL)
            {
                return getIntersectionPointsLC(*line1, *circle2, limited);
            }
            const REllipse *ellipse2 = dynamic_cast<const REllipse *>(&shape2);
            if (ellipse2 != NULL)
            {
                return getIntersectionPointsLE(*line1, *ellipse2, limited);
            }
            const RSpline *spline2 = dynamic_cast<const RSpline *>(&shape2);
            if (spline2 != NULL)
            {
                return getIntersectionPointsLS(*line1, *spline2, limited);
            }
            const RRay *ray2 = dynamic_cast<const RRay *>(&shape2);
            if (ray2 != NULL)
            {
                std::vector<RVector> ret = getIntersectionPointsLL(
                        *line1, ray2->getLineShape(), limited, false);
                if (limited) ret = ray2->filterOnShape(ret, true);
                return ret;
            }
            const RXLine *xline2 = dynamic_cast<const RXLine *>(&shape2);
            if (xline2 != NULL)
            {
                return getIntersectionPointsLL(*line1, xline2->getLineShape(),
                                               limited, false);
            }

            // spline, polyline, ...:
            const RExplodable *explodable2 =
                    dynamic_cast<const RExplodable *>(&shape2);
            if (explodable2 != NULL)
            {
                return getIntersectionPointsLX(*line1, *explodable2, limited);
            }
        }
    }

    {
        const RRay *ray1 = dynamic_cast<const RRay *>(&shape1);
        if (ray1 != NULL)
        {
            if (same) { return empty; }
            RXLine xline1(ray1->getLineShape());
            std::vector<RVector> ret =
                    getIntersectionPoints(xline1, shape2, limited, same, force);
            if (limited) ret = ray1->filterOnShape(ret, true);
            return ret;
        }
    }

    {
        const RXLine *xline1 = dynamic_cast<const RXLine *>(&shape1);
        if (xline1 != NULL)
        {
            if (same) { return empty; }
            RLine line1 = xline1->getLineShape();

            const RLine *line2 = dynamic_cast<const RLine *>(&shape2);
            if (line2 != NULL)
            {
                return getIntersectionPointsLL(line1, *line2, false, limited);
            }
            const RArc *arc2 = dynamic_cast<const RArc *>(&shape2);
            if (arc2 != NULL)
            {
                return getIntersectionPointsLA(line1, *arc2, false, limited);
            }
            const RCircle *circle2 = dynamic_cast<const RCircle *>(&shape2);
            if (circle2 != NULL)
            {
                return getIntersectionPointsLC(line1, *circle2, false);
            }
            const REllipse *ellipse2 = dynamic_cast<const REllipse *>(&shape2);
            if (ellipse2 != NULL)
            {
                return getIntersectionPointsLE(line1, *ellipse2, false,
                                               limited);
            }
            const RSpline *spline2 = dynamic_cast<const RSpline *>(&shape2);
            if (spline2 != NULL)
            {
                return getIntersectionPointsLS(line1, *spline2, false);
            }
            const RRay *ray2 = dynamic_cast<const RRay *>(&shape2);
            if (ray2 != NULL)
            {
                std::vector<RVector> ret = getIntersectionPointsLL(
                        line1, ray2->getLineShape(), false);
                if (limited) ret = ray2->filterOnShape(ret, true);
                return ret;
            }
            const RXLine *xline2 = dynamic_cast<const RXLine *>(&shape2);
            if (xline2 != NULL)
            {
                return getIntersectionPointsLL(line1, xline2->getLineShape(),
                                               false);
            }

            // spline, polyline, ...:
            const RExplodable *explodable2 =
                    dynamic_cast<const RExplodable *>(&shape2);
            if (explodable2 != NULL)
            {
                return getIntersectionPointsLX(line1, *explodable2, false);
            }
        }
    }

    {
        const RArc *arc1 = dynamic_cast<const RArc *>(&shape1);
        if (arc1 != NULL)
        {
            if (same) { return empty; }
            const RLine *line2 = dynamic_cast<const RLine *>(&shape2);
            if (line2 != NULL)
            {
                return getIntersectionPointsLA(*line2, *arc1, limited);
            }
            const RArc *arc2 = dynamic_cast<const RArc *>(&shape2);
            if (arc2 != NULL)
            {
                return getIntersectionPointsAA(*arc1, *arc2, limited);
            }
            const RCircle *circle2 = dynamic_cast<const RCircle *>(&shape2);
            if (circle2 != NULL)
            {
                return getIntersectionPointsAC(*arc1, *circle2, limited);
            }
            const REllipse *ellipse2 = dynamic_cast<const REllipse *>(&shape2);
            if (ellipse2 != NULL)
            {
                return getIntersectionPointsAE(*arc1, *ellipse2, limited);
            }
            const RSpline *spline2 = dynamic_cast<const RSpline *>(&shape2);
            if (spline2 != NULL)
            {
                return getIntersectionPointsAS(*arc1, *spline2, limited);
            }
            const RRay *ray2 = dynamic_cast<const RRay *>(&shape2);
            if (ray2 != NULL)
            {
                std::vector<RVector> ret = getIntersectionPointsLA(
                        ray2->getLineShape(), *arc1, false, limited);
                if (limited) ret = ray2->filterOnShape(ret, true);
                return ret;
            }
            const RXLine *xline2 = dynamic_cast<const RXLine *>(&shape2);
            if (xline2 != NULL)
            {
                return getIntersectionPointsLA(xline2->getLineShape(), *arc1,
                                               false, limited);
            }
            // polyline, ...:
            const RExplodable *explodable2 =
                    dynamic_cast<const RExplodable *>(&shape2);
            if (explodable2 != NULL)
            {
                return getIntersectionPointsAX(*arc1, *explodable2, limited);
            }
        }
    }

    {
        const RCircle *circle1 = dynamic_cast<const RCircle *>(&shape1);
        if (circle1 != NULL)
        {
            if (same) { return empty; }
            const RLine *line2 = dynamic_cast<const RLine *>(&shape2);
            if (line2 != NULL)
            {
                return getIntersectionPointsLC(*line2, *circle1, limited);
            }
            const RArc *arc2 = dynamic_cast<const RArc *>(&shape2);
            if (arc2 != NULL)
            {
                return getIntersectionPointsAC(*arc2, *circle1, limited);
            }
            const RCircle *circle2 = dynamic_cast<const RCircle *>(&shape2);
            if (circle2 != NULL)
            {
                return getIntersectionPointsCC(*circle1, *circle2);
            }
            const REllipse *ellipse2 = dynamic_cast<const REllipse *>(&shape2);
            if (ellipse2 != NULL)
            {
                return getIntersectionPointsCE(*circle1, *ellipse2);
            }
            const RSpline *spline2 = dynamic_cast<const RSpline *>(&shape2);
            if (spline2 != NULL)
            {
                return getIntersectionPointsCS(*circle1, *spline2, limited);
            }
            const RRay *ray2 = dynamic_cast<const RRay *>(&shape2);
            if (ray2 != NULL)
            {
                std::vector<RVector> ret = getIntersectionPointsLC(
                        ray2->getLineShape(), *circle1, false);
                if (limited) ret = ray2->filterOnShape(ret, true);
                return ret;
            }
            const RXLine *xline2 = dynamic_cast<const RXLine *>(&shape2);
            if (xline2 != NULL)
            {
                return getIntersectionPointsLC(xline2->getLineShape(), *circle1,
                                               false);
            }

            // spline, polyline, ...:
            const RExplodable *explodable2 =
                    dynamic_cast<const RExplodable *>(&shape2);
            if (explodable2 != NULL)
            {
                return getIntersectionPointsCX(*circle1, *explodable2, limited);
            }
        }
    }

    {
        const REllipse *ellipse1 = dynamic_cast<const REllipse *>(&shape1);
        if (ellipse1 != NULL)
        {
            if (same) { return empty; }
            const RLine *line2 = dynamic_cast<const RLine *>(&shape2);
            if (line2 != NULL)
            {
                return getIntersectionPointsLE(*line2, *ellipse1, limited);
            }
            const RArc *arc2 = dynamic_cast<const RArc *>(&shape2);
            if (arc2 != NULL)
            {
                return getIntersectionPointsAE(*arc2, *ellipse1, limited);
            }
            const RCircle *circle2 = dynamic_cast<const RCircle *>(&shape2);
            if (circle2 != NULL)
            {
                return getIntersectionPointsCE(*circle2, *ellipse1);
            }
            const REllipse *ellipse2 = dynamic_cast<const REllipse *>(&shape2);
            if (ellipse2 != NULL)
            {
                return getIntersectionPointsEE(*ellipse2, *ellipse1, limited);
            }
            const RSpline *spline2 = dynamic_cast<const RSpline *>(&shape2);
            if (spline2 != NULL)
            {
                return getIntersectionPointsES(*ellipse1, *spline2, limited);
            }
            const RRay *ray2 = dynamic_cast<const RRay *>(&shape2);
            if (ray2 != NULL)
            {
                std::vector<RVector> ret = getIntersectionPointsLE(
                        ray2->getLineShape(), *ellipse1, false, limited);
                if (limited) ret = ray2->filterOnShape(ret, true);
                return ret;
            }
            const RXLine *xline2 = dynamic_cast<const RXLine *>(&shape2);
            if (xline2 != NULL)
            {
                return getIntersectionPointsLE(xline2->getLineShape(),
                                               *ellipse1, false, limited);
            }

            // spline, polyline, ...:
            const RExplodable *explodable2 =
                    dynamic_cast<const RExplodable *>(&shape2);
            if (explodable2 != NULL)
            {
                return getIntersectionPointsEX(*ellipse1, *explodable2,
                                               limited);
            }
        }
    }

    {
        const RSpline *spline1 = dynamic_cast<const RSpline *>(&shape1);
        if (spline1 != NULL)
        {
            const RLine *line2 = dynamic_cast<const RLine *>(&shape2);
            if (line2 != NULL)
            {
                return getIntersectionPointsLS(*line2, *spline1, limited);
            }
            const RArc *arc2 = dynamic_cast<const RArc *>(&shape2);
            if (arc2 != NULL)
            {
                return getIntersectionPointsAS(*arc2, *spline1, limited);
            }
            const RCircle *circle2 = dynamic_cast<const RCircle *>(&shape2);
            if (circle2 != NULL)
            {
                return getIntersectionPointsCS(*circle2, *spline1, limited);
            }
            const REllipse *ellipse2 = dynamic_cast<const REllipse *>(&shape2);
            if (ellipse2 != NULL)
            {
                return getIntersectionPointsES(*ellipse2, *spline1, limited);
            }
            const RRay *ray2 = dynamic_cast<const RRay *>(&shape2);
            if (ray2 != NULL)
            {
                std::vector<RVector> ret = getIntersectionPointsLS(
                        ray2->getLineShape(), *spline1, false);
                if (limited) ret = ray2->filterOnShape(ret, true);
                return ret;
            }
            const RXLine *xline2 = dynamic_cast<const RXLine *>(&shape2);
            if (xline2 != NULL)
            {
                return getIntersectionPointsLS(xline2->getLineShape(), *spline1,
                                               false);
            }
            const RSpline *spline2 = dynamic_cast<const RSpline *>(&shape2);
            if (spline2 != NULL)
            {
                return getIntersectionPointsSS(*spline1, *spline2, limited,
                                               same);
            }

            // spline, polyline, ...:
            const RExplodable *explodable2 =
                    dynamic_cast<const RExplodable *>(&shape2);
            if (explodable2 != NULL)
            {
                return getIntersectionPointsSX(*spline1, *explodable2, limited);
            }
        }
    }

    {
        const RExplodable *explodable1 =
                RShapePrivate::castToExplodable(&shape1);
        if (explodable1 != NULL)
        {
            if (!same)
            {
                const RLine *line2 = dynamic_cast<const RLine *>(&shape2);
                if (line2 != NULL)
                {
                    return getIntersectionPointsLX(*line2, *explodable1,
                                                   limited);
                }
                const RArc *arc2 = dynamic_cast<const RArc *>(&shape2);
                if (arc2 != NULL)
                {
                    return getIntersectionPointsAX(*arc2, *explodable1,
                                                   limited);
                }
                const RCircle *circle2 = dynamic_cast<const RCircle *>(&shape2);
                if (circle2 != NULL)
                {
                    return getIntersectionPointsCX(*circle2, *explodable1,
                                                   limited);
                }
                const REllipse *ellipse2 =
                        dynamic_cast<const REllipse *>(&shape2);
                if (ellipse2 != NULL)
                {
                    return getIntersectionPointsEX(*ellipse2, *explodable1,
                                                   limited);
                }
                const RRay *ray2 = dynamic_cast<const RRay *>(&shape2);
                if (ray2 != NULL)
                {
                    std::vector<RVector> ret = getIntersectionPointsLX(
                            ray2->getLineShape(), *explodable1, false);
                    if (limited) ret = ray2->filterOnShape(ret, true);
                    return ret;
                }
                const RXLine *xline2 = dynamic_cast<const RXLine *>(&shape2);
                if (xline2 != NULL)
                {
                    return getIntersectionPointsLX(xline2->getLineShape(),
                                                   *explodable1, false);
                }
                const RSpline *spline2 = dynamic_cast<const RSpline *>(&shape2);
                if (spline2 != NULL)
                {
                    return getIntersectionPointsSX(*spline2, *explodable1,
                                                   limited);
                }
            }

            // spline, polyline, ...:
            const RExplodable *explodable2 =
                    dynamic_cast<const RExplodable *>(&shape2);
            if (explodable2 != NULL)
            {
                return getIntersectionPointsXX(*explodable1, *explodable2,
                                               limited, same);
            }
        }
    }

    return std::vector<RVector>();
}

std::vector<std::shared_ptr<RShape>>
RShapePrivate::getOffsetArcs(const RShape &shape, double distance, int number,
                             RS::Side side, const RVector &position)
{
    return std::vector<std::shared_ptr<RShape>>();
}

ON_NurbsCurve RShapePrivate::convertShapeToNURBS(const RShape &shape)
{
    return ON_NurbsCurve();
}

ON_NurbsCurve RShapePrivate::convertLineToNURBS(const RLine &line)
{
    return ON_NurbsCurve();
}

ON_NurbsCurve RShapePrivate::convertArcToNURBS(const RArc &arc)
{
    return ON_NurbsCurve();
}

ON_NurbsCurve RShapePrivate::convertEllipseToNURBS(const REllipse &ellipse)
{
    return ON_NurbsCurve();
}

std::vector<RVector> RShapePrivate::getIntersectionPointsLL(const RLine &line1,
                                                            const RLine &line2,
                                                            bool limited1,
                                                            bool limited2)
{
    std::vector<RVector> res;
    double a1 = line1.getEndPoint().y - line1.getStartPoint().y;
    double b1 = line1.getStartPoint().x - line1.getEndPoint().x;
    double c1 = a1 * line1.getStartPoint().x + b1 * line1.getStartPoint().y;

    double a2 = line2.getEndPoint().y - line2.getStartPoint().y;
    double b2 = line2.getStartPoint().x - line2.getEndPoint().x;
    double c2 = a2 * line2.getStartPoint().x + b2 * line2.getStartPoint().y;

    double det = a1 * b2 - a2 * b1;
    if (fabs(det) < 1.0e-6) { return res; }
    else
    {
        RVector v((b2 * c1 - b1 * c2) / det, (a1 * c2 - a2 * c1) / det);

        if ((!limited1 || line1.isOnShape(v)) &&
            (!limited2 || line2.isOnShape(v)))
        {
            res.push_back(v);
            return res;
        }
    }
    return res;
}

std::vector<RVector> RShapePrivate::getIntersectionPointsLA(const RLine &line1,
                                                            const RArc &arc2,
                                                            bool limited1,
                                                            bool limited2)
{

    std::vector<RVector> candidates = RShapePrivate::getIntersectionPointsLC(
            line1, RCircle(arc2.getCenter(), arc2.getRadius()), limited1);
    if (!limited2) { return candidates; }

    std::vector<RVector> res;

    for (int i = 0; i < candidates.size(); i++)
    {
        if (arc2.isOnShape(candidates[i])) { res.push_back(candidates[i]); }
    }
    // ret.setTangent(tangent);

    return res;
}

std::vector<RVector>
RShapePrivate::getIntersectionPointsLC(const RLine &line1,
                                       const RCircle &circle2, bool limited)
{
    std::vector<RVector> res;

    RVector vLineCenter = line1.getVectorTo(circle2.getCenter(), false);
    double dist = vLineCenter.getMagnitude();

    // special case: arc almost touches line (tangent with tiny gap or tiny
    // overlap):
    if (RMath::fuzzyCompare(dist, circle2.getRadius(), 1.0e-6))
    {
        RVector sol = circle2.getCenter() - vLineCenter;
        if (!limited || line1.isOnShape(sol, true, 1.0e-6))
        {
            res.push_back(sol);
        }
        // ret.setTangent(true);
        return res;
    }

    RVector p = line1.getStartPoint();
    RVector d = line1.getEndPoint() - line1.getStartPoint();
    if (d.getMagnitude() < 1.0e-6) { return res; }

    RVector delta = p - circle2.getCenter();

    // root term:
    double term = std::pow(RVector::getDotProduct(d, delta), 2.0) -
                  std::pow(d.getMagnitude(), 2.0) *
                          (std::pow(delta.getMagnitude(), 2.0) -
                           std::pow(circle2.getRadius(), 2.0));

    // no intersection:
    if (term < 0.0) { return res; }

    // one or two intersections:
    double t1 = (-RVector::getDotProduct(d, delta) + sqrt(term)) /
                std::pow(d.getMagnitude(), 2.0);
    double t2;
    bool tangent = false;

    // only one intersection:
    if (fabs(term) < RS::PointTolerance)
    {
        t2 = t1;
        tangent = true;
    }

    // two intersections
    else
    {
        t2 = (-RVector::getDotProduct(d, delta) - sqrt(term)) /
             std::pow(d.getMagnitude(), 2.0);
    }

    RVector sol1;
    RVector sol2 = RVector::invalid;

    sol1 = p + d * t1;

    if (!tangent) { sol2 = p + d * t2; }

    if (!limited || line1.isOnShape(sol1, true, 1.0e-6))
    {
        res.push_back(sol1);
    }
    if (sol2.isValid())
    {
        if (!limited || line1.isOnShape(sol2, true, 1.0e-6))
        {
            res.push_back(sol2);
        }
    }
    // ret.setTangent(tangent);

    // tangent with two intersections very close to each other:
    if (res.size() == 2 && res[0].equalsFuzzy(res[1])) { res.pop_back(); }

    return res;
}

std::vector<RVector>
RShapePrivate::getIntersectionPointsLE(const RLine &line1,
                                       const REllipse &ellipse2, bool limited1,
                                       bool limited2)
{

    std::vector<RVector> res;

    // find out if line1 is (almost) a tangent:
    RVector tangentPoint = ellipse2.getTangentPoint(line1);
    if (tangentPoint.isValid())
    {
        res.push_back(tangentPoint);
        return res;
    }

    // rotate into normal position:
    double ang = ellipse2.getAngle();

    double rx = ellipse2.getMajorRadius();
    double ry = ellipse2.getMinorRadius();
    RVector center = ellipse2.getCenter();
    RVector a1 = line1.getStartPoint();
    a1.rotate(-ang, center);
    RVector a2 = line1.getEndPoint();
    a2.rotate(-ang, center);
    RVector origin = a1;
    RVector dir = a2 - a1;
    RVector diff = origin - center;
    RVector mDir = RVector(dir.x / (rx * rx), dir.y / (ry * ry));
    RVector mDiff = RVector(diff.x / (rx * rx), diff.y / (ry * ry));

    double a = RVector::getDotProduct(dir, mDir);
    double b = RVector::getDotProduct(dir, mDiff);
    double c = RVector::getDotProduct(diff, mDiff) - 1.0;
    double d = b * b - a * c;

    RVector res1 = RVector::invalid;
    RVector res2 = RVector::invalid;

    if (d < 0)
    {
        // no solution
    }
    else if (d > 0)
    {
        double root = sqrt(d);
        double t_a = (-b - root) / a;
        double t_b = (-b + root) / a;

        res1 = a1.getLerp(a2, t_a).rotate(ang, center);
        res2 = a1.getLerp(a2, t_b).rotate(ang, center);
    }
    else
    {
        double t = -b / a;
        if (0 <= t && t <= 1)
        {
            // one solution:
            res1 = a1.getLerp(a2, t).rotate(ang, center);
        }
        else
        {
            // no solution
        }
    }

    if (res1.isValid())
    {
        if ((!limited1 || line1.isOnShape(res1)) &&
            (!limited2 || ellipse2.isOnShape(res1)))
        {
            res.push_back(res1);
        }
    }
    if (res2.isValid())
    {
        if ((!limited1 || line1.isOnShape(res2)) &&
            (!limited2 || ellipse2.isOnShape(res2)))
        {
            res.push_back(res2);
        }
    }

    return res;
}

std::vector<RVector>
RShapePrivate::getIntersectionPointsLS(const RLine &line1,
                                       const RSpline &spline2, bool limited)
{
    // if (RSpline::hasProxy()) {
    //     RSplineProxy *proxy = RSpline::getSplineProxy();
    //     return proxy->getIntersectionPoints(spline2, line1, limited);
    // }

    return std::vector<RVector>();
}

std::vector<RVector> RShapePrivate::getIntersectionPointsLX(
        const RLine &line1, const RExplodable &explodable2, bool limited)
{
    std::vector<RVector> res;

    std::vector<std::shared_ptr<RShape>> sub = explodable2.getExploded();
    std::vector<std::shared_ptr<RShape>>::iterator it;
    for (it = sub.begin(); it != sub.end(); ++it)
    {
        std::shared_ptr<RLine> pLine2 = std::dynamic_pointer_cast<RLine>(*it);
        if (pLine2)
        {
            RLine line2 = *pLine2;
            auto &&ips = RShapePrivate::getIntersectionPointsLL(line1, line2,
                                                                limited, true);
            res.insert(res.end(), ips.begin(), ips.end());
            continue;
        }

        std::shared_ptr<RArc> pArc2 = std::dynamic_pointer_cast<RArc>(*it);
        if (pArc2)
        {
            RArc arc2 = *pArc2;
            auto &&ips = RShapePrivate::getIntersectionPointsLA(line1, arc2,
                                                                limited, true);
            res.insert(res.end(), ips.begin(), ips.end());
            continue;
        }
    }

    return res;
}

std::vector<RVector> RShapePrivate::getIntersectionPointsAA(const RArc &arc1,
                                                            const RArc &arc2,
                                                            bool limited)
{

    std::vector<RVector> candidates = RShapePrivate::getIntersectionPoints(
            RCircle(arc1.getCenter(), arc1.getRadius()),
            RCircle(arc2.getCenter(), arc2.getRadius()));
    if (!limited) { return candidates; }

    std::vector<RVector> res;

    for (int i = 0; i < candidates.size(); i++)
    {
        if (arc1.isOnShape(candidates[i]) && arc2.isOnShape(candidates[i]))
        {
            res.push_back(candidates[i]);
        }
    }
    // ret.setTangent(tangent);

    return res;
}

std::vector<RVector>
RShapePrivate::getIntersectionPointsAC(const RArc &arc1, const RCircle &circle2,
                                       bool limited)
{
    std::vector<RVector> candidates = RShapePrivate::getIntersectionPoints(
            RCircle(arc1.getCenter(), arc1.getRadius()), circle2);
    if (!limited) { return candidates; }

    std::vector<RVector> res;

    for (int i = 0; i < candidates.size(); i++)
    {
        if (arc1.isOnShape(candidates[i])) { res.push_back(candidates[i]); }
    }
    return res;
}

std::vector<RVector>
RShapePrivate::getIntersectionPointsAE(const RArc &arc1,
                                       const REllipse &ellipse2, bool limited)
{
    std::vector<RVector> candidates = RShapePrivate::getIntersectionPointsCE(
            RCircle(arc1.getCenter(), arc1.getRadius()), ellipse2);

    if (!limited) { return candidates; }

    std::vector<RVector> res;

    for (int i = 0; i < candidates.size(); i++)
    {
        RVector c = candidates[i];
        if (arc1.isOnShape(c))
        {
            if (!ellipse2.isFullEllipse())
            {
                double a1 = ellipse2.getCenter().getAngleTo(
                        ellipse2.getStartPoint());
                double a2 =
                        ellipse2.getCenter().getAngleTo(ellipse2.getEndPoint());
                double a = ellipse2.getCenter().getAngleTo(c);
                if (!RMath::isAngleBetween(a, a1, a2, ellipse2.isReversed()))
                {
                    continue;
                }
            }

            res.push_back(c);
        }
    }

    return res;
}

std::vector<RVector>
RShapePrivate::getIntersectionPointsAS(const RArc &arc1, const RSpline &spline2,
                                       bool limited)
{

    return std::vector<RVector>();
}

std::vector<RVector> RShapePrivate::getIntersectionPointsAX(
        const RArc &arc1, const RExplodable &explodable2, bool limited)
{
    std::vector<RVector> res;

    std::vector<std::shared_ptr<RShape>> sub = explodable2.getExploded();
    std::vector<std::shared_ptr<RShape>>::iterator it;
    for (it = sub.begin(); it != sub.end(); ++it)
    {
        std::shared_ptr<RLine> pLine2 = std::dynamic_pointer_cast<RLine>(*it);
        if (pLine2)
        {
            RLine line2 = *pLine2;
            auto &&ips = RShapePrivate::getIntersectionPointsLA(line2, arc1);
            res.insert(res.end(), ips.begin(), ips.end());
            continue;
        }

        std::shared_ptr<RArc> pArc2 = std::dynamic_pointer_cast<RArc>(*it);
        if (pArc2)
        {
            RArc arc2 = *pArc2;
            auto &&ips = RShapePrivate::getIntersectionPointsAA(arc1, arc2);
            res.insert(res.end(), ips.begin(), ips.end());
            continue;
        }
    }

    return res;
}

std::vector<RVector>
RShapePrivate::getIntersectionPointsCC(const RCircle &circle1,
                                       const RCircle &circle2)
{
    double r1 = circle1.getRadius();
    double r2 = circle2.getRadius();
    if (r1 < r2)
    {
        // make sure circle 1 is the larger one (for tangency detection):
        return getIntersectionPointsCC(circle2, circle1);
    }

    std::vector<RVector> res;

    RVector c1 = circle1.getCenter();
    RVector c2 = circle2.getCenter();

    RVector u = c2 - c1;
    double uMag = u.getMagnitude();

    // concentric
    if (uMag < RS::PointTolerance) { return res; }

    double tol = (r1 + r2) / 200000;

    // the two circles (almost) touch externally / internally in one point
    // (tangent):
    if (RMath::fuzzyCompare(uMag, r1 + r2, tol) ||
        RMath::fuzzyCompare(uMag, fabs(r1 - r2), tol))
    {

        u.setMagnitude(r1);
        res.push_back(c1 + u);
        return res;
    }

    RVector v = RVector(u.y, -u.x);

    double s, t1, t2, term;

    s = 1.0 / 2.0 * ((r1 * r1 - r2 * r2) / (std::pow(uMag, 2.0)) + 1.0);

    term = (r1 * r1) / (std::pow(uMag, 2.0)) -s * s;

    // no intersection:
    if (term < 0.0) { return res; }

    // one or two intersections:
    t1 = sqrt(term);
    t2 = -sqrt(term);

    RVector sol1 = c1 + u * s + v * t1;
    RVector sol2 = c1 + u * s + v * t2;

    if (sol1.equalsFuzzy(sol2, tol)) { res.push_back(sol1); }
    else
    {
        res.push_back(sol1);
        res.push_back(sol2);
    }

    return res;
}

std::vector<RVector>
RShapePrivate::getIntersectionPointsCE(const RCircle &circle1,
                                       const REllipse &ellipse2)
{
    REllipse ellipse1(circle1.getCenter(), RVector(circle1.getRadius(), 0.),
                      1.0, 0.0, 2.0 * M_PI, false);

    return getIntersectionPointsEE(ellipse1, ellipse2);
}

std::vector<RVector>
RShapePrivate::getIntersectionPointsCS(const RCircle &circle1,
                                       const RSpline &spline2, bool limited)
{

    return std::vector<RVector>();
}

std::vector<RVector> RShapePrivate::getIntersectionPointsCX(
        const RCircle &circle1, const RExplodable &explodable2, bool limited)
{
    std::vector<RVector> res;

    std::vector<std::shared_ptr<RShape>> sub = explodable2.getExploded();
    std::vector<std::shared_ptr<RShape>>::iterator it;
    for (it = sub.begin(); it != sub.end(); ++it)
    {
        std::shared_ptr<RLine> pLine2 = std::dynamic_pointer_cast<RLine>(*it);
        if (pLine2)
        {
            RLine line2 = *pLine2;
            auto &&ips = RShapePrivate::getIntersectionPointsLC(line2, circle1);
            res.insert(res.end(), ips.begin(), ips.end());
            continue;
        }

        std::shared_ptr<RArc> pArc2 = std::dynamic_pointer_cast<RArc>(*it);
        if (pArc2)
        {
            RArc arc2 = *pArc2;
            auto &&ips = RShapePrivate::getIntersectionPointsAC(arc2, circle1);
            res.insert(res.end(), ips.begin(), ips.end());
            continue;
        }
    }

    return res;
}

std::vector<RVector>
RShapePrivate::getIntersectionPointsEE(const REllipse &ellipse1,
                                       const REllipse &ellipse2, bool limited)
{
    std::vector<RVector> candidates =
            getIntersectionPointsEE(ellipse1, ellipse2);

    if (!limited) { return candidates; }

    std::vector<RVector> ret;

    for (int i = 0; i < candidates.size(); i++)
    {
        RVector c = candidates[i];
        bool onShape = true;

        double a1 = ellipse1.getCenter().getAngleTo(ellipse1.getStartPoint());
        double a2 = ellipse1.getCenter().getAngleTo(ellipse1.getEndPoint());
        double a = ellipse1.getCenter().getAngleTo(c);
        if (!RMath::isAngleBetween(a, a1, a2, ellipse1.isReversed()))
        {
            onShape = false;
        }

        a1 = ellipse2.getCenter().getAngleTo(ellipse2.getStartPoint());
        a2 = ellipse2.getCenter().getAngleTo(ellipse2.getEndPoint());
        a = ellipse2.getCenter().getAngleTo(c);
        if (!RMath::isAngleBetween(a, a1, a2, ellipse2.isReversed()))
        {
            onShape = false;
        }

        if (onShape) { ret.push_back(c); }
    }

    return ret;
}

std::vector<RVector>
RShapePrivate::getIntersectionPointsEE(const REllipse &ellipse1,
                                       const REllipse &ellipse2)
{
    std::vector<RVector> ret;

    // two full ellipses:
    // if bounding boxes don't intersect, ellipses don't either:
    if (ellipse1.isFullEllipse() && ellipse2.isFullEllipse() &&
        !ellipse1.getBoundingBox().intersects(ellipse2.getBoundingBox()))
    {

        return ret;
    }

    // normalize ellipse ratios:
    REllipse ellipse1Copy = ellipse1;
    if (ellipse1Copy.getMajorRadius() < ellipse1Copy.getMinorRadius())
    {
        ellipse1Copy.switchMajorMinor();
    }
    REllipse ellipse2Copy = ellipse2;
    if (ellipse2Copy.getMajorRadius() < ellipse2Copy.getMinorRadius())
    {
        ellipse2Copy.switchMajorMinor();
    }

    // for later comparison, make sure that major points are in
    // quadrant I or II (relative to the ellipse center):
    if (fabs(ellipse1Copy.getMajorPoint().y) < RS::PointTolerance)
    {
        if (ellipse1Copy.getMajorPoint().x < 0.0)
        {
            ellipse1Copy.setMajorPoint(-ellipse1Copy.getMajorPoint());
        }
    }
    else
    {
        if (ellipse1Copy.getMajorPoint().y < 0.0)
        {
            ellipse1Copy.setMajorPoint(-ellipse1Copy.getMajorPoint());
        }
    }
    if (fabs(ellipse2Copy.getMajorPoint().y) < RS::PointTolerance)
    {
        if (ellipse2Copy.getMajorPoint().x < 0.0)
        {
            ellipse2Copy.setMajorPoint(-ellipse2Copy.getMajorPoint());
        }
    }
    else
    {
        if (ellipse2Copy.getMajorPoint().y < 0.0)
        {
            ellipse2Copy.setMajorPoint(-ellipse2Copy.getMajorPoint());
        }
    }

    // for comparison:
    bool identicalCenter = (ellipse1Copy.getCenter() - ellipse2Copy.getCenter())
                                   .getMagnitude() < RS::PointTolerance;

    bool identicalRatio =
            fabs(ellipse1Copy.getRatio() - ellipse2Copy.getRatio()) < 1.0e-4;
    double angleDifference = fabs(RMath::getAngleDifference180(
            ellipse1Copy.getAngle(), ellipse2Copy.getAngle()));
    bool identicalRotation =
            angleDifference < 1.0e-4 || angleDifference > M_PI - 1.0e-4;

    // ellipses are identical (no intersection points):
    //    if (identicalCenter && identicalShape) {
    if (identicalCenter && identicalRatio && identicalRotation) { return ret; }

    // special case: ellipse shapes are identical (different positions):
    if (identicalRatio && identicalRotation)
    {

        double angle = -ellipse1Copy.getAngle();
        double yScale = 1.0 / ellipse1Copy.getRatio();

        RVector circleCenter1 = ellipse1Copy.getCenter();
        circleCenter1.rotate(angle);
        circleCenter1.scale(RVector(1.0, yScale));
        RVector circleCenter2 = ellipse2Copy.getCenter();
        circleCenter2.rotate(angle);
        circleCenter2.scale(RVector(1.0, yScale));

        RCircle circle1(circleCenter1, ellipse1Copy.getMajorRadius());
        RCircle circle2(circleCenter2, ellipse2Copy.getMajorRadius());

        ret = getIntersectionPointsCC(circle1, circle2);

        scaleList(ret, RVector(1.0, 1.0 / yScale));
        rotateList(ret, -angle);

        return ret;
    }

    // transform ellipse2 to coordinate system of ellipse1:
    RVector centerOffset = -ellipse1Copy.getCenter();
    double angleOffset = -ellipse1Copy.getAngle();

    double majorRadius1 = ellipse1Copy.getMajorRadius();
    double majorRadius2 = ellipse2Copy.getMajorRadius();

    // special case: treat first ellipse as a line:
    if (ellipse1Copy.getMinorRadius() < RS::PointTolerance ||
        ellipse1Copy.getRatio() < RS::PointTolerance)
    {

        ellipse2Copy.move(centerOffset);
        ellipse2Copy.rotate(angleOffset);

        RLine line(RVector(-majorRadius1, 0.0), RVector(majorRadius1, 0.0));
        ret = getIntersectionPointsLE(line, ellipse2Copy);
        rotateList(ret, -angleOffset);
        moveList(ret, -centerOffset);

        return ret;
    }

    // special case: treat second ellipse as a line:
    if (ellipse2Copy.getMinorRadius() < RS::PointTolerance ||
        ellipse2Copy.getRatio() < RS::PointTolerance)
    {

        ellipse2Copy.move(centerOffset);
        ellipse2Copy.rotate(angleOffset);

        RLine line(RVector(-majorRadius2, 0.), RVector(majorRadius2, 0.));
        line.rotate(ellipse2Copy.getAngle(), RVector(0., 0.));
        line.move(ellipse2Copy.getCenter());
        ret = getIntersectionPointsLE(line, ellipse1Copy);
        rotateList(ret, -angleOffset);
        moveList(ret, -centerOffset);
        return ret;
    }

    double phi_1 = ellipse1Copy.getAngle();
    double a1 = ellipse1Copy.getMajorRadius();
    double b1 = ellipse1Copy.getMinorRadius();
    double h1 = ellipse1Copy.getCenter().x;
    double k1 = ellipse1Copy.getCenter().y;

    double phi_2 = ellipse2Copy.getAngle();
    double a2 = ellipse2Copy.getMajorRadius();
    double b2 = ellipse2Copy.getMinorRadius();
    double h2 = ellipse2Copy.getCenter().x;
    double k2 = ellipse2Copy.getCenter().y;

    int i, j, k, nroots, nychk, nintpts;
    double AA, BB, CC, DD, EE, FF, H2_TR, K2_TR, A22, B22, PHI_2R;
    double cosphi, cosphi2, sinphi, sinphi2, cosphisinphi;
    double tmp0, tmp1, tmp2, tmp3;
    double cy[5] = {0.0};
    double py[5] = {0.0};
    double r[3][5] = {{0.0}};
    double x1, x2;
    double ychk[5] = {0.0};
    double xint[5];
    double yint[5];

    // each of the ellipse axis lengths must be positive
    if ((!(a1 > 0.0) || !(b1 > 0.0)) || (!(a2 > 0.0) || !(b2 > 0.0)))
    {
        return std::vector<RVector>();
    }

    // the rotation angles should be between -2pi and 2pi (?)
    if (fabs(phi_1) > (M_PI * 2)) { phi_1 = fmod(phi_1, (M_PI * 2)); }
    if (fabs(phi_2) > (M_PI * 2)) { phi_2 = fmod(phi_2, (M_PI * 2)); }

    // determine the two ellipse equations from input parameters:

    // Finding the points of intersection between two general ellipses
    // requires solving a quartic equation. Before attempting to solve the
    // quartic, several quick tests can be used to eliminate some cases
    // where the ellipses do not intersect. Optionally, can whittle away
    // at the problem, by addressing the easiest cases first.

    // Working with the translated+rotated ellipses simplifies the
    // calculations. The ellipses are translated then rotated so that the
    // first ellipse is centered at the origin and oriented with the
    // coordinate axes. Then, the first ellipse will have the implicit
    // (polynomial) form of
    // x^2/A1^2 + y+2/B1^2 = 1

    // For the second ellipse, the center is first translated by the amount
    // required to put the first ellipse at the origin, e.g., by (-H1, -K1)
    // Then, the center of the second ellipse is rotated by the amount
    // required to orient the first ellipse with the coordinate axes, e.g.,
    // through the angle -PHI_1.
    // The translated and rotated center point coordinates for the second
    // ellipse are found with the rotation matrix, derivations are
    // described in the reference.
    cosphi = cos(phi_1);
    sinphi = sin(phi_1);
    H2_TR = (h2 - h1) * cosphi + (k2 - k1) * sinphi;
    K2_TR = (h1 - h2) * sinphi + (k2 - k1) * cosphi;
    PHI_2R = phi_2 - phi_1;
    if (fabs(PHI_2R) > (M_PI * 2)) { PHI_2R = fmod(PHI_2R, (M_PI * 2)); }

    // Calculate implicit (Polynomial) coefficients for the second ellipse
    // in its translated-by (-H1, -H2) and rotated-by -PHI_1 position
    // AA*x^2 + BB*x*y + CC*y^2 + DD*x + EE*y + FF = 0
    // Formulas derived in the reference
    // To speed things up, store multiply-used expressions first
    cosphi = cos(PHI_2R);
    cosphi2 = cosphi * cosphi;
    sinphi = sin(PHI_2R);
    sinphi2 = sinphi * sinphi;
    cosphisinphi = 2.0 * cosphi * sinphi;
    A22 = a2 * a2;
    B22 = b2 * b2;
    tmp0 = (cosphi * H2_TR + sinphi * K2_TR) / A22;
    tmp1 = (sinphi * H2_TR - cosphi * K2_TR) / B22;
    tmp2 = cosphi * H2_TR + sinphi * K2_TR;
    tmp3 = sinphi * H2_TR - cosphi * K2_TR;

    // implicit polynomial coefficients for the second ellipse
    AA = cosphi2 / A22 + sinphi2 / B22;
    BB = cosphisinphi / A22 - cosphisinphi / B22;
    CC = sinphi2 / A22 + cosphi2 / B22;
    DD = -2.0 * cosphi * tmp0 - 2.0 * sinphi * tmp1;
    EE = -2.0 * sinphi * tmp0 + 2.0 * cosphi * tmp1;
    FF = tmp2 * tmp2 / A22 + tmp3 * tmp3 / B22 - 1.0;

    // create and solve the quartic equation to find intersection points:

    // If execution arrives here, the ellipses are at least 'close' to
    // intersecting.
    // Coefficients for the Quartic Polynomial in y are calculated from
    // the two implicit equations.
    // Formulas for these coefficients are derived in the reference.

    cy[4] = pow(a1, 4.0) * AA * AA +
            b1 * b1 * (a1 * a1 * (BB * BB - 2.0 * AA * CC) + b1 * b1 * CC * CC);
    cy[3] = 2.0 * b1 * (b1 * b1 * CC * EE + a1 * a1 * (BB * DD - AA * EE));
    cy[2] = a1 * a1 *
                    ((b1 * b1 * (2.0 * AA * CC - BB * BB) + DD * DD -
                      2.0 * AA * FF) -
                     2.0 * a1 * a1 * AA * AA) +
            b1 * b1 * (2.0 * CC * FF + EE * EE);
    cy[1] = 2.0 * b1 * (a1 * a1 * (AA * EE - BB * DD) + EE * FF);
    cy[0] = (a1 * (a1 * AA - DD) + FF) * (a1 * (a1 * AA + DD) + FF);

    // Once the coefficients for the Quartic Equation in y are known, the
    // roots of the quartic polynomial will represent y-values of the
    // intersection points of the two ellipse curves.
    // The quartic sometimes degenerates into a polynomial of lesser
    // degree, so handle all possible cases.
    if (fabs(cy[4]) > 0.0)
    {
        //  quartic coefficient nonzero, use quartic formula:
        for (i = 0; i <= 3; i++) { py[4 - i] = cy[i] / cy[4]; }
        py[0] = 1.0;
        RMath::getBiQuadRoots(py, r);
        nroots = 4;
    }
    else if (fabs(cy[3]) > 0.0)
    {
        for (i = 0; i <= 2; i++) { py[3 - i] = cy[i] / cy[3]; }
        py[0] = 1.0;
        RMath::getCubicRoots(py, r);
        nroots = 3;
    }
    else if (fabs(cy[2]) > 0.0)
    {
        //  quartic degenerates to quadratic, use quadratic formula:
        for (i = 0; i <= 1; i++) { py[2 - i] = cy[i] / cy[2]; }
        py[0] = 1.0;
        RMath::getQuadRoots(py, r);
        nroots = 2;
    }
    else if (fabs(cy[1]) > 0.0)
    {
        //  quartic degenerates to linear: solve directly:
        //  cy[1]*Y + cy[0] = 0
        r[1][1] = (-cy[0] / cy[1]);
        r[2][1] = 0.0;
        nroots = 1;
    }
    else
    {
        //  completely degenerate quartic: ellipses identical?
        //  a completely degenerate quartic, which would seem to
        //  indicate that the ellipses are identical. However, some
        //  configurations lead to a degenerate quartic with no
        //  points of intersection.
        nroots = 0;
    }

    // check roots of the quartic: are they points of intersection?
    // determine which roots are real, discard any complex roots
    nychk = 0;
    for (i = 1; i <= nroots; i++)
    {
        if (fabs(r[2][i]) < 1.0e-02)
        {
            nychk++;
            ychk[nychk] = r[1][i] * b1;
        }
    }

    // sort the real roots by straight insertion
    for (j = 2; j <= nychk; j++)
    {
        tmp0 = ychk[j];
        for (k = j - 1; k >= 1; k--)
        {
            if (ychk[k] <= tmp0) { break; }
            ychk[k + 1] = ychk[k];
        }
        ychk[k + 1] = tmp0;
    }

    // determine whether polynomial roots are points of intersection
    // for the two ellipses
    nintpts = 0;
    for (i = 1; i <= nychk; i++)
    {

        // check for multiple roots
        if ((i > 1) && (fabs(ychk[i] - ychk[i - 1]) < (1.0e-02 / 2.0)))
        {
            continue;
        }
        // check intersection points for ychk[i]
        if (fabs(ychk[i]) > b1) { x1 = 0.0; }
        else { x1 = a1 * sqrt(1.0 - (ychk[i] * ychk[i]) / (b1 * b1)); }
        x2 = -x1;

        if (fabs(ellipse2tr(x1, ychk[i], AA, BB, CC, DD, EE, FF)) <
            1.0e-02 / 2.0)
        {
            nintpts++;
            if (nintpts > 4)
            {
                //(*rtnCode) = ERROR_INTERSECTION_PTS;
                return std::vector<RVector>();
            }
            xint[nintpts] = x1;
            yint[nintpts] = ychk[i];
        }

        if ((fabs(ellipse2tr(x2, ychk[i], AA, BB, CC, DD, EE, FF)) <
             1.0e-02 / 2.0) &&
            (fabs(x2 - x1) > 1.0e-02 / 2.0))
        {
            nintpts++;
            if (nintpts > 4)
            {
                //(*rtnCode) = ERROR_INTERSECTION_PTS;
                return std::vector<RVector>();
            }
            xint[nintpts] = x2;
            yint[nintpts] = ychk[i];
        }
    }

    for (int i = 1; i <= nintpts; i++)
    {
        RVector v(xint[i], yint[i]);
        v.rotate(-angleOffset);
        v.move(-centerOffset);
        ret.push_back(v);
    }

    return ret;
}

std::vector<RVector>
RShapePrivate::getIntersectionPointsES(const REllipse &ellipse1,
                                       const RSpline &spline2, bool limited)
{
    return std::vector<RVector>();
}

std::vector<RVector> RShapePrivate::getIntersectionPointsEX(
        const REllipse &ellipse1, const RExplodable &explodable2, bool limited)
{
    std::vector<RVector> res;

    std::vector<std::shared_ptr<RShape>> sub = explodable2.getExploded();
    std::vector<std::shared_ptr<RShape>>::iterator it;
    for (it = sub.begin(); it != sub.end(); ++it)
    {
        std::shared_ptr<RLine> pLine2 = std::dynamic_pointer_cast<RLine>(*it);
        if (pLine2)
        {
            RLine line2 = *pLine2;
            auto &&ips =
                    RShapePrivate::getIntersectionPointsLE(line2, ellipse1);
            res.insert(res.end(), ips.begin(), ips.end());
            continue;
        }
        std::shared_ptr<RArc> pArc2 = std::dynamic_pointer_cast<RArc>(*it);
        if (pArc2)
        {
            RArc arc2 = *pArc2;
            auto &&ips = RShapePrivate::getIntersectionPointsAE(arc2, ellipse1);
            res.insert(res.end(), ips.begin(), ips.end());
            continue;
        }
    }

    return res;
}

std::vector<RVector> RShapePrivate::getIntersectionPointsSX(
        const RSpline &spline1, const RExplodable &explodable2, bool limited)
{
    std::vector<RVector> res;

    std::vector<std::shared_ptr<RShape>> sub = explodable2.getExploded();
    std::vector<std::shared_ptr<RShape>>::iterator it;
    for (it = sub.begin(); it != sub.end(); ++it)
    {
        std::shared_ptr<RLine> pLine2 = std::dynamic_pointer_cast<RLine>(*it);
        if (pLine2)
        {
            RLine line2 = *pLine2;
            auto &&ips = RShapePrivate::getIntersectionPointsLS(line2, spline1);
            res.insert(res.end(), ips.begin(), ips.end());
            continue;
        }
        std::shared_ptr<RArc> pArc2 = std::dynamic_pointer_cast<RArc>(*it);
        if (pArc2)
        {
            RArc arc2 = *pArc2;
            auto &&ips = RShapePrivate::getIntersectionPointsAS(arc2, spline1);
            res.insert(res.end(), ips.begin(), ips.end());
            continue;
        }
    }

    return res;
}

std::vector<RVector>
RShapePrivate::getIntersectionPointsSS(const RSpline &spline1,
                                       const RSpline &spline2, bool limited,
                                       bool same, double tolerance)
{
    return std::vector<RVector>();
}

/**
 * \param same True if the two shapes are identical, from the same interpolated
 *      shape (e.g. spline).
 */
std::vector<RVector>
RShapePrivate::getIntersectionPointsXX(const RExplodable &explodable1,
                                       const RExplodable &explodable2,
                                       bool limited, bool same)
{
    std::vector<RVector> res;

    std::vector<std::shared_ptr<RShape>> sub1 = explodable1.getExploded();
    std::vector<std::shared_ptr<RShape>> sub2;

    if (same) { sub2 = sub1; }
    else { sub2 = explodable2.getExploded(); }

    std::vector<std::shared_ptr<RShape>>::iterator it1;
    std::vector<std::shared_ptr<RShape>>::iterator it2;

    int c1, c2;
    for (it1 = sub1.begin(), c1 = 0; it1 != sub1.end(); ++it1, ++c1)
    {
        for (it2 = sub2.begin(), c2 = 0; it2 != sub2.end(); ++it2, ++c2)
        {
            // sub shapes of same, interpolated shape (e.g. spline):
            if (same)
            {
                // segments are connected and therefore don't intersect for a
                // spline:
                if (std::fabs(c1 - c2) <= 1) { continue; }
            }
            auto &&ips = RShapePrivate::getIntersectionPoints(*(*it1), *(*it2));
            res.insert(res.end(), ips.begin(), ips.end());
        }
    }

    return res;
}

const RExplodable *RShapePrivate::castToExplodable(const RShape *shape)
{
    const RPolyline *polyline = dynamic_cast<const RPolyline *>(shape);
    if (polyline != NULL)
    {
        return dynamic_cast<const RExplodable *>(polyline);
    }

    const RSpline *spline = dynamic_cast<const RSpline *>(shape);
    if (spline != NULL) { return dynamic_cast<const RExplodable *>(spline); }

    return NULL;
}
