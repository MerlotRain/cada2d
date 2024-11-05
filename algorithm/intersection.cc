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
#include <cmath>

using namespace cada::shape;

namespace cada {
namespace algorithm {

/* -------------------------------- functions ------------------------------- */
std::vector<Vec2d> cada_getIntersectionPoints(const Shape *shape1,
                                              const Shape *shape2, bool limited,
                                              bool same, bool force);

static double ellipse2tr(double x, double y, double AA, double BB, double CC,
                         double DD, double EE, double FF)
{
    return (AA * x * x + BB * x * y + CC * y * y + DD * x + EE * y + FF);
}
static std::vector<Vec2d> cada_getIntersectionPointsLL(const Line *line1,
                                                       const Line *line2,
                                                       bool limited1,
                                                       bool limited2);
static std::vector<Vec2d> cada_getIntersectionPointsLA(const Line *line1,
                                                       const Arc *arc2,
                                                       bool limited1,
                                                       bool limited2);
static std::vector<Vec2d> cada_getIntersectionPointsLC(const Line *line1,
                                                       const Circle *circle2,
                                                       bool limited);
static std::vector<Vec2d> cada_getIntersectionPointsLE(const Line *line1,
                                                       const Ellipse *ellipse2,
                                                       bool limited1,
                                                       bool limited2);
static std::vector<Vec2d>
cada_getIntersectionPointsLX(const Line *line1, const Polyline *explodable2,
                             bool limited);
static std::vector<Vec2d> cada_getIntersectionPointsLS(const Line *line1,
                                                       const BSpline *spline2,
                                                       bool limited);
static std::vector<Vec2d>
cada_getIntersectionPointsAA(const Arc *arc1, const Arc *arc2, bool limited);
static std::vector<Vec2d> cada_getIntersectionPointsAC(const Arc *arc1,
                                                       const Circle *circle2,
                                                       bool limited);
static std::vector<Vec2d> cada_getIntersectionPointsAE(const Arc *arc1,
                                                       const Ellipse *ellipse2,
                                                       bool limited);
static std::vector<Vec2d> cada_getIntersectionPointsAS(const Arc *arc1,
                                                       const BSpline *spline2,
                                                       bool limited);
static std::vector<Vec2d>
cada_getIntersectionPointsAX(const Arc *arc1, const Polyline *explodable2,
                             bool limited);
static std::vector<Vec2d> cada_getIntersectionPointsCC(const Circle *circle1,
                                                       const Circle *circle2);
static std::vector<Vec2d> cada_getIntersectionPointsCE(const Circle *circle1,
                                                       const Ellipse *ellipse2);
static std::vector<Vec2d> cada_getIntersectionPointsCS(const Circle *circle1,
                                                       const BSpline *spline2,
                                                       bool limited);
static std::vector<Vec2d>
cada_getIntersectionPointsCX(const Circle *circle1, const Polyline *explodable2,
                             bool limited);
static std::vector<Vec2d> cada_getIntersectionPointsEE(const Ellipse *ellipse1,
                                                       const Ellipse *ellipse2);
static std::vector<Vec2d> cada_getIntersectionPointsEE(const Ellipse *ellipse1,
                                                       const Ellipse *ellipse2,
                                                       bool limited);
static std::vector<Vec2d> cada_getIntersectionPointsES(const Ellipse *ellipse1,
                                                       const BSpline *spline2,
                                                       bool limited);
static std::vector<Vec2d>
cada_getIntersectionPointsEX(const Ellipse *ellipse1,
                             const Polyline *explodable2, bool limited);
static std::vector<Vec2d> cada_getIntersectionPointsSS(const BSpline *spline1,
                                                       const BSpline *spline2,
                                                       bool limited, bool same,
                                                       double tol);
static std::vector<Vec2d>
cada_getIntersectionPointsSX(const BSpline *spline1,
                             const Polyline *explodable2, bool limited);
static std::vector<Vec2d>
cada_getIntersectionPointsXX(const Polyline *explodable1,
                             const Polyline *explodable2, bool limited,
                             bool same);

/* ---------------------------------- impls --------------------------------- */
std::vector<Vec2d> cada_getIntersectionPoints(const Shape *shape1,
                                              const Shape *shape2, bool limited,
                                              bool same, bool force)
{
    assert(shape1);
    assert(shape2);

    std::vector<Vec2d> empty;
    bool gotInfiniteShape = false;
    if (shape1->getShapeType() == NS::XLine ||
        shape2->getShapeType() == NS::XLine ||
        shape1->getShapeType() == NS::Ray ||
        shape2->getShapeType() == NS::Ray) {

        gotInfiniteShape = true;
    }

    if (limited && !gotInfiniteShape) {
        BBox bb1 = shape1->getBoundingBox().grow(1e-2);
        BBox bb2 = shape2->getBoundingBox().grow(1e-2);
        if (!bb1.intersects(bb2)) {
            return empty;
        }
    }

    {
        const Line *line1 = dynamic_cast<const Line *>(shape1);
        if (line1) {
            if (same) {
                return empty;
            }
            const Line *line2 = dynamic_cast<const Line *>(shape2);
            if (line2) {
                return cada_getIntersectionPointsLL(line1, line2, limited,
                                                    limited);
            }
            const Arc *arc2 = dynamic_cast<const Arc *>(shape2);
            if (arc2) {
                return cada_getIntersectionPointsLA(line1, arc2, limited,
                                                    limited);
            }
            const Circle *circle2 = dynamic_cast<const Circle *>(shape2);
            if (circle2) {
                return cada_getIntersectionPointsLC(line1, circle2, limited);
            }
            const Ellipse *ellipse2 = dynamic_cast<const Ellipse *>(shape2);
            if (ellipse2) {
                return cada_getIntersectionPointsLE(line1, ellipse2, limited,
                                                    limited);
            }
            const BSpline *spline2 = dynamic_cast<const BSpline *>(shape2);
            if (spline2) {
                return cada_getIntersectionPointsLS(line1, spline2, limited);
            }
            const Ray *ray2 = dynamic_cast<const Ray *>(shape2);
            if (ray2) {
                std::vector<Vec2d> ret = cada_getIntersectionPointsLL(
                    line1, ray2->getLineShape().get(), limited, false);
                if (limited) {
                    ret = ray2->filterOnShape(ret, true);
                }
                return ret;
            }
            const XLine *xline2 = dynamic_cast<const XLine *>(shape2);
            if (xline2) {
                return cada_getIntersectionPointsLL(
                    line1, xline2->getLineShape().release(), limited, false);
            }
            const Polyline *explodable2 =
                dynamic_cast<const Polyline *>(shape2);
            if (explodable2) {
                return cada_getIntersectionPointsLX(line1, explodable2,
                                                    limited);
            }
        }
    }

    {
        const Ray *ray1 = dynamic_cast<const Ray *>(shape1);
        if (ray1) {
            if (same)
                return empty;
        }
        auto &&xline1 = ray1->getLineShape();
        std::vector<Vec2d> ret = cada_getIntersectionPoints(
            xline1.get(), shape2, limited, same, force);
        if (limited)
            ret = ray1->filterOnShape(ret, true);
        return ret;
    }

    {
        const XLine *xline1 = dynamic_cast<const XLine *>(shape1);
        if (xline1) {
            if (same)
                return empty;

            std::unique_ptr<Line> line1 = xline1->getLineShape();

            const Line *line2 = dynamic_cast<const Line *>(shape2);
            if (line2) {
                return cada_getIntersectionPointsLL(line1.release(), line2,
                                                    false, limited);
            }
            const Arc *arc2 = dynamic_cast<const Arc *>(shape2);
            if (arc2) {
                return cada_getIntersectionPointsLA(line1.release(), arc2,
                                                    false, limited);
            }
            const Circle *circle2 = dynamic_cast<const Circle *>(shape2);
            if (circle2) {
                return cada_getIntersectionPointsLC(line1.release(), circle2,
                                                    false);
            }
            const Ellipse *ellipse2 = dynamic_cast<const Ellipse *>(shape2);
            if (ellipse2) {
                return cada_getIntersectionPointsLE(line1.release(), ellipse2,
                                                    false, limited);
            }
            const BSpline *spline2 = dynamic_cast<const BSpline *>(shape2);
            if (spline2) {
                return cada_getIntersectionPointsLS(line1.release(), spline2,
                                                    false);
            }
            const Ray *ray2 = dynamic_cast<const Ray *>(shape2);
            if (ray2) {
                std::vector<Vec2d> ret = cada_getIntersectionPointsLL(
                    line1.release(), ray2->getLineShape().get(), false, false);
                if (limited)
                    ret = ray2->filterOnShape(ret, true);
                return ret;
            }
            const XLine *xline2 = dynamic_cast<const XLine *>(shape2);
            if (xline2) {
                return cada_getIntersectionPointsLL(
                    line1.release(), xline2->getLineShape().release(), false,
                    false);
            }

            const Polyline *explodable2 =
                dynamic_cast<const Polyline *>(shape2);
            if (explodable2) {
                return cada_getIntersectionPointsLX(line1.release(),
                                                    explodable2, false);
            }
        }
    }

    {
        const Arc *arc1 = dynamic_cast<const Arc *>(shape1);
        if (arc1) {
            if (same)
                return empty;

            const Line *line2 = dynamic_cast<const Line *>(shape2);
            if (line2) {
                return cada_getIntersectionPointsLA(line2, arc1, limited,
                                                    limited);
            }
            const Arc *arc2 = dynamic_cast<const Arc *>(shape2);
            if (arc2) {
                return cada_getIntersectionPointsAA(arc1, arc2, limited);
            }
            const Circle *circle2 = dynamic_cast<const Circle *>(shape2);
            if (circle2) {
                return cada_getIntersectionPointsAC(arc1, circle2, limited);
            }

            const Ellipse *ellipse2 = dynamic_cast<const Ellipse *>(shape2);
            if (ellipse2) {
                return cada_getIntersectionPointsAE(arc1, ellipse2, limited);
            }
            const BSpline *spline2 = dynamic_cast<const BSpline *>(shape2);
            if (spline2 != NULL) {
                return cada_getIntersectionPointsAS(arc1, spline2, limited);
            }
            const Ray *ray2 = dynamic_cast<const Ray *>(shape2);
            if (ray2 != NULL) {
                std::vector<Vec2d> ret = cada_getIntersectionPointsLA(
                    ray2->getLineShape().get(), arc1, false, limited);
                if (limited)
                    ret = ray2->filterOnShape(ret, true);
                return ret;
            }
            const XLine *xline2 = dynamic_cast<const XLine *>(shape2);
            if (xline2 != NULL) {
                return cada_getIntersectionPointsLA(
                    xline2->getLineShape().release(), arc1, false, limited);
            }

            // polyline, ...:
            const Polyline *explodable2 =
                dynamic_cast<const Polyline *>(shape2);
            if (explodable2 != NULL) {
                return cada_getIntersectionPointsAX(arc1, explodable2, limited);
            }
        }
    }
    {
        const Circle *circle1 = dynamic_cast<const Circle *>(shape1);
        if (circle1 != NULL) {
            if (same) {
                return empty;
            }
            const Line *line2 = dynamic_cast<const Line *>(shape2);
            if (line2 != NULL) {
                return cada_getIntersectionPointsLC(line2, circle1, limited);
            }
            const Arc *arc2 = dynamic_cast<const Arc *>(shape2);
            if (arc2 != NULL) {
                return cada_getIntersectionPointsAC(arc2, circle1, limited);
            }
            const Circle *circle2 = dynamic_cast<const Circle *>(shape2);
            if (circle2 != NULL) {
                return cada_getIntersectionPointsCC(circle1, circle2);
            }
            const Ellipse *ellipse2 = dynamic_cast<const Ellipse *>(shape2);
            if (ellipse2 != NULL) {
                return cada_getIntersectionPointsCE(circle1, ellipse2);
            }
            const BSpline *spline2 = dynamic_cast<const BSpline *>(shape2);
            if (spline2 != NULL) {
                return cada_getIntersectionPointsCS(circle1, spline2, limited);
            }
            const Ray *ray2 = dynamic_cast<const Ray *>(shape2);
            if (ray2 != NULL) {
                std::vector<Vec2d> ret = cada_getIntersectionPointsLC(
                    ray2->getLineShape().get(), circle1, false);
                if (limited)
                    ret = ray2->filterOnShape(ret, true);
                return ret;
            }
            const XLine *xline2 = dynamic_cast<const XLine *>(shape2);
            if (xline2 != NULL) {
                return cada_getIntersectionPointsLC(
                    xline2->getLineShape().release(), circle1, false);
            }

            // spline, polyline, ...:
            const Polyline *explodable2 =
                dynamic_cast<const Polyline *>(shape2);
            if (explodable2 != NULL) {
                return cada_getIntersectionPointsCX(circle1, explodable2,
                                                    limited);
            }
        }
    }

    {
        const Ellipse *ellipse1 = dynamic_cast<const Ellipse *>(shape1);
        if (ellipse1 != NULL) {
            if (same) {
                return empty;
            }
            const Line *line2 = dynamic_cast<const Line *>(shape2);
            if (line2 != NULL) {
                return cada_getIntersectionPointsLE(line2, ellipse1, limited,
                                                    limited);
            }
            const Arc *arc2 = dynamic_cast<const Arc *>(shape2);
            if (arc2 != NULL) {
                return cada_getIntersectionPointsAE(arc2, ellipse1, limited);
            }
            const Circle *circle2 = dynamic_cast<const Circle *>(shape2);
            if (circle2 != NULL) {
                return cada_getIntersectionPointsCE(circle2, ellipse1);
            }
            const Ellipse *ellipse2 = dynamic_cast<const Ellipse *>(shape2);
            if (ellipse2 != NULL) {
                return cada_getIntersectionPointsEE(ellipse2, ellipse1,
                                                    limited);
            }
            const BSpline *spline2 = dynamic_cast<const BSpline *>(shape2);
            if (spline2 != NULL) {
                return cada_getIntersectionPointsES(ellipse1, spline2, limited);
            }
            const Ray *ray2 = dynamic_cast<const Ray *>(shape2);
            if (ray2 != NULL) {
                std::vector<Vec2d> ret = cada_getIntersectionPointsLE(
                    ray2->getLineShape().get(), ellipse1, false, limited);
                if (limited)
                    ret = ray2->filterOnShape(ret, true);
                return ret;
            }
            const XLine *xline2 = dynamic_cast<const XLine *>(shape2);
            if (xline2 != NULL) {
                return cada_getIntersectionPointsLE(
                    xline2->getLineShape().release(), ellipse1, false, limited);
            }

            // spline, polyline, ...:
            const Polyline *explodable2 =
                dynamic_cast<const Polyline *>(shape2);
            if (explodable2 != NULL) {
                return cada_getIntersectionPointsEX(ellipse1, explodable2,
                                                    limited);
            }
        }
    }

    {
        const BSpline *spline1 = dynamic_cast<const BSpline *>(shape1);
        if (spline1 != NULL) {
            const Line *line2 = dynamic_cast<const Line *>(shape2);
            if (line2 != NULL) {
                return cada_getIntersectionPointsLS(line2, spline1, limited);
            }
            const Arc *arc2 = dynamic_cast<const Arc *>(shape2);
            if (arc2 != NULL) {
                return cada_getIntersectionPointsAS(arc2, spline1, limited);
            }
            const Circle *circle2 = dynamic_cast<const Circle *>(shape2);
            if (circle2 != NULL) {
                return cada_getIntersectionPointsCS(circle2, spline1, limited);
            }
            const Ellipse *ellipse2 = dynamic_cast<const Ellipse *>(shape2);
            if (ellipse2 != NULL) {
                return cada_getIntersectionPointsES(ellipse2, spline1, limited);
            }
            const Ray *ray2 = dynamic_cast<const Ray *>(shape2);
            if (ray2 != NULL) {
                std::vector<Vec2d> ret = cada_getIntersectionPointsLS(
                    ray2->getLineShape().get(), spline1, false);
                if (limited)
                    ret = ray2->filterOnShape(ret, true);
                return ret;
            }
            const XLine *xline2 = dynamic_cast<const XLine *>(shape2);
            if (xline2 != NULL) {
                return cada_getIntersectionPointsLS(
                    xline2->getLineShape().release(), spline1, false);
            }
            const BSpline *spline2 = dynamic_cast<const BSpline *>(shape2);
            if (spline2 != NULL) {
                return cada_getIntersectionPointsSS(spline1, spline2, limited,
                                                    same, NS::PointTolerance);
            }

            // spline, polyline, ...:
            const Polyline *explodable2 =
                dynamic_cast<const Polyline *>(shape2);
            if (explodable2 != NULL) {
                return cada_getIntersectionPointsSX(spline1, explodable2,
                                                    limited);
            }
        }
    }

    {
        const Polyline *explodable1 = dynamic_cast<const Polyline *>(shape1);
        if (explodable1 != NULL) {
            if (!same) {
                const Line *line2 = dynamic_cast<const Line *>(shape2);
                if (line2 != NULL) {
                    return cada_getIntersectionPointsLX(line2, explodable1,
                                                        limited);
                }
                const Arc *arc2 = dynamic_cast<const Arc *>(shape2);
                if (arc2 != NULL) {
                    return cada_getIntersectionPointsAX(arc2, explodable1,
                                                        limited);
                }
                const Circle *circle2 = dynamic_cast<const Circle *>(shape2);
                if (circle2 != NULL) {
                    return cada_getIntersectionPointsCX(circle2, explodable1,
                                                        limited);
                }
                const Ellipse *ellipse2 = dynamic_cast<const Ellipse *>(shape2);
                if (ellipse2 != NULL) {
                    return cada_getIntersectionPointsEX(ellipse2, explodable1,
                                                        limited);
                }
                const Ray *ray2 = dynamic_cast<const Ray *>(shape2);
                if (ray2 != NULL) {
                    std::vector<Vec2d> ret = cada_getIntersectionPointsLX(
                        ray2->getLineShape().get(), explodable1, false);
                    if (limited)
                        ret = ray2->filterOnShape(ret, true);
                    return ret;
                }
                const XLine *xline2 = dynamic_cast<const XLine *>(shape2);
                if (xline2 != NULL) {
                    return cada_getIntersectionPointsLX(
                        xline2->getLineShape().release(), explodable1, false);
                }
                const BSpline *spline2 = dynamic_cast<const BSpline *>(shape2);
                if (spline2 != NULL) {
                    return cada_getIntersectionPointsSX(spline2, explodable1,
                                                        limited);
                }
            }

            // spline, polyline, ...:
            const Polyline *explodable2 =
                dynamic_cast<const Polyline *>(shape2);
            if (explodable2 != NULL) {
                return cada_getIntersectionPointsXX(explodable1, explodable2,
                                                    limited, same);
            }
        }
    }

    return std::vector<Vec2d>();
}

/* ---------------------------------- impls --------------------------------- */

std::vector<Vec2d> cada_getIntersectionPointsLL(const Line *line1,
                                                const Line *line2,
                                                bool limited1, bool limited2)
{
    assert(line1);
    assert(line2);
    std::vector<Vec2d> res;
    double a1 = line1->getEndPoint().y - line1->getStartPoint().y;
    double b1 = line1->getEndPoint().x - line1->getStartPoint().x;
    double c1 = a1 * line1->getStartPoint().x + b1 * line1->getStartPoint().y;

    double a2 = line2->getEndPoint().y - line2->getStartPoint().y;
    double b2 = line2->getEndPoint().x - line2->getStartPoint().x;
    double c2 = a2 * line2->getStartPoint().x + b2 * line2->getStartPoint().y;

    double det = a1 * b2 - a2 * b1;
    if (fabs(det) < 1.0e-6) {
        return res;
    }
    else {
        Vec2d v((b2 * c1 - b1 * c2) / det, (a1 * c2 - a2 * c1) / det);

        if ((!limited1 || line1->isOnShape(v)) &&
            (!limited2 || line2->isOnShape(v))) {
            res.push_back(v);
            return res;
        }
    }
    return res;
}

std::vector<Vec2d> cada_getIntersectionPointsLA(const Line *line1,
                                                const Arc *arc2, bool limited1,
                                                bool limited2)
{
    assert(line1);
    assert(arc2);

    std::unique_ptr<Circle> arc_c = ShapeFactory::instance()->createCircle(
        arc2->getCenter(), arc2->getRadius());
    std::vector<Vec2d> candidates =
        cada_getIntersectionPointsLC(line1, arc_c.release(), limited1);

    if (!limited2) {
        return candidates;
    }
    std::vector<Vec2d> res;

    for (size_t i = 0; i < candidates.size(); ++i) {
        if (arc2->isOnShape(candidates[i]))
            res.push_back(candidates[i]);
    }

    return res;
}

std::vector<Vec2d> cada_getIntersectionPointsLC(const Line *line1,
                                                const Circle *circle2,
                                                bool limited)
{
    assert(line1);
    assert(circle2);
    std::vector<Vec2d> res;

    Vec2d vLineCenter = line1->getVectorTo(circle2->getCenter(), false);
    double dist = vLineCenter.getMagnitude();

    // special case: arc almost touches line (tangent with tiny gap or tiny
    // overlap):
    if (Math::fuzzyCompare(dist, circle2->getRadius(), 1.0e-6)) {
        Vec2d sol = circle2->getCenter() - vLineCenter;
        if (!limited || line1->isOnShape(sol, true, 1.0e-6)) {
            res.push_back(sol);
        }
        // ret.setTangent(true);
        return res;
    }

    Vec2d p = line1->getStartPoint();
    Vec2d d = line1->getEndPoint() - line1->getStartPoint();
    if (d.getMagnitude() < 1.0e-6) {
        return res;
    }

    Vec2d delta = p - circle2->getCenter();

    // root term:
    double term =
        std::pow(Vec2d::getDotProduct(d, delta), 2.0) -
        std::pow(d.getMagnitude(), 2.0) * (std::pow(delta.getMagnitude(), 2.0) -
                                           std::pow(circle2->getRadius(), 2.0));

    // no intersection:
    if (term < 0.0) {
        return res;
    }

    // one or two intersections:
    double t1 = (-Vec2d::getDotProduct(d, delta) + sqrt(term)) /
                std::pow(d.getMagnitude(), 2.0);
    double t2;
    bool tangent = false;

    // only one intersection:
    if (fabs(term) < NS::PointTolerance) {
        t2 = t1;
        tangent = true;
    }

    // two intersections
    else {
        t2 = (-Vec2d::getDotProduct(d, delta) - sqrt(term)) /
             std::pow(d.getMagnitude(), 2.0);
    }

    Vec2d sol1;
    Vec2d sol2 = Vec2d::invalid;

    sol1 = p + d * t1;

    if (!tangent) {
        sol2 = p + d * t2;
    }

    if (!limited || line1->isOnShape(sol1, true, 1.0e-6)) {
        res.push_back(sol1);
    }
    if (sol2.isValid()) {
        if (!limited || line1->isOnShape(sol2, true, 1.0e-6)) {
            res.push_back(sol2);
        }
    }
    // ret.setTangent(tangent);

    // tangent with two intersections very close to each other:
    if (res.size() == 2 && res[0].equalsFuzzy(res[1])) {
        res.pop_back();
    }

    return res;
}

std::vector<Vec2d> cada_getIntersectionPointsLE(const Line *line1,
                                                const Ellipse *ellipse2,
                                                bool limited1, bool limited2)
{
    assert(line1);
    assert(ellipse2);
    std::vector<Vec2d> res;

    // find out if line1 is (almost) a tangent:
    Vec2d tangentPoint = ellipse2->getTangentPoint(line1);
    if (tangentPoint.isValid()) {
        res.push_back(tangentPoint);
        return res;
    }

    // rotate into normal position:
    double ang = ellipse2->getAngle();

    double rx = ellipse2->getMajorRadius();
    double ry = ellipse2->getMinorRadius();
    Vec2d center = ellipse2->getCenter();
    Vec2d a1 = line1->getStartPoint();
    a1.rotate(-ang, center);
    Vec2d a2 = line1->getEndPoint();
    a2.rotate(-ang, center);
    Vec2d origin = a1;
    Vec2d dir = a2 - a1;
    Vec2d diff = origin - center;
    Vec2d mDir = Vec2d(dir.x / (rx * rx), dir.y / (ry * ry));
    Vec2d mDiff = Vec2d(diff.x / (rx * rx), diff.y / (ry * ry));

    double a = Vec2d::getDotProduct(dir, mDir);
    double b = Vec2d::getDotProduct(dir, mDiff);
    double c = Vec2d::getDotProduct(diff, mDiff) - 1.0;
    double d = b * b - a * c;

    Vec2d res1 = Vec2d::invalid;
    Vec2d res2 = Vec2d::invalid;

    if (d < 0) {
        // no solution
    }
    else if (d > 0) {
        double root = sqrt(d);
        double t_a = (-b - root) / a;
        double t_b = (-b + root) / a;

        res1 = a1.getLerp(a2, t_a).rotate(ang, center);
        res2 = a1.getLerp(a2, t_b).rotate(ang, center);
    }
    else {
        double t = -b / a;
        if (0 <= t && t <= 1) {
            // one solution:
            res1 = a1.getLerp(a2, t).rotate(ang, center);
        }
        else {
            // no solution
        }
    }

    if (res1.isValid()) {
        if ((!limited1 || line1->isOnShape(res1)) &&
            (!limited2 || ellipse2->isOnShape(res1))) {
            res.push_back(res1);
        }
    }
    if (res2.isValid()) {
        if ((!limited1 || line1->isOnShape(res2)) &&
            (!limited2 || ellipse2->isOnShape(res2))) {
            res.push_back(res2);
        }
    }

    return res;
}

std::vector<Vec2d> cada_getIntersectionPointsLX(const Line *line1,
                                                const Polyline *explodable2,
                                                bool limited)
{
    return std::vector<Vec2d>();
}

std::vector<Vec2d> cada_getIntersectionPointsLS(const Line *line1,
                                                const BSpline *spline2,
                                                bool limited)
{
    return std::vector<Vec2d>();
}

std::vector<Vec2d> cada_getIntersectionPointsAA(const Arc *arc1,
                                                const Arc *arc2, bool limited)
{
    assert(arc1);
    assert(arc2);

    std::unique_ptr<Circle> c1 = ShapeFactory::instance()->createCircle(
        arc1->getCenter(), arc1->getRadius());
    std::unique_ptr<Circle> c2 = ShapeFactory::instance()->createCircle(
        arc2->getCenter(), arc2->getRadius());

    std::vector<Vec2d> candidates =
        cada_getIntersectionPointsCC(c1.release(), c2.release());
    if (!limited) {
        return candidates;
    }

    std::vector<Vec2d> res;
    for (size_t i = 0; i < candidates.size(); ++i) {
        if (arc1->isOnShape(candidates[i]) && arc2->isOnShape(candidates[i])) {
            res.push_back(candidates[i]);
        }
    }

    return res;
}

std::vector<Vec2d> cada_getIntersectionPointsAC(const Arc *arc1,
                                                const Circle *circle2,
                                                bool limited)
{
    assert(arc1);
    assert(circle2);

    std::unique_ptr<Circle> c1 = ShapeFactory::instance()->createCircle(
        arc1->getCenter(), arc1->getRadius());

    std::vector<Vec2d> candidates =
        cada_getIntersectionPointsCC(c1.release(), circle2);
    if (!limited) {
        return candidates;
    }

    std::vector<Vec2d> res;
    for (size_t i = 0; i < candidates.size(); ++i) {
        if (arc1->isOnShape(candidates[i])) {
            res.push_back(candidates[i]);
        }
    }

    return res;
}

std::vector<Vec2d> cada_getIntersectionPointsAE(const Arc *arc1,
                                                const Ellipse *ellipse2,
                                                bool limited)
{
    return std::vector<Vec2d>();
}

std::vector<Vec2d> cada_getIntersectionPointsAS(const Arc *arc1,
                                                const BSpline *spline2,
                                                bool limited)
{
    return std::vector<Vec2d>();
}

std::vector<Vec2d> cada_getIntersectionPointsAX(const Arc *arc1,
                                                const Polyline *explodable2,
                                                bool limited)
{
    return std::vector<Vec2d>();
}

std::vector<Vec2d> cada_getIntersectionPointsCC(const Circle *circle1,
                                                const Circle *circle2)
{
    return std::vector<Vec2d>();
}

std::vector<Vec2d> cada_getIntersectionPointsCE(const Circle *circle1,
                                                const Ellipse *ellipse2)
{
    return std::vector<Vec2d>();
}

std::vector<Vec2d> cada_getIntersectionPointsCS(const Circle *circle1,
                                                const BSpline *spline2,
                                                bool limited)
{
    return std::vector<Vec2d>();
}

std::vector<Vec2d> cada_getIntersectionPointsCX(const Circle *circle1,
                                                const Polyline *explodable2,
                                                bool limited)
{
    return std::vector<Vec2d>();
}

std::vector<Vec2d> cada_getIntersectionPointsEE(const Ellipse *ellipse1,
                                                const Ellipse *ellipse2)
{
    return std::vector<Vec2d>();
}

std::vector<Vec2d> cada_getIntersectionPointsEE(const Ellipse *ellipse1,
                                                const Ellipse *ellipse2,
                                                bool limited)
{
    return std::vector<Vec2d>();
}

std::vector<Vec2d> cada_getIntersectionPointsES(const Ellipse *ellipse1,
                                                const BSpline *spline2,
                                                bool limited)
{
    return std::vector<Vec2d>();
}

std::vector<Vec2d> cada_getIntersectionPointsEX(const Ellipse *ellipse1,
                                                const Polyline *explodable2,
                                                bool limited)
{
    return std::vector<Vec2d>();
}

std::vector<Vec2d> cada_getIntersectionPointsSS(const BSpline *spline1,
                                                const BSpline *spline2,
                                                bool limited, bool same,
                                                double tol)
{
    return std::vector<Vec2d>();
}

std::vector<Vec2d> cada_getIntersectionPointsSX(const BSpline *spline1,
                                                const Polyline *explodable2,
                                                bool limited)
{
    return std::vector<Vec2d>();
}

std::vector<Vec2d> cada_getIntersectionPointsXX(const Polyline *explodable1,
                                                const Polyline *explodable2,
                                                bool limited, bool same)
{
    return std::vector<Vec2d>();
}

} // namespace algorithm
} // namespace cada