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

static constexpr auto epsTolerance = 1.0e-02;

/* -------------------------------- functions ------------------------------- */
std::vector<Vec2d> cada_getIntersectionPoints(const Shape *shape1,
                                              const Shape *shape2, bool limited,
                                              bool same, bool force);

static double ellipse2tr(double x, double y, double AA, double BB, double CC,
                         double DD, double EE, double FF)
{
    return (AA * x * x + BB * x * y + CC * y * y + DD * x + EE * y + FF);
}

static void getQuadRoots(double p[], double r[][5])
{
    /**
     * Array r[3][5] p[5]
     * Roots of poly p[0]*x^2 + p[1]*x + p[2]=0
     * x=r[1][k] + i r[2][k] k=1,2
     */
    double b, c, d;
    b = -p[1] / (2.0 * p[0]);
    c = p[2] / p[0];
    d = b * b - c;
    if (d >= 0.0) {
        if (b > 0.0)
            b = (r[1][2] = (sqrt(d) + b));
        else
            b = (r[1][2] = (-sqrt(d) + b));
        r[1][1] = c / b;
        r[2][1] = (r[2][2] = 0.0);
    }
    else {
        d = (r[2][1] = sqrt(-d));
        r[2][2] = -d;
        r[1][1] = (r[1][2] = b);
    }
}

static void getCubicRoots(double p[], double r[][5])
{
    /**
     * Array r[3][5] p[5]
     * Roots of poly p[0]*x^3 + p[1]*x^2 + p[2]*x + p[3] = 0
     * x=r[1][k] + i r[2][k] k=1,...,3
     * Assumes 0<arctan(x)<pi/2 for x>0
     */
    double s, t, b, c, d;
    int k;
    if (p[0] != 1.0) {
        for (k = 1; k < 4; k++)
            p[k] = p[k] / p[0];
        p[0] = 1.0;
    }
    s = p[1] / 3.0;
    t = s * p[1];
    b = 0.5 * (s * (t / 1.5 - p[2]) + p[3]);
    t = (t - p[2]) / 3.0;
    c = t * t * t;
    d = b * b - c;
    if (d >= 0.0) {
        d = pow((sqrt(d) + fabs(b)), 1.0 / 3.0);
        if (d != 0.0) {
            if (b > 0.0)
                b = -d;
            else
                b = d;
            c = t / b;
        }
        d = r[2][2] = sqrt(0.75) * (b - c);
        b = b + c;
        c = r[1][2] = -0.5 * b - s;
        if ((b > 0.0 && s <= 0.0) || (b < 0.0 && s > 0.0)) {
            r[1][1] = c;
            r[2][1] = -d;
            r[1][3] = b - s;
            r[2][3] = 0.0;
        }
        else {
            r[1][1] = b - s;
            r[2][1] = 0.0;
            r[1][3] = c;
            r[2][3] = -d;
        }

    } /* end 2 equal or complex roots */
    else {
        if (b == 0.0)
            d = atan(1.0) / 1.5;
        else
            d = atan(sqrt(-d) / fabs(b)) / 3.0;
        if (b < 0.0)
            b = 2.0 * sqrt(t);
        else
            b = -2.0 * sqrt(t);
        c = cos(d) * b;
        t = -sqrt(0.75) * sin(d) * b - 0.5 * c;
        d = -t - c - s;
        c = c - s;
        t = t - s;
        if (fabs(c) > fabs(t)) {
            r[1][3] = c;
        }
        else {
            r[1][3] = t;
            t = c;
        }
        if (fabs(d) > fabs(t)) {
            r[1][2] = d;
        }
        else {
            r[1][2] = t;
            t = d;
        }
        r[1][1] = t;
        for (k = 1; k < 4; k++)
            r[2][k] = 0.0;
    }
}

static void getBiQuadRoots(double p[], double r[][5])
{
    /**
     *Array r[3][5] p[5]
     *Roots of poly p[0]*x^4 + p[1]*x^3 + p[2]*x^2 + p[3]*x + p[4] = 0
     *x=r[1][k] + i r[2][k] k=1,...,4
     */
    double a, b, c, d, e;
    int k, j;
    if (p[0] != 1.0) {
        for (k = 1; k < 5; k++)
            p[k] = p[k] / p[0];
        p[0] = 1.0;
    }
    e = 0.25 * p[1];
    b = 2.0 * e;
    c = b * b;
    d = 0.75 * c;
    b = p[3] + b * (c - p[2]);
    a = p[2] - d;
    c = p[4] + e * (e * a - p[3]);
    a = a - d;
    p[1] = 0.5 * a;
    p[2] = (p[1] * p[1] - c) * 0.25;
    p[3] = b * b / (-64.0);
    if (p[3] < 0.0) {
        getCubicRoots(p, r);
        for (k = 1; k < 4; k++) {
            if (r[2][k] == 0.0 && r[1][k] > 0.0) {
                d = r[1][k] * 4.0;
                a = a + d;
                if (a >= 0.0 && b >= 0.0)
                    p[1] = sqrt(d);
                else if (a <= 0.0 && b <= 0.0)
                    p[1] = sqrt(d);
                else
                    p[1] = -sqrt(d);
                b = 0.5 * (a + b / p[1]);
                goto QUAD;
            }
        }
    }
    if (p[2] < 0.0) {
        b = sqrt(c);
        d = b + b - a;
        p[1] = 0.0;
        if (d > 0.0)
            p[1] = sqrt(d);
    }
    else {
        if (p[1] > 0.0)
            b = sqrt(p[2]) * 2.0 + p[1];
        else
            b = -sqrt(p[2]) * 2.0 + p[1];
        if (b != 0.0) {
            p[1] = 0.0;
        }
        else {
            for (k = 1; k < 5; k++) {
                r[1][k] = -e;
                r[2][k] = 0.0;
            }
            goto END;
        }
    }
QUAD:
    p[2] = c / b;
    getQuadRoots(p, r);
    for (k = 1; k < 3; k++)
        for (j = 1; j < 3; j++)
            r[j][k + 2] = r[j][k];
    p[1] = -p[1];
    p[2] = b;
    getQuadRoots(p, r);
    for (k = 1; k < 5; k++)
        r[1][k] = r[1][k] - e;
END:
    return;
}

static void moveList(std::vector<Vec2d> &list, const Vec2d &offset)
{
    for (size_t i = 0; i < list.size(); i++) {
        list[i].move(offset);
    }
}

static void scaleList(std::vector<Vec2d> &list, const Vec2d &factors,
                      const Vec2d &center = Vec2d::nullVector)
{
    for (size_t i = 0; i < list.size(); i++) {
        list[i].scale(factors, center);
    }
}

static void rotateList(std::vector<Vec2d> &list, double rotation)
{
    for (size_t i = 0; i < list.size(); i++) {
        list[i].rotate(rotation);
    }
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
                                                       const Spline *spline2,
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
                                                       const Spline *spline2,
                                                       bool limited);
static std::vector<Vec2d>
cada_getIntersectionPointsAX(const Arc *arc1, const Polyline *explodable2,
                             bool limited);
static std::vector<Vec2d> cada_getIntersectionPointsCC(const Circle *circle1,
                                                       const Circle *circle2);
static std::vector<Vec2d> cada_getIntersectionPointsCE(const Circle *circle1,
                                                       const Ellipse *ellipse2);
static std::vector<Vec2d> cada_getIntersectionPointsCS(const Circle *circle1,
                                                       const Spline *spline2,
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
                                                       const Spline *spline2,
                                                       bool limited);
static std::vector<Vec2d>
cada_getIntersectionPointsEX(const Ellipse *ellipse1,
                             const Polyline *explodable2, bool limited);
static std::vector<Vec2d> cada_getIntersectionPointsSS(const Spline *spline1,
                                                       const Spline *spline2,
                                                       bool limited, bool same,
                                                       double tol);
static std::vector<Vec2d>
cada_getIntersectionPointsSX(const Spline *spline1, const Polyline *explodable2,
                             bool limited);
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
            const Spline *spline2 = dynamic_cast<const Spline *>(shape2);
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
                    line1, xline2->getLineShape().get(), limited, false);
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

            auto &&xline1 = ray1->getLineShape();
            std::vector<Vec2d> ret = cada_getIntersectionPoints(
                xline1.get(), shape2, limited, same, force);
            if (limited)
                ret = ray1->filterOnShape(ret, true);
            return ret;
        }
    }

    {
        const XLine *xline1 = dynamic_cast<const XLine *>(shape1);
        if (xline1) {
            if (same)
                return empty;

            std::unique_ptr<Line> line1 = xline1->getLineShape();

            const Line *line2 = dynamic_cast<const Line *>(shape2);
            if (line2) {
                return cada_getIntersectionPointsLL(line1.get(), line2, false,
                                                    limited);
            }
            const Arc *arc2 = dynamic_cast<const Arc *>(shape2);
            if (arc2) {
                return cada_getIntersectionPointsLA(line1.get(), arc2, false,
                                                    limited);
            }
            const Circle *circle2 = dynamic_cast<const Circle *>(shape2);
            if (circle2) {
                return cada_getIntersectionPointsLC(line1.get(), circle2,
                                                    false);
            }
            const Ellipse *ellipse2 = dynamic_cast<const Ellipse *>(shape2);
            if (ellipse2) {
                return cada_getIntersectionPointsLE(line1.get(), ellipse2,
                                                    false, limited);
            }
            const Spline *spline2 = dynamic_cast<const Spline *>(shape2);
            if (spline2) {
                return cada_getIntersectionPointsLS(line1.get(), spline2,
                                                    false);
            }
            const Ray *ray2 = dynamic_cast<const Ray *>(shape2);
            if (ray2) {
                std::vector<Vec2d> ret = cada_getIntersectionPointsLL(
                    line1.get(), ray2->getLineShape().get(), false, false);
                if (limited)
                    ret = ray2->filterOnShape(ret, true);
                return ret;
            }
            const XLine *xline2 = dynamic_cast<const XLine *>(shape2);
            if (xline2) {
                return cada_getIntersectionPointsLL(
                    line1.get(), xline2->getLineShape().get(), false, false);
            }

            const Polyline *explodable2 =
                dynamic_cast<const Polyline *>(shape2);
            if (explodable2) {
                return cada_getIntersectionPointsLX(line1.get(), explodable2,
                                                    false);
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
            const Spline *spline2 = dynamic_cast<const Spline *>(shape2);
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
                    xline2->getLineShape().get(), arc1, false, limited);
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
            const Spline *spline2 = dynamic_cast<const Spline *>(shape2);
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
                    xline2->getLineShape().get(), circle1, false);
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
            const Spline *spline2 = dynamic_cast<const Spline *>(shape2);
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
                    xline2->getLineShape().get(), ellipse1, false, limited);
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
        const Spline *spline1 = dynamic_cast<const Spline *>(shape1);
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
                    xline2->getLineShape().get(), spline1, false);
            }
            const Spline *spline2 = dynamic_cast<const Spline *>(shape2);
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
                        xline2->getLineShape().get(), explodable1, false);
                }
                const Spline *spline2 = dynamic_cast<const Spline *>(shape2);
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
    double b1 = line1->getStartPoint().x - line1->getEndPoint().x;
    double c1 = a1 * line1->getStartPoint().x + b1 * line1->getStartPoint().y;

    double a2 = line2->getEndPoint().y - line2->getStartPoint().y;
    double b2 = line2->getStartPoint().x - line2->getEndPoint().x;
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
        cada_getIntersectionPointsLC(line1, arc_c.get(), limited1);

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
    assert(line1);
    assert(explodable2);
    std::vector<Vec2d> res;

    auto &&sub = explodable2->getExploded();
    for (auto &&it : sub) {
        shape::Line *line2 = dynamic_cast<shape::Line *>(it.get());
        if (line2) {
            auto points =
                cada_getIntersectionPointsLL(line1, line2, limited, true);
            res.insert(res.end(), points.begin(), points.end());
            continue;
        }
        shape::Arc *arc2 = dynamic_cast<shape::Arc *>(it.get());
        if (arc2) {
            auto points =
                cada_getIntersectionPointsLA(line1, arc2, limited, true);
            res.insert(res.end(), points.begin(), points.end());
            continue;
        }
    }

    return res;
}

std::vector<Vec2d> cada_getIntersectionPointsLS(const Line *line1,
                                                const Spline *spline2,
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
        cada_getIntersectionPointsCC(c1.get(), c2.get());
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
        cada_getIntersectionPointsCC(c1.get(), circle2);
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
    assert(arc1);
    assert(ellipse2);

    std::unique_ptr<Circle> c1 = ShapeFactory::instance()->createCircle(
        arc1->getCenter(), arc1->getRadius());

    std::vector<Vec2d> candidates =
        cada_getIntersectionPointsCE(c1.get(), ellipse2);

    if (!limited) {
        return candidates;
    }

    std::vector<Vec2d> res;
    double a1 = ellipse2->getCenter().getAngleTo(ellipse2->getStartPoint());
    double a2 = ellipse2->getCenter().getAngleTo(ellipse2->getEndPoint());

    for (size_t i = 0; i < candidates.size(); i++) {
        Vec2d c = candidates[i];
        if (arc1->isOnShape(c)) {
            if (!ellipse2->isFullEllipse()) {
                double a = ellipse2->getCenter().getAngleTo(c);
                if (!Math::isAngleBetween(a, a1, a2, ellipse2->isReversed())) {
                    continue;
                }
            }

            res.push_back(c);
        }
    }
    return res;
}

std::vector<Vec2d> cada_getIntersectionPointsAS(const Arc *arc1,
                                                const Spline *spline2,
                                                bool limited)
{
    return std::vector<Vec2d>();
}

std::vector<Vec2d> cada_getIntersectionPointsAX(const Arc *arc1,
                                                const Polyline *explodable2,
                                                bool limited)
{
    (void)limited;
    assert(arc1);
    assert(explodable2);
    std::vector<Vec2d> res;

    auto &&sub = explodable2->getExploded();
    for (auto &&it : sub) {
        shape::Line *line2 = dynamic_cast<shape::Line *>(it.get());
        if (line2) {
            auto points = cada_getIntersectionPointsLA(line2, arc1, true, true);
            res.insert(res.end(), points.begin(), points.end());
            continue;
        }
        shape::Arc *arc2 = dynamic_cast<shape::Arc *>(it.get());
        if (arc2) {
            auto points = cada_getIntersectionPointsAA(arc1, arc2, true);
            res.insert(res.end(), points.begin(), points.end());
            continue;
        }
    }

    return res;
}

std::vector<Vec2d> cada_getIntersectionPointsCC(const Circle *circle1,
                                                const Circle *circle2)
{
    assert(circle1);
    assert(circle2);
    double r1 = circle1->getRadius();
    double r2 = circle2->getRadius();
    if (r1 < r2) {
        // make sure circle 1 is the larger one (for tangency detection):
        return cada_getIntersectionPointsCC(circle2, circle1);
    }

    std::vector<Vec2d> res;

    Vec2d c1 = circle1->getCenter();
    Vec2d c2 = circle2->getCenter();

    Vec2d u = c2 - c1;
    double uMag = u.getMagnitude();

    // concentric
    if (uMag < NS::PointTolerance) {
        return res;
    }

    double tol = (r1 + r2) / 200000;

    // the two circles (almost) touch externally / internally in one point
    // (tangent):
    if (Math::fuzzyCompare(uMag, r1 + r2, tol) ||
        Math::fuzzyCompare(uMag, fabs(r1 - r2), tol)) {

        u.setMagnitude(r1);
        res.push_back(c1 + u);
        return res;
    }

    Vec2d v = Vec2d(u.y, -u.x);

    double s, t1, t2, term;

    s = 1.0 / 2.0 * ((r1 * r1 - r2 * r2) / (std::pow(uMag, 2.0)) + 1.0);

    term = (r1 * r1) / (std::pow(uMag, 2.0))-s * s;

    // no intersection:
    if (term < 0.0) {
        return res;
    }

    // one or two intersections:
    t1 = sqrt(term);
    t2 = -sqrt(term);

    Vec2d sol1 = c1 + u * s + v * t1;
    Vec2d sol2 = c1 + u * s + v * t2;

    if (sol1.equalsFuzzy(sol2, tol)) {
        res.push_back(sol1);
    }
    else {
        res.push_back(sol1);
        res.push_back(sol2);
    }

    return res;
}

std::vector<Vec2d> cada_getIntersectionPointsCE(const Circle *circle1,
                                                const Ellipse *ellipse2)
{
    auto ellipse1 = ShapeFactory::instance()->createEllipse(
        circle1->getCenter(), Vec2d(circle1->getRadius(), 0.), 1.0, 0.0,
        2.0 * M_PI, false);

    return cada_getIntersectionPointsEE(ellipse1.get(), ellipse2);
}

std::vector<Vec2d> cada_getIntersectionPointsCS(const Circle *circle1,
                                                const Spline *spline2,
                                                bool limited)
{
    return std::vector<Vec2d>();
}

std::vector<Vec2d> cada_getIntersectionPointsCX(const Circle *circle1,
                                                const Polyline *explodable2,
                                                bool limited)
{
    (void)limited;
    assert(circle1);
    assert(explodable2);
    std::vector<Vec2d> res;

    auto &&sub = explodable2->getExploded();
    for (auto &&it : sub) {
        shape::Line *line2 = dynamic_cast<shape::Line *>(it.get());
        if (line2) {
            auto points = cada_getIntersectionPointsLC(line2, circle1, true);
            res.insert(res.end(), points.begin(), points.end());
            continue;
        }
        shape::Arc *arc2 = dynamic_cast<shape::Arc *>(it.get());
        if (arc2) {
            auto points = cada_getIntersectionPointsAC(arc2, circle1, true);
            res.insert(res.end(), points.begin(), points.end());
            continue;
        }
    }

    return res;
}

std::vector<Vec2d> cada_getIntersectionPointsEE(const Ellipse *ellipse1,
                                                const Ellipse *ellipse2)
{
    assert(ellipse1);
    assert(ellipse2);
    std::vector<Vec2d> ret;

    // two full ellipses:
    // if bounding boxes don't intersect, ellipses don't either:
    if (ellipse1->isFullEllipse() && ellipse2->isFullEllipse() &&
        !ellipse1->getBoundingBox().intersects(ellipse2->getBoundingBox())) {

        return ret;
    }

    // normalize ellipse ratios:
    std::unique_ptr<Ellipse> ellipse1Copy = ellipse1->clone();
    if (ellipse1Copy->getMajorRadius() < ellipse1Copy->getMinorRadius()) {
        ellipse1Copy->switchMajorMinor();
    }
    std::unique_ptr<Ellipse> ellipse2Copy = ellipse2->clone();
    if (ellipse2Copy->getMajorRadius() < ellipse2Copy->getMinorRadius()) {
        ellipse2Copy->switchMajorMinor();
    }

    // for later comparison, make sure that major points are in
    // quadrant I or II (relative to the ellipse center):
    if (fabs(ellipse1Copy->getMajorPoint().y) < NS::PointTolerance) {
        if (ellipse1Copy->getMajorPoint().x < 0.0) {
            ellipse1Copy->setMajorPoint(-ellipse1Copy->getMajorPoint());
        }
    }
    else {
        if (ellipse1Copy->getMajorPoint().y < 0.0) {
            ellipse1Copy->setMajorPoint(-ellipse1Copy->getMajorPoint());
        }
    }
    if (fabs(ellipse2Copy->getMajorPoint().y) < NS::PointTolerance) {
        if (ellipse2Copy->getMajorPoint().x < 0.0) {
            ellipse2Copy->setMajorPoint(-ellipse2Copy->getMajorPoint());
        }
    }
    else {
        if (ellipse2Copy->getMajorPoint().y < 0.0) {
            ellipse2Copy->setMajorPoint(-ellipse2Copy->getMajorPoint());
        }
    }

    // for comparison:
    bool identicalCenter =
        (ellipse1Copy->getCenter() - ellipse2Copy->getCenter()).getMagnitude() <
        NS::PointTolerance;
    //    bool identicalShape =
    //        (ellipse1Copy->getMajorPoint() -
    //        ellipse2Copy->getMajorPoint()).getMagnitude() < NS::PointTolerance
    //        && fabs(ellipse1Copy->getMajorRadius() -
    //        ellipse2Copy->getMajorRadius()) < NS::PointTolerance &&
    //        fabs(ellipse1Copy->getMinorRadius() -
    //        ellipse2Copy->getMinorRadius()) < NS::PointTolerance;
    bool identicalRatio =
        fabs(ellipse1Copy->getRatio() - ellipse2Copy->getRatio()) < 1.0e-4;
    double angleDifference = fabs(Math::getAngleDifference180(
        ellipse1Copy->getAngle(), ellipse2Copy->getAngle()));
    bool identicalRotation =
        angleDifference < 1.0e-4 || angleDifference > M_PI - 1.0e-4;

    // ellipses are identical (no intersection points):
    //    if (identicalCenter && identicalShape) {
    if (identicalCenter && identicalRatio && identicalRotation) {
        return ret;
    }

    // special case: ellipse shapes are identical (different positions):
    // if (identicalShape) {
    if (identicalRatio && identicalRotation) {
        double angle = -ellipse1Copy->getAngle();
        double yScale = 1.0 / ellipse1Copy->getRatio();

        Vec2d circleCenter1 = ellipse1Copy->getCenter();
        circleCenter1.rotate(angle);
        circleCenter1.scale(Vec2d(1.0, yScale));
        Vec2d circleCenter2 = ellipse2Copy->getCenter();
        circleCenter2.rotate(angle);
        circleCenter2.scale(Vec2d(1.0, yScale));

        auto circle1 = ShapeFactory::instance()->createCircle(
            circleCenter1, ellipse1Copy->getMajorRadius());
        auto circle2 = ShapeFactory::instance()->createCircle(
            circleCenter2, ellipse2Copy->getMajorRadius());

        ret = cada_getIntersectionPointsCC(circle1.get(), circle2.get());

        scaleList(ret, Vec2d(1.0, 1.0 / yScale));
        rotateList(ret, -angle);

        return ret;
    }

    // transform ellipse2 to coordinate system of ellipse1:
    Vec2d centerOffset = -ellipse1Copy->getCenter();
    double angleOffset = -ellipse1Copy->getAngle();

    double majorRadius1 = ellipse1Copy->getMajorRadius();
    double majorRadius2 = ellipse2Copy->getMajorRadius();

    // special case: treat first ellipse as a line:
    if (ellipse1Copy->getMinorRadius() < NS::PointTolerance ||
        ellipse1Copy->getRatio() < NS::PointTolerance) {

        ellipse2Copy->move(centerOffset);
        ellipse2Copy->rotate(angleOffset);

        auto line = ShapeFactory::instance()->createLine(
            Vec2d(-majorRadius1, 0.0), Vec2d(majorRadius1, 0.0));
        ret = cada_getIntersectionPointsLE(line.get(), ellipse2Copy.get(), true,
                                           true);
        rotateList(ret, -angleOffset);
        moveList(ret, -centerOffset);
        return ret;
    }

    // special case: treat second ellipse as a line:
    if (ellipse2Copy->getMinorRadius() < NS::PointTolerance ||
        ellipse2Copy->getRatio() < NS::PointTolerance) {

        ellipse2Copy->move(centerOffset);
        ellipse2Copy->rotate(angleOffset);

        auto line = ShapeFactory::instance()->createLine(
            Vec2d(-majorRadius2, 0.), Vec2d(majorRadius2, 0.));
        line->rotate(ellipse2Copy->getAngle(), Vec2d(0., 0.));
        line->move(ellipse2Copy->getCenter());
        ret = cada_getIntersectionPointsLE(line.get(), ellipse1Copy.get(), true,
                                           true);
        rotateList(ret, -angleOffset);
        moveList(ret, -centerOffset);
        return ret;
    }

    double phi_1 = ellipse1Copy->getAngle();
    double a1 = ellipse1Copy->getMajorRadius();
    double b1 = ellipse1Copy->getMinorRadius();
    double h1 = ellipse1Copy->getCenter().x;
    double k1 = ellipse1Copy->getCenter().y;

    double phi_2 = ellipse2Copy->getAngle();
    double a2 = ellipse2Copy->getMajorRadius();
    double b2 = ellipse2Copy->getMinorRadius();
    double h2 = ellipse2Copy->getCenter().x;
    double k2 = ellipse2Copy->getCenter().y;

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
    if ((!(a1 > 0.0) || !(b1 > 0.0)) || (!(a2 > 0.0) || !(b2 > 0.0))) {
        return std::vector<Vec2d>();
    }

    // the rotation angles should be between -2pi and 2pi (?)
    if (fabs(phi_1) > (M_PI * 2)) {
        phi_1 = fmod(phi_1, (M_PI * 2));
    }
    if (fabs(phi_2) > (M_PI * 2)) {
        phi_2 = fmod(phi_2, (M_PI * 2));
    }

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
    if (fabs(PHI_2R) > (M_PI * 2)) {
        PHI_2R = fmod(PHI_2R, (M_PI * 2));
    }

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

    printf("second ellipse:\n");
    printf("AA: %.2f", AA);
    printf("BB: %.2f", BB);
    printf("CC: %.2f", CC);
    printf("DD: %.2f", DD);
    printf("EE: %.2f", EE);
    printf("FF: %.2f", FF);

    // create and solve the quartic equation to find intersection points:

    // If execution arrives here, the ellipses are at least 'close' to
    // intersecting.
    // Coefficients for the Quartic Polynomial in y are calculated from
    // the two implicit equations.
    // Formulas for these coefficients are derived in the reference.

    cy[4] = pow(a1, 4.0) * AA * AA +
            b1 * b1 * (a1 * a1 * (BB * BB - 2.0 * AA * CC) + b1 * b1 * CC * CC);
    cy[3] = 2.0 * b1 * (b1 * b1 * CC * EE + a1 * a1 * (BB * DD - AA * EE));
    cy[2] =
        a1 * a1 *
            ((b1 * b1 * (2.0 * AA * CC - BB * BB) + DD * DD - 2.0 * AA * FF) -
             2.0 * a1 * a1 * AA * AA) +
        b1 * b1 * (2.0 * CC * FF + EE * EE);
    cy[1] = 2.0 * b1 * (a1 * a1 * (AA * EE - BB * DD) + EE * FF);
    cy[0] = (a1 * (a1 * AA - DD) + FF) * (a1 * (a1 * AA + DD) + FF);

    for (i = 0; i < 5; i++) {
        printf("cy[ %d ]: %.6f \n", i, cy[i]);
    }

    // Once the coefficients for the Quartic Equation in y are known, the
    // roots of the quartic polynomial will represent y-values of the
    // intersection points of the two ellipse curves.
    // The quartic sometimes degenerates into a polynomial of lesser
    // degree, so handle all possible cases.
    if (fabs(cy[4]) > 0.0) {
        printf("quartic\n");
        //  quartic coefficient nonzero, use quartic formula:
        for (i = 0; i <= 3; i++) {
            py[4 - i] = cy[i] / cy[4];
        }
        py[0] = 1.0;
        for (i = 0; i < 5; i++) {
            printf("py[ %d ]: %.6f \n", i, py[i]);
        }
        getBiQuadRoots(py, r);
        nroots = 4;
    }
    else if (fabs(cy[3]) > 0.0) {
        printf("cubic\n");
        //  quartic degenerates to cubic, use cubic formula:
        for (i = 0; i <= 2; i++) {
            py[3 - i] = cy[i] / cy[3];
        }
        py[0] = 1.0;
        getCubicRoots(py, r);
        nroots = 3;
    }
    else if (fabs(cy[2]) > 0.0) {
        printf("quadratic\n");
        //  quartic degenerates to quadratic, use quadratic formula:
        for (i = 0; i <= 1; i++) {
            py[2 - i] = cy[i] / cy[2];
        }
        py[0] = 1.0;
        getQuadRoots(py, r);
        nroots = 2;
    }
    else if (fabs(cy[1]) > 0.0) {
        printf("linear\n");
        //  quartic degenerates to linear: solve directly:
        //  cy[1]*Y + cy[0] = 0
        r[1][1] = (-cy[0] / cy[1]);
        r[2][1] = 0.0;
        nroots = 1;
    }
    else {
        printf("degenerate\n");
        //  completely degenerate quartic: ellipses identical?
        //  a completely degenerate quartic, which would seem to
        //  indicate that the ellipses are identical. However, some
        //  configurations lead to a degenerate quartic with no
        //  points of intersection.
        nroots = 0;
    }

    printf("nroots:%d\n", nroots);

    // check roots of the quartic: are they points of intersection?
    // determine which roots are real, discard any complex roots
    nychk = 0;
    for (i = 1; i <= nroots; i++) {
        if (fabs(r[2][i]) < epsTolerance) {
            nychk++;
            ychk[nychk] = r[1][i] * b1;
            printf("real root: ychk[nychk]: %.6f\n", ychk[nychk]);
        }
    }

    // sort the real roots by straight insertion
    for (j = 2; j <= nychk; j++) {
        tmp0 = ychk[j];
        for (k = j - 1; k >= 1; k--) {
            if (ychk[k] <= tmp0) {
                break;
            }
            ychk[k + 1] = ychk[k];
        }
        ychk[k + 1] = tmp0;
    }

    // determine whether polynomial roots are points of intersection
    // for the two ellipses
    nintpts = 0;
    for (i = 1; i <= nychk; i++) {
        // check for multiple roots
        if ((i > 1) && (fabs(ychk[i] - ychk[i - 1]) < (epsTolerance / 2.0))) {
            // multiple roots;
            continue;
        }
        // check intersection points for ychk[i]
        if (fabs(ychk[i]) > b1) {
            x1 = 0.0;
        }
        else {
            x1 = a1 * sqrt(1.0 - (ychk[i] * ychk[i]) / (b1 * b1));
        }
        x2 = -x1;

        if (fabs(ellipse2tr(x1, ychk[i], AA, BB, CC, DD, EE, FF)) <
            epsTolerance / 2.0) {
            nintpts++;
            if (nintpts > 4) {
                return std::vector<Vec2d>();
            }
            xint[nintpts] = x1;
            yint[nintpts] = ychk[i];
        }

        if ((fabs(ellipse2tr(x2, ychk[i], AA, BB, CC, DD, EE, FF)) <
             epsTolerance / 2.0) &&
            (fabs(x2 - x1) > epsTolerance / 2.0)) {
            nintpts++;
            if (nintpts > 4) {
                return std::vector<Vec2d>();
            }
            xint[nintpts] = x2;
            yint[nintpts] = ychk[i];
        }
    }

    for (int i = 1; i <= nintpts; i++) {
        Vec2d v(xint[i], yint[i]);
        v.rotate(-angleOffset);
        v.move(-centerOffset);
        ret.push_back(v);
    }
    return ret;
}

std::vector<Vec2d> cada_getIntersectionPointsEE(const Ellipse *ellipse1,
                                                const Ellipse *ellipse2,
                                                bool limited)
{
    assert(ellipse1);
    assert(ellipse2);
    std::vector<Vec2d> candidates =
        cada_getIntersectionPointsEE(ellipse1, ellipse2);

    if (!limited) {
        return candidates;
    }

    std::vector<Vec2d> ret;

    for (size_t i = 0; i < candidates.size(); i++) {
        Vec2d c = candidates[i];
        bool onShape = true;

        double a1 = ellipse1->getCenter().getAngleTo(ellipse1->getStartPoint());
        double a2 = ellipse1->getCenter().getAngleTo(ellipse1->getEndPoint());
        double a = ellipse1->getCenter().getAngleTo(c);
        if (Math::isAngleBetween(a, a1, a2, ellipse1->isReversed())) {
            onShape = false;
        }

        a1 = ellipse2->getCenter().getAngleTo(ellipse2->getStartPoint());
        a2 = ellipse2->getCenter().getAngleTo(ellipse2->getEndPoint());
        a = ellipse2->getCenter().getAngleTo(c);
        if (!Math::isAngleBetween(a, a1, a2, ellipse2->isReversed())) {

            onShape = false;
        }

        if (onShape) {
            ret.push_back(c);
        }
    }

    return ret;
}

std::vector<Vec2d> cada_getIntersectionPointsES(const Ellipse *ellipse1,
                                                const Spline *spline2,
                                                bool limited)
{
    return std::vector<Vec2d>();
}

std::vector<Vec2d> cada_getIntersectionPointsEX(const Ellipse *ellipse1,
                                                const Polyline *explodable2,
                                                bool limited)
{
    assert(ellipse1);
    assert(explodable2);
    std::vector<Vec2d> res;

    auto &&sub = explodable2->getExploded();
    for (auto &&it : sub) {
        shape::Line *line2 = dynamic_cast<shape::Line *>(it.get());
        if (line2) {
            auto points =
                cada_getIntersectionPointsLE(line2, ellipse1, true, true);
            res.insert(res.end(), points.begin(), points.end());
            continue;
        }
        shape::Arc *arc2 = dynamic_cast<shape::Arc *>(it.get());
        if (arc2) {
            auto points = cada_getIntersectionPointsAE(arc2, ellipse1, limited);
            res.insert(res.end(), points.begin(), points.end());
            continue;
        }
    }

    return res;
}

std::vector<Vec2d> cada_getIntersectionPointsSS(const Spline *spline1,
                                                const Spline *spline2,
                                                bool limited, bool same,
                                                double tol)
{
    return std::vector<Vec2d>();
}

std::vector<Vec2d> cada_getIntersectionPointsSX(const Spline *spline1,
                                                const Polyline *explodable2,
                                                bool limited)
{
    (void)limited;
    assert(spline1);
    assert(explodable2);
    std::vector<Vec2d> res;

    auto &&sub = explodable2->getExploded();
    for (auto &&it : sub) {
        shape::Line *line2 = dynamic_cast<shape::Line *>(it.get());
        if (line2) {
            auto points = cada_getIntersectionPointsLS(line2, spline1, true);
            res.insert(res.end(), points.begin(), points.end());
            continue;
        }
        shape::Arc *arc2 = dynamic_cast<shape::Arc *>(it.get());
        if (arc2) {
            auto points = cada_getIntersectionPointsAS(arc2, spline1, true);
            res.insert(res.end(), points.begin(), points.end());
            continue;
        }
    }

    return res;
}

std::vector<Vec2d> cada_getIntersectionPointsXX(const Polyline *explodable1,
                                                const Polyline *explodable2,
                                                bool limited, bool same)
{
    assert(explodable1);
    assert(explodable2);
    (void)limited;
    std::vector<Vec2d> res;

    auto &&sub1 = explodable1->getExploded();
    auto &&sub2 = explodable2->getExploded();
    for (auto &&it1 : sub1) {
        for (auto &&it2 : sub2) {
            auto points = cada_getIntersectionPoints(it1.get(), it2.get(),
                                                     limited, same, false);
            res.insert(res.end(), points.begin(), points.end());
        }
    }
    return res;
}

} // namespace algorithm
} // namespace cada
