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

#include <assert.h>
#include <cmath>
#include <limits>
#include <numeric>
#include <cada_shape.h>

using namespace cada::shape;

namespace cada {
namespace algorithm {

/* -------------------------------- functions ------------------------------- */
shape::Vec2d cada_getVectorTo(const shape::Shape *shape,
                              const shape::Vec2d &point, bool limited,
                              double strictRange);

shape::Vec2d cada_line_getVectorTo(const shape::Line *l,
                                   const shape::Vec2d &point, bool limited,
                                   double strictRange);

shape::Vec2d cada_ellipse_getVectorTo(const shape::Ellipse *e,
                                      const shape::Vec2d &point, bool limited,
                                      double strictRange);

shape::Vec2d cada_xline_getVectorTo(const shape::XLine *x,
                                    const shape::Vec2d &point, bool limited,
                                    double strictRange);

shape::Vec2d cada_ray_getVectorTo(const shape::Ray *r,
                                  const shape::Vec2d &point, bool limited,
                                  double strictRange);

/* ---------------------------------- impls --------------------------------- */
shape::Vec2d cada_getVectorTo(const shape::Shape *shape,
                              const shape::Vec2d &point, bool limited,
                              double strictRange)
{
    assert(shape != nullptr);
    switch (shape->getShapeType()) {
    case NS::Point: {
        auto p = dynamic_cast<const Point *>(shape);
        return point - p->getPosition();
    }
    case NS::Line:
        return cada_line_getVectorTo(dynamic_cast<const Line *>(shape), point,
                                     limited, strictRange);
    case NS::Arc: {
        auto a = dynamic_cast<const Arc *>(shape);
        double angle = a->getCenter().getAngleTo(point);
        if (limited &&
            !Math::isAngleBetween(angle, a->getStartAngle(), a->getEndAngle(),
                                  a->isReversed())) {
            return Vec2d::invalid;
        }

        Vec2d v = point - a->getCenter();
        return Vec2d::createPolar(v.getMagnitude() - a->getRadius(),
                                  v.getAngle());
    }
    case NS::Circle: {
        auto c = dynamic_cast<const Circle *>(shape);
        Vec2d v = point - c->getCenter();

        // point is at the center of the circle, infinite solutions:
        if (v.getMagnitude() < NS::PointTolerance) {
            return Vec2d::invalid;
        }

        return Vec2d::createPolar(v.getMagnitude() - c->getRadius(),
                                  v.getAngle());
    }
    case NS::Ellipse:
        return cada_ellipse_getVectorTo(dynamic_cast<const Ellipse *>(shape),
                                        point, limited, strictRange);
    case NS::XLine:
        return cada_xline_getVectorTo(dynamic_cast<const XLine *>(shape), point,
                                      limited, strictRange);
    case NS::Ray:
        return cada_ray_getVectorTo(dynamic_cast<const Ray *>(shape), point,
                                    limited, strictRange);
    case NS::Polyline: {
        auto p = dynamic_cast<const Polyline *>(shape);
        Vec2d ret = Vec2d::invalid;

        auto &&sub = p->getExploded();
        for (int i = 0; i < sub.size(); i++) {
            auto &&ss = sub.at(i);
            bool lim = limited;
            if (i != 0 && i != sub.size() - 1) {
                // segments in the middle: always limited:
                lim = true;
            }
            Vec2d v = ss->getVectorTo(point, lim, strictRange);
            if (v.isValid() &&
                (!ret.isValid() || v.getMagnitude() < ret.getMagnitude())) {
                ret = v;
            }
        }

        return ret;
    }
    case NS::Spline:
    default:
        break;
    }
    return Vec2d::invalid;
}

shape::Vec2d cada_line_getVectorTo(const shape::Line *l,
                                   const shape::Vec2d &point, bool limited,
                                   double strictRange)
{
    Vec2d ae = l->getEndPoint() - l->getStartPoint();
    Vec2d ap = point - l->getStartPoint();

    if (ae.getMagnitude() < 1.0e-6) {
        return Vec2d::invalid;
    }

    if (ap.getMagnitude() < 1.0e-6) {
        // distance to start point is very small:
        return Vec2d(0, 0);
    }

    double b = Vec2d::getDotProduct(ap, ae) / Vec2d::getDotProduct(ae, ae);

    if (limited && (b < 0 || b > 1.0)) {
        // orthogonal to line does not cross line, use distance to end
        // point:
        Vec2d ret = l->getVectorFromEndpointTo(point);
        if (ret.getMagnitude() < strictRange) {
            return ret;
        }
        else {
            // not within given range:
            return Vec2d::invalid;
        }
    }

    Vec2d closestPoint = l->getStartPoint() + ae * b;

    return point - closestPoint;
}

shape::Vec2d cada_ellipse_getVectorTo(const shape::Ellipse *e,
                                      const shape::Vec2d &point, bool limited,
                                      double strictRange)
{
    Vec2d ret = Vec2d::invalid;

    double ang = e->getAngle();
    // double dDistance = RMAXDOUBLE;
    bool swap = false;
    bool majorSwap = false;

    Vec2d normalized = (point - e->getCenter()).rotate(-ang);

    // special case: point in line with major axis:
    if (fabs(normalized.getAngle()) < NS::AngleTolerance ||
        fabs(normalized.getAngle()) > 2 * M_PI - NS::AngleTolerance) {
        ret = Vec2d(e->getMajorRadius(), 0.0);
        // dDistance = ret.distanceTo(normalized);
    }

    else if (fabs(normalized.getAngle() - M_PI) < NS::AngleTolerance) {
        ret = Vec2d(-(e->getMajorRadius()), 0.0);
        // dDistance = ret.distanceTo(normalized);
    }
    else {
        double dU = normalized.x;
        double dV = normalized.y;
        double dA = e->getMajorRadius();
        double dB = e->getMinorRadius();
        double dEpsilon = 1.0e-8;
        // iteration maximum
        int iMax = 32;
        double rdX = 0.0;
        double rdY = 0.0;

        if (dA < dB) {
            double dum = dA;
            dA = dB;
            dB = dum;
            dum = dU;
            dU = dV;
            dV = dum;
            majorSwap = true;
        }

        if (dV < 0.0) {
            dV *= -1.0;
            swap = true;
        }

        // initial guess:
        double dT = dB * (dV - dB);

        // newton's method:
        int i;
        for (i = 0; i < iMax; i++) {
            double dTpASqr = dT + dA * dA;
            double dTpBSqr = dT + dB * dB;
            double dInvTpASqr = 1.0 / dTpASqr;
            double dInvTpBSqr = 1.0 / dTpBSqr;
            double dXDivA = dA * dU * dInvTpASqr;
            double dYDivB = dB * dV * dInvTpBSqr;
            double dXDivASqr = dXDivA * dXDivA;
            double dYDivBSqr = dYDivB * dYDivB;
            double dF = dXDivASqr + dYDivBSqr - 1.0;
            if (fabs(dF) < dEpsilon) {
                // f(t0) is very close to zero:
                rdX = dXDivA * dA;
                rdY = dYDivB * dB;
                break;
            }
            double dFDer =
                2.0 * (dXDivASqr * dInvTpASqr + dYDivBSqr * dInvTpBSqr);

            double dRatio = dF / dFDer;

            if (fabs(dRatio) < dEpsilon) {
                // t1-t0 is very close to zero:
                rdX = dXDivA * dA;
                rdY = dYDivB * dB;
                break;
            }
            dT += dRatio;
        }

        if (i == iMax) {
            ret = Vec2d::invalid;
        }
        else {
            ret = Vec2d(rdX, rdY);
        }
    }

    if (ret.isValid()) {
        if (swap) {
            ret.y *= -1.0;
        }
        if (majorSwap) {
            double dum = ret.x;
            ret.x = ret.y;
            ret.y = dum;
        }
        ret = (ret.rotate(ang) + e->getCenter());

        if (limited) {
            double a1 = e->getCenter().getAngleTo(e->getStartPoint());
            double a2 = e->getCenter().getAngleTo(e->getEndPoint());
            double a = e->getCenter().getAngleTo(ret);
            if (!Math::isAngleBetween(a, a1, a2, e->isReversed())) {
                ret = Vec2d::invalid;
            }
        }
    }

    return point - ret;
}

shape::Vec2d cada_xline_getVectorTo(const shape::XLine *x,
                                    const shape::Vec2d &point, bool limited,
                                    double strictRange)
{
    assert(x);
    std::unique_ptr<Line> l = ShapeFactory::instance()->createLine(
        x->getBasePoint(), x->getDirectionVector());

    return cada_line_getVectorTo(l.get(), point, limited, strictRange);
}

shape::Vec2d cada_ray_getVectorTo(const shape::Ray *r,
                                  const shape::Vec2d &point, bool limited,
                                  double strictRange)
{
    if (!limited) {
        return cada_xline_getVectorTo(static_cast<const shape::XLine *>(r),
                                      point, limited, strictRange);
    }
    else {
        Vec2d p = r->getClosestPointOnShape(point, false);
        if (fabs(Math::getAngleDifference180(
                r->getDirection1(), r->getStartPoint().getAngleTo(p))) < 0.1) {
            return point - p;
        }
        return Vec2d::invalid;
    }
}

} // namespace algorithm
} // namespace cada
