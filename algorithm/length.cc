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

#include "length.h"
#include <assert.h>
#include <cada_shape.h>
#include <cmath>
#include <limits>
#include <numeric>

using namespace cada::shape;

namespace cada {
namespace algorithm {

Length::Length(shape::Shape *shape) : mShape(shape)
{
}

double Length::getLength() const
{
    assert(mShape);
    switch (mShape->getShapeType()) {
    case NS::Point: {
        return 0.0;
    }
    case NS::Line: {
        auto line = dynamic_cast<Line *>(mShape);
        return line->getStartPoint().getDistanceTo(line->getEndPoint());
    }
    case NS::Arc: {
        auto arc = dynamic_cast<Arc *>(mShape);
        return std::fabs(arc->getAngleLength(false)) * arc->getRadius();
    }
    case NS::Circle: {
        auto circle = dynamic_cast<Circle *>(mShape);
        return 2 * circle->getRadius() * M_PI;
    }
    case NS::Ellipse: {
        return getEllipseLength();
    }
    case NS::XLine:
    case NS::Ray: {
        return std::numeric_limits<double>::quiet_NaN();
    }
    case NS::Polyline:
    case NS::BSpline:
        break;
    default:
        break;
    }
    return 0.0;
}

double Length::getLength(shape::Shape *shape)
{
    Length l(shape);
    return l.getLength();
}

double Length::getEllipseLength() const
{
    Ellipse *ellipse = dynamic_cast<Ellipse *>(mShape);
    assert(ellipse);

    double a1, a2;
    double a = ellipse->getMajorRadius();
    double b = ellipse->getMinorRadius();

    if (ellipse->isFullEllipse()) {
        a1 = 0.0;
        a2 = 2 * M_PI;

        if (Math::fuzzyCompare((a + b), 0.0)) {
            return 0.0;
        }
        double h = pow((a - b) / (a + b), 2);

        return M_PI * (a + b) *
               ((135168 - 85760 * h - 5568 * h * h + 3867 * h * h * h) /
                (135168 - 119552 * h + 22208 * h * h - 345 * h * h * h));
    }
    else {
        a1 = Math::getNormalizedAngle(ellipse->getStartParam());
        a2 = Math::getNormalizedAngle(ellipse->getEndParam());
    }

    if (ellipse->isReversed()) {
        double t = a1;
        a1 = a2;
        a2 = t;
    }

    if (Math::fuzzyCompare(a2, 0.0)) {
        a2 = 2 * M_PI;
    }

    if (fabs(a1 - a2) < NS::AngleTolerance) {
        return 0.0;
    }

    if (a1 < a2) {
        if (a1 < M_PI && a2 <= M_PI) {
            return getEllipseSimpsonLength(a, b, a1, a2);
        }
        if (a1 < M_PI && a2 > M_PI) {
            return getEllipseSimpsonLength(a, b, a1, M_PI) +
                   getEllipseSimpsonLength(a, b, M_PI, a2);
        }
        if (a1 >= M_PI && a2 > M_PI) {
            return getEllipseSimpsonLength(a, b, a1, a2);
        }
    }
    else {
        if (a1 > M_PI && a2 < M_PI) {
            return getEllipseSimpsonLength(a, b, a1, 2 * M_PI) +
                   getEllipseSimpsonLength(a, b, 0, a2);
        }
        if (a1 > M_PI && a2 > M_PI) {
            return getEllipseSimpsonLength(a, b, a1, 2 * M_PI) +
                   getEllipseSimpsonLength(a, b, 0, M_PI) +
                   getEllipseSimpsonLength(a, b, M_PI, a2);
        }
        if (a1 < M_PI && a2 < M_PI) {
            return getEllipseSimpsonLength(a, b, a1, M_PI) +
                   getEllipseSimpsonLength(a, b, M_PI, 2 * M_PI) +
                   getEllipseSimpsonLength(a, b, 0, a2);
        }
    }

    return std::numeric_limits<double>::quiet_NaN();
}

double Length::getEllipseSimpsonLength(double majorR, double minorR, double a1,
                                       double a2) const
{
    int interval = 20;
    double df = (a2 - a1) / interval;
    double sum = 0.0;
    double q = 1.0;

    for (int i = 0; i <= interval; ++i) {
        double y = sqrt(::pow(majorR * sin(a1 + i * df), 2) +
                        ::pow(minorR * cos(a1 + i * df), 2));
        if (i == 0 || i == interval) {
            q = 1.0;
        }
        else {
            if (i % 2 == 0) {
                q = 2.0;
            }
            else {
                q = 4.0;
            }
        }

        sum += q * y;
    }

    return (df / 3.0) * sum;
}

} // namespace algorithm
} // namespace cada