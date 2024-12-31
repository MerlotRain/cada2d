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

#include <cada2d/private/RShapePrivate.h>

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

std::vector<RVector> RShapePrivate::RShapePrivate::getIntersectionPointsLL(
    const RLine &line1, const RLine &line2, bool limited)
{
    return std::vector<RVector>();
}

std::vector<RVector> RShapePrivate::RShapePrivate::getIntersectionPointsLL(
    const RLine &line1, const RLine &line2, bool limited1, bool limited2)
{
    return std::vector<RVector>();
}

std::vector<RVector> RShapePrivate::RShapePrivate::getIntersectionPointsLA(
    const RLine &line1, const RArc &arc2, bool limited)
{
    return std::vector<RVector>();
}

std::vector<RVector> RShapePrivate::RShapePrivate::getIntersectionPointsLA(
    const RLine &line1, const RArc &arc2, bool limited1, bool limited2)
{
    return std::vector<RVector>();
}

std::vector<RVector>
RShapePrivate::getIntersectionPointsLC(const RLine &line1,
                                       const RCircle &circle2, bool limited)
{
    return std::vector<RVector>();
}

std::vector<RVector>
RShapePrivate::getIntersectionPointsLE(const RLine &line1,
                                       const REllipse &ellipse2, bool limited)
{
    return std::vector<RVector>();
}

std::vector<RVector> RShapePrivate::getIntersectionPointsLE(
    const RLine &line1, const REllipse &ellipse2, bool limited1, bool limited2)
{
    return std::vector<RVector>();
}

std::vector<RVector>
RShapePrivate::getIntersectionPointsLS(const RLine &line1,
                                       const RSpline &spline2, bool limited)
{
    return std::vector<RVector>();
}

std::vector<RVector> RShapePrivate::getIntersectionPointsLX(
    const RLine &line1, const RExplodable &explodable2, bool limited)
{
    return std::vector<RVector>();
}

std::vector<RVector> RShapePrivate::getIntersectionPointsAA(const RArc &arc1,
                                                            const RArc &arc2,
                                                            bool limited)
{
    return std::vector<RVector>();
}

std::vector<RVector>
RShapePrivate::getIntersectionPointsAC(const RArc &arc1, const RCircle &circle2,
                                       bool limited)
{
    return std::vector<RVector>();
}

std::vector<RVector>
RShapePrivate::getIntersectionPointsAE(const RArc &arc1,
                                       const REllipse &ellipse2, bool limited)
{
    return std::vector<RVector>();
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
    return std::vector<RVector>();
}

std::vector<RVector>
RShapePrivate::getIntersectionPointsCC(const RCircle &circle1,
                                       const RCircle &circle2)
{
    return std::vector<RVector>();
}

std::vector<RVector>
RShapePrivate::getIntersectionPointsCE(const RCircle &circle1,
                                       const REllipse &ellipse2)
{
    return std::vector<RVector>();
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
    return std::vector<RVector>();
}

std::vector<RVector>
RShapePrivate::getIntersectionPointsEE(const REllipse &ellipse1,
                                       const REllipse &ellipse2)
{
    return std::vector<RVector>();
}

std::vector<RVector>
RShapePrivate::getIntersectionPointsEE(const REllipse &ellipse1,
                                       const REllipse &ellipse2, bool limited)
{
    return std::vector<RVector>();
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
    return std::vector<RVector>();
}

std::vector<RVector> RShapePrivate::getIntersectionPointsSX(
    const RSpline &spline1, const RExplodable &explodable2, bool limited)
{
    return std::vector<RVector>();
}

std::vector<RVector> RShapePrivate::RShapePrivate::getIntersectionPointsSS(
    const RSpline &spline1, const RSpline &spline2, bool limited, bool same,
    double tolerance)
{
    return std::vector<RVector>();
}

std::vector<RVector> RShapePrivate::RShapePrivate::getIntersectionPointsXX(
    const RExplodable &explodable1, const RExplodable &explodable2,
    bool limited, bool same)
{
    return std::vector<RVector>();
}