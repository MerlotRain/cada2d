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

#ifndef CAD2D_PRIVATE_RSHAPEPRIVATE_H
#define CAD2D_PRIVATE_RSHAPEPRIVATE_H

#include <cada2d/RShape.h>
#include <vector>
#include <memory>
#include <opennurbs.h>

class RShapePrivate {
public:
    static std::vector<std::shared_ptr<RShape>>
    getReversedShapeList(const std::vector<std::shared_ptr<RShape>> &shapes);

    static std::shared_ptr<RShape>
    scaleArc(const RShape &shape, const RVector &scaleFactors,
             const RVector &center = RDEFAULT_RVECTOR);

    static std::vector<RVector> getIntersectionPoints(const RShape &shape1,
                                                      const RShape &shape2,
                                                      bool limited = true,
                                                      bool same = false,
                                                      bool force = false);

    static std::vector<std::shared_ptr<RShape>>
    getOffsetArcs(const RShape &shape, double distance, int number,
                  RS::Side side, const RVector &position = RVector::invalid);

    static ON_NurbsCurve convertShapeToNURBS(const RShape &shape);

private:
    static ON_NurbsCurve convertLineToNURBS(const RLine &line);
    static ON_NurbsCurve convertArcToNURBS(const RArc &arc);
    static ON_NurbsCurve convertEllipseToNURBS(const REllipse &ellipse);

    static std::vector<RVector> getIntersectionPointsLL(const RLine &line1,
                                                        const RLine &line2,
                                                        bool limited = true);
    static std::vector<RVector> getIntersectionPointsLL(const RLine &line1,
                                                        const RLine &line2,
                                                        bool limited1,
                                                        bool limited2);
    static std::vector<RVector> getIntersectionPointsLA(const RLine &line1,
                                                        const RArc &arc2,
                                                        bool limited = true);
    static std::vector<RVector> getIntersectionPointsLA(const RLine &line1,
                                                        const RArc &arc2,
                                                        bool limited1,
                                                        bool limited2);
    static std::vector<RVector> getIntersectionPointsLC(const RLine &line1,
                                                        const RCircle &circle2,
                                                        bool limited = true);
    static std::vector<RVector>
    getIntersectionPointsLE(const RLine &line1, const REllipse &ellipse2,
                            bool limited = true);
    static std::vector<RVector>
    getIntersectionPointsLE(const RLine &line1, const REllipse &ellipse2,
                            bool limited1, bool limited2);

    static std::vector<RVector> getIntersectionPointsLS(const RLine &line1,
                                                        const RSpline &spline2,
                                                        bool limited = true);
    static std::vector<RVector>
    getIntersectionPointsLX(const RLine &line1, const RExplodable &explodable2,
                            bool limited = true);

    static std::vector<RVector> getIntersectionPointsAA(const RArc &arc1,
                                                        const RArc &arc2,
                                                        bool limited = true);
    static std::vector<RVector> getIntersectionPointsAC(const RArc &arc1,
                                                        const RCircle &circle2,
                                                        bool limited = true);
    static std::vector<RVector>
    getIntersectionPointsAE(const RArc &arc1, const REllipse &ellipse2,
                            bool limited = true);
    static std::vector<RVector> getIntersectionPointsAS(const RArc &arc1,
                                                        const RSpline &spline2,
                                                        bool limited = true);
    static std::vector<RVector>
    getIntersectionPointsAX(const RArc &arc1, const RExplodable &explodable2,
                            bool limited = true);

    static std::vector<RVector> getIntersectionPointsCC(const RCircle &circle1,
                                                        const RCircle &circle2);
    static std::vector<RVector>
    getIntersectionPointsCE(const RCircle &circle1, const REllipse &ellipse2);
    static std::vector<RVector> getIntersectionPointsCS(const RCircle &circle1,
                                                        const RSpline &spline2,
                                                        bool limited = true);
    static std::vector<RVector>
    getIntersectionPointsCX(const RCircle &circle1,
                            const RExplodable &explodable2,
                            bool limited = true);

    static std::vector<RVector>
    getIntersectionPointsEE(const REllipse &ellipse1, const REllipse &ellipse2);
    static std::vector<RVector>
    getIntersectionPointsEE(const REllipse &ellipse1, const REllipse &ellipse2,
                            bool limited);
    static std::vector<RVector>
    getIntersectionPointsES(const REllipse &ellipse1, const RSpline &spline2,
                            bool limited = true);
    static std::vector<RVector>
    getIntersectionPointsEX(const REllipse &ellipse1,
                            const RExplodable &explodable2,
                            bool limited = true);

    static std::vector<RVector>
    getIntersectionPointsSX(const RSpline &spline1,
                            const RExplodable &explodable2, bool limited);

    static std::vector<RVector>
    getIntersectionPointsSS(const RSpline &spline1, const RSpline &spline2,
                            bool limited = true, bool same = false,
                            double tolerance = RS::PointTolerance);

    static std::vector<RVector>
    getIntersectionPointsXX(const RExplodable &explodable1,
                            const RExplodable &explodable2, bool limited = true,
                            bool same = false);
};

#endif // CAD2D_PRIVATE_RSHAPEPRIVATE_H
