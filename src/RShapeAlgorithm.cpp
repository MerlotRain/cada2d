#include "RShapeAlgorithm.h"
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

#include <cada2d/RShapeAlgorithm.h>

extern "C" {
#include <apollonius.h>
}

std::vector<RVector>
RShapeAlgorithm::calculateEquidistantPointsOnLine(const RVector &v1,
                                                  const RVector &v2, size_t n)
{
    return std::vector<RVector>();
}

std::vector<RVector>
RShapeAlgorithm::calculateEquidistantDistributionPointsOnSurface(
    const RVector &v1, const RVector &v2, const RVector &v3, const RVector &v4,
    size_t col, size_t row)
{
    return std::vector<RVector>();
}

std::vector<RLine> RShapeAlgorithm::calculateAngleBisectorOfTwoLineSegments(
    const RLine &l1, const RLine &l2, const RVector &pos1, const RVector &pos2,
    double line_length, int line_number)
{
    return std::vector<RLine>();
}

std::vector<RLine>
RShapeAlgorithm::calculateCommonTangentBetweenTwoCircles(const RCircle &c1,
                                                         const RCircle &c2)
{
    return std::vector<RLine>();
}

std::vector<RLine>
RShapeAlgorithm::calculateOrthogonalTangentBetweenShapeAndLine(
    const RLine &line, const RShape &shape)
{
    return std::vector<RLine>();
}

std::vector<std::shared_ptr<RShape>> RShapeAlgorithm::autoSplit(
    const RVector &pos, const RShape &shape,
    const std::vector<std::shared_ptr<RShape>> &intersecting_shapes,
    bool extend)
{
    return std::vector<std::shared_ptr<RShape>>();
}

std::vector<std::shared_ptr<RShape>> RShapeAlgorithm::autoSplitManual(
    const RShape &shp, double cutDist1, double cutDist2, RVector cutPos1,
    RVector cutPos2, const RVector &position, bool extend)
{
    return std::vector<std::shared_ptr<RShape>>();
}

bool RShapeAlgorithm::breakOutGap(
    const RVector &pos, const RShape &shape,
    std::vector<std::shared_ptr<RShape>> &additional)
{
    return false;
}

std::vector<std::shared_ptr<RShape>>
RShapeAlgorithm::bevelShapes(const RShape &shp1, const RVector &clickPos1,
                             const RShape &shp2, const RVector &clickPos2,
                             bool trim, bool samePolyline, double distance1,
                             double distance2)
{
    return std::vector<std::shared_ptr<RShape>>();
}

std::vector<std::shared_ptr<RShape>>
RShapeAlgorithm::roundRhapes(const RShape &shp1, const RVector &pos1,
                             const RShape &shp2, const RVector &pos2, bool trim,
                             bool samepolyline, double radius,
                             const RVector &solutionPos)
{
    return std::vector<std::shared_ptr<RShape>>();
}

bool RShapeAlgorithm::lengthen(RShape &shape, const RVector &position,
                               bool trim_start, double amount)
{
    return false;
}

std::vector<RCircle> RShapeAlgorithm::apolloniusSolutions(const RShape &shape1,
                                                          const RShape &shape2,
                                                          const RShape &shape3)
{
    return std::vector<RCircle>();
}