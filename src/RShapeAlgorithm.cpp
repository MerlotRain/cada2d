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
#include <cada2d/RArc.h>
#include <cada2d/RCircle.h>
#include <cada2d/REllipse.h>
#include <cada2d/RLine.h>
#include <cada2d/RPolyline.h>
#include <cada2d/RRay.h>
#include <cada2d/RShapeAlgorithm.h>
#include <cada2d/RSpline.h>
#include <cada2d/RXLine.h>
#include <cada2d/private/RShapePrivate.h>

extern "C" {
#include <apollonius.h>
}

std::vector<RVector>
RShapeAlgorithm::calculateEquidistantPointsOnLine(const RVector &v1,
                                                  const RVector &v2, size_t n)
{
    std::vector<RVector> rets;
    assert(n > 2);

    double dx = v2.x - v1.x;
    double dy = v2.y - v1.y;

    double step_x = dx / (n - 1);
    double step_y = dy / (n - 1);

    for (size_t i = 0; i < n; ++i)
    {
        rets.emplace_back(RVector(v1.x + step_x * i, v1.y + step_y * i));
    }
    return rets;
}

std::vector<RVector>
RShapeAlgorithm::calculateEquidistantDistributionPointsOnSurface(
        const RVector &v1, const RVector &v2, const RVector &v3,
        const RVector &v4, size_t col, size_t row)
{
    assert(col > 2);
    assert(row > 2);

    std::vector<RVector> line1_pts =
            calculateEquidistantPointsOnLine(v1, v4, row);
    std::vector<RVector> line2_pts =
            calculateEquidistantPointsOnLine(v2, v3, row);

    assert(line1_pts.size() != 0);
    assert(line2_pts.size() != 0);
    assert(line1_pts.size() == line2_pts.size());

    std::vector<RVector> rets;
    for (size_t i = 0; i < line1_pts.size(); ++i)
    {
        auto &p1 = line1_pts.at(i);
        auto &p2 = line2_pts.at(i);
        auto &&pps = calculateEquidistantPointsOnLine(p1, p2, col);
        rets.insert(rets.end(), pps.begin(), pps.end());
    }
    return rets;
}

std::vector<RLine> RShapeAlgorithm::calculateAngleBisectorOfTwoLineSegments(
        const RLine &l1, const RLine &l2, const RVector &pos1,
        const RVector &pos2, double line_length, int line_number)
{
    assert(line_number > 0);
    assert(line_length > 0);

    auto ips = l1.getIntersectionPoints(l2, false);
    if (ips.empty()) { return std::vector<RLine>(); }

    std::vector<RLine> rets;

    RVector ip = ips[0];

    double angle1 = ip.getAngleTo(l1.getClosestPointOnShape(pos1));
    double angle2 = ip.getAngleTo(l2.getClosestPointOnShape(pos2));
    double angleDiff = RMath::getAngleDifference(angle1, angle2);
    if (angleDiff > M_PI) { angleDiff = angleDiff - 2 * M_PI; }

    for (int i = 0; i < line_number; ++i)
    {
        double angle = angle1 + (angleDiff / (line_number + 1) * i);
        RVector vec;
        vec.setPolar(line_length, angle);
        rets.push_back(RLine(ip, ip + vec));
    }
    return rets;
}

std::vector<RLine>
RShapeAlgorithm::calculateCommonTangentBetweenTwoCircles(const RCircle &c1,
                                                         const RCircle &c2)
{

    RVector offs1, offs2;

    RVector cc1 = c1.getCenter();
    RVector cc2 = c2.getCenter();
    double cr1 = c1.getRadius();
    double cr2 = c2.getRadius();

    double angle1 = cc1.getAngleTo(cc2);
    double dist1 = cc1.getDistanceTo(cc2);
    if (dist1 < 1.0e-6) { return std::vector<RLine>(); }

    std::vector<RLine> tangents;

    // outer tangents:
    double dist2 = cr2 - cr1;
    if (dist1 > dist2)
    {
        double angle2 = asin(dist2 / dist1);
        double angt1 = angle1 + angle2 + M_PI / 2.0;
        double angt2 = angle1 - angle2 - M_PI / 2.0;
        offs1 = RVector();
        offs2 = RVector();

        offs1.setPolar(cr1, angt1);
        offs2.setPolar(cr2, angt1);

        tangents.emplace_back(RLine(cc1 + offs1, cc2 + offs2));

        offs1.setPolar(cr1, angt2);
        offs2.setPolar(cr2, angt2);

        tangents.emplace_back(RLine(cc1 - offs1, cc2 + offs2));
    }

    // inner tangents:
    double dist3 = cr2 + cr1;
    if (dist1 > dist3)
    {
        double angle3 = asin(dist3 / dist1);
        double angt3 = angle1 + angle3 + M_PI / 2.0;
        double angt4 = angle1 - angle3 - M_PI / 2.0;
        offs1 = RVector();
        offs2 = RVector();

        offs1.setPolar(cr1, angt3);
        offs2.setPolar(cr2, angt3);

        tangents.emplace_back(RLine(cc1 - offs1, cc2 + offs2));

        offs1.setPolar(cr1, angt4);
        offs2.setPolar(cr2, angt4);

        tangents.emplace_back(RLine(cc1 - offs1, cc2 + offs2));
    }

    return tangents;
}

std::vector<RLine>
RShapeAlgorithm::calculateOrthogonalTangentBetweenShapeAndLine(
        const RLine &line, const RShape &shape)
{
    std::vector<RLine> ret;

    RLine auxLine1, auxLine2;
    std::vector<RVector> ips, ips1, ips2;

    double lineAngle = line.getAngle();

    if (RShapePrivate::isCircleShape(shape) || RShapePrivate::isArcShape(shape))
    {

        // line parallel to line through center of circle:
        auxLine1 = RLine(shape.getCenterPoints().at(0), lineAngle, 100.0);

        // intersections of parallel with circle:
        ips1 = shape.getIntersectionPoints(auxLine1, false);
        for (size_t i = 0; i < ips1.size(); i++)
        {
            // candidate:
            auxLine2 = RLine(ips1[i], lineAngle + M_PI / 2, 100.0);
            ips2 = line.getIntersectionPoints(auxLine2, false);
            if (ips2.size() == 1)
            {
                ret.emplace_back(std::move(RLine(ips1[i], ips2[0])));
            }
        }
    }
    else if (RShapePrivate::isEllipseShape(shape))
    {
        const REllipse *e = dynamic_cast<const REllipse *>(&shape);
        RVector center = e->getCenter();
        // circle around ellipse:
        auto auxCircle = RCircle(center, e->getMajorRadius());

        std::vector<RVector> foci = e->getFoci();
        auxLine1 = RLine(foci[0], lineAngle, 100.0);
        auxLine2 = RLine(foci[1], lineAngle, 100.0);

        ips1 = auxLine1.getIntersectionPoints(auxCircle, false);
        ips2 = auxLine2.getIntersectionPoints(auxCircle, false);
        RVector pointOfContact1 = RVector::invalid;
        RVector pointOfContact2 = RVector::invalid;

        if (ips1.size() >= 1 && ips2.size() >= 1)
        {
            if (ips1[0].equalsFuzzy(ips2[0])) { pointOfContact1 = ips1[0]; }
            else
            {
                auxLine1 = RLine(ips1[0], ips2[0]);
                ips = shape.getIntersectionPoints(auxLine1, false);
                if (ips.size() >= 1) { pointOfContact1 = ips[0]; }
            }
        }

        if (ips1.size() >= 2 && ips2.size() >= 2)
        {
            if (ips1[1].equalsFuzzy(ips2[1])) { pointOfContact2 = ips1[1]; }
            else
            {
                auxLine2 = RLine(ips1[1], ips2[1]);
                ips = shape.getIntersectionPoints(auxLine2, false);
                if (ips.size() >= 1) { pointOfContact2 = ips[0]; }
            }
        }

        if (pointOfContact1.isValid())
        {
            RVector pointOnLine1 =
                    line.getClosestPointOnShape(pointOfContact1, false);
            ret.emplace_back(std::move(RLine(pointOfContact1, pointOnLine1)));
        }
        if (pointOfContact1.isValid())
        {
            RVector pointOnLine2 =
                    line.getClosestPointOnShape(pointOfContact2, false);
            ret.emplace_back(std::move(RLine(pointOfContact2, pointOnLine2)));
        }
    }

    return ret;
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

std::vector<std::shared_ptr<RShape>>
RShapeAlgorithm::trim(const RShape &trimShape, const RVector &trimClickPos,
                      const RShape &limitingShape,
                      const RVector &limitingClickPos, bool trimBoth,
                      bool samePolyline)
{
    int i1 = 0;
    std::shared_ptr<RShape> segment;
    std::shared_ptr<RShape> trimShapeSimple = trimShape.clone();
    if (RShapePrivate::isPolylineShape(trimShape))
    {
        const RPolyline &polyline = dynamic_cast<const RPolyline &>(trimShape);
        i1 = polyline.getClosestSegment(trimClickPos);
        if (i1 < 0) { return std::vector<std::shared_ptr<RShape>>(); }
        segment = polyline.getSegmentAt(i1);
        if (!segment) { return std::vector<std::shared_ptr<RShape>>(); }
        trimShapeSimple = segment;
    }

    int i2 = 0;
    std::shared_ptr<RShape> limitingShapeSimple = limitingShape.clone();
    if (RShapePrivate::isPolylineShape(limitingShape))
    {
        const RPolyline &polyline =
                dynamic_cast<const RPolyline &>(limitingShape);
        i2 = polyline.getClosestSegment(limitingClickPos);
        if (i2 < 0) { return std::vector<std::shared_ptr<RShape>>(); }
        segment = polyline.getSegmentAt(i2);
        if (!segment) { return std::vector<std::shared_ptr<RShape>>(); }
        limitingShapeSimple = segment;
    }

    // possible trim points:
    std::vector<RVector> sol =
            trimShapeSimple->getIntersectionPoints(*limitingShapeSimple, false);
    if (sol.empty())
    {
        // check if two lines are in line and replace with one line
        if (RShapePrivate::isLineShape(trimShape) &&
            RShapePrivate::isLineShape(limitingShape))
        {
            const RLine &trimLine = dynamic_cast<const RLine &>(trimShape);
            const RLine &limitingLine =
                    dynamic_cast<const RLine &>(limitingShape);

            if (trimLine.isCollinear(limitingLine))
            {
                std::vector<RVector> pts;
                pts.push_back(trimLine.getStartPoint());
                pts.push_back(trimLine.getEndPoint());
                pts.push_back(limitingLine.getStartPoint());
                pts.push_back(limitingLine.getEndPoint());
                RVector minP = RVector::getMinimum(pts);
                pts = RVector::getSortedByDistance(pts, minP);
                RLine newTrimmedLine(pts[1], pts[3]);

                return {std::shared_ptr<RShape>(newTrimmedLine.clone())};
            }
        }

        return std::vector<std::shared_ptr<RShape>>();
    }

    std::shared_ptr<RShape> trimmedTrimShape;
    std::shared_ptr<RShape> trimmedLimitingShape;
    RVector c;
    double r, am, a1, a2;
    RVector mp;

    if (RShapePrivate::isCircleShape(trimShape))
    {
        const RCircle &circle = dynamic_cast<const RCircle &>(trimShape);
        // convert circle to trimmable arc:
        c = circle.getCenter();
        r = circle.getRadius();
        am = c.getAngleTo(trimClickPos);
        a1 = RMath::getNormalizedAngle(am - M_PI / 2);
        a2 = RMath::getNormalizedAngle(am + M_PI / 2);
        trimmedTrimShape =
                std::shared_ptr<RShape>(new RArc(c, r, a1, a2, false));
    }
    else if (RShapePrivate::isFullEllipseShape(trimShape))
    {
        const REllipse &ellipse = dynamic_cast<const REllipse &>(trimShape);
        c = ellipse.getCenter();
        mp = ellipse.getMajorPoint();
        r = ellipse.getRatio();
        am = ellipse.getParamTo(trimClickPos);
        a1 = RMath::getNormalizedAngle(am - M_PI / 2);
        a2 = RMath::getNormalizedAngle(am + M_PI / 2);
        trimmedTrimShape =
                std::shared_ptr<RShape>(new REllipse(c, mp, r, a1, a2, false));
    }
    else
    {
        if (samePolyline) { trimmedTrimShape = trimShapeSimple->clone(); }
        else { trimmedTrimShape = trimShape.clone(); }
    }

    if (trimBoth)
    {
        if (RShapePrivate::isCircleShape(limitingShape))
        {
            const RCircle &circle =
                    dynamic_cast<const RCircle &>(limitingShape);
            // convert circle to trimmable arc:
            c = circle.getCenter();
            r = circle.getRadius();
            am = c.getAngleTo(trimClickPos);
            a1 = RMath::getNormalizedAngle(am - M_PI / 2);
            a2 = RMath::getNormalizedAngle(am + M_PI / 2);
            trimmedLimitingShape =
                    std::shared_ptr<RShape>(new RArc(c, r, a1, a2, false));
        }
        else
        {
            if (samePolyline)
            {
                trimmedLimitingShape = limitingShapeSimple->clone();
            }
            else { trimmedLimitingShape = limitingShape.clone(); }
        }
    }

    // find trim (intersection) point:
    bool isIdx;
    if (trimBoth || RShapePrivate::isEllipseShape(trimShape))
    {
        isIdx = trimClickPos.getClosestIndex(sol);
    }
    else { isIdx = limitingClickPos.getClosestIndex(sol); }
    RVector is = sol[isIdx];

    RVector is2;
    if (sol.size() == 1 || isIdx != 0) { is2 = sol[0]; }
    else { is2 = sol[1]; }

    // trim trim entity:
    RS::Ending ending1 = trimmedTrimShape->getTrimEnd(is, trimClickPos);

    switch (ending1)
    {
        case RS::EndingStart:
            trimmedTrimShape->trimStartPoint(is, trimClickPos);
            if (RShapePrivate::isCircleShape(trimShape) ||
                RShapePrivate::isFullEllipseShape(trimShape))
            {
                trimmedTrimShape->trimEndPoint(is2, trimClickPos);
            }
            break;
        case RS::EndingEnd:
            trimmedTrimShape->trimEndPoint(is, trimClickPos);
            if (RShapePrivate::isCircleShape(trimShape) ||
                RShapePrivate::isFullEllipseShape(trimShape))
            {
                trimmedTrimShape->trimStartPoint(is2, trimClickPos);
            }
            break;
        default:
            break;
    }

    if (RShapePrivate::isXLineShape(*trimmedTrimShape))
    {
        std::shared_ptr<const RXLine> xline =
                std::dynamic_pointer_cast<const RXLine>(trimmedTrimShape);
        trimmedTrimShape = std::shared_ptr<RShape>(
                new RRay(xline->getBasePoint(), xline->getDirectionVector()));
    }
    else if (RShapePrivate::isRayShape(trimShape) && ending1 == RS::EndingEnd)
    {
        std::shared_ptr<RRay> ray =
                std::dynamic_pointer_cast<RRay>(trimmedTrimShape);
        trimmedTrimShape = std::shared_ptr<RShape>(
                new RLine(ray->getBasePoint(), ray->getSecondPoint()));
    }

    // trim limiting shape if possible (not possible for splines):
    RS::Ending ending2 = RS::EndingNone;
    if (trimBoth && trimmedLimitingShape)
    {
        ending2 = trimmedLimitingShape->getTrimEnd(is, limitingClickPos);

        switch (ending2)
        {
            case RS::EndingStart:
                trimmedLimitingShape->trimStartPoint(is, limitingClickPos);
                if (RShapePrivate::isCircleShape(limitingShape) ||
                    RShapePrivate::isFullEllipseShape(limitingShape))
                {
                    trimmedLimitingShape->trimEndPoint(is2, limitingClickPos);
                }
                break;
            case RS::EndingEnd:
                trimmedLimitingShape->trimEndPoint(is, limitingClickPos);
                if (RShapePrivate::isCircleShape(limitingShape) ||
                    RShapePrivate::isFullEllipseShape(limitingShape))
                {
                    trimmedLimitingShape->trimStartPoint(is2, limitingClickPos);
                }
                break;
            default:
                break;
        }

        if (RShapePrivate::isXLineShape(*trimmedLimitingShape))
        {
            std::shared_ptr<RXLine> xline =
                    std::dynamic_pointer_cast<RXLine>(trimmedLimitingShape);
            trimmedLimitingShape = std::shared_ptr<RShape>(new RRay(
                    xline->getBasePoint(), xline->getDirectionVector()));
        }
        else if (RShapePrivate::isRayShape(*trimmedLimitingShape) &&
                 ending2 == RS::EndingEnd)
        {
            std::shared_ptr<RRay> ray =
                    std::dynamic_pointer_cast<RRay>(trimmedLimitingShape);
            trimmedLimitingShape = std::shared_ptr<RShape>(
                    new RLine(ray->getBasePoint(), ray->getSecondPoint()));
        }
    }

    std::vector<std::shared_ptr<RShape>> ret;

    if (samePolyline && RShapePrivate::isPolylineShape(trimShape))
    {
        const RPolyline &polyline = dynamic_cast<const RPolyline &>(trimShape);
        RPolyline pl = polyline.modifyPolylineCorner(*trimmedTrimShape, ending1,
                                                     i1, *trimmedLimitingShape,
                                                     ending2, i2);
        ret.push_back(std::shared_ptr<RShape>(pl.clone()));
    }
    else
    {
        ret.push_back(trimmedTrimShape);
        ret.push_back(trimmedLimitingShape);
    }

    return ret;
}
