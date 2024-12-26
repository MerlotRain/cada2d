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

#include <cada2d/RBox.h>
#include <cada2d/RPolyline.h>
#include <cada2d/RRay.h>

RRay::RRay() : RXLine()
{
}

RRay::RRay(const RLine &line) : RXLine(line)
{
}

RRay::RRay(const RVector &basePoint, const RVector &directionVector)
    : RXLine(basePoint, directionVector)
{
}

RRay::RRay(const RVector &basePoint, double angle, double distance)
    : RXLine(basePoint, angle, distance)
{
}

RRay::~RRay()
{
}

RS::ShapeType RRay::getShapeType() const
{
    return RS::Ray;
}

RRay *RRay::clone() const
{
    return new RRay(*this);
}

RVector RRay::getVectorTo(const RVector &point, bool limited,
                          double strictRange) const
{
    if (!limited) {
        return RXLine::getVectorTo(point, false, strictRange);
    }
    else {
        RVector p = RXLine::getClosestPointOnShape(point, false);
        if (fabs(RMath::getAngleDifference180(
                getDirection1(), getStartPoint().getAngleTo(p))) < 0.1) {
            return point - p;
        }
        return RVector::invalid;
    }
}

bool RRay::reverse()
{
    return false;
}

RLine RRay::getClippedLine(const RBox &box) const
{
    RLine ret = RXLine::getClippedLine(box);

    if (box.contains(getBasePoint())) {
        ret.setStartPoint(getBasePoint());
    }

    if (!RMath::isSameDirection(getDirection1(),
                                getBasePoint().getAngleTo(ret.getEndPoint()),
                                0.1)) {
        ret = getLineShape();
    }

    return ret;
}

bool RRay::trimEndPoint(const RVector &trimPoint, const RVector &clickPoint,
                        bool extend)
{
    RVector tp = getClosestPointOnShape(trimPoint, false);
    if (!tp.isValid()) {
        return false;
    }
    m_directionVector = tp - m_basePoint;
    return true;
}

std::vector<RVector> RRay::getPointsWithDistanceToEnd(double distance,
                                                      int from) const
{
    std::vector<RVector> ret;
    double a1 = getAngle();

    RVector dv;
    dv.setPolar(distance, a1);

    if (from & RS::FromStart) {
        ret.push_back(m_basePoint + dv);
    }

    return ret;
}

bool RRay::stretch(const RPolyline &area, const RVector &offset)
{
    bool ret = false;

    if (area.contains(m_basePoint, true)) {
        m_basePoint += offset;
        ret = true;
    }

    return ret;
}

std::vector<std::shared_ptr<RShape>>
RRay::splitAt(const std::vector<RVector> &points) const
{
    if (points.size() == 0) {
        return RShape::splitAt(points);
    }

    std::vector<std::shared_ptr<RShape>> ret;

    std::vector<RVector> sortedPoints =
        RVector::getSortedByDistance(points, m_basePoint);

    if (!m_basePoint.equalsFuzzy(sortedPoints[0])) {
        sortedPoints.insert(sortedPoints.begin(), m_basePoint);
    }

    for (int i = 0; i < sortedPoints.size() - 1; i++) {
        if (sortedPoints[i].equalsFuzzy(sortedPoints[i + 1])) {
            continue;
        }

        ret.push_back(std::shared_ptr<RShape>(
            new RLine(sortedPoints[i], sortedPoints[i + 1])));
    }

    ret.push_back(std::shared_ptr<RShape>(
        new RRay(sortedPoints[sortedPoints.size() - 1], m_directionVector)));

    return ret;
}