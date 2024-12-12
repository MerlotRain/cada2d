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
#include <cada2d/RXLine.h>
#include <cada2d/private/RShapePrivate.h>

RXLine::RXLine()
    : m_basePoint(RVector::invalid), m_directionVector(RVector::invalid)
{
}

RXLine::RXLine(const RLine &line)
    : m_basePoint(line.getStartPoint()),
      m_directionVector(line.getEndPoint() - line.getStartPoint())
{
}

RXLine::RXLine(const RVector &basePoint, const RVector &directionVector)
    : m_basePoint(basePoint), m_directionVector(directionVector)
{
}

RXLine::RXLine(const RVector &basePoint, double angle, double distance)
    : m_basePoint(basePoint),
      m_directionVector(RVector::createPolar(distance, angle))
{
}

RXLine::~RXLine()
{
}

RS::ShapeType RXLine::getShapeType() const
{
    return RS::XLine;
}

RLine RXLine::getLineShape() const
{
    return RLine(m_basePoint, m_basePoint + m_directionVector);
}

RXLine *RXLine::clone() const
{
    return new RXLine(*this);
}

bool RXLine::isDirected() const
{
    return true;
}

RBox RXLine::getBoundingBox() const
{
    return RBox(RVector::getMinimum(m_basePoint, getSecondPoint()),
                RVector::getMaximum(m_basePoint, getSecondPoint()));
}

double RXLine::getLength() const
{
    return RNANDOUBLE;
}

double RXLine::getAngle() const
{
    return m_directionVector.getAngle();
}

void RXLine::setAngle(double a)
{
    m_directionVector.setAngle(a);
}

void RXLine::setLength(double l)
{
}

double RXLine::getDirection1() const
{
    return m_directionVector.getAngle();
}

double RXLine::getDirection2() const
{
    return getSecondPoint().getAngleTo(m_basePoint);
}

RS::Side RXLine::getSideOfPoint(const RVector &point) const
{
    return getLineShape().getSideOfPoint(point);
}

RVector RXLine::getStartPoint() const
{
    return m_basePoint;
}

RVector RXLine::getEndPoint() const
{
    return getSecondPoint();
}

bool RXLine::trimStartPoint(const RVector &trimPoint, const RVector &clickPoint,
                            bool extend)
{
    return false;
}

bool RXLine::trimEndPoint(const RVector &trimPoint, const RVector &clickPoint,
                          bool extend)
{
    RVector tp = getClosestPointOnShape(trimPoint, false);
    if (!tp.isValid()) {
        return false;
    }
    m_basePoint = tp;
    m_directionVector = -m_directionVector;
    return true;
}

bool RXLine::trimStartPoint(double trimDist)
{
    return true;
}

bool RXLine::trimEndPoint(double trimDist)
{
    return RShape::trimEndPoint(trimDist);
}

RS::Ending RXLine::getTrimEnd(const RVector &trimPoint,
                              const RVector &clickPoint)
{
    return getLineShape().getTrimEnd(trimPoint, clickPoint);
}

double RXLine::getDistanceFromStart(const RVector &p) const
{
    double ret = m_basePoint.getDistanceTo(p);

    RVector p2 = getClosestPointOnShape(p, false);
    double angle = m_basePoint.getAngleTo(p2);
    if (RMath::isSameDirection(getAngle(), angle, M_PI / 2)) {
        return ret;
    }
    else {
        return -ret;
    }
}

RVector RXLine::getBasePoint() const
{
    return m_basePoint;
}

void RXLine::setBasePoint(const RVector &vector)
{
    m_basePoint = vector;
}

RVector RXLine::getSecondPoint() const
{
    return m_basePoint + m_directionVector;
}

void RXLine::setSecondPoint(const RVector &vector)
{
    m_directionVector = vector - m_basePoint;
}

RVector RXLine::getDirectionVector() const
{
    return m_directionVector;
}

void RXLine::setDirectionVector(const RVector &vector)
{
    m_directionVector = vector;
}

RVector RXLine::getMiddlePoint() const
{
    return RVector::invalid;
}

std::vector<RVector> RXLine::getEndPoints() const
{
    return std::vector<RVector>();
}

std::vector<RVector> RXLine::getMiddlePoints() const
{
    return std::vector<RVector>();
}

std::vector<RVector> RXLine::getCenterPoints() const
{
    return std::vector<RVector>();
}

std::vector<RVector> RXLine::getPointsWithDistanceToEnd(double distance,
                                                        int from) const
{
    return std::vector<RVector>();
}

std::vector<RVector> RXLine::getPointCloud(double segmentLength) const
{
    return std::vector<RVector>();
}

double RXLine::getAngleAt(double distance, RS::From from) const
{
    return getAngle();
}

RVector RXLine::getVectorTo(const RVector &point, bool limited,
                            double strictRange) const
{
    return getLineShape().getVectorTo(point, false, strictRange);
}

RLine RXLine::getClippedLine(const RBox &box) const
{
    RLine ret = getLineShape();

    RPolyline pl = box.getPolyline();

    std::vector<RVector> ips =
        RShapePrivate::getIntersectionPoints(getLineShape(), pl, false);
    std::vector<RVector> sol;
    for (int i = 0; i < ips.size(); i++) {
        if (pl.isOnShape(ips[i])) {
            RVector p = ips[i].getClosest(sol);
            if (p.equalsFuzzy(ips[i])) {
                continue;
            }
            sol.push_back(ips[i]);
        }
    }

    if (sol.size() == 2) {
        ret = RLine(sol[0], sol[1]);
        if (!RMath::isSameDirection(ret.getDirection1(), getDirection1(),
                                    1.0e-2)) {
            ret.reverse();
        }
    }

    return ret;
}

bool RXLine::move(const RVector &offset)
{
    if (!offset.isValid() || offset.getMagnitude() < RS::PointTolerance) {
        return false;
    }
    m_basePoint += offset;
    return true;
}

bool RXLine::rotate(double rotation, const RVector &center)
{
    if (fabs(rotation) < RS::AngleTolerance) {
        return false;
    }
    m_basePoint.rotate(rotation, center);
    m_directionVector.rotate(rotation);
    return true;
}

bool RXLine::scale(const RVector &scaleFactors, const RVector &center)
{
    m_basePoint.scale(scaleFactors, center);
    m_directionVector.scale(scaleFactors);
    return true;
}

bool RXLine::mirror(const RLine &axis)
{
    RVector sp = getSecondPoint();
    m_basePoint.mirror(axis.getStartPoint(), axis.getEndPoint());
    sp.mirror(axis.getStartPoint(), axis.getEndPoint());
    setSecondPoint(sp);
    return true;
}

std::vector<std::shared_ptr<RShape>>
RXLine::getOffsetShapes(double distance, int number, RS::Side side,
                        RS::JoinType join, const RVector &position)
{
    return RShapePrivate::getOffsetLines(*this, distance, number, side,
                                         position);
}

bool RXLine::reverse()
{
    RVector sp = getSecondPoint();
    RVector bp = m_basePoint;
    setBasePoint(sp);
    setSecondPoint(bp);
    return true;
}

bool RXLine::stretch(const RPolyline &area, const RVector &offset)
{
    return false;
}

std::vector<std::shared_ptr<RShape>>
RXLine::splitAt(const std::vector<RVector> &points) const
{
    if (points.size() == 0) {
        return RShape::splitAt(points);
    }

    std::vector<std::shared_ptr<RShape>> ret;

    std::vector<RVector> sortedPoints = RVector::getSortedByDistance(
        points, m_basePoint - m_directionVector * 1e9);

    ret.push_back(
        std::shared_ptr<RShape>(new RRay(sortedPoints[0], -m_directionVector)));

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