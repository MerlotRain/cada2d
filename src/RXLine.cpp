/**
 * Copyright (c) 2011-2018 by Andrew Mustun. All rights reserved.
 *
 * This file is part of the QCAD project.
 *
 * QCAD is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * QCAD is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with QCAD.
 */
#include <cada2d/RBox.h>
#include <cada2d/RPolyline.h>
#include <cada2d/RRay.h>
#include <cada2d/RXLine.h>

/**
 * Creates an xline object with invalid base point and direction.
 */
RXLine::RXLine()
    : m_basePoint(RVector::invalid), m_directionVector(RVector::invalid)
{
}

RXLine::RXLine(const RLine &line)
    : m_basePoint(line.getStartPoint()),
      m_directionVector(line.getEndPoint() - line.getStartPoint())
{
}

/**
 * Creates an xline object with the given base point and direction.
 */
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
    return;
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
    RVector tp = getClosestPointOnShape(trimPoint, false);
    if (!tp.isValid()) {
        return false;
    }
    m_basePoint = tp;
    return true;
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
        RShape::getIntersectionPoints(getLineShape(), pl, false);
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