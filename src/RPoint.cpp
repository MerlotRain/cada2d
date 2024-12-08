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
#include <cmath>

#include <cada2d/RBox.h>
#include <cada2d/RPoint.h>
#include <cada2d/RLine.h>

/**
 * Creates a point object with an invalid position
 */
RPoint::RPoint()
{
}

// RPoint::RPoint(const RPoint& other) : RShape() {
//     this->position = other.position;
// }

RPoint::RPoint(double x, double y) : m_position(x, y)
{
}

/**
 * Creates a point object with the given position.
 *
 * \param position the position
 *
 */
RPoint::RPoint(const RVector &position) : m_position(position)
{
}

RPoint::~RPoint()
{
}

RBox RPoint::getBoundingBox() const
{
    return RBox(m_position, m_position);
}

double RPoint::getLength() const
{
    return 0.0;
}

std::vector<RVector> RPoint::getEndPoints() const
{
    std::vector<RVector> ret;
    ret.push_back(m_position);
    return ret;
}

std::vector<RVector> RPoint::getMiddlePoints() const
{
    std::vector<RVector> ret;
    ret.push_back(m_position);
    return ret;
}

std::vector<RVector> RPoint::getCenterPoints() const
{
    std::vector<RVector> ret;
    ret.push_back(m_position);
    return ret;
}

std::vector<RVector> RPoint::getPointsWithDistanceToEnd(double distance,
                                                        int from) const
{
    std::vector<RVector> ret;
    return ret;
}

std::vector<RVector> RPoint::getPointCloud(double segmentLength) const
{
    std::vector<RVector> ret;
    ret.push_back(getPosition());
    return ret;
}

double RPoint::getAngleAt(double distance, RS::From from) const
{
    return RNANDOUBLE;
}

RVector RPoint::getVectorTo(const RVector &point, bool limited,
                            double strictRange) const
{
    return point - m_position;
}

bool RPoint::move(const RVector &offset)
{
    if (!offset.isValid() || offset.getMagnitude() < RS::PointTolerance) {
        return false;
    }
    m_position += offset;
    return true;
}

bool RPoint::rotate(double rotation, const RVector &center)
{
    if (fabs(rotation) < RS::AngleTolerance) {
        return false;
    }
    m_position.rotate(rotation, center);
    return true;
}

bool RPoint::scale(const RVector &scaleFactors, const RVector &center)
{
    m_position.scale(scaleFactors, center);
    return true;
}

bool RPoint::mirror(const RLine &axis)
{
    m_position.mirror(axis.getStartPoint(), axis.getEndPoint());
    return true;
}

bool RPoint::flipHorizontal()
{
    m_position.flipHorizontal();
    return true;
}

bool RPoint::flipVertical()
{
    m_position.flipVertical();
    return true;
}