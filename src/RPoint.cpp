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

#include <cmath>

#include <cada2d/RBox.h>
#include <cada2d/RPoint.h>
#include <cada2d/RLine.h>

RPoint::RPoint()
{
}

RPoint::RPoint(double x, double y) : m_position(x, y)
{
}

RPoint::RPoint(const RVector &position) : m_position(position)
{
}

RPoint::~RPoint()
{
}

RS::ShapeType RPoint::getShapeType() const
{
    return RS::Point;
}

RVector RPoint::getPosition() const
{
    return m_position;
}

void RPoint::setPosition(const RVector &p)
{
    m_position = p;
}

RPoint *RPoint::clone() const
{
    return new RPoint(*this);
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