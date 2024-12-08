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

#include <cada2d/RLine.h>
#include <cada2d/RMath.h>
#include <sstream>
#include <iomanip>

RLine::RLine() : m_startPoint(RVector::invalid), m_endPoint(RVector::invalid)
{
}

RLine::RLine(double x1, double y1, double x2, double y2)
    : m_startPoint(x1, y1), m_endPoint(x2, y2)
{
}

RLine::RLine(const RVector &m_startPoint, const RVector &m_endPoint)
    : m_startPoint(m_startPoint), m_endPoint(m_endPoint)
{
}

RLine::RLine(const RVector &m_startPoint, double angle, double distance)
    : m_startPoint(m_startPoint)
{

    m_endPoint = m_startPoint + RVector::createPolar(distance, angle);
}

bool RLine::isValid() const
{
    return m_startPoint.isSane() && m_endPoint.isSane();
}

void RLine::setLength(double l, bool fromStart)
{
    if (fromStart) {
        m_endPoint = m_startPoint + RVector::createPolar(l, getAngle());
    }
    else {
        m_startPoint = m_endPoint - RVector::createPolar(l, getAngle());
    }
}

double RLine::getAngle() const
{
    return m_startPoint.getAngleTo(m_endPoint);
}

void RLine::setAngle(double a)
{
    m_endPoint = m_startPoint + RVector::createPolar(getLength(), a);
}

bool RLine::isParallel(const RLine &line) const
{
    double a = getAngle();
    double oa = line.getAngle();

    return RMath::isSameDirection(a, oa) ||
           RMath::isSameDirection(a, oa + M_PI);
}

bool RLine::isVertical(double tolerance) const
{
    return RMath::fuzzyCompare(m_startPoint.x, m_endPoint.x, tolerance);
}

bool RLine::isHorizontal(double tolerance) const
{
    return RMath::fuzzyCompare(m_startPoint.y, m_endPoint.y, tolerance);
}

RVector RLine::getStartPoint() const
{
    return m_startPoint;
}

void RLine::setStartPoint(const RVector &vector)
{
    m_startPoint = vector;
}

RVector RLine::getEndPoint() const
{
    return m_endPoint;
}

void RLine::setEndPoint(const RVector &vector)
{
    m_endPoint = vector;
}

RS::ShapeType RLine::getShapeType() const
{
    return RS::Line;
}

RVector RLine::getMiddlePoint() const
{
    return (m_startPoint + m_endPoint) / 2.0;
}

std::vector<RVector> RLine::getEndPoints() const
{
    std::vector<RVector> ret;
    ret.push_back(m_startPoint);
    ret.push_back(m_endPoint);
    return ret;
}

std::vector<RVector> RLine::getMiddlePoints() const
{
    std::vector<RVector> ret;
    ret.push_back(getMiddlePoint());
    return ret;
}

std::vector<RVector> RLine::getCenterPoints() const
{
    return getMiddlePoints();
}
