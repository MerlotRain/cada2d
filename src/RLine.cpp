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
#include <cada2d/RPolyline.h>
#include <iomanip>
#include <sstream>

RLine::RLine() : m_startPoint(RVector::invalid), m_endPoint(RVector::invalid) {}

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

bool RLine::isDirected() const { return true; }

bool RLine::isValid() const
{
    return m_startPoint.isSane() && m_endPoint.isSane();
}

RBox RLine::getBoundingBox() const
{
    return RBox(RVector::getMinimum(m_startPoint, m_endPoint),
                RVector::getMaximum(m_startPoint, m_endPoint));
}

void RLine::setLength(double l, bool fromStart)
{
    if (fromStart)
    {
        m_endPoint = m_startPoint + RVector::createPolar(l, getAngle());
    }
    else { m_startPoint = m_endPoint - RVector::createPolar(l, getAngle()); }
}

double RLine::getLength() const
{
    return m_startPoint.getDistanceTo(m_endPoint);
}

double RLine::getAngle() const { return m_startPoint.getAngleTo(m_endPoint); }

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

bool RLine::isCollinear(const RLine &line) const
{
    auto checkProc = [](const RVector &v1, const RVector &v2,
                        const RVector &v3) -> double {
        double a = v1.getDistanceTo(v2);
        double b = v2.getDistanceTo(v3);
        double c = v3.getDistanceTo(v1);
        if (RMath::fuzzyCompare(a, 0.0) || RMath::fuzzyCompare(b, 0.0) ||
            RMath::fuzzyCompare(c, 0.0))
        {
            return 0.0;
        }
        double s = (a + b + c) / 2;
        double rootTerm = fabs(s * (s - a) * (s - b) * (s - c));
        return sqrt(rootTerm);
    };

    // two points are collinear with a third point if the area of the triangle
    // formed by these three points is zero:
    if (checkProc(m_startPoint, m_endPoint, line.getStartPoint()) >
        RS::PointTolerance)
    {
        return false;
    }

    if (checkProc(m_startPoint, m_endPoint, line.getEndPoint()) >
        RS::PointTolerance)
    {
        return false;
    }

    return true;
}

double RLine::getDirection1() const
{
    return m_startPoint.getAngleTo(m_endPoint);
}

double RLine::getDirection2() const
{
    return m_endPoint.getAngleTo(m_startPoint);
}

RS::Side RLine::getSideOfPoint(const RVector &point) const
{
    double entityAngle = getAngle();
    double angleToCoord = m_startPoint.getAngleTo(point);
    double angleDiff = RMath::getAngleDifference(entityAngle, angleToCoord);

    if (angleDiff < M_PI) { return RS::LeftHand; }
    else { return RS::RightHand; }
}

void RLine::clipToXY(const RBox &box) {}

bool RLine::move(const RVector &offset)
{
    if (!offset.isValid() || offset.getMagnitude() < RS::PointTolerance)
    {
        return false;
    }
    m_startPoint += offset;
    m_endPoint += offset;
    return true;
}

bool RLine::scale(const RVector &scaleFactors, const RVector &center)
{
    m_startPoint.scale(scaleFactors, center);
    m_endPoint.scale(scaleFactors, center);
    return true;
}

bool RLine::flipHorizontal()
{
    m_startPoint.flipHorizontal();
    m_endPoint.flipHorizontal();
    return true;
}

bool RLine::flipVertical()
{
    m_startPoint.flipVertical();
    m_endPoint.flipVertical();
    return true;
}

bool RLine::stretch(const RPolyline &area, const RVector &offset)
{
    bool ret = false;

    if (area.contains(m_startPoint, true))
    {
        m_startPoint += offset;
        ret = true;
    }
    if (area.contains(m_endPoint, true))
    {
        m_endPoint += offset;
        ret = true;
    }

    return ret;
}

bool RLine::moveTo(const RVector &dest) { return false; }

bool RLine::trimStartPoint(const RVector &trimPoint, const RVector &clickPoint,
                           bool extend)
{
    return false;
}

bool RLine::trimEndPoint(const RVector &trimPoint, const RVector &clickPoint,
                         bool extend)
{
    return false;
}

bool RLine::trimStartPoint(double trimDist) { return false; }

double RLine::getDistanceFromStart(const RVector &p) const { return 0.0; }

std::vector<std::shared_ptr<RShape>>
RLine::splitAt(const std::vector<RVector> &points) const
{
    return std::vector<std::shared_ptr<RShape>>();
}

std::vector<std::shared_ptr<RShape>>
RLine::getOffsetShapes(double distance, int number, RS::Side side,
                       RS::JoinType join, const RVector &position)
{
    return std::vector<std::shared_ptr<RShape>>();
}

bool RLine::trimEndPoint(double trimDist) { return false; }

RS::Ending RLine::getTrimEnd(const RVector &trimPoint,
                             const RVector &clickPoint)
{
    return RS::Ending();
}

bool RLine::reverse() { return false; }

bool RLine::mirror(const RLine &axis)
{
    m_startPoint.mirror(axis.getStartPoint(), axis.getEndPoint());
    m_endPoint.mirror(axis.getStartPoint(), axis.getEndPoint());
    return true;
}

bool RLine::rotate(double rotation, const RVector &center)
{
    if (fabs(rotation) < RS::AngleTolerance) { return false; }
    m_startPoint.rotate(rotation, center);
    m_endPoint.rotate(rotation, center);
    return true;
}

bool RLine::isVertical(double tolerance) const
{
    return RMath::fuzzyCompare(m_startPoint.x, m_endPoint.x, tolerance);
}

bool RLine::isHorizontal(double tolerance) const
{
    return RMath::fuzzyCompare(m_startPoint.y, m_endPoint.y, tolerance);
}

double RLine::getAngleAt(double distance, RS::From from) const { return 0.0; }

RVector RLine::getVectorTo(const RVector &point, bool limited,
                           double strictRange) const
{
    return RVector();
}

RVector RLine::getStartPoint() const { return m_startPoint; }

void RLine::setStartPoint(const RVector &vector) { m_startPoint = vector; }

RVector RLine::getEndPoint() const { return m_endPoint; }

void RLine::setEndPoint(const RVector &vector) { m_endPoint = vector; }

RS::ShapeType RLine::getShapeType() const { return RS::Line; }

std::shared_ptr<RShape> RLine::clone() const
{
    return std::shared_ptr<RShape>(new RLine(*this));
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

std::vector<RVector> RLine::getPointCloud(double segmentLength) const
{
    return std::vector<RVector>();
}

std::vector<RVector> RLine::getCenterPoints() const
{
    return getMiddlePoints();
}

std::vector<RVector> RLine::getPointsWithDistanceToEnd(double distance,
                                                       int from) const
{
    return std::vector<RVector>();
}
