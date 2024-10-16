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

#include "cada_shape.h"

namespace cada {

Line::Line() : startPoint(Vec2d::invalid), endPoint(Vec2d::invalid)
{
}

Line::Line(double x1, double y1, double x2, double y2)
    : startPoint(x1, y1), endPoint(x2, y2)
{
}

Line::Line(const Vec2d &startPoint, const Vec2d &endPoint)
    : startPoint(startPoint), endPoint(endPoint)
{
}

Line::Line(const Vec2d &startPoint, double angle, double distance)
    : startPoint(startPoint)
{

    endPoint = startPoint + Vec2d::createPolar(distance, angle);
}

bool Line::isValid() const
{
    return startPoint.isSane() && endPoint.isSane();
}

double Line::getLength() const
{
    return startPoint.getDistanceTo(endPoint);
}

void Line::setLength(double l, bool fromStart)
{
    if (fromStart) {
        endPoint = startPoint + Vec2d::createPolar(l, getAngle());
    }
    else {
        startPoint = endPoint - Vec2d::createPolar(l, getAngle());
    }
}

double Line::getAngle() const
{
    return startPoint.getAngleTo(endPoint);
}

void Line::setAngle(double a)
{
    endPoint = startPoint + Vec2d::createPolar(getLength(), a);
}

bool Line::isParallel(const Line &line) const
{
    double a = getAngle();
    double oa = line.getAngle();

    return Math::isSameDirection(a, oa) || Math::isSameDirection(a, oa + M_PI);
}

bool Line::isVertical(double tolerance) const
{
    return Math::fuzzyCompare(startPoint.x, endPoint.x, tolerance);
}

bool Line::isHorizontal(double tolerance) const
{
    return Math::fuzzyCompare(startPoint.y, endPoint.y, tolerance);
}

Vec2d Line::getStartPoint() const
{
    return startPoint;
}

void Line::setStartPoint(const Vec2d &vector)
{
    startPoint = vector;
}

Vec2d Line::getEndPoint() const
{
    return endPoint;
}

void Line::setEndPoint(const Vec2d &vector)
{
    endPoint = vector;
}

Vec2d Line::getMiddlePoint() const
{
    return (startPoint + endPoint) / 2.0;
}

std::vector<Vec2d> Line::getEndPoints() const
{
    std::vector<Vec2d> ret;
    ret.push_back(startPoint);
    ret.push_back(endPoint);
    return ret;
}

std::vector<Vec2d> Line::getMiddlePoints() const
{
    std::vector<Vec2d> ret;
    ret.push_back(getMiddlePoint());
    return ret;
}

std::vector<Vec2d> Line::getCenterPoints() const
{
    return getMiddlePoints();
}

void Line::clipTo(const BBox &box)
{
    double x1 = startPoint.x;
    double y1 = startPoint.y;
    double x2 = endPoint.x;
    double y2 = endPoint.y;
    double xmin = box.getMinimum().x;
    double ymin = box.getMinimum().y;
    double xmax = box.getMaximum().x;
    double ymax = box.getMaximum().y;

    double deltaX, deltaY, p, q;
    double u1 = 0.0, u2 = 1.0;
    double r;

    deltaX = (x2 - x1);
    deltaY = (y2 - y1);

    // left edge, right edge, bottom edge and top edge checking
    double pPart[] = {-1 * deltaX, deltaX, -1 * deltaY, deltaY};
    double qPart[] = {x1 - xmin, xmax - x1, y1 - ymin, ymax - y1};

    bool accept = true;

    for (int i = 0; i < 4; i++) {
        p = pPart[i];
        q = qPart[i];

        if (p == 0 && q < 0) {
            accept = false;
            break;
        }

        r = q / p;

        if (p < 0) {
            u1 = std::max(u1, r);
        }

        if (p > 0) {
            u2 = std::min(u2, r);
        }

        if (u1 > u2) {
            accept = false;
            break;
        }
    }

    if (accept) {
        if (u2 < 1) {
            x2 = x1 + u2 * deltaX;
            y2 = y1 + u2 * deltaY;
        }
        if (u1 > 0) {
            x1 = x1 + u1 * deltaX;
            y1 = y1 + u1 * deltaY;
        }

        startPoint = Vec2d(x1, y1);
        endPoint = Vec2d(x2, y2);
    }
    else {
        startPoint = Vec2d::invalid;
        endPoint = Vec2d::invalid;
    }
}

} // namespace cada