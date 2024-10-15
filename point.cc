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

using namespace cada;

Point::Point()
{
}

Point::Point(double x, double y) : position(x, y)
{
}

Point::Point(const Vec3d &position) : position(position)
{
}

Point::~Point()
{
}

BBox Point::getBoundingBox() const
{
    return BBox(position, position);
}

double Point::getLength() const
{
    return 0.0;
}

std::vector<Vec3d> Point::getEndPoints() const
{
    std::vector<Vec3d> ret;
    ret.push_back(position);
    return ret;
}

std::vector<Vec3d> Point::getMiddlePoints() const
{
    std::vector<Vec3d> ret;
    ret.push_back(position);
    return ret;
}

std::vector<Vec3d> Point::getCenterPoints() const
{
    std::vector<Vec3d> ret;
    ret.push_back(position);
    return ret;
}

std::vector<Vec3d> Point::getPointsWithDistanceToEnd(double distance,
                                                     int from) const
{
    // Q_UNUSED(distance)
    // Q_UNUSED(from)

    std::vector<Vec3d> ret;
    return ret;
}

std::vector<Vec3d> Point::getPointCloud(double segmentLength) const
{
    // Q_UNUSED(segmentLength)

    std::vector<Vec3d> ret;
    ret.push_back(getPosition());
    return ret;
}

double Point::getAngleAt(double distance, NS::From from) const
{
    // Q_UNUSED(distance)
    // Q_UNUSED(from)
    return std::numeric_limits<double>::quiet_NaN();
}

Vec3d Point::getVectorTo(const Vec3d &point, bool limited,
                         double strictRange) const
{
    // Q_UNUSED(limited)
    // Q_UNUSED(strictRange)

    return point - position;
}

bool Point::move(const Vec3d &offset)
{
    if (!offset.isValid() || offset.getMagnitude() < NS::PointTolerance) {
        return false;
    }
    position += offset;
    return true;
}

bool Point::rotate(double rotation, const Vec3d &center)
{
    if (fabs(rotation) < NS::AngleTolerance) {
        return false;
    }
    position.rotate(rotation, center);
    return true;
}

bool Point::scale(const Vec3d &scaleFactors, const Vec3d &center)
{
    position.scale(scaleFactors, center);
    return true;
}

bool Point::mirror(const Line &axis)
{
    position.mirror(axis);
    return true;
}

bool Point::flipHorizontal()
{
    position.flipHorizontal();
    return true;
}

bool Point::flipVertical()
{
    position.flipVertical();
    return true;
}