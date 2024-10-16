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

Triangle::Triangle()
{
}

Triangle::Triangle(const Vec2d &p1, const Vec2d &p2, const Vec2d &p3)
{
    corner[0] = p1;
    corner[1] = p2;
    corner[2] = p3;
}

Triangle::~Triangle()
{
}

Polyline Triangle::getPolyline() const
{
    std::vector<Vec2d> vertices;
    vertices << corner[0];
    vertices << corner[1];
    vertices << corner[2];
    return Polyline(vertices, true);
}

NS::Orientation Triangle::getOrientation() const
{
    double val = (corner[1].y - corner[0].y) * (corner[2].x - corner[1].x) -
                 (corner[1].x - corner[0].x) * (corner[2].y - corner[1].y);

    if (val > 0.0) {
        return NS::CW;
    }
    else {
        return NS::CCW;
    }
}

bool Triangle::reverse()
{
    Vec2d dummy = corner[0];
    corner[0] = corner[2];
    corner[2] = dummy;
    return true;
}

Triangle Triangle::createArrow(const Vec2d &position, double direction,
                               double arrowSize)
{
    double cosv1, sinv1, cosv2, sinv2;
    double arrowSide = arrowSize / cos(0.165);

    cosv1 = cos(direction + 0.165) * arrowSide;
    sinv1 = sin(direction + 0.165) * arrowSide;
    cosv2 = cos(direction - 0.165) * arrowSide;
    sinv2 = sin(direction - 0.165) * arrowSide;

    Vec2d p1(position.x - cosv1, position.y - sinv1);
    Vec2d p2(position.x - cosv2, position.y - sinv2);

    return Triangle(position, p1, p2);
}

BBox Triangle::getBoundingBox() const
{
    return BBox(
        Vec2d::getMinimum(Vec2d::getMinimum(corner[0], corner[1]), corner[2]),
        Vec2d::getMaximum(Vec2d::getMaximum(corner[0], corner[1]), corner[2]));
}

double Triangle::getLength() const
{
    return corner[0].getDistanceTo(corner[1]) +
           corner[1].getDistanceTo(corner[2]) +
           corner[2].getDistanceTo(corner[0]);
}

double Triangle::getArea() const
{
    double a = corner[0].getDistanceTo(corner[1]);
    double b = corner[1].getDistanceTo(corner[2]);
    double c = corner[2].getDistanceTo(corner[0]);
    if (Math::fuzzyCompare(a, 0.0) || Math::fuzzyCompare(b, 0.0) ||
        Math::fuzzyCompare(c, 0.0)) {
        return 0.0;
    }
    double s = (a + b + c) / 2;
    double rootTerm = fabs(s * (s - a) * (s - b) * (s - c));
    return sqrt(rootTerm);
}

Vec2d Triangle::getCorner(int i) const
{
    if (i < 0 || i > 2) {
        return Vec2d::invalid;
    }

    return corner[i];
}

void Triangle::setCorner(int i, const Vec2d &p)
{
    if (i < 0 || i > 2) {
        return;
    }

    corner[i] = p;
}

void Triangle::setCorners(const Vec2d &c1, const Vec2d &c2, const Vec2d &c3)
{
    corner[0] = c1;
    corner[1] = c2;
    corner[2] = c3;
}

double Triangle::getDistanceTo(const Vec2d &point, bool limited,
                               double strictRange) const
{
    Vec2d normal = getNormal();
    double d = getD();
    double distance =
        (normal.x * point.x + normal.y * point.y + normal.z * point.z + d) /
        (normal.getMagnitude());

    if (!limited ||
        isPointInTriangle(point - normal.getUnitVector() * distance)) {
        return distance;
    }

    return RMAXDOUBLE;
}

Vec2d Triangle::getVectorTo(const Vec2d &point, bool limited,
                            double strictRange) const
{
    // assert(false);
    Line l1(corner[0], corner[1]);
    Line l2(corner[1], corner[2]);
    Line l3(corner[2], corner[0]);

    Vec2d v1 = l1.getVectorTo(point, limited, strictRange);
    Vec2d v2 = l2.getVectorTo(point, limited, strictRange);
    Vec2d v3 = l3.getVectorTo(point, limited, strictRange);

    double m1 = v1.getMagnitude();
    double m2 = v2.getMagnitude();
    double m3 = v3.getMagnitude();

    if (m1 < m2 && m1 < m3) {
        return v1;
    }
    else if (m2 < m3) {
        return v2;
    }
    else {
        return v3;
    }
}

/*
Vec2d Triangle::getVectorTo(const Line& / *line* /, bool / *limited* /)
const { assert(false); return Vec2d();
}
*/

std::vector<Vec2d> Triangle::getEndPoints() const
{
    std::vector<Vec2d> c;

    c.push_back(corner[0]);
    c.push_back(corner[1]);
    c.push_back(corner[2]);

    return c;
}

std::vector<Vec2d> Triangle::getMiddlePoints() const
{
    std::vector<Vec2d> c;

    c.push_back((corner[0] + corner[1]) / 2.0);
    c.push_back((corner[1] + corner[2]) / 2.0);
    c.push_back((corner[2] + corner[0]) / 2.0);

    return c;
}

std::vector<Vec2d> Triangle::getCenterPoints() const
{
    return getMiddlePoints();
}

std::vector<Vec2d> Triangle::getPointsWithDistanceToEnd(double distance,
                                                        int from) const
{
    std::vector<Vec2d> c;

    Line l1(corner[0], corner[1]);
    Line l2(corner[1], corner[2]);
    Line l3(corner[2], corner[0]);

    c.push_back(l1.getPointsWithDistanceToEnd(distance));
    c.push_back(l2.getPointsWithDistanceToEnd(distance));
    c.push_back(l3.getPointsWithDistanceToEnd(distance));

    return c;
}

Vec2d Triangle::getNormal() const
{
    return Vec2d::getCrossProduct(corner[0] - corner[2], corner[1] - corner[2]);
}

bool Triangle::isPointInTriangle(const Vec2d &ip, bool treatAsQuadrant) const
{
    Vec2d normal = getNormal();

    Vec2d f;
    if (std::fabs(normal.x) > std::fabs(normal.y) &&
        std::fabs(normal.x) > std::fabs(normal.z)) {
        // drop x component for inside test:
        f = Vec2d(0, 1, 1);
    }
    else if (std::fabs(normal.y) > std::fabs(normal.z)) {
        // drop y component for inside test:
        f = Vec2d(1, 0, 1);
    }
    else {
        // drop z component for inside test:
        f = Vec2d(1, 1, 0);
    }

    Vec2d p = ip.getMultipliedComponents(f);
    Vec2d a = corner[0].getMultipliedComponents(f);
    Vec2d b = corner[1].getMultipliedComponents(f);
    Vec2d c = corner[2].getMultipliedComponents(f);

    Vec2d v0 = c - a;
    Vec2d v1 = b - a;
    Vec2d v2 = p - a;

    double dot00 = Vec2d::getDotProduct(v0, v0);
    double dot01 = Vec2d::getDotProduct(v0, v1);
    double dot02 = Vec2d::getDotProduct(v0, v2);
    double dot11 = Vec2d::getDotProduct(v1, v1);
    double dot12 = Vec2d::getDotProduct(v1, v2);

    double invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
    double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    double v = (dot00 * dot12 - dot01 * dot02) * invDenom;

    return (u > 0.0 && v > 0.0 && (treatAsQuadrant || u + v < 1.0));
}

bool Triangle::isPointInQuadrant(const Vec2d &ip) const
{
    return isPointInTriangle(ip, true);
}

double Triangle::getD() const
{
    Vec2d normal = getNormal();
    return -normal.x * corner[0].x - normal.y * corner[0].y -
           normal.z * corner[0].z;
}

std::vector<std::shared_ptr<Shape>> Triangle::getExploded(int segments) const
{
    std::vector<std::shared_ptr<Shape>> ret;

    for (int i = 0; i < 3; i++) {
        ret.push_back(
            std::shared_ptr<Shape>(new Line(corner[i], corner[(i + 1) % 3])));
    }

    return ret;
}

} // namespace cada